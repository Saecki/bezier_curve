use eframe::{CreationContext, NativeOptions};
use egui::{
    CentralPanel, Color32, DragValue, Frame, Id, Pos2, Rect, Sense, SidePanel, Slider, Stroke, Vec2,
};
use serde_derive::{Deserialize, Serialize};

fn main() {
    let native_options = NativeOptions {
        follow_system_theme: true,
        ..Default::default()
    };
    eframe::run_native(
        "bezier curve",
        native_options,
        Box::new(|cc| Box::new(SplineApp::new(cc))),
    )
    .expect("error running app");
}

#[derive(Clone, Debug, Serialize, Deserialize)]
struct SplineApp {
    control_points: Vec<Pos2>,
    num_line_segments: u32,
    curve_points: Vec<Pos2>,
    bernstein: bool,

    show_interp_lines: bool,
    show_interp_points: bool,
    show_last_interp_point: bool,
    animate_interp_val: bool,
    animate_direction: f32,
    interp_val: f32,
    #[serde(default)]
    interp_points: Vec<Vec<Pos2>>,
}

impl Default for SplineApp {
    fn default() -> Self {
        let bernstein = false;
        let control_points = vec![
            Pos2::new(100.0, 400.0),
            Pos2::new(150.0, 100.0),
            Pos2::new(550.0, 100.0),
            Pos2::new(600.0, 400.0),
        ];
        let num_line_segments = 50;
        let curve_points = compute_points(&control_points, num_line_segments, bernstein);

        let interp_val = 0.3;
        let interp_points = compute_interp_points(interp_val, &control_points);
        Self {
            control_points,
            num_line_segments: 50,
            curve_points,
            bernstein,
            show_interp_lines: true,
            show_interp_points: true,
            show_last_interp_point: true,
            animate_interp_val: false,
            animate_direction: 1.0,
            interp_val,
            interp_points,
        }
    }
}

impl SplineApp {
    fn new(cc: &CreationContext) -> Self {
        if let Some(storage) = cc.storage {
            if let Some(app) = eframe::get_value(storage, eframe::APP_KEY) {
                return app;
            }
        }

        Self::default()
    }
}

impl eframe::App for SplineApp {
    fn save(&mut self, storage: &mut dyn eframe::Storage) {
        eframe::set_value(storage, eframe::APP_KEY, self);
    }

    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        let point_radius = 8.0;

        let mut changed = false;
        SidePanel::right("settings")
            .resizable(false)
            .exact_width(200.0)
            .show(ctx, |ui| {
                changed |= ui.checkbox(&mut self.bernstein, "bernstein").changed();
                ui.add_space(30.0);
                ui.label("line segments");
                changed |= ui
                    .add(DragValue::new(&mut self.num_line_segments).clamp_range(1..=1000))
                    .changed();

                ui.add_space(30.0);
                ui.horizontal(|ui| {
                    if ui.button("-").clicked() && self.control_points.len() > 2 {
                        let mid = (self.control_points.len() - 1) / 2;
                        self.control_points.remove(mid);
                        changed = true;
                    }

                    if ui.button("+").clicked() {
                        let mid = (self.control_points.len() - 1) / 2;
                        let mid_point =
                            self.control_points[mid].lerp(self.control_points[mid + 1], 0.5);
                        self.control_points.insert(mid + 1, mid_point);
                        changed = true;
                    }
                });

                ui.add_space(30.0);
                ui.checkbox(&mut self.show_interp_lines, "show interp lines");
                ui.checkbox(&mut self.show_interp_points, "show interp points");
                ui.checkbox(&mut self.show_last_interp_point, "show last interp point");
                ui.checkbox(&mut self.animate_interp_val, "animate");
                ui.label("u");

                let slider = Slider::new(&mut self.interp_val, 0.0..=1.0)
                    .fixed_decimals(4)
                    .drag_value_speed(0.002);
                changed |= ui.add(slider).changed();
            });

        CentralPanel::default()
            .frame(Frame::none().fill(Color32::from_gray(0x20)))
            .show(ctx, |ui| {
                let clamp_margin = Vec2::splat(point_radius);
                let clamp_min = clamp_margin.to_pos2();
                let clamp_max = (ui.available_size() - clamp_margin).to_pos2();
                for (i, pos) in self.control_points.iter_mut().enumerate() {
                    let rect = Rect::from_center_size(*pos, Vec2::splat(2.0 * point_radius));
                    let resp = ui.interact(rect, Id::new("control_point").with(i), Sense::drag());
                    if resp.dragged() {
                        *pos += resp.drag_delta();

                        changed = true;
                    }

                    // clamp inside screen bounds
                    *pos = pos.clamp(clamp_min, clamp_max);
                }

                if self.animate_interp_val {
                    // force refresh when animating
                    ui.ctx().request_repaint();

                    let delta = ui.input(|i| i.stable_dt) / 2.5;
                    self.interp_val += self.animate_direction * delta;
                    self.interp_val = self.interp_val.clamp(0.0, 1.0);

                    if self.interp_val == 1.0 {
                        self.animate_direction = -1.0;
                    } else if self.interp_val == 0.0 {
                        self.animate_direction = 1.0;
                    }
                    changed = true;
                }

                if changed {
                    self.curve_points = compute_points(
                        &self.control_points,
                        self.num_line_segments,
                        self.bernstein,
                    );

                    self.interp_points =
                        compute_interp_points(self.interp_val, &self.control_points);
                }

                let painter = ui.painter();

                // curve
                for p in self.curve_points.windows(2) {
                    let stroke = Stroke::new(3.0, Color32::GREEN);
                    painter.line_segment([p[0], p[1]], stroke);
                }

                // control point lines
                for p in self.control_points.windows(2) {
                    let stroke = Stroke::new(2.0, Color32::BLUE);
                    painter.line_segment([p[0], p[1]], stroke);
                }

                // interpolated points
                for points in self.interp_points.iter() {
                    if self.show_interp_lines {
                        for p in points.windows(2) {
                            let stroke = Stroke::new(1.0, Color32::LIGHT_BLUE);
                            painter.line_segment([p[0], p[1]], stroke);
                        }
                    }

                    if points.len() == 1 {
                        if self.show_last_interp_point {
                            let color = Color32::from_rgb(0xF0, 0x20, 0xF0);
                            painter.circle_filled(points[0], point_radius, color);
                        }
                    } else if self.show_interp_points {
                        for p in points.iter() {
                            painter.circle_filled(*p, point_radius, Color32::GOLD);
                        }
                    }
                }

                // control points
                for p in self.control_points.iter() {
                    painter.circle_filled(*p, point_radius, Color32::RED)
                }
            });
    }
}

fn compute_points(control_points: &[Pos2], num_line_segments: u32, bernstein: bool) -> Vec<Pos2> {
    let mut line = Vec::new();
    for i in 0..=num_line_segments {
        let u = i as f32 / num_line_segments as f32;
        let point = if bernstein {
            compute_point_bernstein(u, control_points)
        } else {
            compute_point(u, control_points)
        };
        line.push(point);
    }
    line
}

fn compute_point_bernstein(u: f32, control_points: &[Pos2]) -> Pos2 {
    let mut accum = Pos2::ZERO;
    let n = control_points.len() - 1;
    for (i, p) in control_points.iter().enumerate() {
        let coefficient = (factorial(n)) / (factorial(n - i) * factorial(i));
        let weight = coefficient as f32 * u.powi(i as i32) * (1.0 - u).powi((n - i) as i32);
        accum += weight * p.to_vec2();
    }
    accum
}

fn factorial(n: usize) -> usize {
    match n {
        0 => 1,
        1 => 1,
        _ => (2..=n).product(),
    }
}

fn compute_interp_points(u: f32, control_points: &[Pos2]) -> Vec<Vec<Pos2>> {
    let mut interp_points = vec![lerp(u, control_points)];
    for i in 1..control_points.len() - 1 {
        let next_points = lerp(u, &interp_points[i - 1]);
        interp_points.push(next_points);
    }
    interp_points
}

fn compute_point(u: f32, control_points: &[Pos2]) -> Pos2 {
    let mut current_points = lerp(u, control_points);
    while current_points.len() > 1 {
        current_points = lerp(u, &current_points);
    }
    current_points[0]
}

fn lerp(u: f32, points: &[Pos2]) -> Vec<Pos2> {
    points.windows(2).map(|p| p[0].lerp(p[1], u)).collect()
}

#[test]
fn factorial_test() {
    assert_eq!(factorial(0), 1);
    assert_eq!(factorial(1), 1);
    assert_eq!(factorial(2), 2);
    assert_eq!(factorial(3), 6);
    assert_eq!(factorial(4), 24);
    assert_eq!(factorial(5), 120);
}
