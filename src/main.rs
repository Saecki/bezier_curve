use eframe::{CreationContext, NativeOptions};
use egui::{
    Align2, CentralPanel, Color32, DragValue, FontFamily, FontId, Frame, Id, Pos2, Rect, Sense,
    SidePanel, Slider, Stroke, Vec2,
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
    distance_table: Vec<f32>,
    animate_constant_speed: bool,
    bernstein: bool,

    show_lerp_lines: bool,
    show_lerp_points: bool,
    show_last_lerp_point: bool,
    animate_u: bool,
    animate_curve: bool,
    animate_curve_idx: usize,
    animate_direction: f32,
    u: f32,
    constant_speed_u: f32,
    #[serde(default)]
    lerp_points: Vec<Vec<Pos2>>,
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
        let distance_table = compute_distance_table(&curve_points);

        let u = 0.3;
        let animate_curve_idx = find_last_line_idx(u, &curve_points);
        let constant_speed_u = constant_speed_u(u, &distance_table);
        let lerp_points = compute_lerp_points(u, &control_points);
        Self {
            control_points,
            num_line_segments: 50,
            curve_points,
            distance_table,
            bernstein,
            show_lerp_lines: true,
            show_lerp_points: true,
            show_last_lerp_point: true,
            animate_u: false,
            animate_curve: false,
            animate_curve_idx,
            animate_constant_speed: false,
            animate_direction: 1.0,
            u,
            constant_speed_u,
            lerp_points,
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
        let control_point_radius = 12.5;
        let point_radius = 8.0;

        let mut changed = false;
        SidePanel::right("settings")
            .resizable(false)
            .exact_width(400.0)
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
                ui.checkbox(&mut self.show_lerp_lines, "show construction lines");
                ui.checkbox(&mut self.show_lerp_points, "show construction points");
                ui.checkbox(&mut self.show_last_lerp_point, "show animated point");
                ui.checkbox(&mut self.animate_u, "animate");
                ui.checkbox(&mut self.animate_curve, "animate curve");
                ui.checkbox(&mut self.animate_constant_speed, "constant speed");
                ui.label("u");

                let slider = Slider::new(&mut self.u, 0.0..=1.0)
                    .fixed_decimals(4)
                    .drag_value_speed(0.002);
                changed |= ui.add(slider).changed();

                let slider = Slider::new(&mut self.constant_speed_u, 0.0..=1.0).fixed_decimals(4);
                ui.add_enabled(false, slider);

                ui.add_space(30.0);
                for (i, p) in self.control_points.iter().enumerate() {
                    ui.horizontal(|ui| {
                        ui.label(format!("{i}"));

                        let Pos2 { x, y } = p;
                        ui.label(format!("({x}, {y})"));
                    });
                }
            });

        CentralPanel::default()
            .frame(Frame::none().fill(Color32::from_gray(0x20)))
            .show(ctx, |ui| {
                let clamp_margin = Vec2::splat(control_point_radius);
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

                if self.animate_u {
                    // force refresh when animating
                    ui.ctx().request_repaint();

                    let delta = ui.input(|i| i.stable_dt) / 2.5;
                    self.u += self.animate_direction * delta;
                    self.u = self.u.clamp(0.0, 1.0);

                    if self.u == 1.0 {
                        self.animate_direction = -1.0;
                    } else if self.u == 0.0 {
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

                    self.distance_table = compute_distance_table(&self.curve_points);
                    self.constant_speed_u = constant_speed_u(self.u, &self.distance_table);
                    let u = if self.animate_constant_speed {
                        self.constant_speed_u
                    } else {
                        self.u
                    };
                    self.animate_curve_idx = find_last_line_idx(u, &self.curve_points);
                    self.lerp_points = compute_lerp_points(u, &self.control_points);
                }

                let painter = ui.painter();

                // curve
                let curve_stroke = Stroke::new(3.0, Color32::GREEN);
                if !self.animate_curve {
                    for p in self.curve_points.windows(2) {
                        painter.line_segment([p[0], p[1]], curve_stroke);
                    }
                } else {
                    for p in self.curve_points[0..=self.animate_curve_idx].windows(2) {
                        painter.line_segment([p[0], p[1]], curve_stroke);
                    }
                    let last_point = self.curve_points[self.animate_curve_idx];
                    let lerp_point = self.lerp_points[self.lerp_points.len() - 1][0];
                    painter.line_segment([last_point, lerp_point], curve_stroke);
                }

                // control point lines
                for p in self.control_points.windows(2) {
                    let stroke = Stroke::new(2.0, Color32::BLUE);
                    painter.line_segment([p[0], p[1]], stroke);
                }

                // interpolated points
                for points in self.lerp_points.iter() {
                    if self.show_lerp_lines {
                        for p in points.windows(2) {
                            let stroke = Stroke::new(1.0, Color32::LIGHT_BLUE);
                            painter.line_segment([p[0], p[1]], stroke);
                        }
                    }

                    if points.len() == 1 {
                        if self.show_last_lerp_point {
                            let color = Color32::from_rgb(0xF0, 0x20, 0xF0);
                            painter.circle_filled(points[0], point_radius, color);
                        }
                    } else if self.show_lerp_points {
                        for p in points.iter() {
                            painter.circle_filled(*p, point_radius, Color32::GOLD);
                        }
                    }
                }

                // control points
                for (i, p) in self.control_points.iter().enumerate() {
                    painter.circle_filled(*p, control_point_radius, Color32::RED);
                    let font = FontId::new(1.5 * control_point_radius, FontFamily::Proportional);
                    painter.text(
                        *p,
                        Align2::CENTER_CENTER,
                        i.to_string(),
                        font,
                        Color32::WHITE,
                    );
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

fn compute_lerp_points(u: f32, control_points: &[Pos2]) -> Vec<Vec<Pos2>> {
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

fn compute_distance_table(points: &[Pos2]) -> Vec<f32> {
    let total_distance: f32 = points.windows(2).map(|p| p[0].distance(p[1])).sum();
    let mut current_distance = 0.0;
    let mut table = vec![0.0];
    for p in points.windows(2) {
        let dist = p[0].distance(p[1]);
        current_distance += dist;
        table.push(current_distance / total_distance);
    }
    table
}

fn constant_speed_u(u: f32, distance_table: &[f32]) -> f32 {
    if u == 0.0 || u == 1.0 {
        return u;
    }

    let i = distance_table.iter().position(|t| *t > u).unwrap();
    let in_a = distance_table[i - 1];
    let in_b = distance_table[i];
    let out_a = (i - 1) as f32 / (distance_table.len() - 1) as f32;
    let out_b = (i) as f32 / (distance_table.len() - 1) as f32;

    remap(u, in_a, in_b, out_a, out_b)
}

fn remap(val: f32, in_a: f32, in_b: f32, out_a: f32, out_b: f32) -> f32 {
    let norm = (val - in_a) / (in_b - in_a);
    out_a + norm * (out_b - out_a)
}

fn find_last_line_idx(u: f32, points: &[Pos2]) -> usize {
    let n = (points.len() - 1) as f32;
    (u * n).floor() as usize
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
