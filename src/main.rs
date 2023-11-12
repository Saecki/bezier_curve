use eframe::{CreationContext, NativeOptions};
use egui::{
    Align2, CentralPanel, Color32, DragValue, FontFamily, FontId, Frame, Id, Pos2, Rect, Sense,
    SidePanel, Slider, Stroke, Vec2,
};
use egui_plot::{Line, Plot, PlotPoints};
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
    params: Params,
    #[serde(skip)]
    output: Output,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
struct Params {
    control_points: Vec<Pos2>,
    num_line_segments: u32,
    animate_constant_speed: bool,
    bernstein: bool,
    show_lerp_lines: bool,
    show_lerp_points: bool,
    show_last_lerp_point: bool,
    animate: bool,
    animate_curve: bool,

    animate_direction: f32,
    u: f32,
}

#[derive(Clone, Debug, Default)]
struct Output {
    curve_points: Vec<Pos2>,
    distance_table: Vec<f32>,
    animate_curve_idx: usize,
    constant_speed_u: f32,
    lerp_points: Vec<Vec<Pos2>>,
}

impl Default for SplineApp {
    fn default() -> Self {
        let params = Params {
            control_points: vec![
                Pos2::new(100.0, 400.0),
                Pos2::new(150.0, 100.0),
                Pos2::new(550.0, 100.0),
                Pos2::new(600.0, 400.0),
            ],
            bernstein: false,
            num_line_segments: 50,
            show_lerp_lines: true,
            show_lerp_points: true,
            show_last_lerp_point: true,
            animate: false,
            animate_constant_speed: false,
            animate_curve: false,

            animate_direction: 1.0,
            u: 0.3,
        };
        let output = compute(&params);
        Self { params, output }
    }
}

impl SplineApp {
    fn new(cc: &CreationContext) -> Self {
        if let Some(storage) = cc.storage {
            if let Some(mut app) = eframe::get_value::<SplineApp>(storage, eframe::APP_KEY) {
                app.output = compute(&app.params);
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
                let params = &mut self.params;

                ui.add_space(30.0);
                ui.checkbox(&mut params.show_lerp_lines, "show construction lines");
                ui.checkbox(&mut params.show_lerp_points, "show construction points");
                ui.checkbox(&mut params.show_last_lerp_point, "show constructed point");
                ui.checkbox(&mut params.animate, "animate");
                ui.checkbox(&mut params.animate_curve, "animate curve");
                ui.checkbox(&mut params.animate_constant_speed, "constant speed");
                ui.label("u");

                let slider = Slider::new(&mut params.u, 0.0..=1.0)
                    .fixed_decimals(4)
                    .drag_value_speed(0.002);
                changed |= ui.add(slider).changed();

                let slider =
                    Slider::new(&mut self.output.constant_speed_u, 0.0..=1.0).fixed_decimals(4);
                ui.add_enabled(false, slider);

                ui.add_space(30.0);
                changed |= ui.checkbox(&mut params.bernstein, "bernstein").changed();

                ui.add_space(30.0);
                ui.label("line segments");
                changed |= ui
                    .add(DragValue::new(&mut params.num_line_segments).clamp_range(1..=1000))
                    .changed();

                ui.add_space(30.0);
                ui.horizontal(|ui| {
                    if ui.button("-").clicked() && params.control_points.len() > 2 {
                        let mid = (params.control_points.len() - 1) / 2;
                        params.control_points.remove(mid);
                        changed = true;
                    }

                    if ui.button("+").clicked() {
                        let mid = (params.control_points.len() - 1) / 2;
                        let mid_point =
                            params.control_points[mid].lerp(params.control_points[mid + 1], 0.5);
                        params.control_points.insert(mid + 1, mid_point);
                        changed = true;
                    }
                });

                for (i, p) in params.control_points.iter().enumerate() {
                    ui.horizontal(|ui| {
                        ui.label(format!("{i}"));

                        let Pos2 { x, y } = p;
                        ui.label(format!("({x}, {y})"));
                    });
                }

                ui.add_space(30.0);
                Plot::new("distance_table").show_axes(false).show(ui, |ui| {
                    let values = self.output.distance_table.iter().enumerate().map(|(i, d)| {
                        let n = self.output.distance_table.len() - 1;
                        let x = i as f64 / n as f64;
                        [x, *d as f64]
                    });
                    let points = PlotPoints::from_iter(values);
                    let line = Line::new(points);
                    ui.line(line);
                });
            });

        CentralPanel::default()
            .frame(Frame::none().fill(Color32::from_gray(0x20)))
            .show(ctx, |ui| {
                let params = &mut self.params;

                let clamp_margin = Vec2::splat(control_point_radius);
                let clamp_min = clamp_margin.to_pos2();
                let clamp_max = (ui.available_size() - clamp_margin).to_pos2();
                for (i, pos) in params.control_points.iter_mut().enumerate() {
                    let rect = Rect::from_center_size(*pos, Vec2::splat(2.0 * point_radius));
                    let resp = ui.interact(rect, Id::new("control_point").with(i), Sense::drag());
                    if resp.dragged() {
                        *pos += resp.drag_delta();

                        changed = true;
                    }

                    // clamp inside screen bounds
                    *pos = pos.clamp(clamp_min, clamp_max);
                }

                if params.animate {
                    // force refresh when animating
                    ui.ctx().request_repaint();

                    let delta = ui.input(|i| i.stable_dt) / 2.5;
                    params.u += params.animate_direction * delta;
                    params.u = params.u.clamp(0.0, 1.0);

                    if params.u == 1.0 {
                        params.animate_direction = -1.0;
                    } else if params.u == 0.0 {
                        params.animate_direction = 1.0;
                    }
                    changed = true;
                }

                if changed {
                    self.output = compute(params);
                }

                let out = &self.output;
                let painter = ui.painter();

                // curve
                let curve_stroke = Stroke::new(3.0, Color32::GREEN);
                if !params.animate_curve {
                    for p in out.curve_points.windows(2) {
                        painter.line_segment([p[0], p[1]], curve_stroke);
                    }
                } else {
                    for p in out.curve_points[0..=out.animate_curve_idx].windows(2) {
                        painter.line_segment([p[0], p[1]], curve_stroke);
                    }
                    let last_point = out.curve_points[out.animate_curve_idx];
                    let lerp_point = out.lerp_points[out.lerp_points.len() - 1][0];
                    painter.line_segment([last_point, lerp_point], curve_stroke);
                }

                // control point lines
                for p in params.control_points.windows(2) {
                    let stroke = Stroke::new(2.0, Color32::BLUE);
                    painter.line_segment([p[0], p[1]], stroke);
                }

                // interpolated points
                for points in out.lerp_points.iter() {
                    if params.show_lerp_lines {
                        for p in points.windows(2) {
                            let stroke = Stroke::new(1.0, Color32::LIGHT_BLUE);
                            painter.line_segment([p[0], p[1]], stroke);
                        }
                    }

                    if points.len() == 1 {
                        if params.show_last_lerp_point {
                            let color = Color32::from_rgb(0xF0, 0x20, 0xF0);
                            painter.circle_filled(points[0], point_radius, color);
                        }
                    } else if params.show_lerp_points {
                        for p in points.iter() {
                            painter.circle_filled(*p, point_radius, Color32::GOLD);
                        }
                    }
                }

                // control points
                for (i, p) in params.control_points.iter().enumerate() {
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

fn compute(params: &Params) -> Output {
    let curve_points = compute_points(
        &params.control_points,
        params.num_line_segments,
        params.bernstein,
    );
    let distance_table = compute_distance_table(&curve_points);

    let constant_speed_u = constant_speed_u(params.u, &distance_table);
    let u = if params.animate_constant_speed {
        constant_speed_u
    } else {
        params.u
    };
    let lerp_points = compute_lerp_points(u, &params.control_points);
    let animate_curve_idx = find_last_line_idx(u, &curve_points);
    Output {
        curve_points,
        distance_table,
        animate_curve_idx,
        constant_speed_u,
        lerp_points,
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
