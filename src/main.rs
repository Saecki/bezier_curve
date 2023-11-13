use eframe::{CreationContext, NativeOptions};
use egui::emath::Rot2;
use egui::scroll_area::ScrollBarVisibility;
use egui::{
    Align2, Button, CentralPanel, Color32, DragValue, FontFamily, FontId, Frame, Id, Key,
    Modifiers, Painter, Pos2, Rect, ScrollArea, Sense, SidePanel, Slider, Stroke, Ui, Vec2,
};
use egui_plot::{Arrows, Corner, Legend, Line, Plot, PlotPoints, PlotUi};
use serde_derive::{Deserialize, Serialize};

const DISTANCE_MAP_COLOR: Color32 = Color32::from_rgb(0xF0, 0x60, 0x80);
const VELOCITY_COLOR: Color32 = Color32::from_rgb(0xF0, 0xB0, 0x20);
const ACC_COLOR: Color32 = Color32::from_rgb(0x20, 0xB0, 0xF0);
const WIDGET_SPACING: f32 = 8.0;

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

impl Default for SplineApp {
    fn default() -> Self {
        let params = Params {
            control_points: vec![
                Pos2::new(100.0, 400.0),
                Pos2::new(150.0, 100.0),
                Pos2::new(550.0, 100.0),
                Pos2::new(600.0, 400.0),
            ],
            num_line_segments: 50,
            show_control_lines: true,
            show_control_points: true,
            show_lerp_lines: true,
            show_lerp_points: true,
            show_last_lerp_point: true,
            show_velocity_vector_on_curve: false,
            show_acc_vector_on_curve: false,
            show_velocity_vector_in_plot: false,
            show_acc_vector_in_plot: false,
            animate: false,
            animation_time: 2.5,
            wrap_animation: false,
            animate_constant_speed: false,
            animate_curve: false,

            movement_direction: 1.0,
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

#[derive(Clone, Debug, Serialize, Deserialize)]
struct Params {
    control_points: Vec<Pos2>,
    num_line_segments: u32,
    animate_constant_speed: bool,
    show_control_lines: bool,
    show_control_points: bool,
    show_lerp_lines: bool,
    show_lerp_points: bool,
    show_last_lerp_point: bool,
    show_velocity_vector_on_curve: bool,
    show_acc_vector_on_curve: bool,
    show_velocity_vector_in_plot: bool,
    show_acc_vector_in_plot: bool,
    animate: bool,
    animation_time: f32,
    wrap_animation: bool,
    animate_curve: bool,

    movement_direction: f32,
    u: f32,
}

#[derive(Clone, Debug, Default)]
struct Output {
    curve_points: Vec<Pos2>,
    distance_table: Vec<f32>,
    velocity_curve: Vec<Vec2>,
    acc_curve: Vec<Vec2>,
    animate_curve_idx: usize,
    constant_speed_u: f32,
    lerp_points: Vec<Vec<Pos2>>,
    current_velocity: Vec2,
    current_acc: Vec2,
}

impl Output {
    fn constructed_point(&self) -> Pos2 {
        self.lerp_points[self.lerp_points.len() - 1][0]
    }
}

impl eframe::App for SplineApp {
    fn save(&mut self, storage: &mut dyn eframe::Storage) {
        eframe::set_value(storage, eframe::APP_KEY, self);
    }

    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        let control_point_radius = 12.5;
        let point_radius = 8.0;
        let velocity_scale = 0.1;
        let acc_scale = 0.025;

        let mut changed = false;

        ctx.input_mut(|i| {
            if i.consume_key(Modifiers::COMMAND, Key::Space) {
                self.params.animate.toggle();
            } else if i.consume_key(Modifiers::COMMAND, Key::ArrowRight) {
                self.params.movement_direction = 1.0;
            } else if i.consume_key(Modifiers::COMMAND, Key::ArrowLeft) {
                self.params.movement_direction = -1.0;
            } else if i.consume_key(Modifiers::COMMAND, Key::ArrowUp) {
                self.params.animation_time += 0.1;
            } else if i.consume_key(Modifiers::COMMAND, Key::ArrowDown) {
                self.params.animation_time -= 0.1;
            }
        });

        SidePanel::right("settings")
            .resizable(false)
            .show_separator_line(false)
            .exact_width(400.0)
            .show(ctx, |ui| {
                ScrollArea::vertical()
                    .max_height(ui.available_height() - 2.0 * WIDGET_SPACING - ui.available_width())
                    .scroll_bar_visibility(ScrollBarVisibility::AlwaysVisible)
                    .show(ui, |ui| {
                        changed |= draw_sidebar(ui, self);

                        // fill entire width
                        ui.allocate_space(Vec2::new(ui.available_width(), 0.0));
                    });

                ui.add_space(WIDGET_SPACING);
                Plot::new("distance_table")
                    .height(ui.available_width())
                    .show_axes(false)
                    .data_aspect(1.0)
                    .view_aspect(1.0)
                    .legend(
                        Legend::default()
                            .background_alpha(0.5)
                            .position(Corner::LeftTop),
                    )
                    .show(ui, |ui| {
                        let params = &self.params;
                        let out = &self.output;
                        let values = out.distance_table.iter().enumerate().map(|(i, d)| {
                            let n = out.distance_table.len() - 1;
                            let x = i as f64 / n as f64;
                            [x, *d as f64]
                        });
                        let points = PlotPoints::from_iter(values);
                        let line = Line::new(points)
                            .name("1. distance map")
                            .color(DISTANCE_MAP_COLOR)
                            .width(2.0);
                        ui.line(line);

                        // velocity curve
                        draw_vector_curve(
                            ui,
                            "2. velocity",
                            &out.velocity_curve,
                            out.current_velocity,
                            out.animate_curve_idx,
                            VELOCITY_COLOR,
                            params.animate_curve,
                            params.show_velocity_vector_in_plot,
                        );

                        // acc curve
                        draw_vector_curve(
                            ui,
                            "3. acceleration",
                            &out.acc_curve,
                            out.current_acc,
                            out.animate_curve_idx,
                            ACC_COLOR,
                            params.animate_curve,
                            params.show_acc_vector_in_plot,
                        );
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

                    let delta = ui.input(|i| i.stable_dt) / params.animation_time;
                    params.u += params.movement_direction * delta;
                    params.u = params.u.clamp(0.0, 1.0);

                    if params.u == 1.0 {
                        if params.wrap_animation {
                            params.u = 0.0;
                        } else {
                            params.movement_direction = -1.0;
                        }
                    } else if params.u == 0.0 {
                        if params.wrap_animation {
                            params.u = 1.0;
                        } else {
                            params.movement_direction = 1.0;
                        }
                    }
                    changed = true;
                }

                if changed {
                    self.output = compute(params);
                }

                let out = &self.output;
                let clip_rect = Rect::from_min_size(ui.cursor().min, ui.available_size());
                let painter = ui.painter_at(clip_rect);

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
                    let lerp_point = out.constructed_point();
                    painter.line_segment([last_point, lerp_point], curve_stroke);
                }

                // control point lines
                if params.show_control_lines {
                    for p in params.control_points.windows(2) {
                        let stroke = Stroke::new(2.0, Color32::BLUE);
                        painter.line_segment([p[0], p[1]], stroke);
                    }
                }

                // vectors
                if params.show_velocity_vector_on_curve {
                    let stroke = Stroke::new(2.0, VELOCITY_COLOR);
                    let lerp_point = out.constructed_point();
                    let v = params.movement_direction * velocity_scale * out.current_velocity;
                    draw_arrow(&painter, lerp_point, v, 20.0, stroke);
                }
                if params.show_acc_vector_on_curve {
                    let stroke = Stroke::new(2.0, ACC_COLOR);
                    let lerp_point = out.constructed_point();
                    let v = acc_scale * out.current_acc;
                    draw_arrow(&painter, lerp_point, v, 20.0, stroke);
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
                if params.show_control_points {
                    for (i, p) in params.control_points.iter().enumerate() {
                        painter.circle_filled(*p, control_point_radius, Color32::RED);
                        let font =
                            FontId::new(1.5 * control_point_radius, FontFamily::Proportional);
                        painter.text(
                            *p,
                            Align2::CENTER_CENTER,
                            i.to_string(),
                            font,
                            Color32::WHITE,
                        );
                    }
                }
            });
    }
}

fn draw_sidebar(ui: &mut Ui, app: &mut SplineApp) -> bool {
    let mut changed = false;
    let params = &mut app.params;

    ui.add_space(WIDGET_SPACING);
    ui.checkbox(&mut params.show_control_lines, "show control lines");
    ui.checkbox(&mut params.show_control_points, "show control points");

    ui.add_space(WIDGET_SPACING);
    ui.checkbox(&mut params.show_lerp_lines, "show construction lines");
    ui.checkbox(&mut params.show_lerp_points, "show construction points");
    ui.checkbox(&mut params.show_last_lerp_point, "show constructed point");

    ui.add_space(WIDGET_SPACING);
    ui.checkbox(
        &mut params.show_velocity_vector_on_curve,
        "show velocity vector on curve",
    );
    ui.checkbox(
        &mut params.show_acc_vector_on_curve,
        "show acceleration vector on curve",
    );

    ui.add_space(WIDGET_SPACING);
    ui.horizontal(|ui| {
        ui.selectable_value(&mut params.movement_direction, -1.0, "\u{23ea}");
        let play_pause = if params.animate {
            "\u{23f8}"
        } else {
            "\u{25b6}"
        };
        if ui.add(Button::new(play_pause)).clicked() {
            params.animate.toggle();
        }
        ui.selectable_value(&mut params.movement_direction, 1.0, "\u{23e9}");
    });
    ui.checkbox(&mut params.wrap_animation, "wrap animation");
    ui.checkbox(&mut params.animate_curve, "animate curve");
    ui.checkbox(&mut params.animate_constant_speed, "constant speed");
    ui.horizontal(|ui| {
        ui.add(
            DragValue::new(&mut params.animation_time)
                .speed(0.1)
                .clamp_range(0.1..=100.0),
        );
        ui.label("animation time");
    });

    ui.horizontal(|ui| {
        let slider = Slider::new(&mut params.u, 0.0..=1.0)
            .fixed_decimals(4)
            .drag_value_speed(0.002);
        let resp = ui.add(slider);
        ui.label("u");

        if resp.changed() {
            params.movement_direction = resp.drag_delta().x.signum();
            changed = true;
        }
    });

    ui.horizontal(|ui| {
        let slider = Slider::new(&mut app.output.constant_speed_u, 0.0..=1.0).fixed_decimals(4);
        ui.add_enabled(false, slider);
        ui.label("u constant speed");
    });

    ui.add_space(WIDGET_SPACING);
    ui.horizontal(|ui| {
        changed |= ui
            .add(DragValue::new(&mut params.num_line_segments).clamp_range(1..=1000))
            .changed();
        ui.label("line segments");
    });

    ui.add_space(WIDGET_SPACING);
    ui.horizontal(|ui| {
        if ui.button("-").clicked() && params.control_points.len() > 2 {
            let mid = (params.control_points.len() - 1) / 2;
            params.control_points.remove(mid);
            changed = true;
        }

        if ui.button("+").clicked() {
            let mid = (params.control_points.len() - 1) / 2;
            let mid_point = params.control_points[mid].lerp(params.control_points[mid + 1], 0.5);
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

    ui.add_space(WIDGET_SPACING);
    ui.checkbox(
        &mut params.show_velocity_vector_in_plot,
        "show velocity vector in plot",
    );
    ui.checkbox(
        &mut params.show_acc_vector_in_plot,
        "show acceleration vector in plot",
    );

    changed
}

fn draw_arrow(painter: &Painter, origin: Pos2, vec: Vec2, tip_length: f32, stroke: Stroke) {
    let rot = Rot2::from_angle(std::f32::consts::TAU / 10.0);
    let tip = origin + vec;
    let dir = vec.normalized();
    painter.line_segment([origin, tip], stroke);
    painter.line_segment([tip, tip - tip_length * (rot * dir)], stroke);
    painter.line_segment([tip, tip - tip_length * (rot.inverse() * dir)], stroke);
}

fn draw_vector_curve(
    ui: &mut PlotUi,
    name: &str,
    values: &[Vec2],
    current: Vec2,
    current_idx: usize,
    color: Color32,
    animate: bool,
    show_vector: bool,
) {
    fn vec2_to_plot_point(v: Vec2) -> [f64; 2] {
        // invert y axis
        [0.5 + v.x as f64, 0.5 - v.y as f64]
    }

    let max = values
        .iter()
        .map(|v| v.abs().max_elem())
        .max_by(|a, b| a.total_cmp(&b))
        .unwrap();
    let scale = 0.5 / max;
    let points = if !animate {
        let values = values.iter().map(|v| vec2_to_plot_point(*v * scale));
        PlotPoints::from_iter(values)
    } else {
        let values = values[0..=current_idx]
            .iter()
            .chain(std::iter::once(&current))
            .map(|v| vec2_to_plot_point(*v * scale));
        PlotPoints::from_iter(values)
    };
    let line = Line::new(points).color(color).name(name).width(2.0);
    ui.line(line);

    if show_vector {
        let tip = vec2_to_plot_point(scale * current);
        let arrow = Arrows::new(vec![[0.5; 2]], vec![tip])
            .color(color)
            .highlight(true)
            .tip_length(15.0);
        ui.arrows(arrow);
    }
}

fn compute(params: &Params) -> Output {
    let curve_points = compute_points(&params.control_points, params.num_line_segments);
    let distance_table = compute_distance_table(&curve_points);

    let velocity_curve = compute_vectors(
        &params.control_points,
        params.num_line_segments,
        compute_velocity,
    );
    let acc_curve = compute_vectors(
        &params.control_points,
        params.num_line_segments,
        compute_acc,
    );

    let constant_speed_u = constant_speed_u(params.u, &distance_table);
    let u = if params.animate_constant_speed {
        constant_speed_u
    } else {
        params.u
    };
    let lerp_points = compute_lerp_points(u, &params.control_points);
    let animate_curve_idx = find_last_line_idx(u, &curve_points);
    let current_velocity = compute_velocity(u, &params.control_points);
    let current_acc = compute_acc(u, &params.control_points);
    Output {
        curve_points,
        distance_table,
        velocity_curve,
        acc_curve,
        animate_curve_idx,
        constant_speed_u,
        lerp_points,
        current_velocity,
        current_acc,
    }
}

fn compute_points(control_points: &[Pos2], num_line_segments: u32) -> Vec<Pos2> {
    let mut line = Vec::new();
    for i in 0..=num_line_segments {
        let u = i as f32 / num_line_segments as f32;
        let point = compute_point(u, control_points);
        line.push(point);
    }
    line
}

fn compute_point(u: f32, control_points: &[Pos2]) -> Pos2 {
    let n = control_points.len() - 1;
    let mut accum = Pos2::ZERO;
    for (i, p) in control_points.iter().enumerate() {
        let weight = weight(u, n, i);
        accum += weight * p.to_vec2();
    }
    accum
}

fn compute_vectors(
    control_points: &[Pos2],
    num_line_segments: u32,
    fun: fn(f32, &[Pos2]) -> Vec2,
) -> Vec<Vec2> {
    let mut line = Vec::new();
    for i in 0..=num_line_segments {
        let u = i as f32 / num_line_segments as f32;
        let point = fun(u, control_points);
        line.push(point);
    }
    line
}

fn compute_velocity(u: f32, control_points: &[Pos2]) -> Vec2 {
    let n = control_points.len() - 1;
    let mut accum = Vec2::ZERO;
    for (i, p) in control_points.windows(2).enumerate() {
        let weight = weight(u, n - 1, i);
        accum += weight * (p[1] - p[0]);
    }
    n as f32 * accum
}

fn compute_acc(u: f32, control_points: &[Pos2]) -> Vec2 {
    let n = control_points.len() - 1;
    let mut accum = Vec2::ZERO;
    for (i, p) in control_points.windows(3).enumerate() {
        let weight = weight(u, n - 2, i);
        accum += weight * ((p[2] - p[1]) - (p[1] - p[0]));
    }
    (n * (n - 1)) as f32 * accum
}

fn weight(u: f32, n: usize, i: usize) -> f32 {
    let coefficient = (factorial(n)) / (factorial(n - i) * factorial(i));
    coefficient as f32 * u.powi(i as i32) * (1.0 - u).powi((n - i) as i32)
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

trait BoolExt {
    fn toggle(&mut self);
}

impl BoolExt for bool {
    fn toggle(&mut self) {
        *self = !*self;
    }
}
