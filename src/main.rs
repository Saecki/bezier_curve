use std::f32::consts::TAU;
use std::ops::RangeInclusive;

use eframe::{CreationContext, NativeOptions};
use egui::emath::Rot2;
use egui::scroll_area::ScrollBarVisibility;
use egui::{
    Align, Align2, Button, CentralPanel, Color32, DragValue, FontFamily, FontId, Frame, Id, Key,
    Label, LayerId, Layout, Margin, Modifiers, Order, Painter, Pos2, Rect, RichText, Rounding,
    ScrollArea, Sense, Shape, SidePanel, Slider, Stroke, Ui, Vec2,
};
use egui_plot::{Arrows, Corner, Legend, Line, Plot, PlotBounds, PlotPoint, PlotPoints, PlotUi};
use serde_derive::{Deserialize, Serialize};

#[cfg(test)]
mod test;

const DISTANCE_COLOR: Color32 = Color32::from_rgb(0x70, 0xF0, 0x20);
const VELOCITY_COLOR: Color32 = Color32::from_rgb(0xF0, 0xB0, 0x20);
const ACC_COLOR: Color32 = Color32::from_rgb(0x20, 0xB0, 0xF0);
const CURVATURE_COLOR: Color32 = Color32::from_rgb(0xFF, 0x50, 0xF0);
const WIDGET_SPACING: f32 = 8.0;

const CONTROL_POINT_RADIUS: f32 = 10.0;
const POINT_RADIUS: f32 = 5.0;
const VELOCITY_SCALE: f32 = 0.1;
const ACC_SCALE: f32 = 0.025;

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
            num_line_segments: 250,
            show_control_lines: true,
            show_control_points: true,
            show_lerp_lines: true,
            show_lerp_points: true,
            show_last_lerp_point: true,
            show_velocity_vector_on_curve: false,
            show_acc_vector_on_curve: false,
            show_curvature_circle_on_curve: false,
            adaptive_curvature_color: false,
            curvature_arc_length: 800.0,
            show_velocity_vector_in_plot: false,
            show_acc_vector_in_plot: false,
            animate: false,
            animation_time: 2.5,
            wrap_animation: false,
            animate_constant_speed: false,
            animate_curve: false,

            plot_tab: PlotTab::Time,
            movement_direction: 1.0,
            u: 0.3,

            drag_state: None,
            hovered_point: None,
        };
        let output = compute(&params);
        Self { params, output }
    }
}

impl SplineApp {
    fn new(cc: &CreationContext) -> Self {
        cc.egui_ctx.style_mut(|style| {
            style.spacing.slider_width = 200.0;
        });

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
    show_curvature_circle_on_curve: bool,
    adaptive_curvature_color: bool,
    curvature_arc_length: f32,
    show_velocity_vector_in_plot: bool,
    show_acc_vector_in_plot: bool,
    animate: bool,
    animation_time: f32,
    wrap_animation: bool,
    animate_curve: bool,

    plot_tab: PlotTab,
    movement_direction: f32,
    u: f32,

    #[serde(skip)]
    drag_state: Option<DragState>,
    hovered_point: Option<RangeInclusive<usize>>,
}

#[derive(Clone, Debug)]
struct DragState {
    locked_axis: Option<Axis>,
    raw_delta: Vec2,
}

impl DragState {
    fn delta(&self) -> Vec2 {
        match self.locked_axis {
            Some(Axis::X) => Vec2::new(self.raw_delta.x, 0.0),
            Some(Axis::Y) => Vec2::new(0.0, self.raw_delta.y),
            None => self.raw_delta,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Axis {
    X,
    Y,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
enum PlotTab {
    Time,
    Spacial,
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

            if i.pointer.is_decidedly_dragging() {
                if let (true, Some(origin), Some(pos)) = (
                    i.pointer.middle_down(),
                    i.pointer.press_origin(),
                    i.pointer.interact_pos(),
                ) {
                    let raw_delta = pos - origin;
                    let drag_state = self.params.drag_state.get_or_insert_with(|| DragState {
                        locked_axis: None,
                        raw_delta,
                    });

                    drag_state.raw_delta = raw_delta;
                    if i.consume_key(Modifiers::NONE, Key::X) {
                        drag_state.locked_axis = match drag_state.locked_axis {
                            Some(Axis::X) => None,
                            Some(Axis::Y) | None => Some(Axis::X),
                        };
                    } else if i.consume_key(Modifiers::NONE, Key::Y) {
                        drag_state.locked_axis = match drag_state.locked_axis {
                            Some(Axis::Y) => None,
                            Some(Axis::X) | None => Some(Axis::Y),
                        };
                    }
                } else {
                    // drag released
                    if let Some(drag_state) = &self.params.drag_state {
                        for p in self.params.control_points.iter_mut() {
                            *p += drag_state.delta();
                        }
                    }
                    changed = true;
                    self.params.drag_state = None;
                };
            }
        });

        SidePanel::right("settings")
            .frame(
                Frame::none()
                    .fill(Color32::from_gray(0x16))
                    .outer_margin(Margin::same(WIDGET_SPACING)),
            )
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
                draw_plots(ui, self);
            });

        CentralPanel::default()
            .frame(Frame::none().fill(Color32::from_gray(0x20)))
            .show(ctx, |ui| {
                main_content(ui, self, changed);
            });
    }
}

fn draw_sidebar(ui: &mut Ui, app: &mut SplineApp) -> bool {
    let mut changed = false;
    let params = &mut app.params;

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
    ui.checkbox(
        &mut params.show_curvature_circle_on_curve,
        "show curvature circle on curve",
    );
    ui.checkbox(
        &mut params.adaptive_curvature_color,
        "adaptive curvature color",
    );
    ui.horizontal(|ui| {
        ui.add(
            DragValue::new(&mut params.curvature_arc_length)
                .speed(1.0)
                .clamp_range(10.0..=4000.0),
        );
        ui.label("curvature arc length");
    });

    ui.add_space(WIDGET_SPACING);
    ui.checkbox(
        &mut params.show_velocity_vector_in_plot,
        "show velocity vector in plot",
    );
    ui.checkbox(
        &mut params.show_acc_vector_in_plot,
        "show acceleration vector in plot",
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
            .drag_value_speed(0.0002);
        let resp = ui.add(slider);
        ui.label("u");

        let delta = resp.drag_delta().x;
        if delta != 0.0 {
            params.movement_direction = delta.signum();
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

    // curve points
    ui.add_space(WIDGET_SPACING);
    ui.heading("Curve points");

    params.hovered_point = None;
    let resp = {
        let (id, rect) = ui.allocate_space(Vec2::new(200.0, 4.0));
        let resp = ui.interact(rect, id, Sense::click());
        if resp.hovered() {
            let painter = ui.painter();
            painter.rect_filled(rect, Rounding::same(2.0), Color32::GRAY);
        }
        resp
    };
    if resp.clicked() {
        let dir = app.output.velocity_curve.first().unwrap().normalized();
        let offset = -2.5 * CONTROL_POINT_RADIUS * dir;
        let pos = *params.control_points.first().unwrap() + offset;
        params.control_points.insert(0, pos);
        changed = true;
    }
    if resp.hovered() {
        params.hovered_point = Some(0..=0);
    }

    let mut i = 0;
    while i < params.control_points.len() {
        let p = &mut params.control_points[i];

        let mut removed = false;
        let resp = Frame::none()
            .rounding(Rounding::same(4.0))
            .fill(Color32::from_gray(0x20))
            .show(ui, |ui| {
                ui.allocate_space(Vec2::new(200.0, 0.0));
                ui.horizontal(|ui| {
                    removed = ui
                        .add_sized(
                            Vec2::new(20.0, ui.style().spacing.interact_size.y),
                            Button::new("\u{1f5d9}").fill(Color32::TRANSPARENT),
                        )
                        .clicked();

                    ui.add_sized(
                        Vec2::new(20.0, ui.style().spacing.interact_size.y),
                        Label::new(format!("{i}")),
                    );

                    ui.add_space(10.0);
                    let Pos2 { x, y } = p;
                    let coord_size = Vec2::new(60.0, ui.style().spacing.interact_size.y);
                    let layout = Layout::centered_and_justified(egui::Direction::LeftToRight)
                        .with_main_align(Align::RIGHT);

                    // preview positions when dragging
                    let (mut preview_x, mut preview_y) = (*x, *y);
                    let (x, y) = match &params.drag_state {
                        Some(drag_state) => {
                            let delta = drag_state.delta();
                            preview_x += delta.x;
                            preview_y += delta.y;
                            (&mut preview_x, &mut preview_y)
                        }
                        None => (x, y),
                    };

                    ui.allocate_ui_with_layout(coord_size, layout, |ui| {
                        changed |= ui.add(DragValue::new(x).fixed_decimals(1)).changed();
                    });

                    ui.allocate_ui_with_layout(coord_size, layout, |ui| {
                        changed |= ui.add(DragValue::new(y).fixed_decimals(1)).changed();
                    });
                })
                .response
            })
            .inner;

        if resp.hovered() {
            resp.highlight();
            params.hovered_point = Some(i..=i);
        }

        let resp = {
            let (id, rect) = ui.allocate_space(Vec2::new(200.0, 4.0));
            let resp = ui.interact(rect, id, Sense::click());
            if resp.hovered() {
                let painter = ui.painter();
                painter.rect_filled(rect, Rounding::same(2.0), Color32::GRAY);
            }
            resp
        };
        if resp.clicked() {
            if i < params.control_points.len() - 1 {
                let mid_point = params.control_points[i].lerp(params.control_points[i + 1], 0.5);
                params.control_points.insert(i + 1, mid_point);
            } else {
                let dir = app.output.velocity_curve.last().unwrap().normalized();
                let offset = 2.5 * CONTROL_POINT_RADIUS * dir;
                let point = *params.control_points.last().unwrap() + offset;
                params.control_points.push(point);
            }
            changed = true;
        }
        if resp.hovered() {
            if i < params.control_points.len() - 1 {
                params.hovered_point = Some(i..=i + 1);
            } else {
                params.hovered_point = Some(i..=i);
            }
        }

        if removed && params.control_points.len() > 3 {
            params.control_points.remove(i);
        } else {
            i += 1;
        }
    }

    changed
}

fn draw_plots(ui: &mut Ui, app: &mut SplineApp) {
    ui.horizontal(|ui| {
        ui.selectable_value(&mut app.params.plot_tab, PlotTab::Time, "time");
        ui.selectable_value(&mut app.params.plot_tab, PlotTab::Spacial, "spacial");
    });

    match app.params.plot_tab {
        PlotTab::Time => draw_time_plots(ui, app),
        PlotTab::Spacial => draw_spacial_plots(ui, app),
    }
}

fn draw_time_plots(ui: &mut Ui, app: &SplineApp) {
    let params = &app.params;
    let out = &app.output;
    Plot::new("time_plot")
        .height(ui.available_width())
        .show_axes(false)
        .allow_scroll(false)
        .allow_drag(false)
        .allow_zoom(false)
        .data_aspect(1.0)
        .view_aspect(1.0)
        .legend(
            Legend::default()
                .background_alpha(0.5)
                .position(Corner::LeftTop),
        )
        .show(ui, |ui| {
            ui.set_plot_bounds(PlotBounds::from_min_max([-0.1; 2], [1.1; 2]));

            // distance
            let values = out.distance_table.iter().enumerate().map(|(i, d)| {
                let n = out.distance_table.len() - 1;
                let x = i as f64 / n as f64;
                [x, *d as f64]
            });
            let points = PlotPoints::from_iter(values);
            let line = Line::new(points)
                .name("1. distance")
                .color(DISTANCE_COLOR)
                .width(2.0);
            ui.line(line);

            // velocity
            draw_time_plot(
                ui,
                "2. velocity",
                &out.velocity_curve,
                VELOCITY_COLOR,
                |v| v.length(),
            );

            // acc
            draw_time_plot(ui, "3. acceleration", &out.acc_curve, ACC_COLOR, |a| {
                a.length()
            });

            // curvature
            let max_curvature = out
                .acc_curve
                .iter()
                .zip(out.velocity_curve.iter())
                .map(|(a, v)| compute_curvature(*v, *a).abs())
                .max_by(|a, b| a.total_cmp(b))
                .unwrap();
            let curvature_scale = 0.5 / max_curvature;
            let values = out
                .acc_curve
                .iter()
                .zip(out.velocity_curve.iter())
                .enumerate()
                .map(|(i, (a, v))| {
                    let n = out.acc_curve.len() - 1;
                    let x = i as f64 / n as f64;
                    let curvature = curvature_scale * compute_curvature(*v, *a);
                    [x, 0.5 + curvature as f64]
                });
            let points = PlotPoints::from_iter(values);
            let line = Line::new(points)
                .name("4. curvature")
                .color(CURVATURE_COLOR)
                .width(2.0);
            ui.line(line);

            // cursor
            let current_u = current_u(params, out);
            let points = PlotPoints::Owned(vec![
                PlotPoint::new(current_u, -0.2),
                PlotPoint::new(current_u, 1.2),
            ]);
            let cursor = Line::new(points).width(1.5).color(Color32::RED);
            ui.line(cursor);
        });
}

fn draw_time_plot<F>(ui: &mut PlotUi, name: &str, values: &[Vec2], color: Color32, compute: F)
where
    F: Fn(&Vec2) -> f32,
{
    let max = values
        .iter()
        .map(|v| compute(v))
        .max_by(|a, b| a.total_cmp(b))
        .unwrap();
    let scale = 1.0 / max;
    let values = values.iter().enumerate().map(|(i, v)| {
        let n = values.len() - 1;
        let x = i as f64 / n as f64;
        let v = scale * compute(v);
        [x, v as f64]
    });
    let points = PlotPoints::from_iter(values);
    let line = Line::new(points).name(name).color(color).width(2.0);
    ui.line(line);
}

fn draw_spacial_plots(ui: &mut Ui, app: &SplineApp) {
    let params = &app.params;
    let out = &app.output;
    Plot::new("spacial_plot")
        .height(ui.available_width())
        .show_axes(false)
        .allow_scroll(false)
        .allow_drag(false)
        .allow_zoom(false)
        .data_aspect(1.0)
        .view_aspect(1.0)
        .legend(
            Legend::default()
                .background_alpha(0.5)
                .position(Corner::LeftTop),
        )
        .show(ui, |ui| {
            ui.set_plot_bounds(PlotBounds::from_min_max([-0.6; 2], [0.6; 2]));

            // velocity curve
            draw_vector_plot(
                ui,
                "2. velocity",
                &out.velocity_curve,
                out.animate_curve_idx,
                out.current_velocity,
                VELOCITY_COLOR,
                params.animate_curve,
                params.show_velocity_vector_in_plot,
            );

            // acc curve
            draw_vector_plot(
                ui,
                "3. acceleration",
                &out.acc_curve,
                out.animate_curve_idx,
                out.current_acc,
                ACC_COLOR,
                params.animate_curve,
                params.show_acc_vector_in_plot,
            );
        });
}

fn draw_vector_plot(
    ui: &mut PlotUi,
    name: &str,
    values: &[Vec2],
    current_idx: usize,
    current: Vec2,
    color: Color32,
    animate: bool,
    show_vector: bool,
) {
    fn vec2_to_plot_point(v: Vec2) -> [f64; 2] {
        // invert y axis
        [v.x as f64, -v.y as f64]
    }

    let max = values
        .iter()
        .map(|v| v.abs().max_elem())
        .max_by(|a, b| a.total_cmp(b))
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
        let arrow = Arrows::new(vec![[0.0; 2]], vec![tip])
            .color(color)
            .highlight(true)
            .tip_length(15.0);
        ui.arrows(arrow);
    }
}

fn main_content(ui: &mut Ui, app: &mut SplineApp, mut changed: bool) {
    let params = &mut app.params;

    let clamp_margin = Vec2::splat(CONTROL_POINT_RADIUS);
    let clamp_min = clamp_margin.to_pos2();
    let clamp_max = (ui.available_size() - clamp_margin).to_pos2();
    for (i, pos) in params.control_points.iter_mut().enumerate() {
        let rect = Rect::from_center_size(*pos, Vec2::splat(2.0 * POINT_RADIUS));
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
        app.output = compute(params);
    }

    let out = &app.output;
    let available_rect = Rect::from_min_size(ui.cursor().min, ui.available_size());
    let mut clip_rect = available_rect;
    if let Some(drag_state) = &params.drag_state {
        clip_rect = clip_rect.translate(-drag_state.delta());
    }
    let main_layer_id = LayerId::new(Order::Middle, Id::new("main_content"));
    let painter = ui.painter_at(clip_rect).with_layer_id(main_layer_id);

    // curve
    let curve_stroke = Stroke::new(3.0, Color32::WHITE);
    if !params.animate_curve {
        painter.add(Shape::line(out.curve_points.clone(), curve_stroke));
    } else {
        let lerp_point = out.constructed_point();
        let points = out.curve_points[0..=out.animate_curve_idx]
            .iter()
            .copied()
            .chain(std::iter::once(lerp_point))
            .collect();
        painter.add(Shape::line(points, curve_stroke));
    }

    // control point lines
    if params.show_control_lines {
        let stroke = Stroke::new(2.0, Color32::GRAY);
        painter.add(Shape::line(params.control_points.clone(), stroke));
    }

    // interpolated lines
    for points in out.lerp_points.iter() {
        if params.show_lerp_lines && points.len() > 1 {
            let stroke = Stroke::new(1.0, Color32::GREEN);
            painter.add(Shape::line(points.clone(), stroke));
        }
    }

    // vectors
    if params.show_velocity_vector_on_curve {
        let stroke = Stroke::new(2.0, VELOCITY_COLOR);
        let lerp_point = out.constructed_point();
        let v = params.movement_direction * VELOCITY_SCALE * out.current_velocity;
        draw_arrow(&painter, lerp_point, v, 20.0, stroke);
    }
    if params.show_acc_vector_on_curve {
        let stroke = Stroke::new(2.0, ACC_COLOR);
        let lerp_point = out.constructed_point();
        let v = ACC_SCALE * out.current_acc;
        draw_arrow(&painter, lerp_point, v, 20.0, stroke);
    }

    // curvature circle
    if params.show_curvature_circle_on_curve {
        fn curvature_color_channel(c: f32) -> u8 {
            let max_c = 0.04;
            let normalized = c.clamp(0.0, max_c) / max_c;
            let mapped = normalized.powf(0.3);
            (255.0 * mapped).round() as u8
        }

        let lerp_point = out.constructed_point();
        let curvature = compute_curvature(out.current_velocity, out.current_acc);
        let radius = 1.0 / curvature;
        let color = if params.adaptive_curvature_color {
            let left = curvature_color_channel(curvature);
            let right = curvature_color_channel(-curvature);
            let r = 0xFF - left;
            let b = 0xFF - right;
            // let g = 0xFF_u8.saturating_sub(left).saturating_sub(right);
            Color32::from_rgb(r, 0, b)
        } else {
            CURVATURE_COLOR
        };
        let stroke = Stroke::new(2.0, color);

        if radius.is_infinite() {
            let l = out.current_velocity.normalized() * 0.5 * params.curvature_arc_length;
            let start = lerp_point + l;
            let end = lerp_point - l;
            painter.line_segment([start, end], stroke);
        } else {
            let signed_radius = 1.0 / curvature;
            let v_norm = out.current_velocity.normalized();
            let center = lerp_point + signed_radius * Vec2::new(-v_norm.y, v_norm.x);
            draw_half_open_circle(
                &painter,
                lerp_point,
                center,
                signed_radius,
                params.curvature_arc_length,
                stroke,
            );
        }
    }

    // interpolated points
    for points in out.lerp_points.iter() {
        if points.len() == 1 {
            if params.show_last_lerp_point {
                let color = Color32::from_rgb(0xF0, 0x20, 0xF0);
                painter.circle_filled(points[0], POINT_RADIUS, color);
            }
        } else if params.show_lerp_points {
            for p in points.iter() {
                painter.circle_filled(*p, POINT_RADIUS, Color32::GOLD);
            }
        }
    }

    // control points
    if params.show_control_points {
        for (i, p) in params.control_points.iter().enumerate() {
            let mut fill = Color32::RED;
            let mut stroke = Stroke::new(2.0, Color32::TRANSPARENT);
            if params
                .hovered_point
                .as_ref()
                .is_some_and(|r| r.contains(&i))
            {
                fill = Color32::from_rgb(0xFF, 0x50, 0x30);
                stroke.color = Color32::WHITE;
            }
            painter.circle(*p, CONTROL_POINT_RADIUS, fill, stroke);
            let font = FontId::new(1.2 * CONTROL_POINT_RADIUS, FontFamily::Proportional);
            painter.text(
                *p,
                Align2::CENTER_CENTER,
                i.to_string(),
                font,
                Color32::WHITE,
            );
        }
    }

    if let Some(drag_state) = &params.drag_state {
        let delta = drag_state.delta();
        ui.ctx().translate_layer(main_layer_id, delta);

        // draw translation delta
        let max = available_rect.right_bottom() - Vec2::splat(16.0);
        let coord_size = Vec2::new(40.0, 16.0);
        let box_size = Vec2::new(2.0 * 40.0 + 2.0 * 8.0 + 8.0, 16.0 + 2.0 * 8.0);
        let min = max - box_size;
        let rect = Rect::from_min_max(min, max);

        let layer_id = LayerId::new(Order::Foreground, Id::new("drag_overlay"));
        ui.with_layer_id(layer_id, |ui| {
            ui.allocate_ui_at_rect(rect, |ui| {
                Frame::none()
                    .fill(Color32::from_rgba_unmultiplied(0x80, 0x80, 0x80, 0xF0))
                    .rounding(Rounding::same(8.0))
                    .inner_margin(8.0)
                    .show(ui, |ui| {
                        ui.horizontal_centered(|ui| {
                            let layout =
                                Layout::centered_and_justified(egui::Direction::LeftToRight)
                                    .with_main_align(Align::RIGHT);
                            let text_color = Color32::from_gray(0x20);
                            let Vec2 { x, y } = delta;
                            ui.allocate_ui_with_layout(coord_size, layout, |ui| {
                                let text = RichText::new(format!("{x:.1}")).color(text_color);
                                ui.add(Label::new(text));
                            });
                            ui.allocate_ui_with_layout(coord_size, layout, |ui| {
                                let text = RichText::new(format!("{y:.1}")).color(text_color);
                                ui.add(Label::new(text));
                            });
                        });
                    });
            });
        });
    }
}

fn draw_arrow(painter: &Painter, origin: Pos2, vec: Vec2, tip_length: f32, stroke: Stroke) {
    let rot = Rot2::from_angle(std::f32::consts::TAU / 10.0);
    let tip = origin + vec;
    let dir = vec.normalized();
    painter.line_segment([origin, tip], stroke);
    painter.line_segment([tip, tip - tip_length * (rot * dir)], stroke);
    painter.line_segment([tip, tip - tip_length * (rot.inverse() * dir)], stroke);
}

fn draw_half_open_circle(
    painter: &Painter,
    curve_point: Pos2,
    center: Pos2,
    signed_radius: f32,
    arc_length: f32,
    stroke: Stroke,
) {
    let radius = signed_radius.abs();
    let arc_angle_range = (arc_length / radius).min(TAU);

    let neutral_angle = (curve_point - center).angle();
    let start_angle = neutral_angle - 0.5 * arc_angle_range;
    for i in 0..100 {
        let a = i as f32 / 100.0;
        let b = (i + 1) as f32 / 100.0;
        let angle_a = start_angle + a * arc_angle_range;
        let angle_b = start_angle + b * arc_angle_range;
        let point_a = center + Vec2::splat(radius) * Vec2::new(angle_a.cos(), angle_a.sin());
        let point_b = center + Vec2::splat(radius) * Vec2::new(angle_b.cos(), angle_b.sin());
        painter.line_segment([point_a, point_b], stroke);
    }
}

fn current_u(params: &Params, out: &Output) -> f32 {
    if params.animate_constant_speed {
        out.constant_speed_u
    } else {
        params.u
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

    let constant_speed_u = compute_constant_speed_u(params.u, &distance_table);
    let u = if params.animate_constant_speed {
        constant_speed_u
    } else {
        params.u
    };
    let lerp_points = compute_lerp_points(u, &params.control_points);
    let animate_curve_idx = {
        let n = (curve_points.len() - 1) as f32;
        (u * n).floor() as usize
    };
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
        let point = compute_position(u, control_points);
        line.push(point);
    }
    line
}

fn compute_position(u: f32, control_points: &[Pos2]) -> Pos2 {
    let n = control_points.len() - 1;
    let mut accum = Pos2::ZERO;
    for (i, p) in control_points.iter().enumerate() {
        let weight = bernstein_weight(u, n, i);
        accum += weight * p.to_vec2();
    }
    accum
}

fn compute_vectors(
    control_points: &[Pos2],
    num_line_segments: u32,
    compute: fn(f32, &[Pos2]) -> Vec2,
) -> Vec<Vec2> {
    let mut line = Vec::new();
    for i in 0..=num_line_segments {
        let u = i as f32 / num_line_segments as f32;
        let vec = compute(u, control_points);
        line.push(vec);
    }
    line
}

fn compute_velocity(u: f32, control_points: &[Pos2]) -> Vec2 {
    let n = control_points.len() - 1;
    let mut accum = Vec2::ZERO;
    for (i, p) in control_points.windows(2).enumerate() {
        let weight = bernstein_weight(u, n - 1, i);
        accum += weight * (p[1] - p[0]);
    }
    n as f32 * accum
}

fn compute_acc(u: f32, control_points: &[Pos2]) -> Vec2 {
    let n = control_points.len() - 1;
    let mut accum = Vec2::ZERO;
    for (i, p) in control_points.windows(3).enumerate() {
        let weight = bernstein_weight(u, n - 2, i);
        accum += weight * ((p[2] - p[1]) - (p[1] - p[0]));
    }
    (n * (n - 1)) as f32 * accum
}

fn compute_curvature(v: Vec2, a: Vec2) -> f32 {
    (v.x * a.y - v.y * a.x) / (v.x * v.x + v.y * v.y).powf(1.5)
}

fn bernstein_weight(u: f32, n: usize, i: usize) -> f32 {
    let coefficient = (factorial(n)) / (factorial(n - i) * factorial(i));
    coefficient as f32 * u.powi(i as i32) * (1.0 - u).powi((n - i) as i32)
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

fn compute_constant_speed_u(u: f32, distance_table: &[f32]) -> f32 {
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

fn factorial(n: usize) -> usize {
    match n {
        0 => 1,
        1 => 1,
        _ => (2..=n).product(),
    }
}

trait BoolExt {
    fn toggle(&mut self);
}

impl BoolExt for bool {
    fn toggle(&mut self) {
        *self = !*self;
    }
}
