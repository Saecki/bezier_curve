use egui::{Pos2, Vec2};

use crate::{Output, Params};

pub fn all(params: &Params) -> Output {
    let curve_points = points(&params.control_points, params.num_line_segments);
    let distance_table = distance_table(&curve_points);

    let velocity_curve = vectors(
        &params.control_points,
        params.num_line_segments,
        velocity,
    );
    let acc_curve = vectors(
        &params.control_points,
        params.num_line_segments,
        acc,
    );

    let constant_speed_u = constant_speed_u(params.u, &distance_table);
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
    let current_velocity = velocity(u, &params.control_points);
    let current_acc = acc(u, &params.control_points);
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

fn points(control_points: &[Pos2], num_line_segments: u32) -> Vec<Pos2> {
    let mut line = Vec::new();
    for i in 0..=num_line_segments {
        let u = i as f32 / num_line_segments as f32;
        let point = position(u, control_points);
        line.push(point);
    }
    line
}

fn position(u: f32, control_points: &[Pos2]) -> Pos2 {
    let n = control_points.len() - 1;
    let mut accum = Pos2::ZERO;
    for (i, p) in control_points.iter().enumerate() {
        let weight = bernstein_weight(u, n, i);
        accum += weight * p.to_vec2();
    }
    accum
}

fn vectors(
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

fn velocity(u: f32, control_points: &[Pos2]) -> Vec2 {
    let n = control_points.len() - 1;
    let mut accum = Vec2::ZERO;
    for (i, p) in control_points.windows(2).enumerate() {
        let weight = bernstein_weight(u, n - 1, i);
        accum += weight * (p[1] - p[0]);
    }
    n as f32 * accum
}

fn acc(u: f32, control_points: &[Pos2]) -> Vec2 {
    let n = control_points.len() - 1;
    let mut accum = Vec2::ZERO;
    for (i, p) in control_points.windows(3).enumerate() {
        let weight = bernstein_weight(u, n - 2, i);
        accum += weight * ((p[2] - p[1]) - (p[1] - p[0]));
    }
    (n * (n - 1)) as f32 * accum
}

pub fn curvature(v: Vec2, a: Vec2) -> f32 {
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

fn distance_table(points: &[Pos2]) -> Vec<f32> {
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

fn factorial(n: usize) -> usize {
    match n {
        0 => 1,
        1 => 1,
        _ => (2..=n).product(),
    }
}
