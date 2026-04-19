use std::ops::Range;

use rand::{Rng, RngExt};
use rand_distr::{Distribution, StandardNormal};

use crate::config::ScreenConfig;
use crate::style::MovementStyle;
use crate::trajectory::Trajectory;

pub type Point = [f64; 2];

fn sub(a: Point, b: Point) -> Point {
    [a[0] - b[0], a[1] - b[1]]
}

fn add(a: Point, b: Point) -> Point {
    [a[0] + b[0], a[1] + b[1]]
}

fn scale(a: Point, s: f64) -> Point {
    [a[0] * s, a[1] * s]
}

fn norm(a: Point) -> f64 {
    a[0].hypot(a[1])
}

fn sign<R: Rng>(rng: &mut R) -> f64 {
    if rng.random_bool(0.5) { 1.0 } else { -1.0 }
}

fn linspace(start: f64, end: f64, n: usize) -> Vec<f64> {
    match n {
        0 => Vec::new(),
        1 => vec![start],
        _ => {
            let step = (end - start) / (n - 1) as f64;
            (0..n).map(|i| start + step * i as f64).collect()
        }
    }
}

fn gradient(f: &[f64], x: &[f64]) -> Vec<f64> {
    let n = f.len();
    let mut out = vec![0.0; n];
    if n < 2 {
        return out;
    }
    out[0] = (f[1] - f[0]) / (x[1] - x[0]);
    out[n - 1] = (f[n - 1] - f[n - 2]) / (x[n - 1] - x[n - 2]);
    for i in 1..n - 1 {
        let hs = x[i] - x[i - 1];
        let hd = x[i + 1] - x[i];
        out[i] = (hs * hs * f[i + 1] + (hd * hd - hs * hs) * f[i] - hd * hd * f[i - 1])
            / (hs * hd * (hs + hd));
    }
    out
}

fn interp(x: f64, xp: &[f64], fp: &[f64]) -> f64 {
    match xp.len() {
        0 => 0.0,
        _ if x <= xp[0] => fp[0],
        n if x >= xp[n - 1] => fp[n - 1],
        _ => {
            let hi = xp.partition_point(|&v| v <= x);
            let lo = hi - 1;
            let span = xp[hi] - xp[lo];
            if span == 0.0 {
                fp[lo]
            } else {
                let t = (x - xp[lo]) / span;
                fp[lo] + t * (fp[hi] - fp[lo])
            }
        }
    }
}

fn convolve_same(raw: &[f64], kernel_size: usize) -> Vec<f64> {
    let n = raw.len();
    let k = kernel_size;
    let offset = (k - 1) / 2;
    let inv_k = 1.0 / k as f64;
    (0..n)
        .map(|i| {
            let m = i + offset;
            let lo = (m + 1).saturating_sub(n);
            let hi = m.min(k - 1);
            let sum: f64 = (lo..=hi).map(|j| raw[m - j]).sum();
            sum * inv_k
        })
        .collect()
}

fn peak_normalize(values: &mut [f64]) {
    let peak = values.iter().copied().fold(0.0_f64, f64::max);
    if peak > 0.0 {
        for v in values.iter_mut() {
            *v /= peak;
        }
    }
}

fn fitts_duration<R: Rng>(
    distance: f64,
    diagonal: f64,
    style: &MovementStyle,
    rng: &mut R,
) -> f64 {
    let rel = distance / diagonal;
    let a = 0.18 + rng.random_range(-0.02..0.02);
    let b = 0.38 + rng.random_range(-0.03..0.03);
    let mut mt = a + b * (1.0 + 12.0 * rel).log2();
    mt /= style.speed_factor;
    mt *= rng.random_range(0.92..1.08);
    mt.clamp(0.06, 2.0)
}

fn minimum_jerk_profile(n: usize, peak_shift: f64) -> Vec<f64> {
    let t = linspace(0.0, 1.0, n);
    let t_shifted: Vec<f64> = t
        .iter()
        .map(|&x| (x - peak_shift * 0.05).clamp(0.0, 1.0))
        .collect();
    let last = *t_shifted.last().unwrap_or(&0.0);
    let t_norm: Vec<f64> = if last > 0.0 {
        t_shifted.iter().map(|&x| x / last).collect()
    } else {
        t_shifted
    };
    let pos: Vec<f64> = t_norm
        .iter()
        .map(|&x| 10.0 * x.powi(3) - 15.0 * x.powi(4) + 6.0 * x.powi(5))
        .collect();

    let all_increasing = t_norm.windows(2).all(|w| w[1] - w[0] > 0.0);
    let grad_x: &[f64] = if all_increasing { &t_norm } else { &t };
    let mut vel = gradient(&pos, grad_x);
    for v in vel.iter_mut() {
        *v = v.max(0.0);
    }
    peak_normalize(&mut vel);
    vel
}

fn sub_movement_profile<R: Rng>(n: usize, style: &MovementStyle, rng: &mut R) -> Vec<f64> {
    let primary = minimum_jerk_profile(n, rng.random_range(-0.5..0.5));
    let mult = rng.random_range(0.85..1.0);
    let mut combined: Vec<f64> = primary.iter().map(|&v| v * mult).collect();

    for k in 1..style.sub_movement_count {
        let kf = k as f64;
        let frac = rng.random_range((0.55 + kf * 0.1)..(0.75 + kf * 0.05));
        let start_idx = (frac * n as f64) as usize;
        let sub_n = n.saturating_sub(start_idx);
        if sub_n < 4 {
            continue;
        }
        let sub = minimum_jerk_profile(sub_n, 0.0);
        let amplitude = rng.random_range(0.15..0.35) / kf;
        for (dst, &s) in combined[start_idx..].iter_mut().zip(&sub) {
            *dst += s * amplitude;
        }
    }

    peak_normalize(&mut combined);
    combined
}

fn cubic_bezier_point(t: f64, p0: Point, p1: Point, p2: Point, p3: Point) -> Point {
    let u = 1.0 - t;
    let uu = u * u;
    let uuu = uu * u;
    let tt = t * t;
    let ttt = tt * t;
    [
        uuu * p0[0] + 3.0 * uu * t * p1[0] + 3.0 * u * tt * p2[0] + ttt * p3[0],
        uuu * p0[1] + 3.0 * uu * t * p1[1] + 3.0 * u * tt * p2[1] + ttt * p3[1],
    ]
}

fn cubic_bezier_curve(ts: &[f64], p0: Point, p1: Point, p2: Point, p3: Point) -> Vec<Point> {
    ts.iter()
        .map(|&t| cubic_bezier_point(t, p0, p1, p2, p3))
        .collect()
}

fn generate_bezier_path<R: Rng>(
    start: Point,
    end: Point,
    distance: f64,
    diagonal: f64,
    style: &MovementStyle,
    rng: &mut R,
) -> (Point, Point, Point, Point) {
    let direction = sub(end, start);
    let d_norm = if distance > 0.0 {
        scale(direction, 1.0 / distance)
    } else {
        [1.0, 0.0]
    };
    let perp = [-d_norm[1], d_norm[0]];
    let rel_dist = distance / diagonal;

    let mut curve_mag = if rel_dist < 0.08 {
        rng.random_range(0.02..0.06)
    } else if rel_dist < 0.25 {
        rng.random_range(0.04..0.12)
    } else {
        rng.random_range(0.06..0.18)
    };
    curve_mag /= style.precision;

    let angle = direction[1].atan2(direction[0]);
    let biomech = angle.sin() * 0.15;
    let side = sign(rng) * (1.0 + biomech * rng.random_range(0.5..1.5));

    let cp1_along = rng.random_range(0.2..0.4);
    let cp2_along = rng.random_range(0.6..0.8);
    let cp1_perp = side * curve_mag * distance * rng.random_range(0.5..1.2);
    let cp2_perp = side * curve_mag * distance * rng.random_range(0.3..0.8) * sign(rng);

    let p1 = add(add(start, scale(direction, cp1_along)), scale(perp, cp1_perp));
    let p2 = add(add(start, scale(direction, cp2_along)), scale(perp, cp2_perp));

    (start, p1, p2, end)
}

fn arc_length_parameterize(
    p0: Point,
    p1: Point,
    p2: Point,
    p3: Point,
    n_fine: usize,
) -> (Vec<f64>, Vec<f64>, f64) {
    let t_fine = linspace(0.0, 1.0, n_fine);
    let pts = cubic_bezier_curve(&t_fine, p0, p1, p2, p3);
    let mut cum_len = Vec::with_capacity(n_fine);
    cum_len.push(0.0);
    for w in pts.windows(2) {
        let dx = w[1][0] - w[0][0];
        let dy = w[1][1] - w[0][1];
        let prev = *cum_len.last().unwrap();
        cum_len.push(prev + dx.hypot(dy));
    }
    let total = *cum_len.last().unwrap_or(&0.0);
    (t_fine, cum_len, total)
}

fn map_velocity_to_bezier_params(
    vel_profile: &[f64],
    t_fine: &[f64],
    cum_len: &[f64],
    total_len: f64,
    time_vals: &[f64],
) -> Vec<f64> {
    let n = vel_profile.len();
    let mut dist_traveled = vec![0.0; n];
    for i in 1..n {
        let dt = time_vals[i] - time_vals[i - 1];
        dist_traveled[i] = dist_traveled[i - 1] + vel_profile[i - 1] * dt;
    }

    let last = dist_traveled[n - 1];
    if last > 0.0 {
        let k = total_len / last;
        for d in dist_traveled.iter_mut() {
            *d *= k;
        }
    } else {
        dist_traveled = linspace(0.0, total_len, n);
    }

    dist_traveled
        .iter()
        .map(|&d| interp(d, cum_len, t_fine))
        .collect()
}

fn add_overshoot<R: Rng>(
    path_x: &mut [f64],
    path_y: &mut [f64],
    end: Point,
    distance: f64,
    style: &MovementStyle,
    rng: &mut R,
) {
    if distance < 30.0 || rng.random::<f64>() > style.overshoot_tendency {
        return;
    }
    let n = path_x.len();
    let start_idx = (n as f64 * rng.random_range(0.82..0.92)) as usize;
    let tail = n.saturating_sub(start_idx);
    if tail < 3 {
        return;
    }

    let dir = [end[0] - path_x[start_idx], end[1] - path_y[start_idx]];
    let d_norm = dir[0].hypot(dir[1]);
    if d_norm < 1e-6 {
        return;
    }
    let direction = [dir[0] / d_norm, dir[1] / d_norm];
    let magnitude = distance * rng.random_range(0.02..0.08) / style.precision;

    let t_os = linspace(0.0, 1.0, tail);
    let mut envelope: Vec<f64> = t_os
        .iter()
        .map(|&t| (std::f64::consts::PI * t).sin() * (1.0 - t).sqrt())
        .collect();
    peak_normalize(&mut envelope);

    for (i, &e) in envelope.iter().enumerate() {
        path_x[start_idx + i] += e * magnitude * direction[0];
        path_y[start_idx + i] += e * magnitude * direction[1];
    }
}

fn add_noise<R: Rng>(
    path_x: &mut [f64],
    path_y: &mut [f64],
    vel_profile: &[f64],
    distance: f64,
    style: &MovementStyle,
    rng: &mut R,
) {
    let n = path_x.len();
    let base_mag = distance * 0.0025 / style.precision * (0.4 + style.nervousness);
    let kernel_size = (n / 12).max(3);
    let raw_x: Vec<f64> = (0..n).map(|_| StandardNormal.sample(rng)).collect();
    let raw_y: Vec<f64> = (0..n).map(|_| StandardNormal.sample(rng)).collect();
    let smooth_x = convolve_same(&raw_x, kernel_size);
    let smooth_y = convolve_same(&raw_y, kernel_size);

    let taper_len = (n / 4).min(6);
    let taper: Vec<f64> = (0..n)
        .map(|i| {
            if taper_len <= 1 {
                1.0
            } else if i < taper_len {
                i as f64 / (taper_len - 1) as f64
            } else if i >= n - taper_len {
                (n - 1 - i) as f64 / (taper_len - 1) as f64
            } else {
                1.0
            }
        })
        .collect();

    for i in 0..n {
        let m = base_mag * (1.0 - vel_profile[i] * 0.7) * taper[i];
        path_x[i] += smooth_x[i] * m;
        path_y[i] += smooth_y[i] * m;
    }
}

fn micro_corrections(path_x: &mut [f64], path_y: &mut [f64], end: Point) {
    let n = path_x.len();
    let zone = ((n as f64 * 0.12) as usize).max(3);
    if zone >= n {
        return;
    }
    let start_idx = n - zone;
    for i in start_idx..n {
        let blend = ((i - start_idx) as f64 / zone as f64).powi(2);
        let w = blend * 0.3;
        path_x[i] = path_x[i] * (1.0 - w) + end[0] * w;
        path_y[i] = path_y[i] * (1.0 - w) + end[1] * w;
    }
}

pub fn generate_single<R: Rng>(
    start: Point,
    end: Point,
    config: &ScreenConfig,
    style: &MovementStyle,
    rng: &mut R,
) -> Trajectory {
    let distance = norm(sub(end, start));

    if distance < 1.0 {
        return Trajectory {
            x: vec![start[0].round() as i32, end[0].round() as i32],
            y: vec![start[1].round() as i32, end[1].round() as i32],
            t: vec![0.0, 10.0],
            v: vec![0.0, 0.0],
        };
    }

    let diagonal = config.diagonal();
    let duration = fitts_duration(distance, diagonal, style, rng);
    let n = ((duration * config.sample_rate as f64) as usize).max(6);

    let vel_profile = sub_movement_profile(n, style, rng);
    let (p0, p1, p2, p3) = generate_bezier_path(start, end, distance, diagonal, style, rng);
    let (t_fine, cum_len, total_len) = arc_length_parameterize(p0, p1, p2, p3, 500);

    let time_vals = linspace(0.0, duration, n);
    let bezier_t =
        map_velocity_to_bezier_params(&vel_profile, &t_fine, &cum_len, total_len, &time_vals);

    let pts = cubic_bezier_curve(&bezier_t, p0, p1, p2, p3);
    let mut path_x: Vec<f64> = pts.iter().map(|p| p[0]).collect();
    let mut path_y: Vec<f64> = pts.iter().map(|p| p[1]).collect();

    add_overshoot(&mut path_x, &mut path_y, end, distance, style, rng);
    add_noise(&mut path_x, &mut path_y, &vel_profile, distance, style, rng);
    micro_corrections(&mut path_x, &mut path_y, end);

    path_x[0] = start[0];
    path_y[0] = start[1];
    path_x[n - 1] = end[0];
    path_y[n - 1] = end[1];

    let w_max = (config.width as f64 - 1.0).max(0.0);
    let h_max = (config.height as f64 - 1.0).max(0.0);
    let xs: Vec<i32> = path_x
        .iter()
        .map(|&v| v.clamp(0.0, w_max).round() as i32)
        .collect();
    let ys: Vec<i32> = path_y
        .iter()
        .map(|&v| v.clamp(0.0, h_max).round() as i32)
        .collect();

    let timestamps: Vec<f64> = linspace(0.0, duration * 1000.0, n)
        .into_iter()
        .map(f64::round)
        .collect();

    let mut velocities = vec![0.0; n];
    for i in 1..n {
        let dt = ((timestamps[i] - timestamps[i - 1]) / 1000.0).max(1e-10);
        let dx = (xs[i] - xs[i - 1]) as f64;
        let dy = (ys[i] - ys[i - 1]) as f64;
        velocities[i] = dx.hypot(dy) / dt;
    }

    Trajectory {
        x: xs,
        y: ys,
        t: timestamps,
        v: velocities,
    }
}

pub fn generate<R: Rng>(
    waypoints: &[Point],
    config: &ScreenConfig,
    style: &MovementStyle,
    rng: &mut R,
) -> Vec<Trajectory> {
    waypoints
        .windows(2)
        .map(|w| generate_single(w[0], w[1], config, style, rng))
        .collect()
}

pub fn merge<R: Rng>(
    trajectories: &[Trajectory],
    delay_range: Range<f64>,
    rng: &mut R,
) -> Trajectory {
    let mut out = Trajectory::empty();
    let mut offset = 0.0;
    for (i, traj) in trajectories.iter().enumerate() {
        if i > 0 {
            offset = out.t.last().copied().unwrap_or(0.0)
                + rng.random_range(delay_range.clone());
        }
        out.x.extend_from_slice(&traj.x);
        out.y.extend_from_slice(&traj.y);
        out.t.extend(traj.t.iter().map(|&t| t + offset));
        out.v.extend_from_slice(&traj.v);
    }
    out
}
