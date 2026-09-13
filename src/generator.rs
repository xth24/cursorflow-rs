//! Trajectory generation and merging.

use std::ops::Range;

use rand::{Rng, RngExt};

use crate::config::ScreenConfig;
use crate::curve::{self, Point};
use crate::motion::Plan;
use crate::noise::Drift;
use crate::style::MovementStyle;
use crate::trajectory::Trajectory;

/// Time constant of the slow path wobble, in seconds.
const WOBBLE_TIME_CONSTANT: f64 = 0.18;

/// Time constant of the fast hand tremor, in seconds, giving it a bandwidth
/// of roughly six hertz.
const TREMOR_TIME_CONSTANT: f64 = 0.025;

/// Wobble amplitude as a fraction of the movement distance, before the
/// nervousness and precision adjustments.
const WOBBLE_RATIO: f64 = 0.0025;

/// Largest wobble amplitude in pixels, so moves across the whole screen do not wander.
const MAX_WOBBLE_PX: f64 = 6.0;

/// Share of the wobble that remains while the cursor is at rest; motor noise
/// grows with the strength of the movement, so a resting hand is steadier.
const REST_WOBBLE: f64 = 0.25;

/// Tremor amplitude in pixels at zero nervousness, and its gain per unit.
const TREMOR_BASE_PX: f64 = 0.15;
const TREMOR_GAIN_PX: f64 = 0.5;

/// Seconds over which noise fades in at the start and out at the end, so the
/// endpoints stay exact.
const NOISE_FADE: f64 = 0.06;

/// Duration in seconds of a movement shorter than a pixel.
const STILL_DURATION: f64 = 0.01;

/// Generates one movement from `start` to `end`.
///
/// The movement is planned from `style`: a curved primary movement lands near
/// `end`, then, while the cursor is still off target and phases remain, a short
/// pause and a corrective movement follow. Hand tremor is added along the way,
/// positions are clamped to the screen and rounded to whole pixels, and the
/// first and last samples sit on the rounded `start` and `end`. Timestamps
/// start at zero and are strictly increasing.
///
/// Style or config values outside their documented ranges are clamped.
/// A move shorter than one pixel yields two samples ten milliseconds apart,
/// and coordinates that are not finite yield an empty trajectory.
///
/// # Examples
///
/// ```
/// use cursorflow::{MovementStyle, ScreenConfig, generate_single};
///
/// let mut rng = rand::rng();
/// let path = generate_single(
///     [100.0, 100.0],
///     [800.0, 500.0],
///     &ScreenConfig::default(),
///     &MovementStyle::default(),
///     &mut rng,
/// );
///
/// assert_eq!((path.x[0], path.y[0]), (100, 100));
/// assert_eq!(path.iter().next_back().map(|s| (s.x, s.y)), Some((800, 500)));
/// assert!(path.duration() > 100.0);
/// ```
pub fn generate_single<R: Rng>(
    start: Point,
    end: Point,
    config: &ScreenConfig,
    style: &MovementStyle,
    rng: &mut R,
) -> Trajectory {
    let style = style.sanitized();
    let config = config.sanitized();
    let distance = curve::norm(curve::sub(end, start));

    if !distance.is_finite() {
        return Trajectory::empty();
    }

    if distance < 1.0 {
        return finish(&[start, end], &[0.0, STILL_DURATION], &config);
    }

    let diagonal = finite_positive_or(config.diagonal(), 1.0);
    let plan = Plan::new(start, end, diagonal, &style, rng);
    let times = sample_times(plan.duration(), &config, rng);
    let mut path: Vec<Point> = times.iter().map(|&time| plan.position(time)).collect();

    add_noise(&mut path, &times, distance, &style, rng);
    path[0] = start;
    path[times.len() - 1] = end;

    finish(&path, &times, &config)
}

/// Generates one movement for each consecutive pair of `waypoints`.
///
/// Every trajectory starts at time zero; use [`merge`] to chain them on one
/// time axis. Fewer than two waypoints yield no trajectories.
///
/// # Examples
///
/// ```
/// use cursorflow::{MovementStyle, ScreenConfig, generate};
///
/// let waypoints = [[10.0, 10.0], [400.0, 300.0], [50.0, 500.0]];
/// let mut rng = rand::rng();
///
/// let segments = generate(&waypoints, &ScreenConfig::default(), &MovementStyle::default(), &mut rng);
///
/// assert_eq!(segments.len(), 2);
/// assert_eq!((segments[1].x[0], segments[1].y[0]), (400, 300));
/// ```
pub fn generate<R: Rng>(
    waypoints: &[Point],
    config: &ScreenConfig,
    style: &MovementStyle,
    rng: &mut R,
) -> Vec<Trajectory> {
    waypoints
        .windows(2)
        .map(|pair| generate_single(pair[0], pair[1], config, style, rng))
        .collect()
}

/// Concatenates trajectories on one time axis with a random pause between them.
///
/// Each pause is drawn from `delay_range`, in milliseconds. A reversed range
/// is swapped, negative bounds are raised to zero, and an empty range yields
/// its lower bound. Empty trajectories are skipped, and speeds are derived
/// again over the joined columns.
///
/// # Examples
///
/// ```
/// use cursorflow::{MovementStyle, ScreenConfig, generate, merge};
///
/// let waypoints = [[10.0, 10.0], [400.0, 300.0], [50.0, 500.0]];
/// let mut rng = rand::rng();
/// let segments = generate(&waypoints, &ScreenConfig::default(), &MovementStyle::default(), &mut rng);
///
/// let path = merge(&segments, 70.0..200.0, &mut rng);
///
/// assert_eq!(path.len(), segments[0].len() + segments[1].len());
/// assert!(path.duration() >= segments[0].duration() + segments[1].duration() + 70.0);
/// ```
pub fn merge<R: Rng>(
    trajectories: &[Trajectory],
    delay_range: Range<f64>,
    rng: &mut R,
) -> Trajectory {
    let mut x = Vec::new();
    let mut y = Vec::new();
    let mut t = Vec::new();

    for trajectory in trajectories
        .iter()
        .filter(|trajectory| !trajectory.is_empty())
    {
        let n = trajectory.len();
        let offset = t.last().map_or(0.0, |&last: &f64| {
            finite_or(last, 0.0) + sample_delay(&delay_range, rng)
        });

        x.extend_from_slice(&trajectory.x[..n]);
        y.extend_from_slice(&trajectory.y[..n]);
        t.extend(trajectory.t[..n].iter().map(|&time| time + offset));
    }

    Trajectory::new(x, y, t)
}

fn sample_times<R: Rng>(duration: f64, config: &ScreenConfig, rng: &mut R) -> Vec<f64> {
    let interval = 1.0 / f64::from(config.sample_rate);
    let regular = (duration / interval - 0.5).max(0.0).floor() as usize + 1;
    let jitter = config.sample_jitter * interval;

    let mut times: Vec<f64> = (0..regular)
        .map(|index| {
            let base = index as f64 * interval;

            if index == 0 || jitter <= 0.0 {
                base
            } else {
                base + rng.random_range(-jitter..jitter)
            }
        })
        .collect();
    times.push(duration);

    times
}

fn add_noise<R: Rng>(
    path: &mut [Point],
    times: &[f64],
    distance: f64,
    style: &MovementStyle,
    rng: &mut R,
) {
    let n = path.len().min(times.len());
    if n < 3 {
        return;
    }

    let duration = times[n - 1];
    let fade = finite_positive_or(NOISE_FADE.min(duration / 2.0), 1.0);
    let wobble_amplitude =
        (distance * WOBBLE_RATIO).min(MAX_WOBBLE_PX) * (0.5 + style.nervousness) / style.precision;
    let tremor_amplitude = (TREMOR_BASE_PX + TREMOR_GAIN_PX * style.nervousness) / style.precision;
    let effort = speed_ratios(path, times);
    let mut wobble = Drift::new(WOBBLE_TIME_CONSTANT, rng);
    let mut tremor = Drift::new(TREMOR_TIME_CONSTANT, rng);

    for index in 0..n {
        let dt = if index == 0 {
            0.0
        } else {
            times[index] - times[index - 1]
        };
        let envelope = smoothstep(times[index].min(duration - times[index]) / fade);
        let gain = REST_WOBBLE + (1.0 - REST_WOBBLE) * effort[index];
        let offset = curve::add(
            curve::scale(wobble.step(dt, rng), wobble_amplitude * gain),
            curve::scale(tremor.step(dt, rng), tremor_amplitude),
        );

        path[index] = curve::add(path[index], curve::scale(offset, envelope));
    }
}

/// Returns each sample's speed as a fraction of the fastest one.
fn speed_ratios(path: &[Point], times: &[f64]) -> Vec<f64> {
    let n = path.len().min(times.len());
    let mut speeds: Vec<f64> = (0..n)
        .map(|index| {
            if index == 0 || index + 1 == n {
                return 0.0;
            }

            let span = times[index + 1] - times[index - 1];
            let hop = curve::norm(curve::sub(path[index + 1], path[index - 1]));

            if span > 0.0 { hop / span } else { 0.0 }
        })
        .collect();
    let peak = speeds.iter().copied().fold(0.0_f64, f64::max);

    if peak > 0.0 {
        for speed in &mut speeds {
            *speed /= peak;
        }
    }

    speeds
}

fn finish(path: &[Point], times: &[f64], config: &ScreenConfig) -> Trajectory {
    let x_max = f64::from(config.width.saturating_sub(1));
    let y_max = f64::from(config.height.saturating_sub(1));
    let x = path
        .iter()
        .map(|point| point[0].clamp(0.0, x_max).round() as i32)
        .collect();
    let y = path
        .iter()
        .map(|point| point[1].clamp(0.0, y_max).round() as i32)
        .collect();
    let t = times.iter().map(|&time| time * 1000.0).collect();

    Trajectory::new(x, y, t)
}

fn sample_delay<R: Rng>(delay_range: &Range<f64>, rng: &mut R) -> f64 {
    let start = finite_or(delay_range.start, 0.0);
    let end = finite_or(delay_range.end, 0.0);
    let low = start.min(end).max(0.0);
    let high = start.max(end).max(0.0);

    if high > low {
        rng.random_range(low..high)
    } else {
        low
    }
}

fn smoothstep(x: f64) -> f64 {
    let x = x.clamp(0.0, 1.0);

    x * x * (3.0 - 2.0 * x)
}

fn finite_or(value: f64, fallback: f64) -> f64 {
    if value.is_finite() { value } else { fallback }
}

fn finite_positive_or(value: f64, fallback: f64) -> f64 {
    if value.is_finite() && value > 0.0 {
        value
    } else {
        fallback
    }
}
