//! Trajectory generation and merging.

use std::ops::Range;

use rand::{Rng, RngExt};
use rand_distr::{Beta, Distribution};

use crate::config::ScreenConfig;
use crate::curve::{self, Point};
use crate::motion::Plan;
use crate::noise::{Drift, Tremor};
use crate::style::MovementStyle;
use crate::target::{Area, Target};
use crate::trajectory::Trajectory;

/// Time constant of the slow path wobble, in seconds of a clock that runs at
/// full rate only at peak speed.
const WOBBLE_TIME_CONSTANT: f64 = 0.18;

/// Wobble amplitude as a fraction of the movement distance, before the
/// nervousness and precision adjustments.
const WOBBLE_RATIO: f64 = 0.0022;

/// Largest wobble amplitude in pixels, so moves across the whole screen do not wander.
const MAX_WOBBLE_PX: f64 = 5.5;

/// Frequency of physiological hand tremor in hertz, which peaks between 8 and 12.
const TREMOR_FREQUENCY: f64 = 9.0;

/// Time constant in seconds over which the tremor amplitude drifts, which
/// spreads its spectrum over about a hertz and a half around the frequency.
const TREMOR_TIME_CONSTANT: f64 = 0.1;

/// Tremor amplitude in pixels at zero nervousness, and its gain per unit.
const TREMOR_BASE_PX: f64 = 0.15;
const TREMOR_GAIN_PX: f64 = 0.5;

/// Seconds over which noise fades out at the end, so the cursor comes to rest
/// on its planned landing.
const NOISE_FADE: f64 = 0.06;

/// Duration in seconds of a movement that starts on its target.
const STILL_DURATION: f64 = 0.01;

/// Shape of a pause within the delay range of [`merge`]: a Beta(2, 4) draw
/// peaks at a quarter of the range and trails off toward its long end.
const PAUSE_SHAPE: (f64, f64) = (2.0, 4.0);

const EXPECT_PAUSE_SHAPE: &str = "pause shape parameters are positive constants";

/// Generates one movement from `start` onto `target`.
///
/// The movement is planned from `style`. A curved primary movement aims at
/// the target center and lands with an error that grows with its speed. While
/// the cursor rests outside the target and phases remain, a corrective
/// movement follows, either after a short pause or blending into the end of
/// the one before, and the last allowed phase always ends inside the target.
/// Movement time follows Fitts's law, so smaller and farther targets take
/// longer. Hand wobble and tremor are added while the hand moves and freeze
/// while it rests, positions are clamped to the screen and rounded to whole
/// pixels, and timestamps start at zero and are strictly increasing.
///
/// The first sample sits on the rounded `start`, clamped to the screen, and
/// the last on the pixel where the cursor came to rest, inside the part of
/// the target that is on screen. When `start` already lies on the target, the
/// result holds still for ten milliseconds. Style or config values outside
/// their documented ranges are clamped, and coordinates that are not finite
/// yield an empty trajectory.
///
/// # Examples
///
/// ```
/// use cursorflow::{MovementStyle, ScreenConfig, Target, generate_single};
///
/// let button = Target::new([800.0, 500.0], [96.0, 32.0]);
/// let mut rng = rand::rng();
///
/// let path = generate_single(
///     [100.0, 100.0],
///     button,
///     &ScreenConfig::default(),
///     &MovementStyle::default(),
///     &mut rng,
/// );
/// let rest = path.iter().next_back().map(|sample| (sample.x, sample.y));
///
/// assert_eq!((path.x[0], path.y[0]), (100, 100));
/// assert!(rest.is_some_and(|(x, y)| (752..=848).contains(&x) && (484..=516).contains(&y)));
/// assert!(path.duration() > 100.0);
/// ```
pub fn generate_single<R: Rng>(
    start: Point,
    target: Target,
    config: &ScreenConfig,
    style: &MovementStyle,
    rng: &mut R,
) -> Trajectory {
    let style = style.sanitized();
    let config = config.sanitized();

    if !is_finite(start) || !is_finite(target.center) {
        return Trajectory::empty();
    }

    let start = config.clamp(start);
    let area = Area::new(&target, &config);

    if area.contains(start) {
        return finish(&[start, start], &[0.0, STILL_DURATION], &config);
    }

    let plan = Plan::new(start, &area, &config, &style, rng);
    let times = sample_times(plan.duration(), &config, rng);
    let mut path: Vec<Point> = times.iter().map(|&time| plan.position(time)).collect();
    let distance = curve::norm(curve::sub(area.aim(), start));

    add_noise(&mut path, &times, distance, &style, rng);
    path[0] = start;
    path[times.len() - 1] = plan.landing();

    finish(&path, &times, &config)
}

/// Generates one movement onto each of `targets` in turn, beginning at `start`.
///
/// Each movement starts where the previous one came to rest, and every
/// trajectory starts at time zero; use [`merge`] to chain them on one time
/// axis.
///
/// # Examples
///
/// ```
/// use cursorflow::{MovementStyle, ScreenConfig, Target, generate};
///
/// let targets = [
///     Target::new([400.0, 300.0], [60.0, 24.0]),
///     Target::point([50.0, 500.0]),
/// ];
/// let mut rng = rand::rng();
///
/// let segments = generate(
///     [10.0, 10.0],
///     targets,
///     &ScreenConfig::default(),
///     &MovementStyle::default(),
///     &mut rng,
/// );
/// let rest = segments[0].iter().next_back().map(|sample| (sample.x, sample.y));
///
/// assert_eq!(segments.len(), 2);
/// assert_eq!(rest, Some((segments[1].x[0], segments[1].y[0])));
/// ```
pub fn generate<R: Rng>(
    start: Point,
    targets: impl IntoIterator<Item = Target>,
    config: &ScreenConfig,
    style: &MovementStyle,
    rng: &mut R,
) -> Vec<Trajectory> {
    let mut position = start;

    targets
        .into_iter()
        .map(|target| {
            let trajectory = generate_single(position, target, config, style, rng);

            if let Some(rest) = trajectory.iter().next_back() {
                position = [f64::from(rest.x), f64::from(rest.y)];
            }

            trajectory
        })
        .collect()
}

/// Concatenates trajectories on one time axis with a random pause between them.
///
/// Each pause is drawn from `delay_range`, in milliseconds, skewed toward its
/// short end the way human pauses are: most fall in the lower half and a few
/// run long. A reversed range is swapped, negative bounds are raised to zero,
/// and an empty range yields its lower bound. Empty trajectories are skipped,
/// and speeds are derived again over the joined columns.
///
/// # Examples
///
/// ```
/// use cursorflow::{MovementStyle, ScreenConfig, Target, generate, merge};
///
/// let targets = [
///     Target::new([400.0, 300.0], [60.0, 24.0]),
///     Target::new([50.0, 500.0], [32.0, 32.0]),
/// ];
/// let mut rng = rand::rng();
/// let segments = generate(
///     [10.0, 10.0],
///     targets,
///     &ScreenConfig::default(),
///     &MovementStyle::default(),
///     &mut rng,
/// );
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

/// Adds hand wobble and tremor to `path`, sampled at `times` seconds.
///
/// Both follow the planned speed relative to its peak. The wobble clock runs
/// with the square of that ratio, so each wobble step stays a small, steady
/// share of the planned progress and never turns a slow movement back, and
/// the tremor reaches the cursor in proportion to its square root. Neither
/// moves the cursor while the planned hand rests, just as a mouse held still
/// sends nothing, and the noise fades out over the last moments so the
/// landing stays exact.
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
    let mut wobble = Drift::still(WOBBLE_TIME_CONSTANT);
    let mut tremor = Tremor::new(TREMOR_FREQUENCY, TREMOR_TIME_CONSTANT, rng);

    for index in 0..n {
        let dt = if index == 0 {
            0.0
        } else {
            times[index] - times[index - 1]
        };
        let effort = effort[index];
        let envelope = smoothstep((duration - times[index]) / fade);
        let offset = curve::add(
            curve::scale(wobble.step(dt * effort * effort, rng), wobble_amplitude),
            curve::scale(
                tremor.step(times[index], dt, rng),
                tremor_amplitude * effort.sqrt(),
            ),
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
    let (x, y) = path
        .iter()
        .map(|&point| {
            let [x, y] = config.clamp(point);

            (x.round() as i32, y.round() as i32)
        })
        .unzip();
    let t = times.iter().map(|&time| time * 1000.0).collect();

    Trajectory::new(x, y, t)
}

fn sample_delay<R: Rng>(delay_range: &Range<f64>, rng: &mut R) -> f64 {
    let start = finite_or(delay_range.start, 0.0);
    let end = finite_or(delay_range.end, 0.0);
    let low = start.min(end).max(0.0);
    let high = start.max(end).max(0.0);

    if high > low {
        let (alpha, beta) = PAUSE_SHAPE;
        let shape = Beta::new(alpha, beta).expect(EXPECT_PAUSE_SHAPE);

        low + (high - low) * shape.sample(rng)
    } else {
        low
    }
}

fn smoothstep(x: f64) -> f64 {
    let x = x.clamp(0.0, 1.0);

    x * x * (3.0 - 2.0 * x)
}

fn is_finite(point: Point) -> bool {
    point.iter().all(|value| value.is_finite())
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
