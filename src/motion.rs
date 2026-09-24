//! Movement planning: submovement timing, landing errors, and speed profiles.

use std::f64::consts::PI;
use std::ops::Range;

use rand::{Rng, RngExt};

use crate::config::ScreenConfig;
use crate::curve::{self, Curve, Point};
use crate::noise::{normal, normal_quantile};
use crate::style::MovementStyle;
use crate::target::Area;

/// Fitts's law intercept and slope of the primary movement, in seconds and
/// seconds per bit of the index of difficulty `log2(distance / width + 1)`.
const PRIMARY_INTERCEPT: f64 = 0.16;
const PRIMARY_SLOPE: f64 = 0.15;

/// Fitts's law intercept and slope of a corrective movement, in seconds and
/// seconds per bit.
const CORRECTION_INTERCEPT: f64 = 0.08;
const CORRECTION_SLOPE: f64 = 0.05;

/// Bounds on any movement's duration in seconds, after style scaling.
const MIN_DURATION: f64 = 0.06;
const MAX_DURATION: f64 = 2.0;

/// Longest a single correction may take, in seconds.
const MAX_CORRECTION_DURATION: f64 = 0.6;

/// Standard deviation of the logarithm of a movement's duration around its
/// Fitts's law value, which skews durations long the way human ones are.
const DURATION_SPREAD: f64 = 0.1;

/// Endpoint spread along the movement axis per unit of average speed, in
/// seconds: a movement covering `d` pixels in `t` seconds lands with a
/// standard deviation of this times `d / t`, as in Meyer's optimized
/// submovement model, so a faster movement misses by more.
const SPREAD_PER_SPEED: f64 = 0.033;

/// Sideways landing spread relative to the spread along the movement axis.
const LATERAL_SPREAD: f64 = 0.45;

/// Bounds on the overshoot chance, which keep its normal quantile finite.
const MIN_OVERSHOOT: f64 = 0.001;
const MAX_OVERSHOOT: f64 = 0.999;

/// Draws of the last allowed landing that may miss the target before the
/// landing is pulled onto the target's nearest point.
const LANDING_ATTEMPTS: usize = 8;

/// Chance that a correction starts while the previous movement is still slowing down.
const OVERLAP_CHANCE: f64 = 0.5;

/// Head start of an overlapping correction, as a fraction of the previous
/// movement's duration.
const OVERLAP: Range<f64> = 0.1..0.3;

/// Median pause in seconds before a correction that waits for the cursor to
/// stop, modeling the visual feedback delay, and the standard deviation of
/// its logarithm.
const PAUSE_MEDIAN: f64 = 0.085;
const PAUSE_SPREAD: f64 = 0.4;

/// Bounds on that pause in seconds.
const MIN_PAUSE: f64 = 0.02;
const MAX_PAUSE: f64 = 0.4;

/// Pause factor when a correction reverses direction: an overshoot is noticed
/// while still moving, so the reaction comes sooner.
const REVERSAL_PAUSE: f64 = 0.5;

/// Skew ranges of the Beta speed profile, whose velocity peaks at
/// `2 / (skew + 1)` of the movement: between 36% and 46% for the primary
/// movement and between 43% and 50% for corrections.
const PRIMARY_SKEW: Range<f64> = 3.35..4.56;
const CORRECTION_SKEW: Range<f64> = 3.0..3.7;

/// Speed modulation amplitude at zero jerk smoothness. It must stay below
/// `1 / (3π)` so the progress function remains monotonic.
const MAX_WOBBLE: f64 = 0.08;

/// Bow of the primary curve as a fraction of distance, chosen by how far the
/// move reaches relative to the screen diagonal.
const SHORT_CURVE: Range<f64> = 0.03..0.10;
const MEDIUM_CURVE: Range<f64> = 0.06..0.20;
const LONG_CURVE: Range<f64> = 0.10..0.28;
const SHORT_REACH: f64 = 0.08;
const MEDIUM_REACH: f64 = 0.25;

/// Bow of a correction as a fraction of its length.
const CORRECTION_CURVE: Range<f64> = 0.03..0.15;

/// Share of a curve's bow set by the person's arc direction; the rest varies
/// from one movement to the next.
const ARC_CONSISTENCY: f64 = 0.7;

/// Largest bow as a fraction of distance, so a very low precision cannot loop.
const MAX_BOW: f64 = 0.3;

/// Maps normalized time to the fraction of a path covered.
#[derive(Debug, Clone)]
struct Profile {
    skew: f64,
    wobble: [f64; 3],
}

impl Profile {
    fn random<R: Rng>(skew: Range<f64>, irregularity: f64, rng: &mut R) -> Self {
        let amplitude = MAX_WOBBLE * irregularity.clamp(0.0, 1.0);

        Self {
            skew: rng.random_range(skew),
            wobble: std::array::from_fn(|_| amplitude * rng.random_range(-1.0..1.0)),
        }
    }

    fn progress(&self, tau: f64) -> f64 {
        let tau = tau.clamp(0.0, 1.0);
        let wobble: f64 = self
            .wobble
            .iter()
            .zip(1..)
            .map(|(&amplitude, k)| amplitude * (f64::from(k) * PI * tau).sin() / f64::from(k))
            .sum();

        beta_progress((tau + wobble).clamp(0.0, 1.0), self.skew)
    }
}

/// Returns the regularized incomplete beta function `I_x(3, b)`, the progress
/// of a movement whose speed follows the Beta(3, b) density.
///
/// At `b = 3` this is the minimum jerk polynomial. Larger `b` keeps its smooth
/// start and moves the speed peak earlier, to `2 / (b + 1)`.
fn beta_progress(x: f64, b: f64) -> f64 {
    1.0 - (1.0 - x).powf(b) * (1.0 + b * x + 0.5 * b * (b + 1.0) * x * x)
}

/// One submovement: a curve walked with a speed profile from `start` seconds.
#[derive(Debug, Clone)]
struct Stroke {
    start: f64,
    duration: f64,
    heading: Point,
    curve: Curve,
    profile: Profile,
}

impl Stroke {
    /// Returns how far the stroke has carried the cursor by `time` seconds.
    fn displacement(&self, time: f64) -> Point {
        let fraction = self.profile.progress((time - self.start) / self.duration);

        curve::sub(self.curve.at_fraction(fraction), self.curve.start())
    }

    fn end(&self) -> f64 {
        self.start + self.duration
    }
}

/// The submovements of one aimed movement on a shared time axis, in seconds.
///
/// Each correction adds its displacement to the ones before it, so one that
/// starts early blends into the slowing movement instead of waiting for it to
/// stop, and the cursor still comes to rest exactly on the last landing.
#[derive(Debug, Clone)]
pub(crate) struct Plan {
    origin: Point,
    strokes: Vec<Stroke>,
    landing: Point,
    duration: f64,
}

impl Plan {
    /// Plans a movement from `start` that comes to rest inside `area`.
    ///
    /// Each phase aims at the target and lands with an error that grows with
    /// its speed. While the cursor is outside the area and phases remain, a
    /// correction follows, and the last allowed phase lands inside the area.
    pub(crate) fn new<R: Rng>(
        start: Point,
        area: &Area,
        config: &ScreenConfig,
        style: &MovementStyle,
        rng: &mut R,
    ) -> Self {
        let diagonal = config.diagonal().max(1.0);
        let irregularity = 1.0 - style.jerk_smoothness;
        let mut plan = Self {
            origin: start,
            strokes: Vec::new(),
            landing: start,
            duration: 0.0,
        };

        for phase in 0..style.sub_movement_count {
            if area.contains(plan.landing) {
                break;
            }

            let from = plan.landing;
            let aim = area.aim();
            let heading = curve::direction(from, aim);
            let distance = curve::norm(curve::sub(aim, from));
            let bits = (distance / area.width_along(heading) + 1.0).log2();
            let previous = plan.strokes.last();
            let (duration, skew, curvature) = match previous {
                None => (
                    primary_duration(bits, style, rng),
                    PRIMARY_SKEW,
                    primary_curvature(distance / diagonal),
                ),
                Some(_) => (
                    correction_duration(bits, style, rng),
                    CORRECTION_SKEW,
                    CORRECTION_CURVE,
                ),
            };
            let onset = previous.map_or(0.0, |previous| {
                correction_onset(previous, heading, style, rng)
            });
            let landing = if phase + 1 == style.sub_movement_count {
                land_inside(from, area, duration, style, rng)
            } else {
                config.clamp(land(from, aim, duration, style, rng))
            };
            let bow = bow(heading, curvature, style, rng);

            plan.push(Stroke {
                start: onset,
                duration,
                heading,
                curve: Curve::bowed(from, landing, bow, rng),
                profile: Profile::random(skew, irregularity, rng),
            });
        }

        plan
    }

    pub(crate) fn duration(&self) -> f64 {
        self.duration
    }

    /// Returns the point the cursor comes to rest on.
    pub(crate) fn landing(&self) -> Point {
        self.landing
    }

    /// Returns the planned position at `time` seconds: the start plus how far
    /// every submovement has carried the cursor by then.
    pub(crate) fn position(&self, time: f64) -> Point {
        self.strokes.iter().fold(self.origin, |point, stroke| {
            curve::add(point, stroke.displacement(time))
        })
    }

    fn push(&mut self, stroke: Stroke) {
        self.duration = self.duration.max(stroke.end());
        self.landing = stroke.curve.end();
        self.strokes.push(stroke);
    }
}

fn primary_duration<R: Rng>(bits: f64, style: &MovementStyle, rng: &mut R) -> f64 {
    let duration = (PRIMARY_INTERCEPT + PRIMARY_SLOPE * bits) / style.speed_factor;

    (duration * duration_jitter(rng)).clamp(MIN_DURATION, MAX_DURATION)
}

fn correction_duration<R: Rng>(bits: f64, style: &MovementStyle, rng: &mut R) -> f64 {
    let duration = (CORRECTION_INTERCEPT + CORRECTION_SLOPE * bits) / style.speed_factor;

    (duration * duration_jitter(rng)).clamp(MIN_DURATION, MAX_CORRECTION_DURATION)
}

fn duration_jitter<R: Rng>(rng: &mut R) -> f64 {
    (DURATION_SPREAD * normal(rng)).exp()
}

/// Returns when a correction heading along `heading` starts, in seconds.
///
/// It either starts while `previous` is still slowing down, blending into it,
/// or waits a skewed pause after the cursor stops, a shorter one when it
/// reverses direction.
fn correction_onset<R: Rng>(
    previous: &Stroke,
    heading: Point,
    style: &MovementStyle,
    rng: &mut R,
) -> f64 {
    if rng.random_bool(OVERLAP_CHANCE) {
        return previous.end() - rng.random_range(OVERLAP) * previous.duration;
    }

    let pause = PAUSE_MEDIAN * (PAUSE_SPREAD * normal(rng)).exp() / style.speed_factor.sqrt();
    let pause = if curve::dot(previous.heading, heading) < 0.0 {
        pause * REVERSAL_PAUSE
    } else {
        pause
    };

    previous.end() + pause.clamp(MIN_PAUSE, MAX_PAUSE)
}

/// Draws where a movement from `from` aimed at `aim` lands after `duration` seconds.
///
/// The error along the movement axis is shifted so that the style's
/// overshoot chance is the probability of landing beyond `aim`.
fn land<R: Rng>(
    from: Point,
    aim: Point,
    duration: f64,
    style: &MovementStyle,
    rng: &mut R,
) -> Point {
    let heading = curve::direction(from, aim);
    let distance = curve::norm(curve::sub(aim, from));
    let spread = SPREAD_PER_SPEED * distance / duration / style.precision;
    let bias = normal_quantile(style.overshoot_tendency.clamp(MIN_OVERSHOOT, MAX_OVERSHOOT));
    let along = spread * (normal(rng) + bias);
    let lateral = spread * LATERAL_SPREAD * normal(rng);

    curve::add(
        aim,
        curve::add(
            curve::scale(heading, along),
            curve::scale(curve::perpendicular(heading), lateral),
        ),
    )
}

/// Draws the landing of the last allowed movement, which ends inside `area`.
fn land_inside<R: Rng>(
    from: Point,
    area: &Area,
    duration: f64,
    style: &MovementStyle,
    rng: &mut R,
) -> Point {
    let mut landing = land(from, area.aim(), duration, style, rng);

    for _ in 1..LANDING_ATTEMPTS {
        if area.contains(landing) {
            return landing;
        }

        landing = land(from, area.aim(), duration, style, rng);
    }

    area.clamp(landing)
}

fn primary_curvature(reach: f64) -> Range<f64> {
    if reach < SHORT_REACH {
        SHORT_CURVE
    } else if reach < MEDIUM_REACH {
        MEDIUM_CURVE
    } else {
        LONG_CURVE
    }
}

/// Draws the signed bow of a curve heading along `heading`, as a fraction of its length.
///
/// Most of the bow follows the style's arc direction, so one person bends
/// movements in a given direction the same way, and the rest varies.
fn bow<R: Rng>(heading: Point, curvature: Range<f64>, style: &MovementStyle, rng: &mut R) -> f64 {
    let angle = heading[1].atan2(heading[0]);
    let side = ARC_CONSISTENCY * (angle - style.arc_direction).cos()
        + (1.0 - ARC_CONSISTENCY) * rng.random_range(-1.0..1.0);

    (rng.random_range(curvature) / style.precision * side).clamp(-MAX_BOW, MAX_BOW)
}

#[cfg(test)]
mod tests {
    use rand::{SeedableRng, rngs::SmallRng};

    use super::*;
    use crate::target::Target;

    fn config() -> ScreenConfig {
        ScreenConfig::default()
    }

    fn plan(
        start: Point,
        target: Target,
        style: &MovementStyle,
        rng: &mut SmallRng,
    ) -> (Plan, Area) {
        let area = Area::new(&target, &config());

        (Plan::new(start, &area, &config(), style, rng), area)
    }

    #[test]
    fn beta_progress_generalizes_minimum_jerk() {
        for step in 0..=1000 {
            let x = f64::from(step) / 1000.0;
            let minimum_jerk = x * x * x * (10.0 + x * (6.0 * x - 15.0));

            assert!((beta_progress(x, 3.0) - minimum_jerk).abs() < 1e-12);
        }

        for skew in [PRIMARY_SKEW.start, 4.0, PRIMARY_SKEW.end] {
            let steps = 10_000;
            let (peak, _) = (1..=steps)
                .map(|step| {
                    let x = f64::from(step) / f64::from(steps);
                    let before = f64::from(step - 1) / f64::from(steps);

                    (x, beta_progress(x, skew) - beta_progress(before, skew))
                })
                .fold((0.0, 0.0), |best, candidate| {
                    if candidate.1 > best.1 {
                        candidate
                    } else {
                        best
                    }
                });

            assert!(
                (peak - 2.0 / (skew + 1.0)).abs() < 1e-3,
                "peak at {peak} for skew {skew}"
            );
        }
    }

    #[test]
    fn progress_stays_monotonic_at_maximum_irregularity() {
        let profile = Profile {
            skew: PRIMARY_SKEW.end,
            wobble: [MAX_WOBBLE, -MAX_WOBBLE, MAX_WOBBLE],
        };

        assert_eq!(profile.progress(0.0), 0.0);
        assert_eq!(profile.progress(1.0), 1.0);

        let mut previous = 0.0;
        for step in 1..=2000 {
            let value = profile.progress(f64::from(step) / 2000.0);

            assert!(value >= previous, "progress fell at step {step}");
            previous = value;
        }
    }

    #[test]
    fn plan_starts_at_origin_and_rests_inside_the_target() {
        let mut rng = SmallRng::seed_from_u64(11);
        let style = MovementStyle {
            sub_movement_count: 4,
            ..MovementStyle::default()
        };
        let start = [100.0, 100.0];

        for _ in 0..200 {
            let (plan, area) = plan(
                start,
                Target::new([1200.0, 800.0], [12.0, 12.0]),
                &style,
                &mut rng,
            );
            let rest = plan.position(plan.duration());

            assert_eq!(plan.position(0.0), start);
            assert!(curve::norm(curve::sub(rest, plan.landing())) < 1e-9);
            assert!(area.contains(plan.landing()));
            assert!((1..=4).contains(&plan.strokes.len()));
            assert!(plan.duration() > MIN_DURATION);
        }
    }

    #[test]
    fn overlapping_corrections_blend_without_stopping() {
        let mut rng = SmallRng::seed_from_u64(12);
        let style = MovementStyle {
            sub_movement_count: 2,
            ..MovementStyle::default()
        };
        let (mut overlapping, mut blended) = (0, 0);

        for _ in 0..400 {
            let (plan, _) = plan(
                [100.0, 500.0],
                Target::point([900.0, 700.0]),
                &style,
                &mut rng,
            );
            let [primary, correction] = plan.strokes.as_slice() else {
                continue;
            };

            if correction.start >= primary.end()
                || curve::dot(primary.heading, correction.heading) < 0.0
            {
                continue;
            }

            let speed = |time: f64| {
                curve::norm(curve::sub(
                    plan.position(time + 1e-3),
                    plan.position(time - 1e-3),
                )) / 2e-3
            };
            let peak = (0..=200)
                .map(|step| speed(primary.duration * f64::from(step) / 200.0))
                .fold(0.0, f64::max);
            let slowest = (0..=100)
                .map(|step| {
                    speed(correction.start + 0.45 * correction.duration * f64::from(step) / 100.0)
                })
                .fold(f64::INFINITY, f64::min);

            overlapping += 1;
            blended += usize::from(slowest > 0.02 * peak);
        }

        assert!(
            overlapping > 50,
            "only {overlapping} overlapping corrections"
        );
        assert!(
            blended * 10 > overlapping * 8,
            "{blended} of {overlapping} overlapping corrections kept moving"
        );
    }

    #[test]
    fn overshoot_chance_is_the_style_probability() {
        let mut rng = SmallRng::seed_from_u64(13);
        let aim = [1000.0, 540.0];

        for chance in [0.1, 0.3, 0.7] {
            let style = MovementStyle {
                overshoot_tendency: chance,
                sub_movement_count: 2,
                ..MovementStyle::default()
            };
            let trials = 4000;
            let overshoots = (0..trials)
                .filter(|_| {
                    let (plan, _) = plan([200.0, 540.0], Target::point(aim), &style, &mut rng);

                    plan.strokes[0].curve.end()[0] > aim[0]
                })
                .count();
            let observed = overshoots as f64 / f64::from(trials);

            assert!(
                (observed - chance).abs() < 0.03,
                "chance {chance}, observed {observed}"
            );
        }
    }

    #[test]
    fn smaller_targets_take_longer_and_need_more_corrections() {
        let mut rng = SmallRng::seed_from_u64(14);
        let style = MovementStyle::default();
        let summary = |size: f64, rng: &mut SmallRng| {
            let trials = 600;
            let (time, corrected) = (0..trials).fold((0.0, 0), |(time, corrected), _| {
                let (plan, _) = plan(
                    [300.0, 300.0],
                    Target::new([900.0, 300.0], [size, size]),
                    &style,
                    rng,
                );

                (
                    time + plan.duration(),
                    corrected + usize::from(plan.strokes.len() > 1),
                )
            });

            (
                time / f64::from(trials),
                corrected as f64 / f64::from(trials),
            )
        };
        let (large_time, large_rate) = summary(200.0, &mut rng);
        let (medium_time, medium_rate) = summary(30.0, &mut rng);
        let (small_time, small_rate) = summary(4.0, &mut rng);

        assert!(large_time < medium_time && medium_time < small_time);
        assert!(large_rate < 0.1, "large target corrected {large_rate}");
        assert!(large_rate < medium_rate && medium_rate < small_rate);
        assert!(small_rate > 0.9, "small target corrected {small_rate}");
    }

    #[test]
    fn a_person_bows_a_given_direction_the_same_way() {
        let mut rng = SmallRng::seed_from_u64(15);
        let style = MovementStyle {
            arc_direction: 0.0,
            ..MovementStyle::default()
        };
        let rightward = [1.0, 0.0];
        let downward = [0.0, 1.0];
        let right_of_travel = (0..1000)
            .filter(|_| bow(rightward, LONG_CURVE, &style, &mut rng) > 0.0)
            .count();
        let across: f64 = (0..1000)
            .map(|_| bow(downward, LONG_CURVE, &style, &mut rng).abs())
            .sum::<f64>()
            / 1000.0;
        let along: f64 = (0..1000)
            .map(|_| bow(rightward, LONG_CURVE, &style, &mut rng).abs())
            .sum::<f64>()
            / 1000.0;

        assert_eq!(right_of_travel, 1000);
        assert!(across < 0.3 * along, "across {across}, along {along}");
    }
}
