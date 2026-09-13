//! Movement planning: phase timing, landing errors, and speed profiles.

use std::f64::consts::PI;
use std::ops::Range;

use rand::{Rng, RngExt};

use crate::curve::{self, Curve, Point};
use crate::noise::normal;
use crate::style::MovementStyle;

/// Fitts's law intercept in seconds.
const FITTS_INTERCEPT: f64 = 0.18;

/// Fitts's law slope in seconds per bit of difficulty.
const FITTS_SLOPE: f64 = 0.38;

/// Converts distance relative to the screen diagonal into an index of
/// difficulty; `12` makes a move across the full diagonal worth about 3.7 bits.
const FITTS_GAIN: f64 = 12.0;

/// Bounds on the primary movement's duration in seconds, after style scaling.
const MIN_DURATION: f64 = 0.06;
const MAX_DURATION: f64 = 2.0;

/// Longest a single correction may take, in seconds.
const MAX_CORRECTION_DURATION: f64 = 0.6;

/// Endpoint spread of the primary movement as a fraction of its distance,
/// before precision and speed adjustments.
const LANDING_SPREAD: f64 = 0.04;

/// Sideways landing spread relative to the spread along the movement axis.
const LATERAL_SPREAD: f64 = 0.45;

/// Overshoot tendency at which the landing bias along the movement axis is zero.
const NEUTRAL_OVERSHOOT: f64 = 0.35;

/// Landing bias, in spread units, per unit of overshoot tendency above neutral.
const OVERSHOOT_GAIN: f64 = 1.4;

/// Errors below this many pixels vanish when rounding, so they go uncorrected.
const MIN_CORRECTION: f64 = 1.0;

/// Share of the Fitts duration taken by the primary movement when corrections follow.
const PRIMARY_SHARE: Range<f64> = 0.62..0.78;

/// Pause before a correction in seconds, modelling the visual feedback delay.
const DWELL: Range<f64> = 0.04..0.15;

/// Pause factor when a correction reverses direction: an overshoot is noticed
/// while still moving, so the reaction comes sooner.
const REVERSAL_DWELL: f64 = 0.5;

/// The fixed part and the part per bit of a correction's duration, in seconds.
const CORRECTION_INTERCEPT: f64 = 0.07;
const CORRECTION_SLOPE: f64 = 0.04;

/// Pixels of error that count as one bit of correction difficulty.
const CORRECTION_SCALE: f64 = 3.0;

/// Error left after a correction as a fraction of the error it targeted;
/// negative values fall short, which is the more common outcome.
const RESIDUAL_ALONG: Range<f64> = -0.25..0.12;
const RESIDUAL_LATERAL: f64 = 0.08;

/// Exponent ranges for the time warp. Values below one move the velocity
/// peak earlier; the primary range puts it between 36% and 46% of the move.
const PRIMARY_GAMMA: Range<f64> = 0.72..0.92;
const CORRECTION_GAMMA: Range<f64> = 0.86..1.0;

/// Speed modulation amplitude at zero jerk smoothness. It must stay below
/// `1 / (3π)` so the progress function remains monotonic.
const MAX_WOBBLE: f64 = 0.08;

/// Sideways bulge of the primary curve as a fraction of distance, chosen by
/// how far the move reaches relative to the screen diagonal.
const SHORT_CURVE: Range<f64> = 0.02..0.06;
const MEDIUM_CURVE: Range<f64> = 0.04..0.12;
const LONG_CURVE: Range<f64> = 0.06..0.18;
const SHORT_REACH: f64 = 0.08;
const MEDIUM_REACH: f64 = 0.25;

/// Sideways bulge of a correction as a fraction of its length.
const CORRECTION_CURVE: Range<f64> = 0.02..0.10;

/// Maps normalized time to the fraction of a path covered.
#[derive(Debug, Clone)]
struct Profile {
    gamma: f64,
    wobble: [f64; 3],
}

impl Profile {
    fn random<R: Rng>(gamma: Range<f64>, irregularity: f64, rng: &mut R) -> Self {
        let amplitude = MAX_WOBBLE * irregularity.clamp(0.0, 1.0);

        Self {
            gamma: rng.random_range(gamma),
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
        let warped = (tau + wobble).clamp(0.0, 1.0).powf(self.gamma);

        minimum_jerk(warped)
    }
}

fn minimum_jerk(x: f64) -> f64 {
    x * x * x * (10.0 + x * (6.0 * x - 15.0))
}

#[derive(Debug, Clone)]
enum Motion {
    Hold(Point),
    Move { curve: Curve, profile: Profile },
}

#[derive(Debug, Clone)]
struct Phase {
    start: f64,
    duration: f64,
    motion: Motion,
}

impl Phase {
    fn position(&self, time: f64) -> Point {
        match &self.motion {
            Motion::Hold(point) => *point,
            Motion::Move { curve, profile } => {
                curve.at_fraction(profile.progress((time - self.start) / self.duration))
            }
        }
    }

    fn end(&self) -> Point {
        match &self.motion {
            Motion::Hold(point) => *point,
            Motion::Move { curve, .. } => curve.end(),
        }
    }
}

/// The phases of one aimed movement laid out on a shared time axis, in seconds.
#[derive(Debug, Clone)]
pub(crate) struct Plan {
    phases: Vec<Phase>,
    duration: f64,
}

impl Plan {
    pub(crate) fn new<R: Rng>(
        start: Point,
        end: Point,
        diagonal: f64,
        style: &MovementStyle,
        rng: &mut R,
    ) -> Self {
        let distance = curve::norm(curve::sub(end, start));
        let corrections = style.sub_movement_count.saturating_sub(1);
        let irregularity = 1.0 - style.jerk_smoothness;
        let landing = if corrections > 0 {
            landing_error(start, end, style, rng)
        } else {
            [0.0, 0.0]
        };
        let corrected = curve::norm(landing) >= MIN_CORRECTION;
        let landing = if corrected { landing } else { [0.0, 0.0] };
        let duration = fitts_duration(distance, diagonal, style, rng);
        let duration = if corrected {
            duration * rng.random_range(PRIMARY_SHARE)
        } else {
            duration
        };
        let curvature = primary_curvature(distance / diagonal, style, rng);

        let mut plan = Self {
            phases: Vec::new(),
            duration: 0.0,
        };
        plan.push_move(
            Curve::between(start, curve::add(end, landing), curvature, rng),
            Profile::random(PRIMARY_GAMMA, irregularity, rng),
            duration,
        );

        let mut heading = curve::direction(start, end);
        for index in 0..corrections {
            let from = plan.end();
            let remaining = curve::sub(end, from);
            let gap = curve::norm(remaining);

            if gap < MIN_CORRECTION {
                break;
            }

            let residual = if index + 1 == corrections {
                [0.0, 0.0]
            } else {
                residual_error(remaining, style, rng)
            };
            let target = curve::add(end, residual);
            let next = curve::direction(from, target);
            let dwell = rng.random_range(DWELL) / style.speed_factor.sqrt();
            let dwell = if curve::dot(heading, next) < 0.0 {
                dwell * REVERSAL_DWELL
            } else {
                dwell
            };
            let curvature = rng.random_range(CORRECTION_CURVE) / style.precision;

            plan.push_hold(from, dwell);
            plan.push_move(
                Curve::between(from, target, curvature, rng),
                Profile::random(CORRECTION_GAMMA, irregularity, rng),
                correction_duration(gap, style, rng),
            );
            heading = next;
        }

        plan
    }

    pub(crate) fn duration(&self) -> f64 {
        self.duration
    }

    /// Returns the planned position at `time` seconds, holding the endpoints
    /// outside the planned span.
    pub(crate) fn position(&self, time: f64) -> Point {
        let phase = self
            .phases
            .iter()
            .rev()
            .find(|phase| time >= phase.start)
            .or_else(|| self.phases.first());

        phase.map_or([0.0, 0.0], |phase| phase.position(time))
    }

    fn end(&self) -> Point {
        self.phases.last().map_or([0.0, 0.0], Phase::end)
    }

    fn push_move(&mut self, curve: Curve, profile: Profile, duration: f64) {
        self.push(Motion::Move { curve, profile }, duration);
    }

    fn push_hold(&mut self, point: Point, duration: f64) {
        self.push(Motion::Hold(point), duration);
    }

    fn push(&mut self, motion: Motion, duration: f64) {
        self.phases.push(Phase {
            start: self.duration,
            duration,
            motion,
        });
        self.duration += duration;
    }
}

fn fitts_duration<R: Rng>(distance: f64, diagonal: f64, style: &MovementStyle, rng: &mut R) -> f64 {
    let reach = if distance.is_finite() && diagonal > 0.0 {
        (distance / diagonal).max(0.0)
    } else {
        0.0
    };
    let intercept = FITTS_INTERCEPT + rng.random_range(-0.02..0.02);
    let slope = FITTS_SLOPE + rng.random_range(-0.03..0.03);
    let duration = (intercept + slope * (1.0 + FITTS_GAIN * reach).log2()) / style.speed_factor;

    (duration * rng.random_range(0.92..1.08)).clamp(MIN_DURATION, MAX_DURATION)
}

fn correction_duration<R: Rng>(gap: f64, style: &MovementStyle, rng: &mut R) -> f64 {
    let bits = (1.0 + gap / CORRECTION_SCALE).log2();
    let duration = (CORRECTION_INTERCEPT + CORRECTION_SLOPE * bits) / style.speed_factor;

    (duration * rng.random_range(0.9..1.1)).clamp(MIN_DURATION, MAX_CORRECTION_DURATION)
}

fn primary_curvature<R: Rng>(reach: f64, style: &MovementStyle, rng: &mut R) -> f64 {
    let range = if reach < SHORT_REACH {
        SHORT_CURVE
    } else if reach < MEDIUM_REACH {
        MEDIUM_CURVE
    } else {
        LONG_CURVE
    };

    rng.random_range(range) / style.precision
}

fn landing_error<R: Rng>(start: Point, end: Point, style: &MovementStyle, rng: &mut R) -> Point {
    let distance = curve::norm(curve::sub(end, start));
    let spread = distance * LANDING_SPREAD * style.speed_factor.sqrt() / style.precision;
    let bias = (style.overshoot_tendency - NEUTRAL_OVERSHOOT) * OVERSHOOT_GAIN;
    let heading = curve::direction(start, end);
    let along = spread * (normal(rng) + bias);
    let lateral = spread * LATERAL_SPREAD * normal(rng);

    curve::add(
        curve::scale(heading, along),
        curve::scale(curve::perpendicular(heading), lateral),
    )
}

fn residual_error<R: Rng>(remaining: Point, style: &MovementStyle, rng: &mut R) -> Point {
    let gap = curve::norm(remaining);
    let heading = curve::scale(remaining, 1.0 / gap);
    let accuracy = (1.0 / style.precision).clamp(0.5, 2.0);
    let along = gap * rng.random_range(RESIDUAL_ALONG) * accuracy;
    let lateral = gap * RESIDUAL_LATERAL * normal(rng) * accuracy;

    curve::add(
        curve::scale(heading, along),
        curve::scale(curve::perpendicular(heading), lateral),
    )
}

#[cfg(test)]
mod tests {
    use rand::{SeedableRng, rngs::SmallRng};

    use super::*;

    #[test]
    fn progress_stays_monotonic_at_maximum_irregularity() {
        let profile = Profile {
            gamma: PRIMARY_GAMMA.start,
            wobble: [MAX_WOBBLE, -MAX_WOBBLE, MAX_WOBBLE],
        };

        assert_eq!(profile.progress(0.0), 0.0);
        assert_eq!(profile.progress(1.0), 1.0);

        let mut previous = 0.0;
        for step in 1..=2000 {
            let value = profile.progress(step as f64 / 2000.0);

            assert!(value >= previous, "progress fell at step {step}");
            previous = value;
        }
    }

    #[test]
    fn velocity_peaks_before_the_midpoint() {
        let profile = Profile {
            gamma: 0.8,
            wobble: [0.0; 3],
        };
        let steps = 1000;
        let (peak_step, _) = (1..steps)
            .map(|step| {
                let before = profile.progress((step - 1) as f64 / steps as f64);
                let after = profile.progress((step + 1) as f64 / steps as f64);
                (step, after - before)
            })
            .fold((0, 0.0), |best, candidate| {
                if candidate.1 > best.1 {
                    candidate
                } else {
                    best
                }
            });
        let peak = peak_step as f64 / steps as f64;

        assert!((0.35..0.45).contains(&peak), "peak at {peak}");
    }

    #[test]
    fn plan_starts_at_origin_and_ends_on_target() {
        let mut rng = SmallRng::seed_from_u64(11);
        let style = MovementStyle {
            sub_movement_count: 4,
            ..MovementStyle::default()
        };
        let start = [100.0, 100.0];
        let end = [1200.0, 800.0];
        let plan = Plan::new(start, end, 2203.0, &style, &mut rng);

        assert_eq!(plan.position(0.0), start);
        assert!(curve::norm(curve::sub(plan.position(plan.duration()), end)) < MIN_CORRECTION);
        assert!(
            plan.phases.len() >= 3,
            "expected a correction, got {} phases",
            plan.phases.len()
        );
        assert!(plan.duration() > MIN_DURATION);
    }
}
