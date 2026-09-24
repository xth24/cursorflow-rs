//! Correlated noise that mimics an unsteady hand.

use std::f64::consts::TAU;

use rand::Rng;
use rand_distr::{Distribution, StandardNormal};

use crate::curve::Point;

/// Probability below which, and above one minus which, the normal quantile
/// switches from its central approximation to its tail approximation.
const QUANTILE_TAIL: f64 = 0.02425;

/// A two dimensional Ornstein Uhlenbeck process with unit stationary variance.
///
/// Successive values decorrelate over the time constant, so the process looks
/// alike at any sample rate.
#[derive(Debug, Clone)]
pub(crate) struct Drift {
    time_constant: f64,
    value: Point,
}

impl Drift {
    /// Starts the process at a draw from its stationary distribution.
    pub(crate) fn new<R: Rng>(time_constant: f64, rng: &mut R) -> Self {
        Self {
            time_constant,
            value: [normal(rng), normal(rng)],
        }
    }

    /// Starts the process at zero, so its spread builds up over the time constant.
    pub(crate) const fn still(time_constant: f64) -> Self {
        Self {
            time_constant,
            value: [0.0, 0.0],
        }
    }

    /// Advances the process by `dt` seconds and returns its new value.
    ///
    /// A step of zero or less leaves the value unchanged.
    pub(crate) fn step<R: Rng>(&mut self, dt: f64, rng: &mut R) -> Point {
        if dt.is_nan() || dt <= 0.0 {
            return self.value;
        }

        let decay = (-dt / self.time_constant).exp();
        let spread = (1.0 - decay * decay).sqrt();

        for value in &mut self.value {
            *value = *value * decay + spread * normal(rng);
        }

        self.value
    }
}

/// Physiological tremor: a narrow band process with unit variance per axis.
///
/// Two slow [`Drift`] processes set the in phase and quadrature amplitudes of
/// a carrier, so the spectrum peaks at the carrier frequency and spreads over
/// about the inverse of the time constant.
#[derive(Debug, Clone)]
pub(crate) struct Tremor {
    angular_frequency: f64,
    in_phase: Drift,
    quadrature: Drift,
}

impl Tremor {
    /// Starts a tremor at `frequency` hertz whose amplitude drifts over
    /// `time_constant` seconds.
    pub(crate) fn new<R: Rng>(frequency: f64, time_constant: f64, rng: &mut R) -> Self {
        Self {
            angular_frequency: TAU * frequency,
            in_phase: Drift::new(time_constant, rng),
            quadrature: Drift::new(time_constant, rng),
        }
    }

    /// Advances the amplitudes by `dt` seconds and returns the tremor at `time` seconds.
    pub(crate) fn step<R: Rng>(&mut self, time: f64, dt: f64, rng: &mut R) -> Point {
        let (sin, cos) = (self.angular_frequency * time).sin_cos();
        let [ix, iy] = self.in_phase.step(dt, rng);
        let [qx, qy] = self.quadrature.step(dt, rng);

        [ix * cos + qx * sin, iy * cos + qy * sin]
    }
}

pub(crate) fn normal<R: Rng>(rng: &mut R) -> f64 {
    StandardNormal.sample(rng)
}

/// Returns the standard normal quantile of `p`, the inverse of its distribution function.
///
/// Uses Acklam's rational approximation, whose relative error stays below
/// about `1.2e-9`. `p` must lie strictly between zero and one.
pub(crate) fn normal_quantile(p: f64) -> f64 {
    const A: [f64; 6] = [
        -3.969_683_028_665_376e1,
        2.209_460_984_245_205e2,
        -2.759_285_104_469_687e2,
        1.383_577_518_672_69e2,
        -3.066_479_806_614_716e1,
        2.506_628_277_459_239,
    ];
    const B: [f64; 5] = [
        -5.447_609_879_822_406e1,
        1.615_858_368_580_409e2,
        -1.556_989_798_598_866e2,
        6.680_131_188_771_972e1,
        -1.328_068_155_288_572e1,
    ];

    if p < QUANTILE_TAIL {
        return quantile_tail(p);
    }

    if p > 1.0 - QUANTILE_TAIL {
        return -quantile_tail(1.0 - p);
    }

    let q = p - 0.5;
    let r = q * q;
    let numerator = A.iter().fold(0.0, |sum, &a| sum * r + a);
    let denominator = B.iter().fold(0.0, |sum, &b| sum * r + b) * r + 1.0;

    numerator * q / denominator
}

/// Returns the normal quantile of a small lower tail probability `p`.
fn quantile_tail(p: f64) -> f64 {
    const C: [f64; 6] = [
        -7.784_894_002_430_293e-3,
        -3.223_964_580_411_365e-1,
        -2.400_758_277_161_838,
        -2.549_732_539_343_734,
        4.374_664_141_464_968,
        2.938_163_982_698_783,
    ];
    const D: [f64; 4] = [
        7.784_695_709_041_462e-3,
        3.224_671_290_700_398e-1,
        2.445_134_137_142_996,
        3.754_408_661_907_416,
    ];

    let q = (-2.0 * p.ln()).sqrt();
    let numerator = C.iter().fold(0.0, |sum, &c| sum * q + c);
    let denominator = D.iter().fold(0.0, |sum, &d| sum * q + d) * q + 1.0;

    numerator / denominator
}

#[cfg(test)]
mod tests {
    use rand::{SeedableRng, rngs::SmallRng};

    use super::*;

    #[test]
    fn drift_keeps_unit_variance_regardless_of_step_size() {
        let mut rng = SmallRng::seed_from_u64(5);

        for dt in [0.001, 0.008, 0.05] {
            let mut drift = Drift::new(0.025, &mut rng);
            let samples = 20_000;
            let energy: f64 = (0..samples)
                .map(|_| {
                    let [x, y] = drift.step(dt, &mut rng);
                    x * x + y * y
                })
                .sum();
            let variance = energy / (2.0 * samples as f64);

            assert!(
                (0.9..1.1).contains(&variance),
                "variance {variance} at dt {dt}"
            );
        }
    }

    #[test]
    fn drift_holds_still_without_time_passing() {
        let mut rng = SmallRng::seed_from_u64(6);
        let mut drift = Drift::still(0.1);

        assert_eq!(drift.step(0.0, &mut rng), [0.0, 0.0]);

        let moved = drift.step(0.05, &mut rng);

        assert_ne!(moved, [0.0, 0.0]);
        assert_eq!(drift.step(0.0, &mut rng), moved);
    }

    #[test]
    fn tremor_has_unit_variance_and_oscillates_at_its_frequency() {
        let mut rng = SmallRng::seed_from_u64(8);
        let frequency = 9.0;
        let dt = 0.001;
        let samples = 200_000;
        let mut tremor = Tremor::new(frequency, 0.1, &mut rng);
        let values: Vec<f64> = (0..samples)
            .map(|index| tremor.step(index as f64 * dt, dt, &mut rng)[0])
            .collect();
        let variance = values.iter().map(|value| value * value).sum::<f64>() / samples as f64;
        let correlation = |seconds: f64| {
            let lag = (seconds / dt).round() as usize;
            let sum: f64 = values.iter().zip(&values[lag..]).map(|(a, b)| a * b).sum();

            sum / ((samples - lag) as f64 * variance)
        };
        let half_period = correlation(0.5 / frequency);
        let full_period = correlation(1.0 / frequency);

        assert!((0.85..1.15).contains(&variance), "variance {variance}");
        assert!(half_period < -0.4, "half period correlation {half_period}");
        assert!(full_period > 0.2, "full period correlation {full_period}");
    }

    #[test]
    fn normal_quantile_matches_known_values() {
        for (p, expected) in [
            (0.5, 0.0),
            (0.3, -0.524_400_512_708_041),
            (0.975, 1.959_963_984_540_054),
            (0.01, -2.326_347_874_040_841),
            (0.001, -3.090_232_306_167_814),
            (0.999_9, 3.719_016_485_455_709),
        ] {
            let value = normal_quantile(p);

            assert!(
                (value - expected).abs() <= 1e-8 * expected.abs().max(1.0),
                "quantile({p}) = {value}, expected {expected}"
            );
        }
    }
}
