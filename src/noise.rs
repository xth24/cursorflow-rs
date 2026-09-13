//! Correlated noise that mimics an unsteady hand.

use rand::Rng;
use rand_distr::{Distribution, StandardNormal};

use crate::curve::Point;

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
    pub(crate) fn new<R: Rng>(time_constant: f64, rng: &mut R) -> Self {
        Self {
            time_constant,
            value: [normal(rng), normal(rng)],
        }
    }

    /// Advances the process by `dt` seconds and returns its new value.
    pub(crate) fn step<R: Rng>(&mut self, dt: f64, rng: &mut R) -> Point {
        let decay = (-dt.max(0.0) / self.time_constant).exp();
        let spread = (1.0 - decay * decay).sqrt();

        for value in &mut self.value {
            *value = *value * decay + spread * normal(rng);
        }

        self.value
    }
}

pub(crate) fn normal<R: Rng>(rng: &mut R) -> f64 {
    StandardNormal.sample(rng)
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
}
