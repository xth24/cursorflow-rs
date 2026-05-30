use rand::{Rng, RngExt};

#[derive(Debug, Clone, Copy)]
pub struct MovementStyle {
    pub speed_factor: f64,
    pub precision: f64,
    pub nervousness: f64,
    pub overshoot_tendency: f64,
    pub sub_movement_count: usize,
    pub jerk_smoothness: f64,
}

impl Default for MovementStyle {
    fn default() -> Self {
        Self {
            speed_factor: 1.0,
            precision: 1.0,
            nervousness: 0.4,
            overshoot_tendency: 0.3,
            sub_movement_count: 2,
            jerk_smoothness: 0.85,
        }
    }
}

impl MovementStyle {
    pub fn random<R: Rng>(rng: &mut R) -> Self {
        let choices = [1usize, 2, 2, 3];
        Self {
            speed_factor: rng.random_range(0.8..1.3),
            precision: rng.random_range(0.7..1.3),
            nervousness: rng.random_range(0.15..0.65),
            overshoot_tendency: rng.random_range(0.1..0.5),
            sub_movement_count: choices[rng.random_range(0..choices.len())],
            jerk_smoothness: rng.random_range(0.7..0.95),
        }
    }

    pub(crate) fn sanitized(self) -> Self {
        let defaults = Self::default();
        Self {
            speed_factor: finite_or(self.speed_factor, defaults.speed_factor).clamp(0.05, 10.0),
            precision: finite_or(self.precision, defaults.precision).clamp(0.05, 10.0),
            nervousness: finite_or(self.nervousness, defaults.nervousness).clamp(0.0, 2.0),
            overshoot_tendency: finite_or(self.overshoot_tendency, defaults.overshoot_tendency)
                .clamp(0.0, 1.0),
            sub_movement_count: self.sub_movement_count.clamp(1, 4),
            jerk_smoothness: finite_or(self.jerk_smoothness, defaults.jerk_smoothness)
                .clamp(0.0, 1.0),
        }
    }
}

fn finite_or(value: f64, fallback: f64) -> f64 {
    if value.is_finite() { value } else { fallback }
}
