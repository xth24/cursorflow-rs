//! Movement characteristics of one person.

use rand::{Rng, RngExt};

/// How a particular person moves the cursor.
///
/// Every field is a dial on the movement model. Values outside the documented
/// ranges are clamped, and values that are not finite fall back to the defaults, so any
/// style produces a valid trajectory. [`MovementStyle::random`] draws a
/// plausible persona; the [`Default`] is an average user.
///
/// # Examples
///
/// ```
/// use cursorflow::{MovementStyle, ScreenConfig, generate_single};
///
/// let careful = MovementStyle {
///     speed_factor: 0.8,
///     precision: 1.4,
///     ..MovementStyle::default()
/// };
/// let mut rng = rand::rng();
///
/// let path = generate_single([50.0, 50.0], [600.0, 300.0], &ScreenConfig::default(), &careful, &mut rng);
///
/// assert_eq!((path.x[0], path.y[0]), (50, 50));
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MovementStyle {
    /// Scales speed; `1.0` is average and larger is faster. Durations divide by it.
    pub speed_factor: f64,
    /// Steadiness of aim, from `0.05` to `10.0`. Larger values give straighter
    /// paths, smaller landing errors, and less tremor.
    pub precision: f64,
    /// Amount of hand tremor and path wobble, from `0.0` to `2.0`.
    pub nervousness: f64,
    /// Tendency of the primary movement to fly past the target rather than fall
    /// short, from `0.0` to `1.0`.
    pub overshoot_tendency: f64,
    /// Number of movement phases including the primary one, from `1` to `4`.
    /// Each further phase is a pause and a corrective movement, made only while
    /// the cursor is still off target.
    pub sub_movement_count: usize,
    /// Regularity of the speed profile, from `0.0` to `1.0`. Lower values add
    /// speed fluctuations within a movement.
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
    /// Draws a plausible style, varying every field around its default.
    ///
    /// # Examples
    ///
    /// ```
    /// use cursorflow::MovementStyle;
    ///
    /// let style = MovementStyle::random(&mut rand::rng());
    ///
    /// assert!((0.8..1.3).contains(&style.speed_factor));
    /// ```
    pub fn random<R: Rng>(rng: &mut R) -> Self {
        let phases = [1, 2, 2, 3];

        Self {
            speed_factor: rng.random_range(0.8..1.3),
            precision: rng.random_range(0.7..1.3),
            nervousness: rng.random_range(0.15..0.65),
            overshoot_tendency: rng.random_range(0.1..0.5),
            sub_movement_count: phases[rng.random_range(0..phases.len())],
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
