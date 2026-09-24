//! Movement characteristics of one person.

use std::f64::consts::PI;

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
/// use cursorflow::{MovementStyle, ScreenConfig, Target, generate_single};
///
/// let careful = MovementStyle {
///     speed_factor: 0.8,
///     precision: 1.4,
///     ..MovementStyle::default()
/// };
/// let button = Target::new([600.0, 300.0], [80.0, 24.0]);
/// let mut rng = rand::rng();
///
/// let path = generate_single([50.0, 50.0], button, &ScreenConfig::default(), &careful, &mut rng);
///
/// assert_eq!((path.x[0], path.y[0]), (50, 50));
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MovementStyle {
    /// Scales speed; `1.0` is average and larger is faster. Durations divide
    /// by it, and landing errors grow with it, since a faster throw is less
    /// accurate.
    pub speed_factor: f64,
    /// Steadiness of aim, from `0.05` to `10.0`. Larger values give straighter
    /// paths, smaller landing errors, and less tremor.
    pub precision: f64,
    /// Amount of hand tremor and path wobble, from `0.0` to `2.0`.
    pub nervousness: f64,
    /// Chance, from `0.0` to `1.0`, that a movement lands beyond the point it
    /// aims at rather than short of it. The extremes act as `0.001` and `0.999`.
    pub overshoot_tendency: f64,
    /// Largest number of movement phases including the primary one, from `1`
    /// to `4`. Each further phase is a corrective movement, made only while the
    /// cursor is outside the target, and the last allowed phase always ends
    /// inside it.
    pub sub_movement_count: usize,
    /// Regularity of the speed profile, from `0.0` to `1.0`. Lower values add
    /// speed fluctuations within a movement.
    pub jerk_smoothness: f64,
    /// Direction of travel, in radians clockwise from the positive x axis as
    /// seen on screen, in which this person's paths bow the most.
    ///
    /// Movements heading this way bow to their right, movements heading the
    /// opposite way bow to their left, which is the same side of the screen,
    /// and movements across it stay nearly straight. The default `π` makes
    /// horizontal sweeps bow upward.
    pub arc_direction: f64,
}

impl Default for MovementStyle {
    fn default() -> Self {
        Self {
            speed_factor: 1.0,
            precision: 1.0,
            nervousness: 0.4,
            overshoot_tendency: 0.3,
            sub_movement_count: 3,
            jerk_smoothness: 0.85,
            arc_direction: PI,
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
        let phases = [2, 3, 3, 4];

        Self {
            speed_factor: rng.random_range(0.8..1.3),
            precision: rng.random_range(0.7..1.3),
            nervousness: rng.random_range(0.15..0.65),
            overshoot_tendency: rng.random_range(0.15..0.45),
            sub_movement_count: phases[rng.random_range(0..phases.len())],
            jerk_smoothness: rng.random_range(0.7..0.95),
            arc_direction: rng.random_range(-PI..PI),
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
            arc_direction: finite_or(self.arc_direction, defaults.arc_direction),
        }
    }
}

fn finite_or(value: f64, fallback: f64) -> f64 {
    if value.is_finite() { value } else { fallback }
}
