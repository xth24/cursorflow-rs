//! Movement targets and the area a cursor may come to rest in.

use crate::config::ScreenConfig;
use crate::curve::Point;

/// Distance, in pixels, beyond the outermost pixel centers that a resting
/// point may lie and still round onto a pixel of the target.
const ROUNDING_MARGIN: f64 = 0.49;

/// What a movement aims at: a rectangle on screen, such as a button.
///
/// People aim at the `center` and stop once the cursor is anywhere inside,
/// so the size decides how long a movement takes and how often it needs a
/// correction: a small target costs more time and more corrections than a
/// large one at the same distance, as Fitts's law describes. A zero size
/// asks for the one pixel the center rounds to.
///
/// Only the part of the target on screen counts. Negative sizes and `NaN`
/// count as zero.
///
/// # Examples
///
/// Move onto a 120 by 32 pixel button and read where the cursor stopped:
///
/// ```
/// use cursorflow::{MovementStyle, ScreenConfig, Target, generate_single};
///
/// let button = Target::new([640.0, 360.0], [120.0, 32.0]);
/// let config = ScreenConfig::default();
/// let mut rng = rand::rng();
///
/// let path = generate_single([100.0, 100.0], button, &config, &MovementStyle::default(), &mut rng);
/// let rest = path.iter().next_back().map(|sample| (sample.x, sample.y));
///
/// assert!(rest.is_some_and(|(x, y)| (580..=700).contains(&x) && (344..=376).contains(&y)));
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Target {
    /// Point the movement aims at, in pixels.
    pub center: Point,
    /// Width and height in pixels.
    pub size: [f64; 2],
}

impl Target {
    /// Creates a target of the given width and height around `center`.
    ///
    /// # Examples
    ///
    /// ```
    /// use cursorflow::Target;
    ///
    /// let icon = Target::new([24.0, 24.0], [32.0, 32.0]);
    ///
    /// assert_eq!(icon.size, [32.0, 32.0]);
    /// ```
    pub const fn new(center: Point, size: [f64; 2]) -> Self {
        Self { center, size }
    }

    /// Creates a target that only accepts the pixel `center` rounds to.
    ///
    /// Landing on one exact pixel is the hardest aim there is, so movements to
    /// a point take longer and correct more often than movements to a button.
    ///
    /// # Examples
    ///
    /// ```
    /// use cursorflow::{MovementStyle, ScreenConfig, Target, generate_single};
    ///
    /// let mut rng = rand::rng();
    /// let path = generate_single(
    ///     [100.0, 100.0],
    ///     Target::point([800.0, 500.0]),
    ///     &ScreenConfig::default(),
    ///     &MovementStyle::default(),
    ///     &mut rng,
    /// );
    ///
    /// assert_eq!(path.iter().next_back().map(|sample| (sample.x, sample.y)), Some((800, 500)));
    /// ```
    pub const fn point(center: Point) -> Self {
        Self::new(center, [0.0, 0.0])
    }
}

/// The pixels a movement may come to rest on: those of the target that lie on screen.
///
/// Any point inside the area rounds to one of these pixels.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Area {
    first: Point,
    last: Point,
    aim: Point,
}

impl Area {
    /// Returns the part of `target` that lies on the screen of `config`.
    ///
    /// When no pixel of the target is on screen, the area is the screen pixel
    /// nearest its center.
    pub(crate) fn new(target: &Target, config: &ScreenConfig) -> Self {
        let limits = config.limits();
        let spans: [(f64, f64); 2] = std::array::from_fn(|axis| {
            pixel_span(target.center[axis], target.size[axis], limits[axis])
        });
        let first = spans.map(|(first, _)| first);
        let last = spans.map(|(_, last)| last);
        let aim = std::array::from_fn(|axis| {
            let center = target.center[axis];
            let (low, high) = (first[axis], last[axis]);

            if (low - ROUNDING_MARGIN..=high + ROUNDING_MARGIN).contains(&center) {
                center
            } else {
                (low + high) / 2.0
            }
        });

        Self { first, last, aim }
    }

    /// Returns the point people aim at: the target center, or the middle of
    /// the visible part when the center is off screen.
    pub(crate) const fn aim(&self) -> Point {
        self.aim
    }

    pub(crate) fn contains(&self, point: Point) -> bool {
        (0..2).all(|axis| {
            (self.first[axis] - ROUNDING_MARGIN..=self.last[axis] + ROUNDING_MARGIN)
                .contains(&point[axis])
        })
    }

    /// Returns the point of the area nearest `point`.
    pub(crate) fn clamp(&self, point: Point) -> Point {
        std::array::from_fn(|axis| {
            point[axis].clamp(
                self.first[axis] - ROUNDING_MARGIN,
                self.last[axis] + ROUNDING_MARGIN,
            )
        })
    }

    /// Returns the width of the area seen along the unit vector `heading`, in pixels.
    ///
    /// This is the length of the chord through the middle of the area along
    /// the approach, the effective width of a two dimensional target, and it
    /// is at least one pixel.
    pub(crate) fn width_along(&self, heading: Point) -> f64 {
        let reach: [f64; 2] = std::array::from_fn(|axis| {
            (self.last[axis] - self.first[axis] + 1.0) / 2.0 / heading[axis].abs()
        });

        2.0 * reach[0].min(reach[1])
    }
}

/// Returns the first and last pixel, along one axis, inside a target that
/// spans `size` pixels around `center`, limited to `0..=limit`.
fn pixel_span(center: f64, size: f64, limit: f64) -> (f64, f64) {
    let half = size.max(0.0) / 2.0;
    let (first, last) = ((center - half).ceil(), (center + half).floor());
    let (first, last) = if first <= last {
        (first, last)
    } else {
        (center.round(), center.round())
    };
    let (first, last) = (first.max(0.0), last.min(limit));

    if first <= last {
        (first, last)
    } else {
        let nearest = center.round().clamp(0.0, limit);

        (nearest, nearest)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn config() -> ScreenConfig {
        ScreenConfig::new(800, 600)
    }

    #[test]
    fn area_covers_the_pixels_inside_the_target() {
        let area = Area::new(&Target::new([119.5, 50.0], [40.0, 10.0]), &config());

        assert!(area.contains([100.0, 45.0]));
        assert!(area.contains([139.0, 55.0]));
        assert!(!area.contains([99.4, 50.0]));
        assert!(!area.contains([139.6, 50.0]));
        assert_eq!(area.aim(), [119.5, 50.0]);
        assert_eq!(area.clamp([0.0, 0.0]).map(f64::round), [100.0, 45.0]);
    }

    #[test]
    fn point_target_accepts_only_its_rounded_pixel() {
        let area = Area::new(&Target::point([5.5, 7.2]), &config());

        assert_eq!(area.aim(), [6.0, 7.2]);
        assert!(area.contains([6.3, 6.8]));
        assert!(!area.contains([5.5, 7.0]));
        assert_eq!(area.width_along([1.0, 0.0]), 1.0);
    }

    #[test]
    fn area_is_clipped_to_the_screen() {
        let partly = Area::new(&Target::new([-10.0, 300.0], [40.0, 40.0]), &config());

        assert!(partly.contains([0.0, 300.0]));
        assert!(partly.contains([10.0, 300.0]));
        assert!(!partly.contains([11.0, 300.0]));
        assert_eq!(partly.aim(), [5.0, 300.0]);

        let off = Area::new(&Target::point([1000.0, -20.0]), &config());

        assert_eq!(off.aim(), [799.0, 0.0]);
        assert!(off.contains([799.0, 0.0]));

        let everywhere = Area::new(
            &Target::new([0.0, 0.0], [f64::INFINITY, f64::NAN]),
            &config(),
        );

        assert!(everywhere.contains([799.0, 0.0]));
        assert!(!everywhere.contains([0.0, 1.0]));
    }

    #[test]
    fn width_along_follows_the_approach_direction() {
        let area = Area::new(&Target::new([399.5, 299.5], [40.0, 20.0]), &config());
        let diagonal = std::f64::consts::FRAC_1_SQRT_2;

        assert_eq!(area.width_along([1.0, 0.0]), 40.0);
        assert_eq!(area.width_along([0.0, -1.0]), 20.0);
        assert!((area.width_along([diagonal, diagonal]) - 20.0 * 2.0_f64.sqrt()).abs() < 1e-9);
    }
}
