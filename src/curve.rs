//! Cubic Bézier curves with an arc length lookup table.

use rand::{Rng, RngExt};

/// A point or vector in screen space, as `[x, y]` in pixels.
///
/// Coordinates grow to the right and downwards, matching screen conventions.
/// The same type describes positions and displacements.
pub type Point = [f64; 2];

/// Smallest arc length table, used for curves only a few pixels long.
const MIN_TABLE: usize = 24;

/// Largest arc length table; longer curves gain nothing visible past this.
const MAX_TABLE: usize = 512;

/// Pixels of control polygon per arc length table entry.
const PIXELS_PER_ENTRY: f64 = 2.0;

/// Extra sideways bulge for vertical movements, where the forearm pivots.
const BIOMECHANICAL_BIAS: f64 = 0.15;

pub(crate) fn add(a: Point, b: Point) -> Point {
    [a[0] + b[0], a[1] + b[1]]
}

pub(crate) fn sub(a: Point, b: Point) -> Point {
    [a[0] - b[0], a[1] - b[1]]
}

pub(crate) fn scale(a: Point, factor: f64) -> Point {
    [a[0] * factor, a[1] * factor]
}

pub(crate) fn dot(a: Point, b: Point) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}

pub(crate) fn norm(a: Point) -> f64 {
    a[0].hypot(a[1])
}

pub(crate) fn perpendicular(a: Point) -> Point {
    [-a[1], a[0]]
}

/// Returns the unit vector from `from` to `to`, or the x axis when they coincide.
pub(crate) fn direction(from: Point, to: Point) -> Point {
    let delta = sub(to, from);
    let length = norm(delta);

    if length > 0.0 {
        scale(delta, 1.0 / length)
    } else {
        [1.0, 0.0]
    }
}

fn random_sign<R: Rng>(rng: &mut R) -> f64 {
    if rng.random_bool(0.5) { 1.0 } else { -1.0 }
}

/// Interpolates `fp` linearly over the sorted knots `xp`, clamping at both ends.
pub(crate) fn interp(x: f64, xp: &[f64], fp: &[f64]) -> f64 {
    match xp.len() {
        0 => 0.0,
        _ if x.is_nan() || x <= xp[0] => fp[0],
        n if x >= xp[n - 1] => fp[n - 1],
        _ => {
            let hi = xp.partition_point(|&knot| knot <= x);
            let lo = hi - 1;
            let span = xp[hi] - xp[lo];

            if span > 0.0 {
                fp[lo] + (x - xp[lo]) / span * (fp[hi] - fp[lo])
            } else {
                fp[lo]
            }
        }
    }
}

/// A cubic Bézier curve that can be walked by fraction of its arc length.
#[derive(Debug, Clone)]
pub(crate) struct Curve {
    control: [Point; 4],
    params: Vec<f64>,
    lengths: Vec<f64>,
}

impl Curve {
    pub(crate) fn new(control: [Point; 4]) -> Self {
        let [p0, p1, p2, p3] = control;
        let polygon = norm(sub(p1, p0)) + norm(sub(p2, p1)) + norm(sub(p3, p2));
        let entries = ((polygon / PIXELS_PER_ENTRY) as usize).clamp(MIN_TABLE, MAX_TABLE);
        let params: Vec<f64> = (0..entries)
            .map(|i| i as f64 / (entries - 1) as f64)
            .collect();

        let mut lengths = Vec::with_capacity(entries);
        let mut total = 0.0;
        let mut previous = p0;
        for &t in &params {
            let point = point_at(control, t);
            total += norm(sub(point, previous));
            lengths.push(total);
            previous = point;
        }

        Self {
            control,
            params,
            lengths,
        }
    }

    /// Builds a curve from `start` to `end` that bulges sideways by about
    /// `curvature` times the distance, towards a randomly chosen side.
    pub(crate) fn between<R: Rng>(start: Point, end: Point, curvature: f64, rng: &mut R) -> Self {
        let delta = sub(end, start);
        let distance = norm(delta);
        let side = perpendicular(direction(start, end));
        let angle = delta[1].atan2(delta[0]);
        let bias = 1.0 + angle.sin() * BIOMECHANICAL_BIAS * rng.random_range(0.5..1.5);
        let bulge = random_sign(rng) * bias * curvature * distance;

        let first = add(
            add(start, scale(delta, rng.random_range(0.2..0.4))),
            scale(side, bulge * rng.random_range(0.5..1.2)),
        );
        let second = add(
            add(start, scale(delta, rng.random_range(0.6..0.8))),
            scale(side, bulge * rng.random_range(0.3..0.8) * random_sign(rng)),
        );

        Self::new([start, first, second, end])
    }

    pub(crate) fn end(&self) -> Point {
        self.control[3]
    }

    pub(crate) fn length(&self) -> f64 {
        self.lengths.last().copied().unwrap_or(0.0)
    }

    /// Returns the point that lies `fraction` of the arc length along the curve.
    pub(crate) fn at_fraction(&self, fraction: f64) -> Point {
        let target = fraction.clamp(0.0, 1.0) * self.length();

        point_at(self.control, interp(target, &self.lengths, &self.params))
    }
}

fn point_at([p0, p1, p2, p3]: [Point; 4], t: f64) -> Point {
    let u = 1.0 - t;
    let w0 = u * u * u;
    let w1 = 3.0 * u * u * t;
    let w2 = 3.0 * u * t * t;
    let w3 = t * t * t;

    [
        w0 * p0[0] + w1 * p1[0] + w2 * p2[0] + w3 * p3[0],
        w0 * p0[1] + w1 * p1[1] + w2 * p2[1] + w3 * p3[1],
    ]
}

#[cfg(test)]
mod tests {
    use rand::{SeedableRng, rngs::SmallRng};

    use super::*;

    #[test]
    fn fraction_walks_from_start_to_end_without_going_back() {
        let mut rng = SmallRng::seed_from_u64(3);
        let start = [100.0, 100.0];
        let end = [900.0, 400.0];
        let curve = Curve::between(start, end, 0.15, &mut rng);

        assert_eq!(curve.at_fraction(0.0), start);
        assert_eq!(curve.at_fraction(1.0), end);
        assert!(curve.length() > norm(sub(end, start)));

        let mut travelled = 0.0;
        let mut previous = start;
        for step in 1..=200 {
            let point = curve.at_fraction(step as f64 / 200.0);
            let hop = norm(sub(point, previous));
            travelled += hop;
            previous = point;

            assert!((hop - curve.length() / 200.0).abs() < curve.length() * 0.01);
        }

        assert!((travelled - curve.length()).abs() < 1.0);
    }

    #[test]
    fn interp_clamps_outside_the_knots_and_interpolates_inside() {
        let xp = [0.0, 1.0, 1.0, 3.0];
        let fp = [0.0, 10.0, 20.0, 40.0];

        assert_eq!(interp(-1.0, &xp, &fp), 0.0);
        assert_eq!(interp(5.0, &xp, &fp), 40.0);
        assert_eq!(interp(f64::NAN, &xp, &fp), 0.0);
        assert_eq!(interp(0.5, &xp, &fp), 5.0);
        assert_eq!(interp(2.0, &xp, &fp), 30.0);
        assert_eq!(interp(1.0, &[], &[]), 0.0);
    }
}
