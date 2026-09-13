//! Timestamped cursor paths.

use std::iter::FusedIterator;
use std::ops::Range;

/// A cursor path as parallel columns of positions, timestamps, and speeds.
///
/// `x` and `y` hold screen coordinates in whole pixels, `t` holds timestamps in
/// milliseconds from the start of the path, and `v` holds the speed in pixels
/// per second at which each sample was reached, with the first sample at zero.
/// The generators always return columns of equal length with `v` derived from
/// the other three. The fields are public for direct plotting and replay. When
/// building a trajectory by hand, [`Trajectory::new`] derives `v`, and
/// [`len`](Self::len) never reads past the shortest position column.
///
/// # Examples
///
/// ```
/// use cursorflow::Trajectory;
///
/// let path = Trajectory::new(vec![0, 3, 6], vec![0, 4, 8], vec![0.0, 10.0, 20.0]);
///
/// assert_eq!(path.len(), 3);
/// assert_eq!(path.v, vec![0.0, 500.0, 500.0]);
/// ```
#[derive(Debug, Default, Clone, PartialEq)]
pub struct Trajectory {
    /// Horizontal positions in pixels.
    pub x: Vec<i32>,
    /// Vertical positions in pixels.
    pub y: Vec<i32>,
    /// Timestamps in milliseconds from the start of the path.
    pub t: Vec<f64>,
    /// Speed in pixels per second at which each sample was reached.
    pub v: Vec<f64>,
}

/// One sample of a [`Trajectory`], as yielded by [`Trajectory::iter`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sample {
    /// Horizontal position in pixels.
    pub x: i32,
    /// Vertical position in pixels.
    pub y: i32,
    /// Timestamp in milliseconds from the start of the path.
    pub t: f64,
    /// Speed in pixels per second at which the sample was reached.
    pub v: f64,
}

impl Trajectory {
    /// Creates a trajectory from position and timestamp columns, deriving speeds.
    ///
    /// The columns are truncated to the shortest of the three.
    ///
    /// # Examples
    ///
    /// ```
    /// use cursorflow::Trajectory;
    ///
    /// let path = Trajectory::new(vec![0, 10, 20], vec![0, 0, 0], vec![0.0, 100.0]);
    ///
    /// assert_eq!(path.len(), 2);
    /// assert_eq!(path.v, vec![0.0, 100.0]);
    /// ```
    pub fn new(x: Vec<i32>, y: Vec<i32>, t: Vec<f64>) -> Self {
        let mut trajectory = Self {
            x,
            y,
            t,
            v: Vec::new(),
        };
        trajectory.refresh_velocities();

        trajectory
    }

    /// Creates a trajectory with no samples.
    pub const fn empty() -> Self {
        Self {
            x: Vec::new(),
            y: Vec::new(),
            t: Vec::new(),
            v: Vec::new(),
        }
    }

    /// Returns the number of samples, which is the length of the shortest of
    /// the `x`, `y`, and `t` columns.
    pub fn len(&self) -> usize {
        self.x.len().min(self.y.len()).min(self.t.len())
    }

    /// Returns `true` when the trajectory has no samples.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Returns the timestamp of the last sample in milliseconds, or zero when empty.
    ///
    /// # Examples
    ///
    /// ```
    /// use cursorflow::Trajectory;
    ///
    /// let path = Trajectory::new(vec![0, 5], vec![0, 5], vec![0.0, 40.0]);
    ///
    /// assert_eq!(path.duration(), 40.0);
    /// assert_eq!(Trajectory::empty().duration(), 0.0);
    /// ```
    pub fn duration(&self) -> f64 {
        self.t[..self.len()].last().copied().unwrap_or(0.0)
    }

    /// Returns the sample at `index`, or [`None`] past the end.
    ///
    /// A missing speed column entry reads as zero.
    pub fn get(&self, index: usize) -> Option<Sample> {
        (index < self.len()).then(|| Sample {
            x: self.x[index],
            y: self.y[index],
            t: self.t[index],
            v: self.v.get(index).copied().unwrap_or(0.0),
        })
    }

    /// Returns an iterator over the samples in order.
    ///
    /// # Examples
    ///
    /// ```
    /// use cursorflow::Trajectory;
    ///
    /// let path = Trajectory::new(vec![0, 3], vec![0, 4], vec![0.0, 10.0]);
    /// let peak = path.iter().map(|sample| sample.v).fold(0.0, f64::max);
    ///
    /// assert_eq!(peak, 500.0);
    /// ```
    pub fn iter(&self) -> Iter<'_> {
        Iter {
            trajectory: self,
            range: 0..self.len(),
        }
    }

    /// Returns a copy without consecutive samples that repeat a position.
    ///
    /// The first sample of each run is kept and speeds are derived again, so
    /// the result matches the events a mouse would emit: none while at rest.
    ///
    /// # Examples
    ///
    /// ```
    /// use cursorflow::Trajectory;
    ///
    /// let path = Trajectory::new(vec![0, 0, 10], vec![0, 0, 0], vec![0.0, 10.0, 110.0]);
    /// let events = path.deduplicate();
    ///
    /// assert_eq!(events.x, vec![0, 10]);
    /// assert_eq!(events.t, vec![0.0, 110.0]);
    /// ```
    pub fn deduplicate(&self) -> Self {
        let mut samples: Vec<Sample> = self.iter().collect();
        samples.dedup_by(|next, previous| (next.x, next.y) == (previous.x, previous.y));

        samples.into_iter().collect()
    }

    /// Returns a copy with the speed column derived from positions and timestamps.
    ///
    /// Use it after editing the public columns by hand.
    pub fn recompute_velocities(&self) -> Self {
        let n = self.len();

        Self::new(
            self.x[..n].to_vec(),
            self.y[..n].to_vec(),
            self.t[..n].to_vec(),
        )
    }

    pub(crate) fn refresh_velocities(&mut self) {
        let n = self.len();
        self.x.truncate(n);
        self.y.truncate(n);
        self.t.truncate(n);
        self.v = speeds(&self.x, &self.y, &self.t);
    }
}

impl FromIterator<Sample> for Trajectory {
    /// Collects samples into a trajectory, deriving speeds from positions and
    /// timestamps rather than trusting the samples' own.
    fn from_iter<I: IntoIterator<Item = Sample>>(samples: I) -> Self {
        let mut x = Vec::new();
        let mut y = Vec::new();
        let mut t = Vec::new();
        for sample in samples {
            x.push(sample.x);
            y.push(sample.y);
            t.push(sample.t);
        }

        Self::new(x, y, t)
    }
}

impl<'a> IntoIterator for &'a Trajectory {
    type Item = Sample;
    type IntoIter = Iter<'a>;

    fn into_iter(self) -> Iter<'a> {
        self.iter()
    }
}

/// An iterator over the samples of a [`Trajectory`], created by [`Trajectory::iter`].
#[derive(Debug, Clone)]
pub struct Iter<'a> {
    trajectory: &'a Trajectory,
    range: Range<usize>,
}

impl Iterator for Iter<'_> {
    type Item = Sample;

    fn next(&mut self) -> Option<Sample> {
        self.range
            .next()
            .and_then(|index| self.trajectory.get(index))
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        self.range.size_hint()
    }
}

impl DoubleEndedIterator for Iter<'_> {
    fn next_back(&mut self) -> Option<Sample> {
        self.range
            .next_back()
            .and_then(|index| self.trajectory.get(index))
    }
}

impl ExactSizeIterator for Iter<'_> {}

impl FusedIterator for Iter<'_> {}

fn speeds(x: &[i32], y: &[i32], t: &[f64]) -> Vec<f64> {
    let n = x.len().min(y.len()).min(t.len());

    (0..n)
        .map(|i| {
            if i == 0 {
                return 0.0;
            }

            let dt = (t[i] - t[i - 1]) / 1000.0;
            if dt.is_finite() && dt > 0.0 {
                let dx = f64::from(x[i] - x[i - 1]);
                let dy = f64::from(y[i] - y[i - 1]);
                dx.hypot(dy) / dt
            } else {
                0.0
            }
        })
        .collect()
}
