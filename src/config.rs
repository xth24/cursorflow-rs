//! Screen and sampling parameters.

/// Sample rate used by [`ScreenConfig::default`], in hertz.
const DEFAULT_SAMPLE_RATE: u32 = 120;

/// Timestamp jitter used by [`ScreenConfig::default`], as a fraction of the sample interval.
const DEFAULT_SAMPLE_JITTER: f64 = 0.05;

/// Largest accepted jitter; anything below one half keeps timestamps strictly increasing.
const MAX_SAMPLE_JITTER: f64 = 0.45;

/// The screen a trajectory is generated for and how it is sampled.
///
/// Positions are clamped to `0..width` and `0..height`. `sample_rate` is the
/// number of samples per second, and `sample_jitter` displaces each timestamp
/// by up to that fraction of the sample interval, so the stream resembles real
/// input events rather than a metronome. Values outside these ranges are clamped while
/// generating: the sample rate to `1..=`[`MAX_SAMPLE_RATE`](Self::MAX_SAMPLE_RATE)
/// and the jitter to `0.0..=0.45`.
///
/// # Examples
///
/// ```
/// use cursorflow::ScreenConfig;
///
/// let config = ScreenConfig {
///     sample_rate: 1000,
///     ..ScreenConfig::new(2560, 1440)
/// };
///
/// assert!(config.diagonal() > 2937.0);
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ScreenConfig {
    /// Screen width in pixels.
    pub width: u32,
    /// Screen height in pixels.
    pub height: u32,
    /// Samples generated per second.
    pub sample_rate: u32,
    /// Timestamp jitter as a fraction of the sample interval, from `0.0` to `0.45`.
    pub sample_jitter: f64,
}

impl ScreenConfig {
    /// Highest supported sample rate in hertz; larger values are clamped to it.
    pub const MAX_SAMPLE_RATE: u32 = 10_000;

    /// Creates a config for a screen of the given size with default sampling.
    ///
    /// # Examples
    ///
    /// ```
    /// use cursorflow::ScreenConfig;
    ///
    /// let config = ScreenConfig::new(1280, 720);
    ///
    /// assert_eq!(config.sample_rate, ScreenConfig::default().sample_rate);
    /// ```
    pub const fn new(width: u32, height: u32) -> Self {
        Self {
            width,
            height,
            sample_rate: DEFAULT_SAMPLE_RATE,
            sample_jitter: DEFAULT_SAMPLE_JITTER,
        }
    }

    /// Returns the screen diagonal in pixels.
    ///
    /// Movement durations scale with distance relative to this diagonal, so
    /// the same waypoints take longer on a small screen than on a large one.
    ///
    /// # Examples
    ///
    /// ```
    /// use cursorflow::ScreenConfig;
    ///
    /// let diagonal = ScreenConfig::new(300, 400).diagonal();
    ///
    /// assert_eq!(diagonal, 500.0);
    /// ```
    pub fn diagonal(&self) -> f64 {
        f64::from(self.width).hypot(f64::from(self.height))
    }

    pub(crate) fn sanitized(self) -> Self {
        let jitter = if self.sample_jitter.is_finite() {
            self.sample_jitter
        } else {
            DEFAULT_SAMPLE_JITTER
        };

        Self {
            sample_rate: self.sample_rate.clamp(1, Self::MAX_SAMPLE_RATE),
            sample_jitter: jitter.clamp(0.0, MAX_SAMPLE_JITTER),
            ..self
        }
    }
}

impl Default for ScreenConfig {
    fn default() -> Self {
        Self::new(1920, 1080)
    }
}
