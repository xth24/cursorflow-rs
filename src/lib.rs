//! Human like cursor movement trajectories.
//!
//! `cursorflow` generates timestamped cursor paths between points that move
//! the way a hand on a mouse does. Each movement is a fast, slightly curved
//! primary motion that lands near the target, a short pause, and one or more
//! small corrections that home in on it. Speed follows an asymmetric bell
//! curve, the path carries a little tremor, and timestamps jitter like real
//! input events. Everything is drawn from the [`Rng`] you pass in, so a
//! seeded generator reproduces the same path.
//!
//! The entry points are [`generate_single`] for one movement, [`generate`]
//! for a chain of waypoints, and [`merge`] to join the chain on one time
//! axis. [`ScreenConfig`] describes the screen and sampling, and
//! [`MovementStyle`] describes the person. The result is a [`Trajectory`].
//!
//! # Examples
//!
//! Move through a few waypoints and print the first samples:
//!
//! ```
//! use cursorflow::{MovementStyle, ScreenConfig, generate, merge};
//!
//! let config = ScreenConfig::default();
//! let mut rng = rand::rng();
//! let style = MovementStyle::random(&mut rng);
//! let waypoints = [[100.0, 100.0], [900.0, 700.0], [1750.0, 200.0]];
//!
//! let segments = generate(&waypoints, &config, &style, &mut rng);
//! let path = merge(&segments, 70.0..200.0, &mut rng).deduplicate();
//!
//! for sample in path.iter().take(5) {
//!     println!("x={} y={} t={:.1}ms v={:.0}px/s", sample.x, sample.y, sample.t, sample.v);
//! }
//! ```
//!
//! Reproduce a movement with a seeded generator:
//!
//! ```
//! use cursorflow::rand::{SeedableRng, rngs::SmallRng};
//! use cursorflow::{MovementStyle, ScreenConfig, generate_single};
//!
//! let config = ScreenConfig::default();
//! let style = MovementStyle::default();
//! let mut first = SmallRng::seed_from_u64(7);
//! let mut second = SmallRng::seed_from_u64(7);
//!
//! let a = generate_single([0.0, 0.0], [500.0, 300.0], &config, &style, &mut first);
//! let b = generate_single([0.0, 0.0], [500.0, 300.0], &config, &style, &mut second);
//!
//! assert_eq!(a, b);
//! ```
//!
//! [`Rng`]: rand::Rng

#![warn(missing_docs)]
#![deny(rustdoc::broken_intra_doc_links)]

mod config;
mod curve;
mod generator;
mod motion;
mod noise;
mod style;
mod trajectory;

/// The random number generator crate this library builds on, reexported so
/// callers can match its version.
pub use rand;

#[doc(inline)]
pub use config::ScreenConfig;
#[doc(inline)]
pub use curve::Point;
#[doc(inline)]
pub use generator::{generate, generate_single, merge};
#[doc(inline)]
pub use style::MovementStyle;
#[doc(inline)]
pub use trajectory::{Iter, Sample, Trajectory};
