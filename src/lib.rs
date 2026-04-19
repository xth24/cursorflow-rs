pub mod config;
pub mod generator;
pub mod style;
pub mod trajectory;

pub use config::ScreenConfig;
pub use generator::{Point, generate, generate_single, merge};
pub use style::MovementStyle;
pub use trajectory::Trajectory;
