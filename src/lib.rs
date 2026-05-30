pub mod config;
pub mod generator;
pub mod style;
pub mod trajectory;

pub use config::ScreenConfig;
pub use generator::{Point, generate, generate_single, merge};
pub use style::MovementStyle;
pub use trajectory::Trajectory;

#[cfg(test)]
mod tests {
    use rand::{SeedableRng, rngs::SmallRng};

    use super::*;

    fn rng() -> SmallRng {
        SmallRng::seed_from_u64(42)
    }

    #[test]
    fn deduplicate_recomputes_velocity_from_remaining_samples() {
        let traj = Trajectory {
            x: vec![0, 0, 10],
            y: vec![0, 0, 0],
            t: vec![0.0, 10.0, 110.0],
            v: vec![],
        };

        let deduped = traj.deduplicate();

        assert_eq!(deduped.x, vec![0, 10]);
        assert_eq!(deduped.y, vec![0, 0]);
        assert_eq!(deduped.t, vec![0.0, 110.0]);
        assert_eq!(deduped.len(), 2);
        assert_eq!(deduped.v[0], 0.0);
        assert!((deduped.v[1] - 90.909_090_909).abs() < 1e-6);
    }

    #[test]
    fn generate_single_sanitizes_invalid_style_values() {
        let config = ScreenConfig::default();
        let style = MovementStyle {
            speed_factor: 0.0,
            precision: 0.0,
            nervousness: f64::INFINITY,
            overshoot_tendency: f64::NAN,
            sub_movement_count: 99,
            jerk_smoothness: f64::NAN,
        };
        let mut rng = rng();

        let traj = generate_single([100.0, 100.0], [900.0, 700.0], &config, &style, &mut rng);

        assert!(!traj.is_empty());
        assert!(traj.t.windows(2).all(|w| w[1] > w[0]));
        assert!(traj.t.iter().all(|v| v.is_finite()));
        assert!(traj.v.iter().all(|v| v.is_finite()));
    }

    #[test]
    fn generate_single_keeps_high_sample_rate_timestamps_strictly_increasing() {
        let config = ScreenConfig {
            sample_rate: 5_000,
            ..ScreenConfig::default()
        };
        let style = MovementStyle::default();
        let mut rng = rng();

        let traj = generate_single([10.0, 10.0], [500.0, 400.0], &config, &style, &mut rng);

        assert!(traj.len() > 1_000);
        assert!(traj.t.windows(2).all(|w| w[1] > w[0]));
        assert!(traj.v.iter().all(|v| v.is_finite()));
    }

    #[test]
    fn merge_normalizes_reversed_delay_ranges_and_recomputes_velocity() {
        let first = Trajectory {
            x: vec![0, 10],
            y: vec![0, 0],
            t: vec![0.0, 10.0],
            v: vec![0.0, 1.0],
        };
        let second = Trajectory {
            x: vec![10, 20],
            y: vec![0, 0],
            t: vec![0.0, 10.0],
            v: vec![0.0, 1.0],
        };
        let mut rng = rng();

        let merged = merge(&[first, second], 200.0..70.0, &mut rng);

        assert_eq!(merged.len(), 4);
        assert!(merged.t.windows(2).all(|w| w[1] >= w[0]));
        assert!(merged.t[2] >= merged.t[1] + 70.0);
        assert!(merged.t[2] < merged.t[1] + 200.0);
        assert_eq!(merged.v[2], 0.0);
        assert!(merged.v[3] > 0.0);
    }
}
