use cursorflow::rand::{SeedableRng, rngs::SmallRng};
use cursorflow::{MovementStyle, ScreenConfig, Trajectory, generate, generate_single, merge};

fn rng() -> SmallRng {
    SmallRng::seed_from_u64(42)
}

fn strictly_increasing(values: &[f64]) -> bool {
    values.windows(2).all(|pair| pair[1] > pair[0])
}

#[test]
fn deduplicate_recomputes_velocity_from_remaining_samples() {
    let trajectory = Trajectory {
        x: vec![0, 0, 10],
        y: vec![0, 0, 0],
        t: vec![0.0, 10.0, 110.0],
        v: vec![],
    };

    let deduped = trajectory.deduplicate();

    assert_eq!(deduped.x, vec![0, 10]);
    assert_eq!(deduped.y, vec![0, 0]);
    assert_eq!(deduped.t, vec![0.0, 110.0]);
    assert_eq!(deduped.len(), 2);
    assert_eq!(deduped.v[0], 0.0);
    assert!((deduped.v[1] - 90.909_090_909).abs() < 1e-6);
}

#[test]
fn len_ignores_the_speed_column_and_new_truncates_to_the_shortest() {
    let hand_made = Trajectory {
        x: vec![0, 1, 2],
        y: vec![0, 1, 2],
        t: vec![0.0, 1.0, 2.0],
        v: vec![],
    };

    assert_eq!(hand_made.len(), 3);
    assert!(!hand_made.is_empty());
    assert_eq!(hand_made.get(1).map(|sample| sample.v), Some(0.0));

    let built = Trajectory::new(vec![0, 1, 2], vec![0, 1], vec![0.0, 1.0, 2.0]);

    assert_eq!(built.len(), 2);
    assert_eq!(built.x, vec![0, 1]);
    assert_eq!(built.t, vec![0.0, 1.0]);
    assert_eq!(built.v.len(), 2);
    assert_eq!(built.get(2), None);
}

#[test]
fn iter_yields_every_sample_in_order_and_in_reverse() {
    let trajectory = generate_single(
        [10.0, 10.0],
        [400.0, 300.0],
        &ScreenConfig::default(),
        &MovementStyle::default(),
        &mut rng(),
    );

    let forward: Vec<_> = trajectory.iter().collect();
    let backward: Vec<_> = trajectory.iter().rev().collect();

    assert_eq!(trajectory.iter().len(), trajectory.len());
    assert_eq!(forward.len(), trajectory.len());
    assert_eq!(forward.iter().rev().copied().collect::<Vec<_>>(), backward);
    assert!(forward.iter().enumerate().all(|(i, sample)| {
        sample.x == trajectory.x[i]
            && sample.y == trajectory.y[i]
            && sample.t == trajectory.t[i]
            && sample.v == trajectory.v[i]
    }));
    assert_eq!(
        (&trajectory).into_iter().collect::<Trajectory>(),
        trajectory
    );
}

#[test]
fn generate_single_sanitizes_invalid_style_and_config_values() {
    let config = ScreenConfig {
        sample_rate: 0,
        sample_jitter: f64::NAN,
        ..ScreenConfig::default()
    };
    let style = MovementStyle {
        speed_factor: 0.0,
        precision: 0.0,
        nervousness: f64::INFINITY,
        overshoot_tendency: f64::NAN,
        sub_movement_count: 99,
        jerk_smoothness: f64::NAN,
    };

    let trajectory = generate_single([100.0, 100.0], [900.0, 700.0], &config, &style, &mut rng());

    assert!(!trajectory.is_empty());
    assert!(strictly_increasing(&trajectory.t));
    assert!(trajectory.t.iter().all(|value| value.is_finite()));
    assert!(trajectory.v.iter().all(|value| value.is_finite()));
}

#[test]
fn generate_single_keeps_high_sample_rate_timestamps_strictly_increasing() {
    let config = ScreenConfig {
        sample_rate: 5_000,
        ..ScreenConfig::default()
    };

    let trajectory = generate_single(
        [10.0, 10.0],
        [500.0, 400.0],
        &config,
        &MovementStyle::default(),
        &mut rng(),
    );

    assert!(trajectory.len() > 1_000);
    assert!(strictly_increasing(&trajectory.t));
    assert!(trajectory.v.iter().all(|value| value.is_finite()));
}

#[test]
fn maximum_jitter_at_maximum_sample_rate_stays_strictly_increasing() {
    let config = ScreenConfig {
        sample_rate: u32::MAX,
        sample_jitter: 5.0,
        ..ScreenConfig::default()
    };
    let style = MovementStyle {
        sub_movement_count: 4,
        ..MovementStyle::default()
    };

    let trajectory = generate_single([0.0, 0.0], [1500.0, 900.0], &config, &style, &mut rng());
    let interval = 1000.0 / f64::from(ScreenConfig::MAX_SAMPLE_RATE);
    let expected = (trajectory.duration() / interval).round() as usize + 1;

    assert!(strictly_increasing(&trajectory.t));
    assert!(
        trajectory.len().abs_diff(expected) <= 2,
        "{} samples, expected about {expected}",
        trajectory.len()
    );
}

#[test]
fn generate_single_pins_endpoints_and_stays_on_screen() {
    let config = ScreenConfig::new(800, 600);
    let mut rng = rng();

    for (start, end) in [
        ([0.0, 0.0], [799.0, 599.0]),
        ([790.0, 10.0], [5.0, 590.0]),
        ([400.0, 300.0], [400.0, 310.0]),
        ([-50.0, 900.0], [1000.0, -20.0]),
    ] {
        let trajectory = generate_single(
            start,
            end,
            &config,
            &MovementStyle::random(&mut rng),
            &mut rng,
        );
        let first = trajectory.get(0).unwrap();
        let last = trajectory.iter().next_back().unwrap();

        assert_eq!(
            (first.x, first.y),
            (
                start[0].clamp(0.0, 799.0) as i32,
                start[1].clamp(0.0, 599.0) as i32
            )
        );
        assert_eq!(
            (last.x, last.y),
            (
                end[0].clamp(0.0, 799.0) as i32,
                end[1].clamp(0.0, 599.0) as i32
            )
        );
        assert_eq!(first.t, 0.0);
        assert!(strictly_increasing(&trajectory.t));
        assert!(
            trajectory
                .iter()
                .all(|s| (0..800).contains(&s.x) && (0..600).contains(&s.y))
        );
    }
}

#[test]
fn corrections_produce_a_rest_between_movement_phases() {
    let style = MovementStyle {
        sub_movement_count: 3,
        precision: 0.7,
        ..MovementStyle::default()
    };
    let config = ScreenConfig {
        sample_jitter: 0.0,
        ..ScreenConfig::default()
    };

    let trajectory = generate_single([100.0, 100.0], [1500.0, 800.0], &config, &style, &mut rng());
    let peak = trajectory.v.iter().copied().fold(0.0, f64::max);
    let interior = &trajectory.v[trajectory.len() / 4..trajectory.len() - 2];
    let slowest = interior.iter().copied().fold(f64::INFINITY, f64::min);

    assert!(
        slowest < peak * 0.05,
        "slowest interior speed {slowest} vs peak {peak}"
    );
    assert!(trajectory.duration() > 700.0);
}

#[test]
fn same_seed_reproduces_the_same_trajectory() {
    let config = ScreenConfig::default();
    let style = MovementStyle::default();
    let waypoints = [[100.0, 100.0], [900.0, 700.0], [1750.0, 200.0]];

    let first = merge(
        &generate(&waypoints, &config, &style, &mut rng()),
        70.0..200.0,
        &mut rng(),
    );
    let second = merge(
        &generate(&waypoints, &config, &style, &mut rng()),
        70.0..200.0,
        &mut rng(),
    );

    assert_eq!(first, second);
}

#[test]
fn tiny_and_degenerate_inputs_do_not_panic() {
    let config = ScreenConfig::default();
    let style = MovementStyle::default();
    let mut rng = rng();

    let still = generate_single([5.2, 5.2], [5.6, 5.4], &config, &style, &mut rng);
    assert_eq!(still.len(), 2);
    assert_eq!(still.t, vec![0.0, 10.0]);

    let invalid = generate_single([f64::NAN, 0.0], [10.0, 10.0], &config, &style, &mut rng);
    assert!(invalid.is_empty());

    assert!(generate(&[], &config, &style, &mut rng).is_empty());
    assert!(generate(&[[1.0, 1.0]], &config, &style, &mut rng).is_empty());
    assert!(merge(&[], 0.0..10.0, &mut rng).is_empty());
    assert!(merge(&[Trajectory::empty()], 0.0..10.0, &mut rng).is_empty());
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

    let merged = merge(&[first, second], 200.0..70.0, &mut rng());

    assert_eq!(merged.len(), 4);
    assert!(merged.t.windows(2).all(|pair| pair[1] >= pair[0]));
    assert!(merged.t[2] >= merged.t[1] + 70.0);
    assert!(merged.t[2] < merged.t[1] + 200.0);
    assert_eq!(merged.v[2], 0.0);
    assert!(merged.v[3] > 0.0);
}
