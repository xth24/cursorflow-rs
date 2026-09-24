use cursorflow::rand::{RngExt, SeedableRng, rngs::SmallRng};
use cursorflow::{
    MovementStyle, ScreenConfig, Target, Trajectory, generate, generate_single, merge,
};

fn rng() -> SmallRng {
    SmallRng::seed_from_u64(42)
}

fn strictly_increasing(values: &[f64]) -> bool {
    values.windows(2).all(|pair| pair[1] > pair[0])
}

fn rest(trajectory: &Trajectory) -> (i32, i32) {
    let last = trajectory.iter().next_back().expect("trajectory is empty");

    (last.x, last.y)
}

/// Counts interior runs of one unchanged position lasting at least `min_ms`.
fn interior_rests(trajectory: &Trajectory, min_ms: f64) -> usize {
    let positions: Vec<(i32, i32)> = trajectory.iter().map(|s| (s.x, s.y)).collect();
    let mut rests = 0;
    let mut begin = 0;

    for index in 1..=positions.len() {
        if index < positions.len() && positions[index] == positions[begin] {
            continue;
        }

        let interior = begin > 0 && index < positions.len();
        if interior && trajectory.t[index - 1] - trajectory.t[begin] >= min_ms {
            rests += 1;
        }

        begin = index;
    }

    rests
}

/// Counts steps of one sample away from a position and straight back to it.
fn out_and_back(trajectory: &Trajectory) -> usize {
    let positions: Vec<(i32, i32)> = trajectory.iter().map(|s| (s.x, s.y)).collect();

    positions
        .windows(3)
        .filter(|window| window[0] == window[2] && window[0] != window[1])
        .count()
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
        Target::new([400.0, 300.0], [48.0, 24.0]),
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
        arc_direction: f64::NEG_INFINITY,
    };
    let target = Target::new([900.0, 700.0], [f64::NAN, -5.0]);

    let trajectory = generate_single([100.0, 100.0], target, &config, &style, &mut rng());

    assert!(!trajectory.is_empty());
    assert_eq!(rest(&trajectory), (900, 700));
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
        Target::new([500.0, 400.0], [32.0, 32.0]),
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

    let trajectory = generate_single(
        [0.0, 0.0],
        Target::point([1500.0, 900.0]),
        &config,
        &style,
        &mut rng(),
    );
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
fn point_targets_pin_both_ends_and_paths_stay_on_screen() {
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
            Target::point(end),
            &config,
            &MovementStyle::random(&mut rng),
            &mut rng,
        );
        let first = trajectory.get(0).expect("trajectory is empty");
        let clamp = |point: [f64; 2]| {
            (
                point[0].clamp(0.0, 799.0) as i32,
                point[1].clamp(0.0, 599.0) as i32,
            )
        };

        assert_eq!((first.x, first.y), clamp(start));
        assert_eq!(rest(&trajectory), clamp(end));
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
fn movements_rest_inside_their_targets() {
    let config = ScreenConfig::default();
    let mut rng = rng();

    for _ in 0..300 {
        let style = MovementStyle::random(&mut rng);
        let start = [rng.random_range(0.0..1920.0), rng.random_range(0.0..1080.0)];
        let center = [rng.random_range(0.0..1920.0), rng.random_range(0.0..1080.0)];
        let size = [rng.random_range(0.0..160.0), rng.random_range(0.0..80.0)];

        let trajectory =
            generate_single(start, Target::new(center, size), &config, &style, &mut rng);
        let (x, y) = rest(&trajectory);
        let reach = |axis: usize| (size[axis] / 2.0).max(0.5) + 1e-9;

        assert!(
            (f64::from(x) - center[0]).abs() <= reach(0)
                && (f64::from(y) - center[1]).abs() <= reach(1),
            "rested at ({x}, {y}) for a {size:?} target at {center:?}"
        );
    }
}

#[test]
fn starting_on_the_target_holds_still() {
    let config = ScreenConfig::default();
    let style = MovementStyle::default();

    let button = generate_single(
        [410.0, 305.0],
        Target::new([400.0, 300.0], [80.0, 30.0]),
        &config,
        &style,
        &mut rng(),
    );
    let pixel = generate_single(
        [5.2, 5.2],
        Target::point([5.4, 5.4]),
        &config,
        &style,
        &mut rng(),
    );

    assert_eq!(button.x, vec![410, 410]);
    assert_eq!(button.t, vec![0.0, 10.0]);
    assert_eq!(pixel.x, vec![5, 5]);
    assert_eq!(pixel.t, vec![0.0, 10.0]);
}

#[test]
fn smaller_targets_take_longer() {
    let config = ScreenConfig::default();
    let style = MovementStyle::default();
    let mut rng = rng();
    let mean_duration = |size: f64, rng: &mut SmallRng| {
        let trials = 100;
        let total: f64 = (0..trials)
            .map(|_| {
                let target = Target::new([900.0, 540.0], [size, size]);

                generate_single([200.0, 540.0], target, &config, &style, rng).duration()
            })
            .sum();

        total / f64::from(trials)
    };

    let large = mean_duration(64.0, &mut rng);
    let small = mean_duration(8.0, &mut rng);

    assert!(small > 1.3 * large, "small {small}ms, large {large}ms");
}

#[test]
fn corrections_either_pause_or_blend() {
    let config = ScreenConfig::default();
    let mut rng = rng();
    let trials = 300;
    let paused = (0..trials)
        .filter(|_| {
            let style = MovementStyle::random(&mut rng);
            let target = Target::new([1500.0, 800.0], [12.0, 12.0]);
            let trajectory = generate_single([100.0, 100.0], target, &config, &style, &mut rng);

            interior_rests(&trajectory, 60.0) > 0
        })
        .count();

    assert!(
        (trials / 5..trials * 4 / 5).contains(&paused),
        "{paused} of {trials} movements paused between phases"
    );
}

#[test]
fn rests_hold_still_without_flicker() {
    let config = ScreenConfig::default();
    let mut rng = rng();
    let trials = 300;
    let flickers: usize = (0..trials)
        .map(|_| {
            let style = MovementStyle::random(&mut rng);
            let target = Target::point([1500.0, 800.0]);

            out_and_back(&generate_single(
                [100.0, 100.0],
                target,
                &config,
                &style,
                &mut rng,
            ))
        })
        .sum();

    assert!(
        flickers < trials / 5,
        "{flickers} out and back steps over {trials} movements"
    );
}

#[test]
fn generate_chains_each_movement_from_where_the_last_rested() {
    let targets = [
        Target::new([900.0, 700.0], [120.0, 40.0]),
        Target::new([1750.0, 200.0], [32.0, 32.0]),
        Target::point([400.0, 900.0]),
        Target::new([1500.0, 500.0], [64.0, 64.0]),
    ];

    let segments = generate(
        [100.0, 100.0],
        targets,
        &ScreenConfig::default(),
        &MovementStyle::default(),
        &mut rng(),
    );

    assert_eq!(segments.len(), targets.len());
    assert_eq!((segments[0].x[0], segments[0].y[0]), (100, 100));
    for pair in segments.windows(2) {
        assert_eq!(rest(&pair[0]), (pair[1].x[0], pair[1].y[0]));
    }
    assert_eq!(rest(&segments[2]), (400, 900));
}

#[test]
fn same_seed_reproduces_the_same_trajectory() {
    let config = ScreenConfig::default();
    let style = MovementStyle::default();
    let targets = [
        Target::new([900.0, 700.0], [120.0, 40.0]),
        Target::new([1750.0, 200.0], [32.0, 32.0]),
    ];

    let first = merge(
        &generate([100.0, 100.0], targets, &config, &style, &mut rng()),
        70.0..200.0,
        &mut rng(),
    );
    let second = merge(
        &generate([100.0, 100.0], targets, &config, &style, &mut rng()),
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

    let invalid = generate_single(
        [f64::NAN, 0.0],
        Target::point([10.0, 10.0]),
        &config,
        &style,
        &mut rng,
    );
    let nowhere = generate_single(
        [10.0, 10.0],
        Target::point([f64::INFINITY, 10.0]),
        &config,
        &style,
        &mut rng,
    );
    let tiny_screen = generate_single(
        [0.0, 0.0],
        Target::point([500.0, 500.0]),
        &ScreenConfig::new(0, 0),
        &style,
        &mut rng,
    );

    assert!(invalid.is_empty());
    assert!(nowhere.is_empty());
    assert_eq!(tiny_screen.x, vec![0, 0]);
    assert!(generate([1.0, 1.0], [], &config, &style, &mut rng).is_empty());
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

#[test]
fn merge_pauses_lean_toward_the_short_end() {
    let step = Trajectory::new(vec![0, 1], vec![0, 0], vec![0.0, 10.0]);
    let mut rng = rng();
    let mut pauses: Vec<f64> = (0..2000)
        .map(|_| {
            let merged = merge(&[step.clone(), step.clone()], 0.0..100.0, &mut rng);

            merged.t[2] - merged.t[1]
        })
        .collect();
    pauses.sort_by(f64::total_cmp);

    assert!(pauses.iter().all(|pause| (0.0..100.0).contains(pause)));
    assert!(
        (25.0..40.0).contains(&pauses[1000]),
        "median pause {}",
        pauses[1000]
    );
}
