use cursorflow::{MovementStyle, ScreenConfig, generate, merge};

fn main() {
    let config = ScreenConfig::default();
    let mut rng = rand::rng();
    let style = MovementStyle::random(&mut rng);

    let waypoints = [
        [100.0, 100.0],
        [900.0, 700.0],
        [1750.0, 200.0],
        [400.0, 900.0],
        [1500.0, 500.0],
    ];

    let trajectories = generate(&waypoints, &config, &style, &mut rng);
    let combined = merge(&trajectories, 70.0..200.0, &mut rng).deduplicate();

    let peak = combined.v.iter().copied().fold(0.0_f64, f64::max);
    let (sum, count) = combined
        .v
        .iter()
        .copied()
        .filter(|&v| v > 0.0)
        .fold((0.0, 0usize), |(s, c), v| (s + v, c + 1));
    let mean = if count == 0 { 0.0 } else { sum / count as f64 };

    println!("Points: {}", combined.len());
    println!(
        "Duration: {:.0} ms",
        combined.t.last().copied().unwrap_or(0.0)
    );
    println!("Peak velocity: {peak:.0} px/s");
    println!("Mean velocity: {mean:.0} px/s");
}
