# cursorflow

[![Crates.io](https://img.shields.io/crates/v/cursorflow.svg)](https://crates.io/crates/cursorflow)
[![Crates.io](https://img.shields.io/crates/d/cursorflow.svg)](https://crates.io/crates/cursorflow)
[![Docs.rs](https://docs.rs/cursorflow/badge.svg)](https://docs.rs/cursorflow)
[![License](https://img.shields.io/crates/l/cursorflow.svg)](LICENSE)

Generate human like cursor movement trajectories in Rust.

`cursorflow` creates timestamped cursor paths between points, including velocity profiles, curved movement, small corrections, noise, and optional pauses between segments.

## Installation

```bash
cargo add cursorflow rand@0.10
```

Or add it manually:

```toml
[dependencies]
cursorflow = "0.1"
rand = "0.10"
```

## Usage

```rust
use cursorflow::{generate, merge, MovementStyle, ScreenConfig};

fn main() {
    let config = ScreenConfig::default();
    let mut rng = rand::rng();
    let style = MovementStyle::random(&mut rng);

    let waypoints = [
        [100.0, 100.0],
        [900.0, 700.0],
        [1750.0, 200.0],
    ];

    let trajectories = generate(&waypoints, &config, &style, &mut rng);
    let combined = merge(&trajectories, 70.0..200.0, &mut rng).deduplicate();

    for i in 0..combined.len().min(5) {
        println!(
            "x={} y={} t={}ms v={:.0}px/s",
            combined.x[i], combined.y[i], combined.t[i], combined.v[i]
        );
    }
}
```

## API

- `generate_single(start, end, config, style, rng)` creates one movement.
- `generate(waypoints, config, style, rng)` creates one movement per waypoint pair.
- `merge(trajectories, delay_range, rng)` combines multiple movements with random delays.
- `Trajectory` contains `x`, `y`, `t`, and `v` vectors for coordinates, timestamps, and velocity.

## License

MIT
