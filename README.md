# cursorflow

[![Crates.io](https://img.shields.io/crates/v/cursorflow.svg)](https://crates.io/crates/cursorflow)
[![Crates.io](https://img.shields.io/crates/d/cursorflow.svg)](https://crates.io/crates/cursorflow)
[![Docs.rs](https://docs.rs/cursorflow/badge.svg)](https://docs.rs/cursorflow)
[![License](https://img.shields.io/crates/l/cursorflow.svg)](LICENSE)

Generate human like cursor movement trajectories in Rust.

`cursorflow` creates timestamped cursor paths between points that move the way a hand on a mouse does: a fast, curved primary movement that lands near the target, a brief pause, and one or more small corrections that home in on it. Speed follows an asymmetric bell curve, the path carries a little tremor, and timestamps jitter like real input events. Every draw comes from the `Rng` you pass in, so seeded output is reproducible.

![Cursor moving through five waypoints in real time, with a live speed graph underneath](https://raw.githubusercontent.com/xth24/cursorflow-rs/main/assets/trajectory.gif)

## Installation

```bash
cargo add cursorflow rand@0.10
```

Or add it manually:

```toml
[dependencies]
cursorflow = "0.2"
rand = "0.10"
```

The `rand` crate is also available as `cursorflow::rand`, so the versions always match.

## Usage

```rust
use cursorflow::{MovementStyle, ScreenConfig, generate, merge};

fn main() {
    let config = ScreenConfig::default();
    let mut rng = rand::rng();
    let style = MovementStyle::random(&mut rng);

    let waypoints = [
        [100.0, 100.0],
        [900.0, 700.0],
        [1750.0, 200.0],
    ];

    let segments = generate(&waypoints, &config, &style, &mut rng);
    let path = merge(&segments, 70.0..200.0, &mut rng).deduplicate();

    for sample in path.iter().take(5) {
        println!(
            "x={} y={} t={:.1}ms v={:.0}px/s",
            sample.x, sample.y, sample.t, sample.v
        );
    }
}
```

## API

- `generate_single(start, end, &config, &style, &mut rng)` creates one movement.
- `generate(&waypoints, &config, &style, &mut rng)` creates one movement per consecutive waypoint pair.
- `merge(&trajectories, delay_range, &mut rng)` joins movements on one time axis with random pauses, in milliseconds.
- `ScreenConfig` sets the screen size, sample rate, and timestamp jitter.
- `MovementStyle` describes the person: speed, precision, nervousness, overshoot tendency, number of movement phases, and smoothness. `MovementStyle::random` draws a persona.
- `Trajectory` holds `x`, `y`, `t` (milliseconds), and `v` (pixels per second) columns. `iter()` yields `Sample`s, `deduplicate()` drops repeated positions, and `duration()` returns the last timestamp.

Full documentation is on [docs.rs](https://docs.rs/cursorflow).

## Examples

- `cargo run --example trajectory_svg` writes `target/trajectory.svg`: the path colored by speed with a speed graph underneath.
- `cargo run --example trajectory_video -- target/trajectory.mp4` writes a real time animation with a live speed graph. Requires `ffmpeg`; a `.gif` path works too, which is how the animation above was made.

## License

MIT
