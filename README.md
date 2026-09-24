# cursorflow

[![Crates.io](https://img.shields.io/crates/v/cursorflow.svg)](https://crates.io/crates/cursorflow)
[![Crates.io](https://img.shields.io/crates/d/cursorflow.svg)](https://crates.io/crates/cursorflow)
[![Docs.rs](https://docs.rs/cursorflow/badge.svg)](https://docs.rs/cursorflow)
[![License](https://img.shields.io/crates/l/cursorflow.svg)](LICENSE)

Generate human like cursor movement trajectories in Rust.

`cursorflow` creates timestamped cursor paths onto targets on screen that move the way a hand on a mouse does: a fast, curved primary movement aimed at the target and, when it misses, small corrections that either pause briefly or blend into its end until the cursor rests inside the target. Movement time follows Fitts's law, so small and distant targets take longer; landing errors grow with speed; speed follows an asymmetric bell curve; the path carries wobble and tremor that freeze while the hand rests; and timestamps jitter like real input events. Every draw comes from the `Rng` you pass in, so seeded output is reproducible.

![Cursor moving onto four targets in real time, with a live speed graph underneath](https://raw.githubusercontent.com/xth24/cursorflow-rs/main/assets/trajectory.gif)

## Installation

```bash
cargo add cursorflow rand@0.10
```

Or add it manually:

```toml
[dependencies]
cursorflow = "0.3"
rand = "0.10"
```

The `rand` crate is also available as `cursorflow::rand`, so the versions always match.

## Usage

```rust
use cursorflow::{MovementStyle, ScreenConfig, Target, generate, merge};

fn main() {
    let config = ScreenConfig::default();
    let mut rng = rand::rng();
    let style = MovementStyle::random(&mut rng);

    let targets = [
        Target::new([900.0, 700.0], [120.0, 40.0]),
        Target::new([1750.0, 200.0], [32.0, 32.0]),
    ];

    let segments = generate([100.0, 100.0], targets, &config, &style, &mut rng);
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

- `generate_single(start, target, &config, &style, &mut rng)` creates one movement that comes to rest inside `target`.
- `generate(start, targets, &config, &style, &mut rng)` creates one movement per target, each starting where the previous one rested.
- `merge(&trajectories, delay_range, &mut rng)` joins movements on one time axis with random pauses, in milliseconds.
- `Target` is the rectangle a movement aims at: `Target::new(center, [width, height])`, or `Target::point(center)` for one exact pixel.
- `ScreenConfig` sets the screen size, sample rate, and timestamp jitter.
- `MovementStyle` describes the person: speed, precision, nervousness, overshoot chance, maximum number of movement phases, smoothness, and the direction in which their paths bow. `MovementStyle::random` draws a persona.
- `Trajectory` holds `x`, `y`, `t` (milliseconds), and `v` (pixels per second) columns. `iter()` yields `Sample`s, `deduplicate()` drops repeated positions, and `duration()` returns the last timestamp.

Full documentation is on [docs.rs](https://docs.rs/cursorflow).

## Examples

- `cargo run --example trajectory_svg` writes `target/trajectory.svg`: the path colored by speed with a speed graph underneath.
- `cargo run --example trajectory_video -- target/trajectory.mp4` writes a real time animation with a live speed graph. Requires `ffmpeg`; a `.gif` path works too, which is how the animation above was made.

## License

MIT
