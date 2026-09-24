# Changelog

## 0.3.0 - 2026-09-24

### Breaking changes

- Movements aim at a `Target`, a rectangle given by its center and size, instead of a point. `generate_single(start, target, ...)` takes one, and `generate(start, targets, ...)` takes a start point and any iterator of targets. `Target::point` asks for one exact pixel.
- A movement ends where the cursor came to rest inside its target rather than on the target's center, and each movement in a chain starts where the previous one rested. A start that already lies on the target yields a still trajectory.
- Movement time follows Fitts's law on distance and target width, so small targets take longer and need more corrections than large ones. The screen diagonal now only shapes curvature.
- `MovementStyle::overshoot_tendency` is the probability of landing beyond the aim point, so the default `0.3` falls short 70% of the time.
- `MovementStyle::sub_movement_count` is an upper bound: corrections happen only while the cursor is outside the target, and the last allowed phase ends inside it. The default is now `3`, and `MovementStyle::random` draws from 2 to 4.
- `MovementStyle` has a new `arc_direction` field.
- Output differs from 0.2 for the same seed.

### Changed

- Corrections add onto the movement before them. About half start while it is still slowing down and blend into it; the rest wait a skewed pause, a shorter one after an overshoot.
- Landing errors grow with average speed, as in Meyer's optimized submovement model, so a faster movement misses by more.
- The speed profile is a Beta(3, b) curve, which is minimum jerk at `b = 3`, keeping a smooth start while peaking early. The previous time warp started abruptly.
- Each person bows paths in a given direction the same way, S shaped paths are rare, and no path leaves or arrives more than 30° off its line.
- Wobble and tremor move the cursor only while the hand moves, so pauses no longer flicker between pixels, and tremor oscillates near 9 Hz like physiological tremor.
- Pauses in `merge` and before corrections, and movement durations, follow skewed distributions instead of uniform ones.
- Landings past a screen edge stop at the edge, so the next correction starts from where the cursor is.

### Fixed

- Downward movements no longer bow more than upward ones. The curvature bias used a signed sine where it meant vertical movement.

## 0.2.0 - 2026-09-13

### Breaking changes

- Movements are planned as a primary movement followed by pauses and corrective movements, with an asymmetric speed profile, correlated tremor, and timestamp jitter. Output differs from 0.1 for the same seed.
- `Trajectory::len` counts positions only, the shortest of `x`, `y`, and `t`, and no longer reads `v`.
- Modules are private. Import everything from the crate root: `cursorflow::generate` rather than `cursorflow::generator::generate`.
- `ScreenConfig` has a new `sample_jitter` field. Build it with `ScreenConfig::new` or `..ScreenConfig::default()`.
- The demo binary is gone; see `examples/`.

### Added

- `Trajectory::new`, `duration`, `get`, and `iter`, plus the `Sample` and `Iter` types, `FromIterator<Sample>`, `IntoIterator for &Trajectory`, and `PartialEq`.
- `ScreenConfig::new` and `ScreenConfig::MAX_SAMPLE_RATE`.
- `cursorflow::rand`, which exposes the `rand` crate this library uses.
- Documentation and examples for every public item.

## 0.1.1

- Hardened trajectory generation against invalid input.

## 0.1.0

- Initial release.
