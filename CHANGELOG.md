# Changelog

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
