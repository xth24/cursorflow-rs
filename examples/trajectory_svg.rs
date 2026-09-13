//! Renders a trajectory to an SVG: the path colored by speed, with a speed
//! graph underneath. Writes `target/trajectory.svg`.

use std::error::Error;
use std::fmt::Write as _;
use std::fs;

use cursorflow::rand::{SeedableRng, rngs::SmallRng};
use cursorflow::{MovementStyle, Point, ScreenConfig, Trajectory, generate, merge};

const OUTPUT: &str = "target/trajectory.svg";
const GRID_STEP: u32 = 120;
const GRAPH_HEIGHT: u32 = 220;
const GRAPH_MARGIN: f64 = 40.0;

fn main() -> Result<(), Box<dyn Error>> {
    let config = ScreenConfig::default();
    let mut rng = SmallRng::seed_from_u64(7);
    let style = MovementStyle::random(&mut rng);
    let waypoints = [
        [100.0, 100.0],
        [900.0, 700.0],
        [1750.0, 200.0],
        [400.0, 900.0],
        [1500.0, 500.0],
    ];

    let segments = generate(&waypoints, &config, &style, &mut rng);
    let trajectory = merge(&segments, 70.0..200.0, &mut rng).deduplicate();

    fs::write(OUTPUT, render(&trajectory, &waypoints, &config)?)?;
    println!("wrote {OUTPUT}");

    Ok(())
}

fn render(
    trajectory: &Trajectory,
    waypoints: &[Point],
    config: &ScreenConfig,
) -> Result<String, Box<dyn Error>> {
    let width = config.width;
    let height = config.height + GRAPH_HEIGHT;
    let peak = trajectory.iter().map(|sample| sample.v).fold(0.0, f64::max);
    let mut svg = String::new();

    writeln!(
        svg,
        r#"<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}" width="{width}" height="{height}">"#
    )?;
    writeln!(
        svg,
        r##"<rect width="100%" height="100%" fill="#101418"/>"##
    )?;

    writeln!(
        svg,
        r##"<g stroke="#27313a" stroke-width="1" opacity="0.65">"##
    )?;
    for x in (0..=config.width).step_by(GRID_STEP as usize) {
        writeln!(
            svg,
            r#"<line x1="{x}" y1="0" x2="{x}" y2="{}"/>"#,
            config.height
        )?;
    }
    for y in (0..=config.height).step_by(GRID_STEP as usize) {
        writeln!(
            svg,
            r#"<line x1="0" y1="{y}" x2="{}" y2="{y}"/>"#,
            config.width
        )?;
    }
    writeln!(svg, "</g>")?;

    for (from, to) in trajectory.iter().zip(trajectory.iter().skip(1)) {
        let ratio = if peak > 0.0 {
            (to.v / peak).clamp(0.0, 1.0)
        } else {
            0.0
        };
        writeln!(
            svg,
            r#"<line x1="{}" y1="{}" x2="{}" y2="{}" stroke="{}" stroke-width="{:.2}" stroke-linecap="round"/>"#,
            from.x,
            from.y,
            to.x,
            to.y,
            speed_color(ratio),
            2.0 + ratio * 5.0,
        )?;
    }

    writeln!(svg, r##"<g fill="#f8fafc" opacity="0.72">"##)?;
    for sample in trajectory {
        writeln!(
            svg,
            r#"<circle cx="{}" cy="{}" r="2.4"/>"#,
            sample.x, sample.y
        )?;
    }
    writeln!(svg, "</g>")?;

    for [x, y] in waypoints {
        writeln!(
            svg,
            r##"<circle cx="{x:.0}" cy="{y:.0}" r="13" fill="#f8fafc" stroke="#101418" stroke-width="4"/>"##
        )?;
    }

    let baseline = f64::from(height) - GRAPH_MARGIN;
    let graph_span = f64::from(GRAPH_HEIGHT) - 2.0 * GRAPH_MARGIN;
    let duration = trajectory.duration().max(1.0);
    writeln!(
        svg,
        r##"<line x1="0" y1="{}" x2="{width}" y2="{}" stroke="#27313a"/>"##,
        config.height, config.height
    )?;
    writeln!(
        svg,
        r##"<line x1="0" y1="{baseline}" x2="{width}" y2="{baseline}" stroke="#27313a"/>"##
    )?;
    let points: Vec<String> = trajectory
        .iter()
        .map(|sample| {
            let x = sample.t / duration * f64::from(width);
            let y = baseline - if peak > 0.0 { sample.v / peak } else { 0.0 } * graph_span;
            format!("{x:.1},{y:.1}")
        })
        .collect();
    writeln!(
        svg,
        r##"<polyline points="{}" fill="none" stroke="#78c878" stroke-width="2"/>"##,
        points.join(" ")
    )?;

    writeln!(
        svg,
        r##"<text x="24" y="42" fill="#f8fafc" font-family="monospace" font-size="24">points: {} | duration: {:.0}ms | peak: {:.0}px/s</text>"##,
        trajectory.len(),
        trajectory.duration(),
        peak
    )?;
    writeln!(svg, "</svg>")?;

    Ok(svg)
}

fn speed_color(ratio: f64) -> String {
    let r = (70.0 + 185.0 * ratio).round() as u8;
    let g = (210.0 - 120.0 * ratio).round() as u8;
    let b = (255.0 - 215.0 * ratio).round() as u8;

    format!("#{r:02x}{g:02x}{b:02x}")
}
