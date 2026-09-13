//! Renders a trajectory as a real time animation through `ffmpeg`, with the
//! cursor, its trail, and a live speed graph.
//!
//! Pass an output path ending in `.gif` or `.mp4`; the default is
//! `target/trajectory.gif`. The trajectory is not deduplicated so that rests
//! show up as zero speed in the graph.

use std::env;
use std::error::Error;
use std::io::{self, Write};
use std::process::{Command, Stdio};

use cursorflow::rand::{SeedableRng, rngs::SmallRng};
use cursorflow::{MovementStyle, Point, ScreenConfig, Trajectory, generate, merge};

const VIEW_WIDTH: usize = 960;
const VIEW_HEIGHT: usize = 540;
const GRAPH_HEIGHT: usize = 120;
const GRAPH_MARGIN: i32 = 12;
const GIF_FPS: u32 = 25;
const VIDEO_FPS: u32 = 60;

const BG: [u8; 3] = [16, 20, 24];
const GRID: [u8; 3] = [39, 49, 58];
const FUTURE_PATH: [u8; 3] = [54, 72, 84];
const TRAIL: [u8; 3] = [70, 210, 255];
const CURSOR: [u8; 3] = [248, 250, 252];
const OUTLINE: [u8; 3] = [16, 20, 24];
const WAYPOINT: [u8; 3] = [255, 213, 92];
const GRAPH: [u8; 3] = [120, 200, 120];

fn main() -> Result<(), Box<dyn Error>> {
    let output = env::args()
        .nth(1)
        .unwrap_or_else(|| "target/trajectory.gif".to_owned());
    let gif = output.ends_with(".gif");
    let fps = if gif { GIF_FPS } else { VIDEO_FPS };

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
    let trajectory = merge(&segments, 70.0..200.0, &mut rng);
    let frames = ((trajectory.duration() / 1000.0) * f64::from(fps)).ceil() as usize + 1;

    let mut command = Command::new("ffmpeg");
    command.args([
        "-y",
        "-loglevel",
        "error",
        "-f",
        "image2pipe",
        "-framerate",
        &fps.to_string(),
        "-vcodec",
        "ppm",
        "-i",
        "-",
    ]);
    if gif {
        command.args([
            "-filter_complex",
            "[0:v]split[a][b];[a]palettegen[p];[b][p]paletteuse",
            "-loop",
            "0",
        ]);
    } else {
        command.args(["-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "18"]);
    }

    let mut child = command.arg(&output).stdin(Stdio::piped()).spawn()?;
    let mut stdin = child.stdin.take().ok_or("ffmpeg stdin is not piped")?;
    for frame in 0..frames {
        let time_ms = frame as f64 * 1000.0 / f64::from(fps);
        render_frame(&trajectory, &waypoints, &config, time_ms).write_ppm(&mut stdin)?;
    }
    drop(stdin);

    if !child.wait()?.success() {
        return Err(format!("ffmpeg failed to write {output}").into());
    }

    println!(
        "wrote {output} ({frames} frames, {:.2}s)",
        frames as f64 / f64::from(fps)
    );

    Ok(())
}

fn render_frame(
    trajectory: &Trajectory,
    waypoints: &[Point],
    config: &ScreenConfig,
    time_ms: f64,
) -> Image {
    let mut image = Image::new(VIEW_WIDTH, VIEW_HEIGHT + GRAPH_HEIGHT, BG);

    draw_grid(&mut image, config);
    draw_full_path(&mut image, trajectory, config);

    let (cursor_x, cursor_y, trail_end) = cursor_at(trajectory, time_ms);
    draw_trail(
        &mut image, trajectory, config, trail_end, cursor_x, cursor_y,
    );
    draw_waypoints(&mut image, waypoints, config);
    draw_cursor(&mut image, config, cursor_x, cursor_y);
    draw_graph(&mut image, trajectory, time_ms);

    image
}

fn draw_grid(image: &mut Image, config: &ScreenConfig) {
    for x in (0..=config.width).step_by(120) {
        let x = map_x(f64::from(x), config);
        image.draw_line(x, 0, x, VIEW_HEIGHT as i32 - 1, GRID, 1);
    }
    for y in (0..=config.height).step_by(120) {
        let y = map_y(f64::from(y), config);
        image.draw_line(0, y, VIEW_WIDTH as i32 - 1, y, GRID, 1);
    }
}

fn draw_full_path(image: &mut Image, trajectory: &Trajectory, config: &ScreenConfig) {
    for (from, to) in trajectory.iter().zip(trajectory.iter().skip(1)) {
        image.draw_line(
            map_x(f64::from(from.x), config),
            map_y(f64::from(from.y), config),
            map_x(f64::from(to.x), config),
            map_y(f64::from(to.y), config),
            FUTURE_PATH,
            2,
        );
    }

    for sample in trajectory {
        image.draw_disc(
            map_x(f64::from(sample.x), config),
            map_y(f64::from(sample.y), config),
            1,
            FUTURE_PATH,
        );
    }
}

fn draw_trail(
    image: &mut Image,
    trajectory: &Trajectory,
    config: &ScreenConfig,
    trail_end: usize,
    cursor_x: f64,
    cursor_y: f64,
) {
    let travelled = trajectory.iter().take(trail_end + 1);
    for (from, to) in travelled.clone().zip(travelled.clone().skip(1)) {
        image.draw_line(
            map_x(f64::from(from.x), config),
            map_y(f64::from(from.y), config),
            map_x(f64::from(to.x), config),
            map_y(f64::from(to.y), config),
            TRAIL,
            4,
        );
    }

    if let Some(last) = trajectory.get(trail_end) {
        image.draw_line(
            map_x(f64::from(last.x), config),
            map_y(f64::from(last.y), config),
            map_x(cursor_x, config),
            map_y(cursor_y, config),
            TRAIL,
            4,
        );
    }

    for sample in travelled {
        image.draw_disc(
            map_x(f64::from(sample.x), config),
            map_y(f64::from(sample.y), config),
            2,
            CURSOR,
        );
    }
}

fn draw_waypoints(image: &mut Image, waypoints: &[Point], config: &ScreenConfig) {
    for &[x, y] in waypoints {
        let x = map_x(x, config);
        let y = map_y(y, config);
        image.draw_disc(x, y, 8, OUTLINE);
        image.draw_disc(x, y, 5, WAYPOINT);
    }
}

fn draw_cursor(image: &mut Image, config: &ScreenConfig, x: f64, y: f64) {
    let x = map_x(x, config);
    let y = map_y(y, config);
    image.draw_disc(x, y, 12, OUTLINE);
    image.draw_disc(x, y, 8, CURSOR);
    image.draw_disc(x, y, 3, TRAIL);
}

fn draw_graph(image: &mut Image, trajectory: &Trajectory, time_ms: f64) {
    let top = VIEW_HEIGHT as i32 + GRAPH_MARGIN;
    let bottom = (VIEW_HEIGHT + GRAPH_HEIGHT) as i32 - GRAPH_MARGIN;
    let duration = trajectory.duration().max(1.0);
    let peak = trajectory
        .iter()
        .map(|sample| sample.v)
        .fold(0.0, f64::max)
        .max(1.0);
    let graph_x = |t: f64| ((t / duration) * (VIEW_WIDTH - 1) as f64).round() as i32;
    let graph_y = |v: f64| bottom - ((v / peak) * f64::from(bottom - top)).round() as i32;

    image.draw_line(
        0,
        VIEW_HEIGHT as i32,
        VIEW_WIDTH as i32 - 1,
        VIEW_HEIGHT as i32,
        GRID,
        1,
    );
    image.draw_line(0, bottom, VIEW_WIDTH as i32 - 1, bottom, GRID, 1);

    let shown = trajectory.iter().take_while(|sample| sample.t <= time_ms);
    for (from, to) in shown.clone().zip(shown.skip(1)) {
        image.draw_line(
            graph_x(from.t),
            graph_y(from.v),
            graph_x(to.t),
            graph_y(to.v),
            GRAPH,
            1,
        );
    }

    let now = graph_x(time_ms.min(duration));
    image.draw_line(now, top, now, bottom, TRAIL, 1);
}

fn cursor_at(trajectory: &Trajectory, time_ms: f64) -> (f64, f64, usize) {
    let Some(first) = trajectory.get(0) else {
        return (0.0, 0.0, 0);
    };
    if trajectory.len() == 1 || time_ms <= first.t {
        return (f64::from(first.x), f64::from(first.y), 0);
    }

    let hi = trajectory
        .t
        .partition_point(|&t| t <= time_ms)
        .min(trajectory.len() - 1);
    let lo = hi.saturating_sub(1);
    let span = trajectory.t[hi] - trajectory.t[lo];
    let ratio = if span > 0.0 {
        ((time_ms - trajectory.t[lo]) / span).clamp(0.0, 1.0)
    } else {
        0.0
    };

    let x = f64::from(trajectory.x[lo]) + f64::from(trajectory.x[hi] - trajectory.x[lo]) * ratio;
    let y = f64::from(trajectory.y[lo]) + f64::from(trajectory.y[hi] - trajectory.y[lo]) * ratio;

    (x, y, lo)
}

fn map_x(x: f64, config: &ScreenConfig) -> i32 {
    let denominator = f64::from(config.width.saturating_sub(1).max(1));

    ((x / denominator) * (VIEW_WIDTH - 1) as f64).round() as i32
}

fn map_y(y: f64, config: &ScreenConfig) -> i32 {
    let denominator = f64::from(config.height.saturating_sub(1).max(1));

    ((y / denominator) * (VIEW_HEIGHT - 1) as f64).round() as i32
}

struct Image {
    width: usize,
    height: usize,
    pixels: Vec<u8>,
}

impl Image {
    fn new(width: usize, height: usize, color: [u8; 3]) -> Self {
        let pixels = color.repeat(width * height);

        Self {
            width,
            height,
            pixels,
        }
    }

    fn write_ppm(&self, writer: &mut impl Write) -> io::Result<()> {
        write!(writer, "P6\n{} {}\n255\n", self.width, self.height)?;
        writer.write_all(&self.pixels)
    }

    fn draw_line(&mut self, x0: i32, y0: i32, x1: i32, y1: i32, color: [u8; 3], radius: i32) {
        let mut x = x0;
        let mut y = y0;
        let dx = (x1 - x0).abs();
        let dy = -(y1 - y0).abs();
        let sx = if x0 < x1 { 1 } else { -1 };
        let sy = if y0 < y1 { 1 } else { -1 };
        let mut err = dx + dy;

        loop {
            self.draw_disc(x, y, radius, color);

            if x == x1 && y == y1 {
                break;
            }

            let e2 = 2 * err;
            if e2 >= dy {
                err += dy;
                x += sx;
            }

            if e2 <= dx {
                err += dx;
                y += sy;
            }
        }
    }

    fn draw_disc(&mut self, cx: i32, cy: i32, radius: i32, color: [u8; 3]) {
        for y in cy - radius..=cy + radius {
            for x in cx - radius..=cx + radius {
                let dx = x - cx;
                let dy = y - cy;

                if dx * dx + dy * dy <= radius * radius {
                    self.set_pixel(x, y, color);
                }
            }
        }
    }

    fn set_pixel(&mut self, x: i32, y: i32, color: [u8; 3]) {
        let (Ok(x), Ok(y)) = (usize::try_from(x), usize::try_from(y)) else {
            return;
        };
        if x >= self.width || y >= self.height {
            return;
        }

        let offset = (y * self.width + x) * 3;
        self.pixels[offset..offset + 3].copy_from_slice(&color);
    }
}
