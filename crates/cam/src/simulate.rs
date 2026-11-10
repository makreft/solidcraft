//! Stock simulation: a height map of the stock's top, lowered wherever a tool's cutting moves
//! pass, with the tool's own shape (a flat, a ball, a cone). What is left standing shows before
//! anything is cut.

use solidcore_record::{Region2, Vec2, Vec3};
use solidcraft_geom::prelude::*;

use crate::path::Toolpath;
use crate::tool::{Tool, ToolKind};
use crate::{CamError, Result, invalid};

/// Most pixels of one height map (a 1 m × 1 m stock at 0.2 mm).
pub const MAX_PIXELS: usize = 25_000_000;

/// The stock's top surface on a grid: `z[y * width + x]` is the height over the pixel whose
/// centre is `origin + (x + 0.5, y + 0.5) * step`.
#[derive(Clone, Debug, PartialEq)]
pub struct Heightmap {
    pub origin: Vec2,
    pub step: f64,
    pub width: usize,
    pub height: usize,
    pub top: f64,
    pub z: Vec<f32>,
}

impl Heightmap {
    /// An uncut stock from `min` to `max` in XY, `top` high, in pixels `step` wide.
    pub fn new(min: Vec2, max: Vec2, top: f64, step: f64) -> Result<Heightmap> {
        if !min.is_finite() || !max.is_finite() || !top.is_finite() || !(max.x > min.x) || !(max.y > min.y) {
            return invalid("the stock needs finite bounds with max above min");
        }
        if !(step > 0.0) || !step.is_finite() {
            return invalid("the resolution must be above 0 mm per pixel");
        }
        let (w, h) = (((max.x - min.x) / step).ceil(), ((max.y - min.y) / step).ceil());
        if !(w >= 1.0 && h >= 1.0) || w * h > MAX_PIXELS as f64 {
            return Err(CamError::TooLarge(format!("the height map would have more than {MAX_PIXELS} pixels; use a coarser resolution")));
        }
        let (width, height) = (w as usize, h as usize);
        Ok(Heightmap { origin: min, step, width, height, top, z: vec![top as f32; width * height] })
    }

    /// The centre of pixel (`x`, `y`).
    pub fn centre(&self, x: usize, y: usize) -> Vec2 {
        self.origin + Vec2::new((x as f64 + 0.5) * self.step, (y as f64 + 0.5) * self.step)
    }

    /// The lowest point of the surface.
    pub fn deepest(&self) -> f64 {
        self.z.iter().fold(self.top, |m, &v| m.min(f64::from(v)))
    }

    /// The material removed so far (mm³).
    pub fn removed(&self) -> f64 {
        self.z.iter().map(|&v| self.top - f64::from(v)).sum::<f64>() * self.step * self.step
    }

    /// Lowers the surface along every cutting move of `path` as `tool` would: the surface under
    /// the tool is at most the tool's own surface there. Returns the volume removed (mm³).
    pub fn cut(&mut self, tool: &Tool, path: &Toolpath) -> Result<f64> {
        tool.validate()?;
        let before = self.removed();
        let samples = path.points(self.step * 0.25);
        for w in samples.windows(2) {
            let ((a, _), (b, cutting)) = (w[0], w[1]);
            if !cutting || !a.is_finite() || !b.is_finite() {
                continue;
            }
            let n = ((a.dist(b) / (self.step * 0.5)).ceil() as usize).clamp(1, 1_000_000);
            for i in 0..=n {
                let t = i as f64 / n as f64;
                self.stamp(tool, a.lerp(b, t));
            }
        }
        Ok(self.removed() - before)
    }

    /// The tool with its tip at `tip`: every pixel under it drops to the tool's surface.
    fn stamp(&mut self, tool: &Tool, tip: Vec3) {
        let depth = self.top - tip.z;
        if depth <= 0.0 {
            return;
        }
        // Only the part of the tool below the stock top can cut.
        let reach = tool.width_at_depth(depth) * 0.5;
        if !(reach > 0.0) {
            return;
        }
        let lo = Vec2::new(tip.x - reach, tip.y - reach);
        let hi = Vec2::new(tip.x + reach, tip.y + reach);
        let x0 = ((lo.x - self.origin.x) / self.step).floor().max(0.0) as usize;
        let y0 = ((lo.y - self.origin.y) / self.step).floor().max(0.0) as usize;
        let x1 = (((hi.x - self.origin.x) / self.step).ceil().max(0.0) as usize).min(self.width);
        let y1 = (((hi.y - self.origin.y) / self.step).ceil().max(0.0) as usize).min(self.height);
        for y in y0..y1 {
            for x in x0..x1 {
                let d = self.centre(x, y).dist(Vec2::new(tip.x, tip.y));
                if d > reach {
                    continue;
                }
                let surface = (tip.z + tool_rise(tool, d)) as f32;
                let cell = &mut self.z[y * self.width + x];
                if surface < *cell {
                    *cell = surface;
                }
            }
        }
    }

    /// How many pixels inside `region` still stand above `z` (plus a hair), and how many lie in
    /// the region at all.
    pub fn standing_in(&self, region: &Region2, z: f64) -> (usize, usize) {
        let (lo, hi) = region
            .outer
            .polyline(0.1)
            .into_vec()
            .iter()
            .fold((Vec2::new(f64::INFINITY, f64::INFINITY), Vec2::new(f64::NEG_INFINITY, f64::NEG_INFINITY)), |(lo, hi), p| {
                (Vec2::new(lo.x.min(p.x), lo.y.min(p.y)), Vec2::new(hi.x.max(p.x), hi.y.max(p.y)))
            });
        let x0 = ((lo.x - self.origin.x) / self.step).floor().max(0.0) as usize;
        let y0 = ((lo.y - self.origin.y) / self.step).floor().max(0.0) as usize;
        let x1 = (((hi.x - self.origin.x) / self.step).ceil().max(0.0) as usize).min(self.width);
        let y1 = (((hi.y - self.origin.y) / self.step).ceil().max(0.0) as usize).min(self.height);
        let (mut standing, mut inside) = (0, 0);
        for y in y0..y1 {
            for x in x0..x1 {
                let c = self.centre(x, y);
                if !region.outer.contains(c) || region.holes.iter().any(|h| h.contains(c)) {
                    continue;
                }
                inside += 1;
                if f64::from(self.z[y * self.width + x]) > z + 0.01 {
                    standing += 1;
                }
            }
        }
        (standing, inside)
    }
}

/// How far above its tip the tool's surface is, `d` from its axis.
fn tool_rise(tool: &Tool, d: f64) -> f64 {
    match tool.kind {
        ToolKind::FlatEndmill | ToolKind::Drill => 0.0,
        ToolKind::BallEndmill => {
            let r = tool.radius();
            r - (r * r - d * d).max(0.0).sqrt()
        }
        ToolKind::VBit => {
            let tan_half = (tool.tip_angle.unwrap_or(0.0).to_radians() * 0.5).tan();
            if tan_half > 1e-9 { (d - tool.tip_diameter * 0.5).max(0.0) / tan_half } else { 0.0 }
        }
        ToolKind::TaperedBall => {
            let r = tool.tip_diameter * 0.5;
            if d <= r {
                r - (r * r - d * d).max(0.0).sqrt()
            } else {
                let tan = tool.taper_angle.unwrap_or(0.0).to_radians().tan();
                if tan > 1e-9 { r + (d - r) / tan } else { r }
            }
        }
    }
}
