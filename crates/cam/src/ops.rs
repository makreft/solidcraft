//! 2.5D operations: contours and pockets, each also as rest machining after a larger tool.
//! Each one turns planar geometry
//! and cutting data into a [`Toolpath`] and checks it with [`verify`] before returning it.

use std::f64::consts::PI;

use serde::{Deserialize, Serialize};
use solidcraft_geom::{Region2, Vec2, Vec3};

use crate::check::verify;
use crate::offset::{Shape, reversed};
use crate::path::{MAX_MOVES, Move, Toolpath};
use crate::piece::{Piece, turn};
use crate::tool::{Tool, ToolKind};
use crate::{CamError, Result, invalid};

/// Most depth passes of one operation.
const MAX_LEVELS: usize = 10_000;
/// Most offset rings of one pocket.
const MAX_RINGS: usize = 2_000;
/// Most times a ramp goes round its loop; a loop too short for that is plunged into instead.
const MAX_RAMP_LAPS: f64 = 2000.0;
/// Largest coordinate or height (mm).
pub(crate) const MAX_COORD: f64 = 1.0e6;
/// Most tabs on one loop.
const MAX_TABS: usize = 1000;
/// A corner of the toolpath that turns away from the material by more than this (radians) gets
/// a dogbone; the slight bends of a curve drawn as short lines do not.
const DOGBONE_MIN_TURN: f64 = 0.5;
/// The steepest ramp into the material (85°).
const RAMP_LIMIT: f64 = 85.0 * PI / 180.0;

/// The Z levels of an operation (work coordinates, mm).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Heights {
    /// Where the tool travels between cuts.
    pub clearance: f64,
    /// Where rapid motion towards the material stops and feed begins.
    pub retract: f64,
    /// Top of the material to cut.
    pub top: f64,
    /// Final depth.
    pub bottom: f64,
}

impl Heights {
    pub fn validate(&self) -> Result<()> {
        let all = [self.clearance, self.retract, self.top, self.bottom];
        if !all.iter().all(|z| z.is_finite() && z.abs() <= MAX_COORD) {
            return invalid("the heights must be finite numbers");
        }
        if self.clearance < self.retract || self.retract < self.top || self.top < self.bottom {
            return invalid("the heights must be ordered: clearance ≥ retract ≥ top ≥ bottom");
        }
        Ok(())
    }
}

/// Feed rates (mm/min).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Feeds {
    pub cut: f64,
    pub plunge: f64,
}

impl Feeds {
    pub fn validate(&self) -> Result<()> {
        if [self.cut, self.plunge].iter().all(|f| *f > 0.0 && *f <= 1.0e6) { Ok(()) } else { invalid("the feed rates must be above 0") }
    }
}

/// What every operation needs to cut.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Cut {
    pub heights: Heights,
    pub feeds: Feeds,
    /// Most material taken per depth pass (mm).
    pub stepdown: f64,
}

impl Cut {
    pub(crate) fn validate(&self) -> Result<()> {
        self.heights.validate()?;
        self.feeds.validate()?;
        if !(self.stepdown > 0.0) || !self.stepdown.is_finite() {
            return invalid("the stepdown must be above 0");
        }
        Ok(())
    }

    /// Depth passes from the top down to the bottom, evenly spaced and at most a stepdown apart.
    pub(crate) fn levels(&self) -> Result<Vec<f64>> {
        let (top, bottom) = (self.heights.top, self.heights.bottom);
        let n = ((top - bottom) / self.stepdown - 1e-9).ceil().max(1.0);
        if !(n <= MAX_LEVELS as f64) {
            return Err(CamError::TooLarge(format!("more than {MAX_LEVELS} depth passes")));
        }
        let n = n as usize;
        Ok((1..=n).map(|k| if k == n { bottom } else { top - (top - bottom) * k as f64 / n as f64 }).collect())
    }
}

/// Which side of a profile the tool runs on.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Side {
    /// Around the regions: the regions are the part that stays.
    Outside,
    /// Inside the regions' boundary: the regions are the opening.
    Inside,
    /// Tool centre on the boundary.
    On,
}

/// Rest machining: the operation cuts only where an earlier, larger tool could not reach, in
/// the inside corners and the openings it did not fit.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Rest {
    /// The earlier tool's radius at the top of its cut (mm).
    pub radius: f64,
    /// How far from the walls that tool's centre ran: its radius and the stock it left (mm).
    pub wall: f64,
}

impl Rest {
    fn validate(&self, r: f64) -> Result<()> {
        if !(self.radius > 0.0 && self.radius <= MAX_COORD) || !(self.wall >= self.radius && self.wall <= MAX_COORD) {
            return invalid("rest machining needs the earlier tool's radius above 0 and its wall offset at least that");
        }
        if !(self.radius > r + 1e-9) {
            return invalid("rest machining needs a tool smaller than the earlier one");
        }
        Ok(())
    }
}

/// Sample spacing, in tool radii, when the rest of a loop is told apart from what was cut.
const REST_SAMPLE: f64 = 0.25;
/// Material thinner than this (mm) is not worth a rest cut: the slivers a curve drawn as short
/// lines leaves at every vertex, a hair's width.
const REST_MIN: f64 = 0.02;

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ContourParams {
    pub side: Side,
    /// Climb milling (with a clockwise spindle the wall is on the right of the tool's travel);
    /// otherwise conventional.
    pub climb: bool,
    /// Material left on the wall (mm).
    pub stock_to_leave: f64,
    /// Chord error allowed on spline boundaries (mm).
    pub tolerance: f64,
    /// Steepest descent into each depth pass, along the profile (radians); 0 plunges straight down.
    pub ramp_angle: f64,
    /// Bridges left standing on the last passes, so a part cut free stays in the stock.
    #[serde(default)]
    pub tabs: Option<Tabs>,
    /// At every inside corner the tool cannot reach (it leaves its radius there), it goes in
    /// along the corner's bisector until it touches the corner and comes back: a round notch
    /// past the corner, so a square part fits all the way in.
    #[serde(default)]
    pub dogbones: bool,
    /// Only where the earlier tool could not reach; no tabs, dogbones or ramps then.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rest: Option<Rest>,
}

/// Holding tabs of a contour: on the passes below their top the tool rises over each tab and
/// comes back down at 45°, never straight down.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Tabs {
    /// Width of a tab at its top, in the material along the contour (mm).
    pub width: f64,
    /// Height of a tab above the bottom (mm).
    pub height: f64,
    /// About one tab every this far along a loop, at least two (mm). A loop shorter than four
    /// tabs' footprints gets none: a small hole's waste comes free.
    pub spacing: f64,
    /// Whether loops inside another one of the operation (a part's holes) get tabs too, holding
    /// their waste; otherwise only the outlines.
    pub inner: bool,
}

impl Tabs {
    fn validate(&self, h: &Heights) -> Result<()> {
        if !(self.width > 0.0 && self.width <= 1000.0) || !(self.spacing > 0.0 && self.spacing <= MAX_COORD) {
            return invalid("tabs need a width above 0 (at most 1 m) and a spacing above 0");
        }
        if !(self.height > 0.0) || !(h.bottom + self.height < h.top) {
            return invalid("tabs must be above 0 high and lower than the cut is deep");
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct PocketParams {
    /// Distance between passes (mm), at most the tool diameter.
    pub stepover: f64,
    pub climb: bool,
    /// Material left on the walls (mm).
    pub stock_to_leave: f64,
    /// Chord error allowed on spline boundaries (mm).
    pub tolerance: f64,
    /// Steepest descent into uncut material, along the first pass (radians); 0 plunges straight down.
    pub ramp_angle: f64,
    /// Dogbones in the pocket's inside corners, as a contour makes them: so a square part fits
    /// all the way into a pocket made for it.
    #[serde(default)]
    pub dogbones: bool,
    /// Only where the earlier tool could not reach: the wall ring's corners and what it did not
    /// fit into; no dogbones or ramps then.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rest: Option<Rest>,
}

/// Collects moves, keeping the tool's position.
pub(crate) struct Builder {
    moves: Vec<Move>,
    pos: Option<Vec3>,
    h: Heights,
    feeds: Feeds,
}

impl Builder {
    pub(crate) fn new(cut: &Cut) -> Builder {
        Builder { moves: Vec::new(), pos: None, h: cut.heights, feeds: cut.feeds }
    }

    pub(crate) fn push(&mut self, m: Move) -> Result<()> {
        if self.moves.len() >= MAX_MOVES {
            return Err(CamError::TooLarge(format!("the toolpath has more than {MAX_MOVES} moves")));
        }
        if let Some(to) = m.target() {
            self.pos = Some(to);
        }
        self.moves.push(m);
        Ok(())
    }

    /// Where the tool is, once it has moved.
    pub(crate) fn position(&self) -> Option<Vec3> {
        self.pos
    }

    fn at(&self, p: Vec2) -> bool {
        self.pos.is_some_and(|q| Vec2::new(q.x, q.y).dist(p) <= 1e-9)
    }

    /// Up to the clearance height (if below it), then across to `p`.
    pub(crate) fn travel_to(&mut self, p: Vec2) -> Result<()> {
        if let Some(q) = self.pos {
            if self.at(p) && q.z >= self.h.clearance {
                return Ok(());
            }
            if q.z < self.h.clearance {
                self.push(Move::Rapid { to: Vec3::new(q.x, q.y, self.h.clearance) })?;
            }
        }
        self.push(Move::Rapid { to: p.extend(self.h.clearance) })
    }

    /// Straight down to the retract height at the rapid rate, when above it.
    pub(crate) fn rapid_to_retract(&mut self) -> Result<()> {
        let Some(q) = self.pos else { return Err(CamError::Unsafe("a rapid before the first positioning move".into())) };
        if q.z > self.h.retract + 1e-9 {
            self.push(Move::Rapid { to: Vec3::new(q.x, q.y, self.h.retract) })?;
        }
        Ok(())
    }

    /// Straight down to `z`: rapid to the retract height, then at the plunge feed.
    pub(crate) fn plunge_to(&mut self, z: f64) -> Result<()> {
        let Some(q) = self.pos else { return Err(CamError::Unsafe("a plunge before the first positioning move".into())) };
        if q.z > self.h.retract {
            self.push(Move::Rapid { to: Vec3::new(q.x, q.y, self.h.retract) })?;
        }
        if z < self.pos.map_or(z, |p| p.z) {
            self.push(Move::Feed { to: Vec3::new(q.x, q.y, z), feed: self.feeds.plunge })?;
        }
        Ok(())
    }

    pub(crate) fn cut_to(&mut self, p: Vec2, z: f64) -> Result<()> {
        if self.at(p) && self.pos.is_some_and(|q| (q.z - z).abs() <= 1e-9) {
            return Ok(());
        }
        self.push(Move::Feed { to: p.extend(z), feed: self.feeds.cut })
    }

    /// Cuts along `piece` from wherever the tool is (within the offsetting's joining tolerance
    /// of the piece's start), going evenly from depth `z0` to `z1`.
    pub(crate) fn cut_piece(&mut self, piece: &Piece, z0: f64, z1: f64) -> Result<()> {
        match *piece {
            Piece::Line { b, .. } => self.cut_to(b, z1),
            Piece::Arc { c, sweep, .. } => {
                // Halves, so that no arc move comes near a full turn.
                let parts = if sweep.abs() > PI { 2 } else { 1 };
                for k in 1..=parts {
                    let t = k as f64 / parts as f64;
                    let to = piece.point_at(t).extend(z0 + (z1 - z0) * t);
                    self.push(Move::Arc { to, center: c, ccw: sweep > 0.0, feed: self.feeds.cut })?;
                }
                Ok(())
            }
        }
    }

    pub(crate) fn cut_loop(&mut self, lp: &[Piece], z: f64, tabs: Option<&Terrain>) -> Result<()> {
        match tabs {
            Some(t) => self.cut_over(lp, z, z, t),
            None => lp.iter().try_for_each(|piece| self.cut_piece(piece, z, z)),
        }
    }

    /// Once along `lp`, from depth `z0` at its start evenly to `z1` at its end, but never below
    /// the tabs' floor: pieces are split where the floor bends or meets that depth, so every
    /// move stays straight (or a helix) in Z.
    fn cut_over(&mut self, lp: &[Piece], z0: f64, z1: f64, tabs: &Terrain) -> Result<()> {
        let total: f64 = lp.iter().map(Piece::length).sum();
        let nominal = |s: f64| if total > 0.0 { z0 + (z1 - z0) * s / total } else { z1 };
        let z = |s: f64| nominal(s).max(tabs.floor(s));
        let mut s0 = 0.0;
        for piece in lp {
            let len = piece.length();
            let s1 = s0 + len;
            let mut at: Vec<f64> = tabs.kinks().into_iter().filter(|&k| k > s0 && k < s1).collect();
            at.extend([s0, s1]);
            at.sort_by(f64::total_cmp);
            let crossings: Vec<f64> = at
                .windows(2)
                .filter_map(|w| {
                    let (fa, fb) = (nominal(w[0]) - tabs.floor(w[0]), nominal(w[1]) - tabs.floor(w[1]));
                    (fa.is_finite() && fb.is_finite() && fa * fb < 0.0).then(|| w[0] + (w[1] - w[0]) * fa / (fa - fb))
                })
                .collect();
            at.extend(crossings);
            at.sort_by(f64::total_cmp);
            for w in at.windows(2) {
                if w[1] - w[0] <= 1e-9 {
                    continue;
                }
                let part = if len > 0.0 { piece.sub((w[0] - s0) / len, (w[1] - s0) / len) } else { *piece };
                self.cut_piece(&part, z(w[0]), z(w[1]))?;
            }
            s0 = s1;
        }
        Ok(())
    }

    /// Goes down to `z` along `lp`, no steeper than `angle`, round as often as that takes; the
    /// tool ends where the loop starts. Straight down when there is no angle or the loop is too
    /// short to ramp along.
    fn ramp_loop(&mut self, lp: &[Piece], z: f64, angle: f64, tabs: Option<&Terrain>) -> Result<()> {
        let Some(from) = self.pos.map(|p| p.z) else { return Err(CamError::Unsafe("a ramp before the first positioning move".into())) };
        let drop = from - z;
        if !(drop > 1e-9) {
            return Ok(());
        }
        let length: f64 = lp.iter().map(Piece::length).sum();
        let laps = (drop / angle.tan() / length - 1e-9).ceil().max(1.0);
        if !(angle > 0.0) || !(length > 1e-3) || !(laps <= MAX_RAMP_LAPS) {
            return self.plunge_to(z);
        }
        if let Some(t) = tabs {
            for lap in 0..laps as usize {
                let (k0, k1) = (lap as f64 / laps, (lap as f64 + 1.0) / laps);
                self.cut_over(lp, from - drop * k0, from - drop * k1, t)?;
            }
            return Ok(());
        }
        let total = laps * length;
        let mut done = 0.0;
        for _ in 0..laps as usize {
            for piece in lp {
                let z0 = from - drop * done / total;
                done += piece.length();
                self.cut_piece(piece, z0, from - drop * (done / total).min(1.0))?;
            }
        }
        // Exactly at depth, whatever the rounding of the last step.
        match self.pos {
            Some(p) => self.cut_to(Vec2::new(p.x, p.y), z),
            None => Ok(()),
        }
    }

    pub(crate) fn retract(&mut self) -> Result<()> {
        match self.pos {
            Some(q) if q.z < self.h.clearance => self.push(Move::Rapid { to: Vec3::new(q.x, q.y, self.h.clearance) }),
            _ => Ok(()),
        }
    }

    pub(crate) fn finish(mut self) -> Result<Toolpath> {
        self.retract()?;
        let path = Toolpath { moves: self.moves };
        verify(&path, &self.h)?;
        Ok(path)
    }
}

fn check_ramp(angle: f64) -> Result<()> {
    if (0.0..=RAMP_LIMIT).contains(&angle) { Ok(()) } else { invalid("the ramp angle must be between 0 and 85 degrees") }
}

/// A cut deeper than the tool's cutting edges would rub its shank in the slot.
pub(crate) fn check_reach(tool: &Tool, cut: &Cut) -> Result<()> {
    let depth = cut.heights.top - cut.heights.bottom;
    if depth > tool.flute_length + 1e-9 {
        return invalid(format!("the cut is {depth:.3} mm deep, but tool {} cuts only {:.3} mm (its flute length)", tool.number, tool.flute_length));
    }
    Ok(())
}

fn check_leave(leave: f64) -> Result<()> {
    if leave.is_finite() && (0.0..=MAX_COORD).contains(&leave) { Ok(()) } else { invalid("the stock to leave must be 0 or more") }
}

/// A milling operation needs a tool that cuts with its side: a drill only cuts at its point.
pub(crate) fn mills(tool: &Tool) -> Result<()> {
    if tool.kind == ToolKind::Drill { invalid("a drill cannot mill: it cuts only at its point") } else { Ok(()) }
}

/// The loop turned to start at its corner nearest to `p`.
pub(crate) fn start_near(lp: &[Piece], p: Vec2) -> Vec<Piece> {
    let first = lp.iter().enumerate().min_by(|a, b| a.1.start().dist(p).total_cmp(&b.1.start().dist(p))).map_or(0, |(i, _)| i);
    lp.iter().cycle().skip(first).take(lp.len()).copied().collect()
}

/// Follows the boundaries of `regions` on the chosen side, one loop after the other, each
/// down through every depth pass before the next.
pub fn contour(regions: &[Region2], tool: &Tool, params: &ContourParams, cut: &Cut) -> Result<Toolpath> {
    tool.validate()?;
    mills(tool)?;
    cut.validate()?;
    check_reach(tool, cut)?;
    check_leave(params.stock_to_leave)?;
    check_ramp(params.ramp_angle)?;
    if let Some(t) = &params.tabs {
        t.validate(&cut.heights)?;
    }
    let shape = Shape::from_regions(regions, params.tolerance)?;
    // A tool that narrows to its tip (a ball below its equator, a V-bit) is widest at the top of
    // the cut: offsetting by that half-width puts the top edge of the wall on the profile.
    let away = tool.radius_at(cut.heights.top - cut.heights.bottom) + params.stock_to_leave;
    let mut loops = match params.side {
        Side::Outside => shape.offset(-away)?,
        Side::Inside => shape.offset(away)?,
        Side::On => shape.loops().to_vec(),
    };
    if loops.is_empty() {
        return invalid("the tool does not fit the profile");
    }
    if let Some(rest) = params.rest {
        let r = tool.radius_at(cut.heights.top - cut.heights.bottom);
        rest.validate(r)?;
        // Where the earlier tool's centre ran, on the same side.
        let earlier = match params.side {
            Side::Outside => shape.offset(-rest.wall)?,
            Side::Inside => shape.offset(rest.wall)?,
            Side::On => Vec::new(),
        };
        let chains: Vec<(Vec<Piece>, bool)> = loops.iter().flat_map(|lp| rest_parts(lp, &earlier, rest.radius - r, r * REST_SAMPLE)).collect();
        return cut_rest(&chains, &shape, away, r, cut);
    }
    // A cut through the stock frees what a loop encloses: smaller loops first, so every hole is
    // cut while the part round it is still held by its uncut outline.
    let area = |lp: &[Piece]| lp.iter().map(Piece::area_term).sum::<f64>().abs();
    loops.sort_by(|a, b| area(a).total_cmp(&area(b)));
    // Loops come with the regions on their left. Around a part that puts the wall on the left
    // (conventional), inside an opening on the right (climb).
    let natural_is_climb = params.side != Side::Outside;
    let levels = cut.levels()?;
    let mut b = Builder::new(cut);
    for natural in &loops {
        // The material is on the left of a loop round a part, on the right of one inside an opening.
        let material_left = params.side == Side::Outside;
        let lp = match params.side {
            Side::On => natural.clone(),
            _ if params.dogbones => with_dogbones(natural, away, material_left, &shape),
            _ => natural.clone(),
        };
        let lp = if params.climb == natural_is_climb { lp } else { reversed(&lp) };
        // A loop with the regions on its right lies inside another one: a part's hole.
        let inner = natural.iter().map(Piece::area_term).sum::<f64>() < 0.0;
        let planned = params.tabs.filter(|t| t.inner || !inner).and_then(|t| plan_tabs(&lp, &t, tool.diameter, cut.heights.bottom));
        let (lp, tabs) = match planned {
            Some((lp, terrain)) => (lp, Some(terrain)),
            None => (lp, None),
        };
        let Some(first) = lp.first() else { continue };
        b.travel_to(first.start())?;
        // Down to the last pass (through what it cut), then into the material along the loop.
        b.plunge_to(cut.heights.top)?;
        for &z in &levels {
            b.ramp_loop(&lp, z, params.ramp_angle, tabs.as_ref())?;
            b.cut_loop(&lp, z, tabs.as_ref())?;
            b.cut_to(first.start(), z)?;
        }
        b.retract()?;
    }
    b.finish()
}

/// `lp` with a dogbone at every corner where it turns away from the material: out along the
/// corner's bisector until the tool, `away` from the wall, is that far from the corner too, and
/// back. Very sharp corners (where that would reach out more than four times `away`) are left,
/// and so is a bend that is no corner of `shape`: where the offsets of two walls that do not
/// meet cross (two parts, or an island and a wall, closer than the tool), a notch would cut
/// into one of them.
fn with_dogbones(lp: &[Piece], away: f64, material_left: bool, shape: &Shape) -> Vec<Piece> {
    let free = |t: Vec2| if material_left { -t.perp() } else { t.perp() };
    let mut out = Vec::with_capacity(lp.len() * 2);
    for (piece, next) in lp.iter().zip(lp.iter().cycle().skip(1)) {
        out.push(*piece);
        let (t1, t2) = (piece.tangent_at(1.0), next.tangent_at(0.0));
        let bend = turn(t1, t2);
        if bend.abs() < DOGBONE_MIN_TURN || (bend > 0.0) == material_left {
            continue;
        }
        // The tool centre sits `away` from both walls: it is `x` from the corner they meet in.
        let (n1, n2) = (free(t1), free(t2));
        let k = 1.0 + n1.dot(n2);
        if !(k > 1e-9) {
            continue;
        }
        let x = (n1 + n2) * (away / k);
        let reach = x.len() - away;
        let (Some(dir), true) = (x.normalized(), reach > 1e-6 && reach <= 4.0 * away) else { continue };
        let c = piece.end();
        let tip = c - dir * reach;
        if !is_wall_corner(shape, c - x, c, tip, away) {
            continue;
        }
        out.push(Piece::Line { a: c, b: tip });
        out.push(Piece::Line { a: tip, b: c });
    }
    out
}

/// Whether `corner` is where two walls of `shape` meet, and the notch from `c` out to `tip` stays
/// `away` from every other wall: only the walls through the corner may come closer.
fn is_wall_corner(shape: &Shape, corner: Vec2, c: Vec2, tip: Vec2, away: f64) -> bool {
    const ON: f64 = 1e-6;
    if !(shape.dist(corner) <= ON) {
        return false;
    }
    let others = || shape.loops().iter().flatten().filter(|w| !(w.dist(corner) <= ON));
    (0..=8).map(|k| c + (tip - c) * (k as f64 / 8.0)).all(|q| others().all(|w| w.dist(q) >= away - ON))
}

/// The tabs of one loop: stretches (arc length from the loop's start) where the tool stays at
/// `top` on the passes below it, coming down at 45° either side.
pub(crate) struct Terrain {
    flats: Vec<(f64, f64)>,
    top: f64,
}

impl Terrain {
    /// Where the floor bends: the ends of every tab's flat, and the middle between two tabs where
    /// the ramp down from one meets the ramp up to the next. Between two of them the floor is
    /// straight, so a pass's depth crosses it at most once.
    fn kinks(&self) -> Vec<f64> {
        let mut flats = self.flats.clone();
        flats.sort_by(|x, y| x.0.total_cmp(&y.0));
        let mut out: Vec<f64> = flats.iter().flat_map(|&(a, b)| [a, b]).collect();
        out.extend(flats.windows(2).map(|w| (w[0].1 + w[1].0) / 2.0));
        out
    }

    /// The lowest the tool may go at `s` along the loop.
    fn floor(&self, s: f64) -> f64 {
        self.flats.iter().map(|&(a, b)| self.top - (a - s).max(s - b).max(0.0)).fold(f64::NEG_INFINITY, f64::max)
    }
}

/// Where `lp` carries its tabs, with the loop turned to start in the widest gap between them, so
/// each pass ramps in where no tab stands. About one every `spacing`, at least two, each moved
/// onto the nearest straight piece that carries its whole footprint (not over a corner or
/// round a curve) when one is near. `None` for a loop too short for them.
fn plan_tabs(lp: &[Piece], tabs: &Tabs, diameter: f64, bottom: f64) -> Option<(Vec<Piece>, Terrain)> {
    let total: f64 = lp.iter().map(Piece::length).sum();
    // The tool's centre stays up the tab's width plus its own, and comes down over its height.
    let flat = tabs.width + diameter;
    let foot = flat + 2.0 * tabs.height;
    if !(total >= 4.0 * foot) {
        return None;
    }
    let n = ((total / tabs.spacing).ceil() as usize).clamp(2, MAX_TABS);
    let share = total / n as f64;
    let mut room = Vec::new();
    let mut s = 0.0;
    for p in lp {
        let len = p.length();
        if matches!(p, Piece::Line { .. }) && len >= foot {
            room.push((s + foot / 2.0, s + len - foot / 2.0));
        }
        s += len;
    }
    let mut centres: Vec<f64> = (0..n)
        .map(|k| {
            let want = (k as f64 + 0.5) * share;
            room.iter()
                .map(|&(a, b)| want.clamp(a, b))
                .filter(|c| (c - want).abs() <= share / 2.0)
                .min_by(|x, y| (x - want).abs().total_cmp(&(y - want).abs()))
                .unwrap_or(want)
        })
        .collect();
    centres.sort_by(f64::total_cmp);
    centres.dedup_by(|later, kept| *later - *kept < foot);
    if centres.len() > 1 && centres[0] + total - centres[centres.len() - 1] < foot {
        centres.pop();
    }
    // The widest gap between footprints, going round.
    let (gap, _) = centres
        .iter()
        .enumerate()
        .map(|(i, &c)| (i, centres.get(i + 1).map_or(centres[0] + total, |&d| d) - c))
        .max_by(|a, b| a.1.total_cmp(&b.1))?;
    let start = (centres[gap] + foot / 2.0).rem_euclid(total);
    let flats = centres
        .iter()
        .map(|&c| {
            let c = (c - start).rem_euclid(total);
            (c - flat / 2.0, c + flat / 2.0)
        })
        .collect();
    Some((start_at(lp, start), Terrain { flats, top: bottom + tabs.height }))
}

/// The loop starting `s` along it, the piece there split in two.
fn start_at(lp: &[Piece], s: f64) -> Vec<Piece> {
    let mut acc = 0.0;
    for (i, p) in lp.iter().enumerate() {
        let len = p.length();
        if s < acc + len || i + 1 == lp.len() {
            let t = if len > 0.0 { ((s - acc) / len).clamp(0.0, 1.0) } else { 0.0 };
            let mut out = Vec::with_capacity(lp.len() + 1);
            if t < 1.0 - 1e-12 {
                out.push(p.sub(t, 1.0));
            }
            out.extend_from_slice(&lp[i + 1..]);
            out.extend_from_slice(&lp[..i]);
            if t > 1e-12 {
                out.push(p.sub(0.0, t));
            }
            return out;
        }
        acc += len;
    }
    lp.to_vec()
}

/// Clears `regions` (holes are islands) with rings parallel to the boundary, from the inside out.
pub fn pocket(regions: &[Region2], tool: &Tool, params: &PocketParams, cut: &Cut) -> Result<Toolpath> {
    tool.validate()?;
    mills(tool)?;
    cut.validate()?;
    check_reach(tool, cut)?;
    check_leave(params.stock_to_leave)?;
    check_ramp(params.ramp_angle)?;
    // The floor is cleared by the tip: a tool that narrows to a point cannot clear one.
    let floor = tool.floor_diameter();
    if !(floor > 0.0) {
        return invalid("a V-bit without a tip flat cannot clear a pocket: give it a `tip_diameter`, use a flat end mill, or engrave the outline");
    }
    let r = floor * 0.5;
    if !(params.stepover > 0.0) || !(params.stepover <= floor) {
        return invalid(if floor == tool.diameter {
            "the stepover must be above 0 and at most the tool diameter".to_string()
        } else {
            format!("the stepover must be above 0 and at most the tool's width at its tip ({floor:.3} mm)")
        });
    }
    let shape = Shape::from_regions(regions, params.tolerance)?;
    let wall = tool.radius_at(cut.heights.top - cut.heights.bottom) + params.stock_to_leave;
    let mut rings: Vec<Vec<Vec<Piece>>> = Vec::new();
    loop {
        let ring = shape.offset(wall + params.stepover * rings.len() as f64)?;
        if ring.is_empty() {
            break;
        }
        rings.push(ring);
        if rings.len() > MAX_RINGS {
            return Err(CamError::TooLarge(format!("the pocket needs more than {MAX_RINGS} passes; use a larger stepover")));
        }
    }
    if rings.is_empty() {
        return invalid("the tool does not fit the pocket");
    }
    if let Some(rest) = params.rest {
        rest.validate(tool.radius_at(cut.heights.top - cut.heights.bottom))?;
        // Only the band between this wall and the earlier one holds anything: inside it the
        // earlier tool cleared everything.
        let earlier = shape.offset(rest.wall)?;
        let band = rings.iter().enumerate().filter(|(k, _)| wall + params.stepover * *k as f64 <= rest.wall + 1e-9).flat_map(|(_, ring)| ring);
        let chains: Vec<(Vec<Piece>, bool)> = band.flat_map(|lp| rest_parts(lp, &earlier, rest.radius - r, r * REST_SAMPLE)).collect();
        return cut_rest(&chains, &shape, wall, r, cut);
    }
    // Rings more than a radius apart leave material wherever the next ring in does not run
    // within a radius: the middle of a part of the pocket it no longer fits in, a passage that
    // narrows. A ring a radius further in clears it; it is cut where some stretch of it is out
    // of the next ring's reach (corners between rings are cleared by the spurs below).
    if params.stepover > r + 1e-9 {
        let mut passes = Vec::new();
        for (k, ring) in rings.iter().enumerate() {
            passes.push(ring.clone());
            let next: Vec<&Piece> = rings.get(k + 1).into_iter().flatten().flatten().collect();
            let reached = |p: Vec2| next.iter().any(|piece| piece.dist(p) <= r + 1e-6);
            let extra: Vec<Vec<Piece>> = shape
                .offset(wall + params.stepover * k as f64 + r)?
                .into_iter()
                .filter(|lp| lp.iter().any(|piece| [0.25, 0.5, 0.75].iter().any(|t| !reached(piece.point_at(*t)))))
                .collect();
            if !extra.is_empty() {
                passes.push(extra);
            }
        }
        rings = passes;
    }
    // The ring along the walls (the pocket on its left, the material on its right).
    if params.dogbones
        && let Some(walls) = rings.first_mut()
    {
        *walls = walls.iter().map(|lp| with_dogbones(lp, wall, false, &shape)).collect();
    }
    let mut b = Builder::new(cut);
    // The depth already cut wherever this pass's rings run.
    let mut cleared = cut.heights.top;
    for z in cut.levels()? {
        // Whether the tool is down in this level's cut.
        let mut down: Option<Vec2> = None;
        for lp in rings.iter().rev().flatten() {
            // Offset loops keep the pocket on their left, so the wall is on the right: climb.
            let lp = if params.climb { lp.clone() } else { reversed(lp) };
            let lp = start_near(&lp, down.unwrap_or_default());
            let Some(first) = lp.first() else { continue };
            let start = first.start();
            match down {
                Some(from) if link_is_clear(&shape, from, start, wall, r) => b.cut_to(start, z)?,
                _ => {
                    b.travel_to(start)?;
                    b.plunge_to(cleared)?;
                    b.ramp_loop(&lp, z, params.ramp_angle, None)?;
                }
            }
            // Rings far apart do not reach into a sharp corner between them: there the tool
            // steps towards the next ring's corner and back.
            let inward = if params.climb { 1.0 } else { -1.0 };
            for (piece, next) in lp.iter().zip(lp.iter().cycle().skip(1)) {
                b.cut_piece(piece, z, z)?;
                if let Some(to) = corner_spur(piece, next, inward, params.stepover, r).filter(|to| link_is_clear(&shape, piece.end(), *to, wall, r)) {
                    b.cut_to(to, z)?;
                    b.cut_to(piece.end(), z)?;
                }
            }
            b.cut_to(start, z)?;
            down = Some(start);
        }
        cleared = z;
        b.retract()?;
    }
    b.finish()
}

/// The stretches of `lp` where a tool `gap` smaller than the earlier one still meets material
/// worth cutting: more than `gap` (and a hair) from the earlier tool's path `earlier`, which
/// cleared everything within its radius of itself. Each stretch is a chain of pieces; `true` when it is the whole loop.
fn rest_parts(lp: &[Piece], earlier: &[Vec<Piece>], gap: f64, sample: f64) -> Vec<(Vec<Piece>, bool)> {
    let left = |p: Vec2| earlier.iter().flatten().map(|w| w.dist(p)).fold(f64::INFINITY, f64::min) > gap + REST_MIN;
    let mut chains: Vec<Vec<Piece>> = Vec::new();
    let mut open = false;
    let mut all = true;
    for piece in lp {
        let n = ((piece.length() / sample.max(1e-3)).ceil() as usize).clamp(1, 10_000);
        let kept: Vec<bool> = (0..=n).map(|i| left(piece.point_at(i as f64 / n as f64))).collect();
        let mut i = 0;
        while i <= n {
            if !kept[i] {
                all = false;
                open = false;
                i += 1;
                continue;
            }
            let start = i;
            while i < n && kept[i + 1] {
                i += 1;
            }
            // Half a sample beyond the last kept point either way, so nothing is left for want
            // of a sample; the run within [0, 1].
            let (t0, t1) = (((start as f64 - 0.5) / n as f64).max(0.0), ((i as f64 + 0.5) / n as f64).min(1.0));
            if t1 - t0 > 1e-9 {
                let part = piece.sub(t0, t1);
                if open && start == 0 {
                    if let Some(last) = chains.last_mut() {
                        last.push(part);
                    }
                } else {
                    chains.push(vec![part]);
                }
                open = i == n;
            }
            i += 1;
        }
    }
    if all && !chains.is_empty() {
        let whole: Vec<Piece> = chains.into_iter().flatten().collect();
        return vec![(whole, true)];
    }
    // A run over the loop's seam (end of the last piece into the start of the first) is one chain.
    if chains.len() > 1 && open && chains[0].first().map(Piece::start) == chains[chains.len() - 1].last().map(Piece::end) {
        let first = chains.remove(0);
        if let Some(last) = chains.last_mut() {
            last.extend(first);
        }
    }
    chains.into_iter().map(|c| (c, false)).collect()
}

/// Cuts `chains` at every depth pass: down into each one, along it, up and across to the next,
/// or straight across where the way is clear of the walls.
fn cut_rest(chains: &[(Vec<Piece>, bool)], shape: &Shape, wall: f64, r: f64, cut: &Cut) -> Result<Toolpath> {
    if chains.is_empty() {
        return Err(CamError::Nothing("the earlier tool left nothing for this one to cut".into()));
    }
    let mut b = Builder::new(cut);
    let mut cleared = cut.heights.top;
    for z in cut.levels()? {
        let mut down: Option<Vec2> = None;
        for (chain, closed) in chains {
            let Some(first) = chain.first() else { continue };
            let start = first.start();
            match down {
                Some(from) if link_is_clear(shape, from, start, wall, r) => b.cut_to(start, z)?,
                _ => {
                    b.travel_to(start)?;
                    b.plunge_to(cleared)?;
                    b.plunge_to(z)?;
                }
            }
            for piece in chain {
                b.cut_piece(piece, z, z)?;
            }
            if *closed {
                b.cut_to(start, z)?;
            }
            down = b.position().map(|q| Vec2::new(q.x, q.y));
        }
        cleared = z;
        b.retract()?;
    }
    b.finish()
}

/// Where to step to from the corner between `piece` and `next` so that the corner between
/// this ring and the next one in (`gap` away) is cleared: along the corner's bisector, to within
/// a radius of the next ring's corner. `inward`: 1 with the pocket on the loop's left, −1 on
/// its right. `None` where the rings' own passes already clear the corner.
fn corner_spur(piece: &Piece, next: &Piece, inward: f64, gap: f64, r: f64) -> Option<Vec2> {
    let (t_in, t_out) = (piece.tangent_at(1.0), next.tangent_at(0.0));
    let bend = crate::piece::turn(t_in, t_out);
    // Half the corner's inside angle has this sine.
    let sharp = (bend.abs() * 0.5).cos();
    if bend * inward <= 1e-6 || sharp < 0.05 || gap <= r * (1.0 + sharp) + 1e-9 {
        return None;
    }
    let dir = (t_out - t_in).normalized()?;
    Some(piece.end() + dir * (gap / sharp - r))
}

/// Whether the tool can feed straight from `a` to `b` without touching a wall.
fn link_is_clear(shape: &Shape, a: Vec2, b: Vec2, wall: f64, r: f64) -> bool {
    let n = ((a.dist(b) / (r * 0.25).max(1e-3)).ceil()).clamp(1.0, 2000.0) as usize;
    (0..=n).all(|i| shape.clear(a.lerp(b, i as f64 / n as f64), wall))
}
