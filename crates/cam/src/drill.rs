//! Drilling: straight down at each hole, in pecks when asked, with a pause at the bottom. Any
//! tool drills; a V-bit with a depth of its own at every hole makes halftone dots.

use serde::{Deserialize, Serialize};
use solidcraft_geom::{Vec2, Vec3};

use crate::ops::{Builder, Cut, MAX_COORD, check_reach};
use crate::path::{Move, Toolpath};
use crate::tool::Tool;
use crate::{CamError, Result, invalid};

/// Most holes of one drilling operation.
pub const MAX_HOLES: usize = 50_000;
/// A peck re-enters its hole this far above where the last one ended (mm): a rapid down a hole
/// stops above what was cut, and the chips are gone from there.
const PECK_CLEARANCE: f64 = 0.1;
/// Up to this many holes are ordered by the nearest next one; more go row by row.
const GREEDY_LIMIT: usize = 2_000;

/// One hole: where, and how deep its bottom is (work Z, mm).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Hole {
    pub at: Vec2,
    pub bottom: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct DrillParams {
    /// Depth per peck (mm): the tool comes up to the retract height between pecks, so the chips
    /// clear. None: each hole in one go.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub peck: Option<f64>,
    /// Pause at the bottom of every hole (seconds); 0 for none.
    #[serde(default)]
    pub dwell: f64,
}

/// Drills `holes`, the nearest next: rapid over at the clearance height and down to the retract
/// height, feed to the bottom (in pecks, each back up to the retract height), a dwell, and up.
pub fn drill(holes: &[Hole], tool: &Tool, params: &DrillParams, cut: &Cut) -> Result<Toolpath> {
    tool.validate()?;
    cut.validate()?;
    check_reach(tool, cut)?;
    if holes.is_empty() {
        return invalid("there is no hole to drill");
    }
    if holes.len() > MAX_HOLES {
        return Err(CamError::TooLarge(format!("more than {MAX_HOLES} holes")));
    }
    if let Some(peck) = params.peck
        && !(peck > 0.0 && peck <= MAX_COORD)
    {
        return invalid("the peck must be above 0");
    }
    if !(params.dwell >= 0.0 && params.dwell <= 3600.0) {
        return invalid("the dwell must be 0 seconds or longer, at most an hour");
    }
    let (top, bottom) = (cut.heights.top, cut.heights.bottom);
    for h in holes {
        if !h.at.is_finite() || h.at.x.abs() > MAX_COORD || h.at.y.abs() > MAX_COORD || !h.bottom.is_finite() {
            return invalid("a hole has a coordinate that is not a finite number");
        }
        if !(h.bottom < top) || h.bottom < bottom - 1e-9 {
            return invalid("a hole's bottom must be below the top and not below the operation's bottom");
        }
    }
    let mut b = Builder::new(cut);
    for i in order(holes) {
        let Some(h) = holes.get(i) else { continue };
        b.travel_to(h.at)?;
        b.rapid_to_retract()?;
        let mut z = top;
        loop {
            let next = match params.peck {
                Some(peck) => (z - peck).max(h.bottom),
                None => h.bottom,
            };
            if z < top {
                // Back into the hole after a peck: at the rapid rate to just above the chips.
                b.push(Move::Rapid { to: Vec3::new(h.at.x, h.at.y, (z + PECK_CLEARANCE).min(cut.heights.retract)) })?;
            }
            b.push(Move::Feed { to: Vec3::new(h.at.x, h.at.y, next), feed: cut.feeds.plunge })?;
            z = next;
            if z <= h.bottom + 1e-9 {
                break;
            }
            b.push(Move::Rapid { to: Vec3::new(h.at.x, h.at.y, cut.heights.retract) })?;
        }
        if params.dwell > 0.0 {
            b.push(Move::Dwell { seconds: params.dwell })?;
        }
        b.retract()?;
    }
    b.finish()
}

/// The holes in drilling order: from the first, always the nearest next. Many holes (a halftone
/// grid) go row by row instead, each row the other way, which is about as short and quick to plan.
fn order(holes: &[Hole]) -> Vec<usize> {
    if holes.len() <= GREEDY_LIMIT {
        let mut left: Vec<usize> = (1..holes.len()).collect();
        let mut out = vec![0];
        let mut at = holes.first().map_or(Vec2::ZERO, |h| h.at);
        while !left.is_empty() {
            let Some((k, &i)) = left.iter().enumerate().min_by(|a, b| holes[*a.1].at.dist(at).total_cmp(&holes[*b.1].at.dist(at))) else { break };
            left.swap_remove(k);
            at = holes[i].at;
            out.push(i);
        }
        return out;
    }
    let (lo, hi) = holes.iter().fold((Vec2::new(f64::INFINITY, f64::INFINITY), Vec2::new(f64::NEG_INFINITY, f64::NEG_INFINITY)), |(lo, hi), h| {
        (Vec2::new(lo.x.min(h.at.x), lo.y.min(h.at.y)), Vec2::new(hi.x.max(h.at.x), hi.y.max(h.at.y)))
    });
    // Rows about one hole's pitch high, as if the holes were on a square grid.
    let band = (((hi.x - lo.x) * (hi.y - lo.y) / holes.len() as f64).sqrt()).max(1e-6);
    let row = |h: &Hole| ((h.at.y - lo.y) / band).floor() as i64;
    let mut out: Vec<usize> = (0..holes.len()).collect();
    out.sort_by(|&a, &b| {
        let (ra, rb) = (row(&holes[a]), row(&holes[b]));
        ra.cmp(&rb).then_with(|| {
            let (xa, xb) = (holes[a].at.x, holes[b].at.x);
            if ra % 2 == 0 { xa.total_cmp(&xb) } else { xb.total_cmp(&xa) }
        })
    });
    out
}
