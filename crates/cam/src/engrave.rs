//! Engraving: the tool centre follows curves, open or closed, at fixed depth passes. Nothing is
//! offset, so a curve needs no area: a single line, a lip line, the strokes of a letter.

use serde::{Deserialize, Serialize};
use solidcraft_geom::Vec2;

use crate::ops::{Builder, Cut, MAX_COORD, check_reach, mills};
use crate::path::Toolpath;
use crate::tool::Tool;
use crate::{CamError, Result, invalid};

/// Most curves of one engraving.
pub const MAX_CHAINS: usize = 10_000;
/// Most points of all its curves together.
pub const MAX_CHAIN_POINTS: usize = 200_000;

/// A curve to engrave, as the points the tool centre goes through in order. A closed chain comes
/// back to its first point; an open one ends at its last.
#[derive(Clone, Debug, PartialEq)]
pub struct Chain {
    pub points: Vec<Vec2>,
    pub closed: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct EngraveParams {
    /// Ends closer than this are joined into one chain (mm), so a curve drawn in pieces is
    /// engraved in one go.
    pub tolerance: f64,
}

/// Engraves `chains`: each one down through every depth pass before the next, the chains in the
/// order of the shortest way between them. An open chain is cut back along itself on the next
/// pass, so the tool never travels; a closed one goes round again from its start.
pub fn engrave(chains: &[Chain], tool: &Tool, params: &EngraveParams, cut: &Cut) -> Result<Toolpath> {
    tool.validate()?;
    mills(tool)?;
    cut.validate()?;
    check_reach(tool, cut)?;
    if !(params.tolerance > 0.0) || !(params.tolerance <= MAX_COORD) {
        return invalid("the joining tolerance must be above 0");
    }
    if chains.len() > MAX_CHAINS {
        return Err(CamError::TooLarge(format!("more than {MAX_CHAINS} curves to engrave")));
    }
    if chains.iter().map(|c| c.points.len()).sum::<usize>() > MAX_CHAIN_POINTS {
        return Err(CamError::TooLarge(format!("more than {MAX_CHAIN_POINTS} points to engrave")));
    }
    let mut clean = Vec::with_capacity(chains.len());
    for chain in chains {
        let mut pts = Vec::with_capacity(chain.points.len());
        for &p in &chain.points {
            if !p.is_finite() || p.x.abs() > MAX_COORD || p.y.abs() > MAX_COORD {
                return invalid("a curve has a coordinate that is not a finite number");
            }
            if !pts.last().is_some_and(|q: &Vec2| q.dist(p) <= 1e-9) {
                pts.push(p);
            }
        }
        if chain.closed && pts.len() > 1 && pts[0].dist(pts[pts.len() - 1]) <= 1e-9 {
            pts.pop();
        }
        if pts.len() < 2 || (chain.closed && pts.len() < 3) {
            return invalid("a curve to engrave needs at least 2 points (3 when closed)");
        }
        clean.push(Chain { points: pts, closed: chain.closed });
    }
    if clean.is_empty() {
        return invalid("there is no curve to engrave");
    }
    let ordered = order(join_chains(clean, params.tolerance));
    let levels = cut.levels()?;
    let mut b = Builder::new(cut);
    for chain in &ordered {
        let Some(&start) = chain.points.first() else { continue };
        b.travel_to(start)?;
        b.plunge_to(cut.heights.top)?;
        let mut forward = true;
        for &z in &levels {
            // Straight down from the pass before, where the tool stands.
            b.plunge_to(z)?;
            if chain.closed {
                for &p in chain.points.iter().skip(1) {
                    b.cut_to(p, z)?;
                }
                b.cut_to(start, z)?;
            } else if forward {
                for &p in chain.points.iter().skip(1) {
                    b.cut_to(p, z)?;
                }
                forward = false;
            } else {
                for &p in chain.points.iter().rev().skip(1) {
                    b.cut_to(p, z)?;
                }
                forward = true;
            }
        }
        b.retract()?;
    }
    b.finish()
}

fn reversed(chain: &Chain) -> Chain {
    Chain { points: chain.points.iter().rev().copied().collect(), closed: chain.closed }
}

/// Open chains whose ends meet within `tol` become one chain; one that comes back to its own
/// start is closed. Closed chains stay as they are. The chains keep the order they were given.
pub fn join_chains(chains: Vec<Chain>, tol: f64) -> Vec<Chain> {
    let mut slots: Vec<Option<Chain>> = chains.into_iter().map(Some).collect();
    let mut out = Vec::with_capacity(slots.len());
    for i in 0..slots.len() {
        let Some(mut current) = slots[i].take() else { continue };
        if current.closed {
            out.push(current);
            continue;
        }
        // Grow at the end; when nothing fits there, turn round and grow at the other end.
        for _ in 0..2 {
            while let Some(&end) = current.points.last() {
                let touches = |c: &Chain| {
                    !c.closed && (c.points.first().is_some_and(|p| p.dist(end) <= tol) || c.points.last().is_some_and(|p| p.dist(end) <= tol))
                };
                let Some(j) = slots.iter().position(|slot| slot.as_ref().is_some_and(touches)) else { break };
                let Some(piece) = slots[j].take() else { break };
                let piece = if piece.points.first().is_some_and(|p| p.dist(end) <= tol) { piece } else { reversed(&piece) };
                let skip = usize::from(piece.points.first().is_some_and(|p| p.dist(end) <= 1e-9));
                current.points.extend(piece.points.into_iter().skip(skip));
            }
            current = reversed(&current);
        }
        if current.points.len() >= 3 {
            let (first, last) = (current.points[0], current.points[current.points.len() - 1]);
            if first.dist(last) <= tol {
                if first.dist(last) <= 1e-9 {
                    current.points.pop();
                }
                if current.points.len() >= 3 {
                    current.closed = true;
                }
            }
        }
        out.push(current);
    }
    out
}

/// The chains in the order of the shortest way from one to the next, starting with the first:
/// an open chain is reversed when its end is nearer, a closed one turned to start at its
/// nearest point.
fn order(mut chains: Vec<Chain>) -> Vec<Chain> {
    let mut out = Vec::with_capacity(chains.len());
    let mut at: Option<Vec2> = None;
    while !chains.is_empty() {
        let i = match at {
            None => 0,
            Some(from) => chains
                .iter()
                .enumerate()
                .map(|(i, c)| {
                    let d = if c.closed { c.points.first().map_or(f64::INFINITY, |p| p.dist(from)) } else { entry_distance(c, from) };
                    (i, d)
                })
                .min_by(|a, b| a.1.total_cmp(&b.1))
                .map_or(0, |(i, _)| i),
        };
        let mut chain = chains.swap_remove(i);
        if let Some(from) = at {
            if chain.closed {
                let nearest = chain.points.iter().enumerate().min_by(|a, b| a.1.dist(from).total_cmp(&b.1.dist(from))).map_or(0, |(k, _)| k);
                chain.points.rotate_left(nearest);
            } else if chain.points.last().is_some_and(|end| end.dist(from) < chain.points[0].dist(from)) {
                chain = reversed(&chain);
            }
        }
        at = Some(if chain.closed { chain.points[0] } else { chain.points[chain.points.len() - 1] });
        out.push(chain);
    }
    out
}

fn entry_distance(chain: &Chain, from: Vec2) -> f64 {
    let a = chain.points.first().map_or(f64::INFINITY, |p| p.dist(from));
    let b = chain.points.last().map_or(f64::INFINITY, |p| p.dist(from));
    a.min(b)
}
