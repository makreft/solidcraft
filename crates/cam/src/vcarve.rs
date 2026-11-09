//! V-carving: a V-bit follows the inward offsets of a region, each as deep as the cone must go
//! to touch the boundary there. Every edge comes out sharp at the top, however fine; where the
//! offsets meet in the middle, a ridge. Rings deeper than the cut's bottom stay at the bottom:
//! the tip clears a flat floor there.

use serde::{Deserialize, Serialize};
use solidcraft_geom::{Region2, Vec2};

use crate::offset::Shape;
use crate::ops::{Builder, Cut, check_reach, start_near};
use crate::path::Toolpath;
use crate::piece::Piece;
use crate::tool::{Tool, ToolKind};
use crate::{CamError, Result, invalid};

/// Most offset rings of one carving.
const MAX_RINGS: usize = 20_000;
/// Rings this close (in stepovers) are linked with a cut, not a retract and a travel.
const LINK_STEPOVERS: f64 = 2.5;

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct VCarveParams {
    /// Distance between the offset passes (mm); the finer, the smoother the carved walls.
    pub stepover: f64,
    /// Chord error allowed on spline boundaries (mm).
    pub tolerance: f64,
}

/// What a carving comes to.
#[derive(Clone, Debug, PartialEq)]
pub struct VCarve {
    pub path: Toolpath,
    /// The deepest the tool goes (work Z).
    pub deepest: f64,
    /// How deep the carving would go below the top if nothing held it: the cone's depth at the
    /// last ring that still fits. Above the cut's depth when the floor was flattened.
    pub natural_depth: f64,
    /// Whether any ring was held at the cut's bottom (so the tip cleared a flat floor there).
    pub flat_floor: bool,
    pub rings: usize,
}

/// Carves `regions` with the V-bit `tool`: inward offsets `stepover` apart, from the boundary in,
/// each at the depth where the cone's edge meets the boundary, never below the cut's bottom.
pub fn vcarve(regions: &[Region2], tool: &Tool, params: &VCarveParams, cut: &Cut) -> Result<VCarve> {
    tool.validate()?;
    if tool.kind != ToolKind::VBit {
        return invalid("v-carving needs a V-bit: a tool whose depth sets its width");
    }
    cut.validate()?;
    check_reach(tool, cut)?;
    if !(params.stepover >= 0.01) || !(params.stepover <= 1000.0) {
        return invalid("the stepover must be at least 0.01 mm");
    }
    let tan_half = (tool.tip_angle.unwrap_or(0.0).to_radians() * 0.5).tan();
    if !(tan_half > 1e-9) || !tan_half.is_finite() {
        return invalid("the V-bit's tip angle gives it no width");
    }
    let shape = Shape::from_regions(regions, params.tolerance)?;
    let (top, bottom) = (cut.heights.top, cut.heights.bottom);
    let tip = tool.tip_diameter * 0.5;
    // Every ring with its depth, from the boundary inwards.
    let mut rings: Vec<(Vec<Vec<Piece>>, f64)> = Vec::new();
    let mut natural_depth = 0.0;
    let mut flat_floor = false;
    loop {
        let d = tip + params.stepover * (rings.len() + 1) as f64;
        let loops = shape.offset(d)?;
        if loops.is_empty() {
            break;
        }
        natural_depth = (d - tip) / tan_half;
        let z = if top - natural_depth < bottom {
            flat_floor = true;
            bottom
        } else {
            top - natural_depth
        };
        rings.push((loops, z));
        if rings.len() > MAX_RINGS {
            return Err(CamError::TooLarge(format!("the carving needs more than {MAX_RINGS} passes; use a larger stepover")));
        }
    }
    if rings.is_empty() {
        return invalid("the tool does not fit the profile: nothing is wider than its tip");
    }
    let deepest = rings.iter().map(|r| r.1).fold(top, f64::min);
    let mut b = Builder::new(cut);
    // Where the tool is down in the material, so the next ring can be reached with a cut.
    let mut down: Option<Vec2> = None;
    for (loops, z) in &rings {
        for lp in loops {
            let lp = start_near(lp, down.unwrap_or_default());
            let Some(first) = lp.first() else { continue };
            let start = first.start();
            match down {
                Some(from) if from.dist(start) <= LINK_STEPOVERS * params.stepover => b.cut_to(start, *z)?,
                _ => {
                    b.travel_to(start)?;
                    b.plunge_to(*z)?;
                }
            }
            b.cut_loop(&lp, *z, None)?;
            b.cut_to(start, *z)?;
            down = Some(start);
        }
    }
    b.retract()?;
    Ok(VCarve { path: b.finish()?, deepest, natural_depth, flat_floor, rings: rings.len() })
}
