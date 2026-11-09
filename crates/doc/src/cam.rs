//! CAM setups persisted on the design: tools, setups and 2D milling and engraving operations.
//! Toolpaths are never stored; they are computed when posting (`cam.post`).

use serde::{Deserialize, Serialize};
use solidcraft_cam::{Chain, ContourParams, Cut, DrillParams, EngraveParams, Feeds, Heights, Hole, PocketParams, Post, Side, Tool, VCarveParams};
use solidcraft_geom::{Loop2, Region2, Vec2};

use crate::{DocError, Result};

/// Most tools in one design.
pub const MAX_TOOLS: usize = 256;
/// Most setups in one design.
pub const MAX_SETUPS: usize = 64;
/// Most posts saved with one design.
pub const MAX_POSTS: usize = 32;
/// Most operations in one setup.
pub const MAX_OPS: usize = 256;
/// Most vertices on one polygon loop.
pub const MAX_LOOP_POINTS: usize = 10_000;
/// Most holes in one region.
pub const MAX_HOLES: usize = 1_000;
/// Most regions in one operation.
pub const MAX_REGIONS: usize = 256;
/// Most curves in one engraving.
pub const MAX_CHAINS: usize = solidcraft_cam::MAX_CHAINS;
/// Most holes in one drilling operation.
pub const MAX_DRILL_HOLES: usize = solidcraft_cam::MAX_HOLES;

/// The design's CAM data: tools, setups and optional named posts.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct CamSettings {
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub tools: Vec<Tool>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub setups: Vec<CamSetup>,
    /// Named posts saved with the design. Absent means the built-in RS-274 post at post time.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub posts: Vec<Post>,
}

impl CamSettings {
    pub fn is_empty(&self) -> bool {
        self.tools.is_empty() && self.setups.is_empty() && self.posts.is_empty()
    }

    pub fn tool(&self, number: u32) -> Option<&Tool> {
        self.tools.iter().find(|t| t.number == number)
    }

    pub fn setup(&self, name: &str) -> Option<&CamSetup> {
        self.setups.iter().find(|s| s.name == name)
    }

    pub fn setup_mut(&mut self, name: &str) -> Option<&mut CamSetup> {
        self.setups.iter_mut().find(|s| s.name == name)
    }

    pub fn post(&self, name: &str) -> Option<&Post> {
        self.posts.iter().find(|p| p.name == name)
    }

    /// The whole of it as a command would leave it: every tool, setup and post valid, and no
    /// two tools with one number, no two setups (or two operations of a setup, or two posts) with
    /// one name. A design file is read as it is; this is checked before anything is posted.
    pub fn validate(&self) -> Result<()> {
        let twice = |what: &str, name: String| Err(DocError::Invalid(format!("two CAM {what} are both {name}")));
        let mut numbers = std::collections::BTreeSet::new();
        for t in &self.tools {
            t.validate().map_err(|e| DocError::Invalid(e.to_string()))?;
            if !numbers.insert(t.number) {
                return twice("tools", format!("number {}", t.number));
            }
        }
        let mut names = std::collections::BTreeSet::new();
        for setup in &self.setups {
            setup.validate()?;
            if !names.insert(setup.name.as_str()) {
                return twice("setups", format!("`{}`", setup.name));
            }
            let mut ops = std::collections::BTreeSet::new();
            for op in &setup.operations {
                if !ops.insert(op.name()) {
                    return twice(&format!("operations of setup `{}`", setup.name), format!("`{}`", op.name()));
                }
            }
        }
        let mut posts = std::collections::BTreeSet::new();
        for post in &self.posts {
            if !posts.insert(post.name.as_str()) {
                return twice("posts", format!("`{}`", post.name));
            }
        }
        Ok(())
    }
}

/// One machining setup: work offset, optional stock box, and operations.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CamSetup {
    pub name: String,
    /// Work offset 1…6 (`G54`…`G59`).
    #[serde(default = "one")]
    pub work_offset: u8,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub stock: Option<StockBox>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub operations: Vec<CamOperation>,
}

fn one() -> u8 {
    1
}

impl CamSetup {
    pub fn new(name: impl Into<String>) -> CamSetup {
        CamSetup { name: name.into(), work_offset: 1, stock: None, operations: Vec::new() }
    }

    /// Refuses an operation whose rapid moves would run inside the stock: its clearance and
    /// retract heights must be above the stock's top.
    pub fn check_clear_of_stock(&self, op: &CamOperation) -> Result<()> {
        let Some(stock) = &self.stock else { return Ok(()) };
        let top = stock.max[2];
        let h = &op.cut().heights;
        for (what, z) in [("retract", h.retract), ("clearance", h.clearance)] {
            if !(z > top) {
                return Err(DocError::Invalid(format!(
                    "operation `{}`: its {what} height Z{z} is inside the stock of setup `{}`, whose top is Z{top}; give `{what}` above it",
                    op.name(),
                    self.name
                )));
            }
        }
        Ok(())
    }

    pub fn validate(&self) -> Result<()> {
        if self.name.is_empty() || self.name.len() > 128 {
            return Err(DocError::Invalid("a CAM setup name must be 1 to 128 characters".into()));
        }
        if !(1..=6).contains(&self.work_offset) {
            return Err(DocError::Invalid("the work offset must be 1 to 6 (G54 to G59)".into()));
        }
        if self.operations.len() > MAX_OPS {
            return Err(DocError::Invalid(format!("a CAM setup has at most {MAX_OPS} operations")));
        }
        if let Some(stock) = &self.stock {
            stock.validate()?;
        }
        for op in &self.operations {
            op.validate()?;
            self.check_clear_of_stock(op)?;
        }
        Ok(())
    }
}

/// Axis-aligned stock box in work coordinates (mm).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct StockBox {
    pub min: [f64; 3],
    pub max: [f64; 3],
}

impl StockBox {
    pub fn validate(&self) -> Result<()> {
        for (a, b) in self.min.iter().zip(self.max.iter()) {
            if !a.is_finite() || !b.is_finite() || !(b > a) {
                return Err(DocError::Invalid("stock min must be below max on each axis".into()));
            }
            if a.abs() > 1.0e6 || b.abs() > 1.0e6 {
                return Err(DocError::Invalid("stock coordinates must be within 1e6 mm".into()));
            }
        }
        Ok(())
    }
}

/// A planar region as polygon loops (outer CCW, holes CW), work XY in mm.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PolygonRegion {
    pub outer: Vec<[f64; 2]>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub holes: Vec<Vec<[f64; 2]>>,
}

impl PolygonRegion {
    pub fn validate(&self) -> Result<()> {
        validate_loop(&self.outer, "outer")?;
        if self.holes.len() > MAX_HOLES {
            return Err(DocError::Invalid(format!("a region has at most {MAX_HOLES} holes")));
        }
        for (i, hole) in self.holes.iter().enumerate() {
            validate_loop(hole, &format!("hole {i}"))?;
        }
        Ok(())
    }

    pub fn to_region(&self) -> Result<Region2> {
        self.validate()?;
        let outer = Loop2::polygon(&self.outer.iter().map(|p| Vec2::new(p[0], p[1])).collect::<Vec<_>>());
        let holes = self.holes.iter().map(|h| Loop2::polygon(&h.iter().map(|p| Vec2::new(p[0], p[1])).collect::<Vec<_>>())).collect();
        Ok(Region2 { outer, holes })
    }
}

fn validate_loop(pts: &[[f64; 2]], label: &str) -> Result<()> {
    if pts.len() < 3 || pts.len() > MAX_LOOP_POINTS {
        return Err(DocError::Invalid(format!("a {label} loop needs 3 to {MAX_LOOP_POINTS} points")));
    }
    if pts.iter().any(|p| !p[0].is_finite() || !p[1].is_finite() || p[0].abs() > 1.0e6 || p[1].abs() > 1.0e6) {
        return Err(DocError::Invalid(format!("a {label} loop has non-finite or huge coordinates")));
    }
    Ok(())
}

/// A curve to engrave: the points of its polyline in work coordinates (mm), open or closed back
/// to its first point.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PolylineChain {
    pub points: Vec<[f64; 2]>,
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub closed: bool,
}

impl PolylineChain {
    pub fn validate(&self) -> Result<()> {
        let least = if self.closed { 3 } else { 2 };
        if self.points.len() < least || self.points.len() > MAX_LOOP_POINTS {
            return Err(DocError::Invalid(format!("a curve to engrave needs {least} to {MAX_LOOP_POINTS} points")));
        }
        if self.points.iter().any(|p| !p[0].is_finite() || !p[1].is_finite() || p[0].abs() > 1.0e6 || p[1].abs() > 1.0e6) {
            return Err(DocError::Invalid("a curve to engrave has non-finite or huge coordinates".into()));
        }
        Ok(())
    }

    pub fn to_chain(&self) -> Chain {
        Chain { points: self.points.iter().map(|p| Vec2::new(p[0], p[1])).collect(), closed: self.closed }
    }
}

/// One hole to drill: where (work XY, mm) and how deep its bottom is (work Z).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct HolePoint {
    pub at: [f64; 2],
    pub bottom: f64,
}

impl HolePoint {
    pub fn validate(&self) -> Result<()> {
        let fine = |v: f64| v.is_finite() && v.abs() <= 1.0e6;
        if !fine(self.at[0]) || !fine(self.at[1]) || !fine(self.bottom) {
            return Err(DocError::Invalid("a hole has non-finite or huge coordinates".into()));
        }
        Ok(())
    }

    pub fn to_hole(&self) -> Hole {
        Hole { at: Vec2::new(self.at[0], self.at[1]), bottom: self.bottom }
    }
}

/// A 2D operation whose geometry is planar polygons (or polyline curves, or holes) in work coordinates.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum CamOperation {
    Contour {
        name: String,
        tool: u32,
        regions: Vec<PolygonRegion>,
        params: ContourParams,
        cut: Cut,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        spindle_rpm: Option<f64>,
        #[serde(default, skip_serializing_if = "std::ops::Not::not")]
        coolant: bool,
        /// The operation whose larger tool went first, when this one is rest machining.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        rest_from: Option<String>,
    },
    Pocket {
        name: String,
        tool: u32,
        regions: Vec<PolygonRegion>,
        params: PocketParams,
        cut: Cut,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        spindle_rpm: Option<f64>,
        #[serde(default, skip_serializing_if = "std::ops::Not::not")]
        coolant: bool,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        rest_from: Option<String>,
    },
    /// The tool centre follows curves, open or closed, at fixed depth passes.
    Engrave {
        name: String,
        tool: u32,
        chains: Vec<PolylineChain>,
        params: EngraveParams,
        cut: Cut,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        spindle_rpm: Option<f64>,
        #[serde(default, skip_serializing_if = "std::ops::Not::not")]
        coolant: bool,
    },
    /// A V-bit along the inward offsets of the regions, each as deep as the cone touches the
    /// wall there, never below `cut.heights.bottom` (where the tip clears a flat floor).
    VCarve {
        name: String,
        tool: u32,
        regions: Vec<PolygonRegion>,
        params: VCarveParams,
        cut: Cut,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        spindle_rpm: Option<f64>,
        #[serde(default, skip_serializing_if = "std::ops::Not::not")]
        coolant: bool,
    },
    /// Straight down at every hole, each to its own bottom; `cut.heights.bottom` is the deepest.
    Drill {
        name: String,
        tool: u32,
        holes: Vec<HolePoint>,
        params: DrillParams,
        cut: Cut,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        spindle_rpm: Option<f64>,
        #[serde(default, skip_serializing_if = "std::ops::Not::not")]
        coolant: bool,
    },
}

impl CamOperation {
    pub fn name(&self) -> &str {
        match self {
            CamOperation::Contour { name, .. }
            | CamOperation::Pocket { name, .. }
            | CamOperation::Engrave { name, .. }
            | CamOperation::VCarve { name, .. }
            | CamOperation::Drill { name, .. } => name,
        }
    }

    pub fn tool(&self) -> u32 {
        match self {
            CamOperation::Contour { tool, .. }
            | CamOperation::Pocket { tool, .. }
            | CamOperation::Engrave { tool, .. }
            | CamOperation::VCarve { tool, .. }
            | CamOperation::Drill { tool, .. } => *tool,
        }
    }

    pub fn cut(&self) -> &Cut {
        match self {
            CamOperation::Contour { cut, .. }
            | CamOperation::Pocket { cut, .. }
            | CamOperation::Engrave { cut, .. }
            | CamOperation::VCarve { cut, .. }
            | CamOperation::Drill { cut, .. } => cut,
        }
    }

    /// The spindle speed the operation asks for (rev/min), if not the tool's.
    pub fn spindle_rpm(&self) -> Option<f64> {
        match self {
            CamOperation::Contour { spindle_rpm, .. }
            | CamOperation::Pocket { spindle_rpm, .. }
            | CamOperation::Engrave { spindle_rpm, .. }
            | CamOperation::VCarve { spindle_rpm, .. }
            | CamOperation::Drill { spindle_rpm, .. } => *spindle_rpm,
        }
    }

    pub fn coolant(&self) -> bool {
        match self {
            CamOperation::Contour { coolant, .. }
            | CamOperation::Pocket { coolant, .. }
            | CamOperation::Engrave { coolant, .. }
            | CamOperation::VCarve { coolant, .. }
            | CamOperation::Drill { coolant, .. } => *coolant,
        }
    }

    pub fn validate(&self) -> Result<()> {
        let name = self.name();
        if name.is_empty() || name.len() > 128 {
            return Err(DocError::Invalid("a CAM operation name must be 1 to 128 characters".into()));
        }
        let positive = |what: &str, v: f64| if v > 0.0 && v.is_finite() { Ok(()) } else { Err(DocError::Invalid(format!("{what} must be above 0"))) };
        let not_negative =
            |what: &str, v: f64| if v >= 0.0 && v.is_finite() { Ok(()) } else { Err(DocError::Invalid(format!("{what} must be ≥ 0"))) };
        let regions = match self {
            CamOperation::Contour { regions, params, .. } => {
                not_negative("stock_to_leave", params.stock_to_leave)?;
                positive("tolerance", params.tolerance)?;
                not_negative("ramp_angle", params.ramp_angle).map_err(|_| DocError::Invalid("ramp_angle must be ≥ 0 radians".into()))?;
                Some(regions)
            }
            CamOperation::Pocket { regions, params, .. } => {
                positive("stepover", params.stepover)?;
                not_negative("stock_to_leave", params.stock_to_leave)?;
                positive("tolerance", params.tolerance)?;
                not_negative("ramp_angle", params.ramp_angle).map_err(|_| DocError::Invalid("ramp_angle must be ≥ 0 radians".into()))?;
                Some(regions)
            }
            CamOperation::VCarve { regions, params, .. } => {
                positive("stepover", params.stepover)?;
                positive("tolerance", params.tolerance)?;
                Some(regions)
            }
            CamOperation::Engrave { chains, params, .. } => {
                positive("tolerance", params.tolerance)?;
                if chains.is_empty() || chains.len() > MAX_CHAINS {
                    return Err(DocError::Invalid(format!("an engraving needs 1 to {MAX_CHAINS} curves")));
                }
                for c in chains {
                    c.validate()?;
                }
                None
            }
            CamOperation::Drill { holes, params, .. } => {
                if let Some(peck) = params.peck {
                    positive("peck", peck)?;
                }
                not_negative("dwell", params.dwell)?;
                if holes.is_empty() || holes.len() > MAX_DRILL_HOLES {
                    return Err(DocError::Invalid(format!("a drilling needs 1 to {MAX_DRILL_HOLES} holes")));
                }
                for h in holes {
                    h.validate()?;
                }
                None
            }
        };
        if let Some(regions) = regions {
            if regions.is_empty() || regions.len() > MAX_REGIONS {
                return Err(DocError::Invalid(format!("an operation needs 1 to {MAX_REGIONS} regions")));
            }
            for r in regions {
                r.validate()?;
            }
        }
        let cut = self.cut();
        cut.heights.validate().map_err(|e| DocError::Invalid(e.to_string()))?;
        cut.feeds.validate().map_err(|e| DocError::Invalid(e.to_string()))?;
        positive("stepdown", cut.stepdown)?;
        if let Some(rpm) = self.spindle_rpm()
            && (!(rpm > 0.0) || !rpm.is_finite() || rpm > 1.0e6)
        {
            return Err(DocError::Invalid("spindle_rpm must be above 0".into()));
        }
        Ok(())
    }
}

/// Default contour params for a new operation.
pub fn default_contour_params() -> ContourParams {
    ContourParams { side: Side::Outside, climb: true, stock_to_leave: 0.0, tolerance: 0.01, ramp_angle: 0.0, tabs: None, dogbones: false, rest: None }
}

/// Default pocket params; `stepover` must still be set by the caller.
pub fn default_pocket_params(stepover: f64) -> PocketParams {
    PocketParams { stepover, climb: true, stock_to_leave: 0.0, tolerance: 0.01, ramp_angle: 0.0, dogbones: false, rest: None }
}

/// Default v-carving params: passes a tenth of a millimetre apart.
pub fn default_vcarve_params() -> VCarveParams {
    VCarveParams { stepover: 0.1, tolerance: 0.01 }
}

/// Default engraving params: ends within a hundredth of a millimetre are one curve.
pub fn default_engrave_params() -> EngraveParams {
    EngraveParams { tolerance: 0.01 }
}

/// Default cut: clearance 10, retract 2, top 0, bottom `bottom`, feeds 600/200, stepdown from depth.
pub fn default_cut(bottom: f64, stepdown: f64) -> Cut {
    Cut { heights: Heights { clearance: 10.0, retract: 2.0, top: 0.0, bottom }, feeds: Feeds { cut: 600.0, plunge: 200.0 }, stepdown }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn polygon_region_roundtrips_to_region2() {
        let p = PolygonRegion { outer: vec![[0.0, 0.0], [40.0, 0.0], [40.0, 30.0], [0.0, 30.0]], holes: vec![] };
        let r = p.to_region().unwrap();
        assert!((r.outer.signed_area() - 1200.0).abs() < 1e-9);
    }

    #[test]
    fn empty_settings_skip_serialize() {
        assert!(CamSettings::default().is_empty());
    }

    #[test]
    fn a_chain_to_engrave_needs_enough_finite_points_and_reads_back_without_its_closed_flag() {
        let open = PolylineChain { points: vec![[0.0, 0.0], [10.0, 0.0]], closed: false };
        open.validate().unwrap();
        assert!(PolylineChain { points: vec![[0.0, 0.0]], closed: false }.validate().is_err());
        assert!(PolylineChain { points: vec![[0.0, 0.0], [10.0, 0.0]], closed: true }.validate().is_err(), "closed needs three");
        assert!(PolylineChain { points: vec![[0.0, 0.0], [f64::NAN, 0.0]], closed: false }.validate().is_err());
        // Saved without `closed` when open, and read back the same.
        let text = serde_json::to_string(&open).unwrap();
        assert!(!text.contains("closed"), "{text}");
        assert_eq!(serde_json::from_str::<PolylineChain>(&text).unwrap(), open);
        let chain = open.to_chain();
        assert_eq!((chain.points.len(), chain.closed), (2, false));
    }
}
