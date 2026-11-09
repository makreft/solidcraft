//! A design can hold CAM milling data (`cam`: tools, setups with work offset and optional stock,
//! and 2D contour/pocket operations whose regions are planar polygons in work coordinates).
//! Toolpaths are never saved; they are computed when posting. Older designs need no
//! transformation: an absent `cam` field means no setups. The last release refuses a design
//! saved with this (the newer format number).
//!
//! Tools may carry `tip_angle`, `taper_angle` and `tip_diameter`: a V-bit (`kind: vbit`) and a
//! tapered ball end mill (`kind: tapered_ball`) narrow towards their tip. A tool without them is a
//! cylinder, as before.
//!
//! Besides contours and pockets, an operation can be an `engrave` (the tool centre follows
//! polyline `chains`, open or `closed`), a `v_carve` (a V-bit along the inward offsets of its
//! regions) or a `drill` (`holes` with a bottom each, pecks and a dwell). A design without them
//! has none, as before.
//!
//! A contour or pocket may be rest machining (`rest_from` names the earlier operation, and its
//! params carry `rest`: the earlier tool's radius and wall offset): it cuts only where that tool
//! could not reach. Without them it cuts everything, as before.
