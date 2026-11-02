use std::f64::consts::PI;

use solidcraft_geom::{Loop2, Region2, Vec2, Vec3};

use super::*;
use crate::offset::Shape;

fn v(x: f64, y: f64) -> Vec2 {
    Vec2::new(x, y)
}

fn polygon(pts: &[(f64, f64)]) -> Region2 {
    Region2 { outer: Loop2::polygon(&pts.iter().map(|p| v(p.0, p.1)).collect::<Vec<_>>()), holes: vec![] }
}

fn rect(x0: f64, y0: f64, x1: f64, y1: f64) -> Region2 {
    polygon(&[(x0, y0), (x1, y0), (x1, y1), (x0, y1)])
}

fn area(loops: &[Loop2]) -> f64 {
    loops.iter().map(Loop2::signed_area).sum()
}

fn close(a: f64, b: f64, tol: f64) {
    assert!((a - b).abs() <= tol, "{a} is not within {tol} of {b}");
}

/// Every point of the offset loops is `d` from the boundary, and every loop is closed.
fn check_offset(regions: &[Region2], d: f64, loops: &[Loop2]) {
    let shape = Shape::from_regions(regions, 0.01).unwrap();
    for lp in loops {
        assert!(lp.is_closed(1e-3), "open loop");
        for p in lp.polyline(0.001) {
            close(shape.dist(p), d.abs(), 1e-3);
        }
    }
}

fn tool(d: f64) -> Tool {
    Tool::new(1, ToolKind::FlatEndmill, d, 20.0)
}

fn cut(top: f64, bottom: f64, stepdown: f64) -> Cut {
    Cut { heights: Heights { clearance: 10.0, retract: 2.0, top, bottom }, feeds: Feeds { cut: 600.0, plunge: 200.0 }, stepdown }
}

fn cutting_points(path: &Toolpath, below: f64) -> Vec<Vec3> {
    path.points(0.005).into_iter().filter(|(p, cutting)| *cutting && p.z < below).map(|(p, _)| p).collect()
}

/// Distance in XY from `q` to the nearest cut made entirely below `below`.
fn dist_to_cuts(path: &Toolpath, below: f64, q: Vec2) -> f64 {
    let flat = |p: Vec3| Vec3::new(p.x, p.y, 0.0);
    let pts = path.points(0.005);
    pts.windows(2)
        .filter(|w| w[1].1 && w[0].0.z < below && w[1].0.z < below)
        .map(|w| q.extend(0.0).dist_to_segment(flat(w[0].0), flat(w[1].0)))
        .fold(f64::INFINITY, f64::min)
}

#[test]
fn square_offsets_to_a_smaller_square_and_to_a_rounded_larger_one() {
    let r = [rect(0.0, 0.0, 40.0, 30.0)];
    let inner = offset_regions(&r, 5.0, 0.01).unwrap();
    assert_eq!(inner.len(), 1);
    close(area(&inner), 30.0 * 20.0, 1e-6);
    check_offset(&r, 5.0, &inner);
    // Grown by d: area + perimeter × d + π d² (four quarter circles at the corners).
    let outer = offset_regions(&r, -5.0, 0.01).unwrap();
    assert_eq!(outer.len(), 1);
    close(area(&outer), 1200.0 + 140.0 * 5.0 + PI * 25.0, 1e-6);
    check_offset(&r, 5.0, &outer);
}

#[test]
fn inside_corner_of_an_l_shape_is_rounded() {
    // Two 30×10 arms that share a 10×10 square, plus the part of the 5×5 square at the inside
    // corner that is at least 5 from the corner.
    let l = [polygon(&[(0.0, 0.0), (40.0, 0.0), (40.0, 20.0), (20.0, 20.0), (20.0, 40.0), (0.0, 40.0)])];
    let inner = offset_regions(&l, 5.0, 0.01).unwrap();
    assert_eq!(inner.len(), 1);
    close(area(&inner), 500.0 + 25.0 - 25.0 * PI / 4.0, 1e-6);
    check_offset(&l, 5.0, &inner);
}

#[test]
fn circles_and_holes_offset_exactly() {
    let disc = [Region2 { outer: Loop2::circle(v(3.0, -2.0), 10.0), holes: vec![] }];
    close(area(&offset_regions(&disc, 3.0, 0.01).unwrap()), 49.0 * PI, 1e-6);
    close(area(&offset_regions(&disc, -3.0, 0.01).unwrap()), 169.0 * PI, 1e-6);
    // A hole grows as its region shrinks.
    let plate = [Region2 { outer: rect(0.0, 0.0, 40.0, 40.0).outer, holes: vec![Loop2::circle(v(20.0, 20.0), 5.0).reversed()] }];
    let inner = offset_regions(&plate, 3.0, 0.01).unwrap();
    assert_eq!(inner.len(), 2);
    close(area(&inner), 34.0 * 34.0 - 64.0 * PI, 1e-6);
    check_offset(&plate, 3.0, &inner);
    // Hole loops given counter-clockwise are turned round.
    let ccw_hole = [Region2 { outer: rect(0.0, 0.0, 40.0, 40.0).outer, holes: vec![Loop2::circle(v(20.0, 20.0), 5.0)] }];
    close(area(&offset_regions(&ccw_hole, 3.0, 0.01).unwrap()), 34.0 * 34.0 - 64.0 * PI, 1e-6);
}

#[test]
fn narrow_neck_splits_the_offset_and_a_small_region_vanishes() {
    let dumbbell = [polygon(&[
        (0.0, 0.0),
        (20.0, 0.0),
        (20.0, 8.0),
        (30.0, 8.0),
        (30.0, 0.0),
        (50.0, 0.0),
        (50.0, 20.0),
        (30.0, 20.0),
        (30.0, 12.0),
        (20.0, 12.0),
        (20.0, 20.0),
        (0.0, 20.0),
    ])];
    let inner = offset_regions(&dumbbell, 3.0, 0.01).unwrap();
    assert_eq!(inner.len(), 2, "the 4 mm neck is too narrow for a 3 mm offset");
    check_offset(&dumbbell, 3.0, &inner);
    for lp in &inner {
        // A 14×14 square plus a small bulge towards the neck.
        let a = lp.signed_area();
        assert!(a > 196.0 && a < 200.0, "{a}");
    }
    // At 1.5 the neck stays open.
    assert_eq!(offset_regions(&dumbbell, 1.5, 0.01).unwrap().len(), 1);
    assert!(offset_regions(&[rect(0.0, 0.0, 10.0, 10.0)], 6.0, 0.01).unwrap().is_empty());
}

#[test]
fn polygon_with_many_shallow_corners_offsets_to_one_loop() {
    // A regular 720-gon: neighbouring sides turn by half a degree.
    let (n, radius) = (720usize, 25.0);
    let pts: Vec<(f64, f64)> =
        (0..n).map(|i| (radius * (i as f64 * 2.0 * PI / n as f64).cos(), radius * (i as f64 * 2.0 * PI / n as f64).sin())).collect();
    let gon = [polygon(&pts)];
    let half = PI / n as f64;
    let (apothem, perimeter) = (radius * half.cos(), 2.0 * n as f64 * radius * half.sin());
    let inner = offset_regions(&gon, 4.0, 0.01).unwrap();
    assert_eq!(inner.len(), 1);
    close(area(&inner), n as f64 * (apothem - 4.0).powi(2) * half.tan(), 1e-3);
    let outer = offset_regions(&gon, -4.0, 0.01).unwrap();
    assert_eq!(outer.len(), 1);
    close(area(&outer), n as f64 * apothem.powi(2) * half.tan() + perimeter * 4.0 + PI * 16.0, 1e-3);
}

#[test]
fn pocket_stays_inside_the_walls_and_clears_everything_the_tool_can_reach() {
    let regions = [rect(0.0, 0.0, 40.0, 30.0)];
    let params = PocketParams { stepover: 2.5, climb: true, stock_to_leave: 0.0, tolerance: 0.01, ramp_angle: 0.0, dogbones: false };
    let path = pocket(&regions, &tool(6.0), &params, &cut(0.0, -5.0, 2.0)).unwrap();
    let stats = path.stats();
    assert_eq!(stats.lowest_z, Some(-5.0));
    // Three passes of 5/3 mm.
    let mut levels: Vec<i64> = cutting_points(&path, 0.0).iter().map(|p| (p.z * 1e6).round() as i64).collect();
    levels.sort();
    levels.dedup();
    assert!(levels.len() >= 3 && levels.contains(&-5_000_000) && levels.contains(&-1_666_667), "{levels:?}");
    for p in cutting_points(&path, 0.0) {
        assert!(p.x >= 3.0 - 1e-6 && p.x <= 37.0 + 1e-6 && p.y >= 3.0 - 1e-6 && p.y <= 27.0 + 1e-6, "gouge at {p:?}");
    }
    for i in 0..=34 {
        for j in 0..=24 {
            let q = v(3.0 + i as f64, 3.0 + j as f64);
            let d = dist_to_cuts(&path, -5.0 + 1e-6, q);
            assert!(d <= 3.0 + 0.01, "{q:?} is {d} from the nearest pass");
        }
    }
}

#[test]
fn pocket_goes_round_an_island() {
    let island = Loop2::circle(v(30.0, 20.0), 5.0);
    let regions = [Region2 { outer: rect(0.0, 0.0, 60.0, 40.0).outer, holes: vec![island] }];
    let params = PocketParams { stepover: 3.0, climb: false, stock_to_leave: 0.5, tolerance: 0.01, ramp_angle: 0.0, dogbones: false };
    let path = pocket(&regions, &tool(6.0), &params, &cut(0.0, -2.0, 2.0)).unwrap();
    let shape = Shape::from_regions(&regions, 0.01).unwrap();
    let pts = cutting_points(&path, 0.0);
    for p in &pts {
        // Tool radius 3 plus 0.5 left on the walls.
        assert!(shape.clear(v(p.x, p.y), 3.5 - 1e-4), "gouge at {p:?}");
    }
    for i in 0..=60 {
        for j in 0..=40 {
            let q = v(i as f64, j as f64);
            if shape.clear(q, 3.5 + 1e-3) {
                let d = dist_to_cuts(&path, 0.0, q);
                assert!(d <= 3.0 + 0.01, "{q:?} is {d} from the nearest pass");
            }
        }
    }
}

#[test]
fn contour_runs_round_the_part_with_the_wall_on_the_chosen_side() {
    let regions = [rect(0.0, 0.0, 40.0, 30.0)];
    let c = cut(0.0, -6.0, 3.0);
    let run = |climb: bool| {
        let params = ContourParams { side: Side::Outside, climb, stock_to_leave: 0.0, tolerance: 0.01, ramp_angle: 0.0, tabs: None, dogbones: false };
        contour(&regions, &tool(6.0), &params, &c).unwrap()
    };
    let climb = run(true);
    // Two passes round the rounded rectangle 3 mm outside the part, and 8 mm of plunging
    // (retract 2 → −3 → −6).
    close(climb.stats().cut_length, 2.0 * (140.0 + 6.0 * PI) + 8.0, 1e-6);
    close(climb.stats().cut_minutes, 2.0 * (140.0 + 6.0 * PI) / 600.0 + 8.0 / 200.0, 1e-9);
    let turns = |path: &Toolpath| {
        let pts = cutting_points(path, -5.9);
        pts.iter().zip(pts.iter().cycle().skip(1)).map(|(a, b)| a.x * b.y - a.y * b.x).sum::<f64>()
    };
    // Climb round the outside of a part is clockwise (wall on the right of travel).
    assert!(turns(&climb) < 0.0);
    assert!(turns(&run(false)) > 0.0);
    for p in cutting_points(&climb, 0.0) {
        let outside = (p.x.clamp(0.0, 40.0) - p.x).hypot(p.y.clamp(0.0, 30.0) - p.y);
        close(outside, 3.0, 1e-6);
    }
    // Inside an opening the tool must fit.
    let inside =
        ContourParams { side: Side::Inside, climb: true, stock_to_leave: 0.0, tolerance: 0.01, ramp_angle: 0.0, tabs: None, dogbones: false };
    assert!(contour(&[rect(0.0, 0.0, 5.0, 5.0)], &tool(6.0), &inside, &c).is_err());
    let inner = contour(&regions, &tool(6.0), &inside, &c).unwrap();
    close(inner.stats().cut_length, 2.0 * 2.0 * (34.0 + 24.0) + 8.0, 1e-6);
    // Climb inside an opening is counter-clockwise.
    assert!(turns(&inner) > 0.0);
}

#[test]
fn holes_are_cut_before_the_outline_that_holds_the_part() {
    // The hole is given first and last: either way the tool goes round it before the outline.
    let hole = rect(10.0, 10.0, 20.0, 20.0).outer.reversed();
    let params =
        ContourParams { side: Side::Outside, climb: true, stock_to_leave: 0.0, tolerance: 0.01, ramp_angle: 0.0, tabs: None, dogbones: false };
    for holes in [vec![hole.clone()], vec![hole.clone(), Loop2::circle(v(50.0, 15.0), 5.0)]] {
        let plate = [Region2 { outer: rect(0.0, 0.0, 60.0, 30.0).outer, holes }];
        let path = contour(&plate, &tool(6.0), &params, &cut(0.0, -6.0, 6.0)).unwrap();
        let cuts = cutting_points(&path, -5.9);
        let outline = cuts.iter().position(|p| p.x < 0.0 || p.y < 0.0).unwrap();
        assert!(cuts[..outline].iter().any(|p| p.x > 10.0 && p.x < 20.0 && p.y > 10.0 && p.y < 20.0), "the hole comes first");
        assert!(cuts[outline..].iter().all(|p| p.x < 0.0 || p.y < 0.0 || p.x > 60.0 || p.y > 30.0), "nothing after the outline");
    }
}

#[test]
fn dogbones_reach_into_the_inside_corners() {
    // A 60 × 40 part with a 10 mm slot 20 deep from its top edge, and a 20 × 10 window.
    let slotted = polygon(&[(0.0, 0.0), (60.0, 0.0), (60.0, 40.0), (35.0, 40.0), (35.0, 20.0), (25.0, 20.0), (25.0, 40.0), (0.0, 40.0)]);
    let part = [Region2 { outer: slotted.outer, holes: vec![rect(5.0, 5.0, 25.0, 15.0).outer.reversed()] }];
    let c = cut(0.0, -6.0, 6.0);
    let run = |dogbones: bool, climb: bool| {
        let params = ContourParams { side: Side::Outside, climb, stock_to_leave: 0.0, tolerance: 0.01, ramp_angle: 0.0, tabs: None, dogbones };
        contour(&part, &tool(6.0), &params, &c).unwrap()
    };
    // The slot's floor and the window's corners: the tool's edge reaches each one.
    let inside_corners = [(25.0, 20.0), (35.0, 20.0), (5.0, 5.0), (25.0, 5.0), (25.0, 15.0), (5.0, 15.0)];
    for climb in [true, false] {
        let with = run(true, climb);
        for &(x, y) in &inside_corners {
            close(dist_to_cuts(&with, -5.9, v(x, y)), 3.0, 1e-6);
        }
        // The part's outer corners and the slot's mouth were reached already, round arcs (drawn
        // as chords within 0.005 mm here): no notch there.
        for (x, y) in [(0.0, 0.0), (60.0, 40.0), (25.0, 40.0), (35.0, 40.0)] {
            close(dist_to_cuts(&with, -5.9, v(x, y)), 3.0, 0.006);
        }
        // Six notches, each 3·(√2 − 1) out and back.
        let extra = with.stats().cut_length - run(false, climb).stats().cut_length;
        close(extra, 6.0 * 2.0 * 3.0 * (2f64.sqrt() - 1.0), 1e-6);
    }
    let without = run(false, true);
    close(dist_to_cuts(&without, -5.9, v(25.0, 20.0)), 3.0 * 2f64.sqrt(), 1e-6);

    // From the inside of an opening, its corners.
    let params = ContourParams { side: Side::Inside, climb: true, stock_to_leave: 0.0, tolerance: 0.01, ramp_angle: 0.0, tabs: None, dogbones: true };
    let window = contour(&[rect(0.0, 0.0, 30.0, 20.0)], &tool(6.0), &params, &c).unwrap();
    for (x, y) in [(0.0, 0.0), (30.0, 0.0), (30.0, 20.0), (0.0, 20.0)] {
        close(dist_to_cuts(&window, -5.9, v(x, y)), 3.0, 1e-6);
    }

    // A pocket made for a square part, its corners notched along the walls.
    let housing = [rect(0.0, 0.0, 40.0, 18.0)];
    let pocket_params = PocketParams { stepover: 2.5, climb: true, stock_to_leave: 0.0, tolerance: 0.01, ramp_angle: 0.0, dogbones: true };
    let pocketed = pocket(&housing, &tool(6.0), &pocket_params, &cut(0.0, -6.0, 3.0)).unwrap();
    for (x, y) in [(0.0, 0.0), (40.0, 0.0), (40.0, 18.0), (0.0, 18.0)] {
        close(dist_to_cuts(&pocketed, -5.9, v(x, y)), 3.0, 1e-6);
    }
    let plain = pocket(&housing, &tool(6.0), &PocketParams { dogbones: false, ..pocket_params }, &cut(0.0, -6.0, 3.0)).unwrap();
    close(dist_to_cuts(&plain, -5.9, v(0.0, 0.0)), 3.0 * 2f64.sqrt(), 1e-6);
}

/// A bend of the offset that is no corner of the part (two parts, or an island and a wall, closer
/// than the tool) gets no notch: it would cut into one of them.
#[test]
fn dogbones_stay_out_of_parts_closer_than_the_tool() {
    let tight = |path: &Toolpath, shape: &Shape, skip: &[(f64, f64)]| {
        cutting_points(path, -0.1)
            .into_iter()
            .map(|p| v(p.x, p.y))
            .filter(|q| skip.iter().all(|&(x, y)| q.dist(v(x, y)) > 3.5))
            .map(|q| shape.dist(q))
            .fold(f64::INFINITY, f64::min)
    };
    // Two 20 × 20 parts 4 mm apart, cut round the outside with a 6 mm tool: the offsets of their
    // facing sides cross in the gap, where no wall corner is.
    let parts = [rect(0.0, 0.0, 20.0, 20.0), rect(24.0, 0.0, 44.0, 20.0)];
    let shape = Shape::from_regions(&parts, 0.01).unwrap();
    let c = cut(0.0, -6.0, 6.0);
    let params =
        ContourParams { side: Side::Outside, climb: true, stock_to_leave: 0.0, tolerance: 0.01, ramp_angle: 0.0, tabs: None, dogbones: true };
    let with = contour(&parts, &tool(6.0), &params, &c).unwrap();
    assert!(tight(&with, &shape, &[]) >= 3.0 - 0.006, "{}", tight(&with, &shape, &[]));
    let plain = contour(&parts, &tool(6.0), &ContourParams { dogbones: false, ..params }, &c).unwrap();
    close(with.stats().cut_length, plain.stats().cut_length, 1e-9);

    // A 50 × 30 pocket with an island 4 mm from its lower wall: the pocket's own corners are
    // notched, nothing cuts into the island or the wall beside it.
    let housing = [Region2 { outer: rect(0.0, 0.0, 50.0, 30.0).outer, holes: vec![rect(10.0, 4.0, 40.0, 20.0).outer.reversed()] }];
    let shape = Shape::from_regions(&housing, 0.01).unwrap();
    let pp = PocketParams { stepover: 2.5, climb: true, stock_to_leave: 0.0, tolerance: 0.01, ramp_angle: 0.0, dogbones: true };
    let pocketed = pocket(&housing, &tool(6.0), &pp, &cut(0.0, -6.0, 3.0)).unwrap();
    let corners = [(0.0, 0.0), (50.0, 0.0), (50.0, 30.0), (0.0, 30.0)];
    assert!(tight(&pocketed, &shape, &corners) >= 3.0 - 0.006, "{}", tight(&pocketed, &shape, &corners));
    for (x, y) in corners {
        close(dist_to_cuts(&pocketed, -5.9, v(x, y)), 3.0, 1e-6);
    }
}

/// The length the last pass runs at the tabs' height, and its longest stretch there.
fn on_tabs(path: &Toolpath, top: f64) -> (f64, f64) {
    let pts = path.points(0.005);
    let (mut total, mut run, mut longest) = (0.0, 0.0, 0.0f64);
    for w in pts.windows(2) {
        let (a, b) = (w[0].0, w[1].0);
        if w[1].1 && (a.z - top).abs() < 1e-6 && (b.z - top).abs() < 1e-6 {
            let d = Vec2::new(a.x, a.y).dist(Vec2::new(b.x, b.y));
            total += d;
            run += d;
            longest = longest.max(run);
        } else {
            run = 0.0;
        }
    }
    (total, longest)
}

/// Between two tabs on one side, or on one arc, the tool goes back down to depth: it stays at the
/// tabs' height only over each tab, the tab's width plus the tool's.
#[test]
fn tabs_on_one_side_let_the_cut_through_between_them() {
    let tabs = |spacing| Tabs { width: 6.0, height: 2.0, spacing, inner: false };
    // A 200 × 20 strip (the loop about 459 mm, so ten tabs) and a disc of radius 50 (the loop
    // about 333 mm, so four).
    for (part, spacing, count) in
        [(rect(0.0, 0.0, 200.0, 20.0), 50.0, 10.0), (Region2 { outer: Loop2::circle(v(0.0, 0.0), 50.0), holes: vec![] }, 100.0, 4.0)]
    {
        for ramp in [0.0, 3f64.to_radians()] {
            let params = ContourParams {
                side: Side::Outside,
                climb: true,
                stock_to_leave: 0.0,
                tolerance: 0.01,
                ramp_angle: ramp,
                tabs: Some(tabs(spacing)),
                dogbones: false,
            };
            let path = contour(std::slice::from_ref(&part), &tool(6.0), &params, &cut(0.0, -6.0, 3.0)).unwrap();
            let (total, longest) = on_tabs(&path, -4.0);
            assert!(longest <= 12.0 + 1e-3, "spacing {spacing}, ramp {ramp}: {longest} mm on end at the tabs' height");
            if ramp == 0.0 {
                close(total, count * 12.0, 0.01);
            }
            // Most of the loop is cut at full depth on the last pass.
            let deep: f64 = path
                .points(0.005)
                .windows(2)
                .filter(|w| w[1].1 && (w[0].0.z + 6.0).abs() < 1e-6 && (w[1].0.z + 6.0).abs() < 1e-6)
                .map(|w| Vec2::new(w[0].0.x, w[0].0.y).dist(Vec2::new(w[1].0.x, w[1].0.y)))
                .sum();
            let loop_length = if spacing == 50.0 { 440.0 + 6.0 * PI } else { 2.0 * PI * 53.0 };
            // Each tab takes its flat and the two 45° slopes off the loop (arcs sampled as chords).
            assert!(deep >= loop_length - count * (12.0 + 4.0) - 0.05, "spacing {spacing}, ramp {ramp}: {deep} mm at depth");
        }
    }
}

/// A cut deeper than the tool's cutting edges is refused: the shank would rub in the slot.
#[test]
fn a_cut_deeper_than_the_flutes_is_refused() {
    let short = Tool::new(4, ToolKind::FlatEndmill, 3.0, 8.0);
    let part = [rect(0.0, 0.0, 40.0, 30.0)];
    let cp = ContourParams { side: Side::Outside, climb: true, stock_to_leave: 0.0, tolerance: 0.01, ramp_angle: 0.0, tabs: None, dogbones: false };
    let pp = PocketParams { stepover: 1.5, climb: true, stock_to_leave: 0.0, tolerance: 0.01, ramp_angle: 0.0, dogbones: false };
    let deep = cut(0.0, -12.0, 3.0);
    for err in [contour(&part, &short, &cp, &deep).unwrap_err(), pocket(&part, &short, &pp, &deep).unwrap_err()] {
        assert!(err.to_string().contains("flute length"), "{err}");
    }
    assert!(contour(&part, &short, &cp, &cut(0.0, -8.0, 4.0)).is_ok());
}

/// Where the tool rises onto a tab: the moves that end at the tabs' top coming from below it.
fn tab_rises(path: &Toolpath, top: f64) -> Vec<Vec3> {
    let pts = path.points(0.005);
    pts.windows(2).filter(|w| w[1].1 && (w[1].0.z - top).abs() < 1e-6 && w[0].0.z < top - 1e-6).map(|w| w[1].0).collect()
}

#[test]
fn tabs_hold_the_part_and_its_waste() {
    // 200 × 100 with a 100 × 50 window, 6 deep; tabs 2 high and 6 wide, about one every 200 mm.
    let plate = [Region2 { outer: rect(0.0, 0.0, 200.0, 100.0).outer, holes: vec![rect(50.0, 25.0, 150.0, 75.0).outer.reversed()] }];
    let tabs = Tabs { width: 6.0, height: 2.0, spacing: 200.0, inner: true };
    let run = |tabs: Option<Tabs>, ramp: f64| {
        let params =
            ContourParams { side: Side::Outside, climb: true, stock_to_leave: 0.0, tolerance: 0.01, ramp_angle: ramp, tabs, dogbones: false };
        contour(&plate, &tool(6.0), &params, &cut(0.0, -6.0, 3.0))
    };
    for ramp in [0.0, 3f64.to_radians()] {
        let path = run(Some(tabs), ramp).unwrap();
        // The outline (about 619 mm) gets four, the window's loop (about 276 mm) two. A ramp
        // below the tabs' top passes over them too: count where they stand.
        let mut rises = tab_rises(&path, -4.0);
        rises.dedup_by(|a, b| a.dist(*b) < 1e-6);
        rises.sort_by(|a, b| a.x.total_cmp(&b.x).then(a.y.total_cmp(&b.y)));
        rises.dedup_by(|a, b| a.dist(*b) < 1e-6);
        let on_outline = rises.iter().filter(|p| p.x < 0.0 || p.y < 0.0 || p.x > 200.0 || p.y > 100.0).count();
        assert_eq!((on_outline, rises.len()), (4, 6), "ramp {ramp}: {rises:?}");
        // On the straight sides, not round a corner.
        for p in &rises {
            let on_side =
                [p.x + 3.0, p.y + 3.0, p.x - 203.0, p.y - 103.0, p.x - 53.0, p.y - 28.0, p.x - 147.0, p.y - 72.0].iter().any(|d| d.abs() < 1e-6);
            assert!(on_side, "{p:?}");
        }
        // Nothing in the material goes down steeper than 45°; straight down only where a loop's
        // pass starts, between tabs, without a ramp.
        let pts = path.points(0.005);
        let (mut flat, mut plunges) = (0.0, 0);
        for w in pts.windows(2) {
            let (a, b) = (w[0].0, w[1].0);
            let run_xy = Vec2::new(a.x, a.y).dist(Vec2::new(b.x, b.y));
            if w[1].1 && (a.z + 4.0).abs() < 1e-6 && (b.z + 4.0).abs() < 1e-6 {
                flat += run_xy;
            }
            if w[1].1 && b.z < a.z - 1e-9 && a.z < -1e-9 {
                if run_xy > 1e-9 {
                    assert!(a.z - b.z <= run_xy * (1.0 + 1e-6) + 1e-9, "{a:?} → {b:?}");
                } else {
                    plunges += 1;
                }
            }
        }
        assert_eq!(plunges, if ramp > 0.0 { 0 } else { 2 });
        // Without a ramp only the last pass is below the tabs: over each one the tool stays up
        // for the tab's width plus its own.
        if ramp == 0.0 {
            close(flat, 6.0 * 12.0, 1e-6);
        }
        assert!(cutting_points(&path, -5.9).len() > 10, "the rest goes to the bottom");
    }
    // Only the outline, when the window's waste may come free.
    let outline_only = run(Some(Tabs { inner: false, ..tabs }), 0.0).unwrap();
    assert_eq!(tab_rises(&outline_only, -4.0).len(), 4);
    // Tabs as high as the cut is deep would leave nothing to cut.
    assert!(run(Some(Tabs { height: 6.0, ..tabs }), 0.0).is_err());
    assert!(run(Some(Tabs { width: 0.0, ..tabs }), 0.0).is_err());
    // An operation saved before tabs and dogbones existed reads as one without them.
    let old = serde_json::json!({"side": "outside", "climb": true, "stock_to_leave": 0.0, "tolerance": 0.01, "ramp_angle": 0.0});
    let params: ContourParams = serde_json::from_value(old).unwrap();
    assert_eq!((params.tabs, params.dogbones), (None, false));
}

/// The steepest descent of a cutting move (radians), and whether any goes straight down below `top`.
fn steepest(path: &Toolpath, top: f64) -> (f64, bool) {
    let pts = path.points(0.002);
    let (mut angle, mut plunges) = (0.0f64, false);
    for w in pts.windows(2) {
        let (a, b) = (w[0].0, w[1].0);
        if !w[1].1 || b.z >= a.z - 1e-9 || a.z > top + 1e-9 {
            continue;
        }
        let run = (b.x - a.x).hypot(b.y - a.y);
        plunges |= run < 1e-9;
        angle = angle.max((a.z - b.z).atan2(run));
    }
    (angle, plunges)
}

#[test]
fn ramps_replace_plunges_into_material() {
    let ramp = 3.0f64.to_radians();
    let regions = [rect(0.0, 0.0, 40.0, 30.0)];
    // Round the part: each pass ramps once round the 140 + 6π loop, then cuts it flat.
    let c = cut(0.0, -6.0, 3.0);
    let params =
        ContourParams { side: Side::Outside, climb: true, stock_to_leave: 0.0, tolerance: 0.01, ramp_angle: ramp, tabs: None, dogbones: false };
    let path = contour(&regions, &tool(6.0), &params, &c).unwrap();
    let lap = 140.0 + 6.0 * PI;
    close(path.stats().cut_length, 2.0 * (lap.hypot(3.0) + lap) + 2.0, 1e-6);
    let (angle, plunges) = steepest(&path, 0.0);
    assert!(!plunges && angle <= ramp + 1e-6 && angle > 0.01, "{angle} {plunges}");
    assert_eq!(path.stats().lowest_z, Some(-6.0));

    // In a pocket the first ring is short, so the ramp goes round it several times.
    let params = PocketParams { stepover: 2.5, climb: true, stock_to_leave: 0.0, tolerance: 0.01, ramp_angle: ramp, dogbones: false };
    let path = pocket(&regions, &tool(6.0), &params, &cut(0.0, -5.0, 2.5)).unwrap();
    let (angle, plunges) = steepest(&path, 0.0);
    assert!(!plunges && angle <= ramp + 1e-6 && angle > 0.01, "{angle} {plunges}");
    assert_eq!(path.stats().lowest_z, Some(-5.0));
    for p in cutting_points(&path, 0.0) {
        assert!(p.x >= 3.0 - 1e-6 && p.x <= 37.0 + 1e-6 && p.y >= 3.0 - 1e-6 && p.y <= 27.0 + 1e-6, "gouge at {p:?}");
    }
    // The floor is still cleared: the flat pass follows the ramp.
    for i in 0..=34 {
        for j in 0..=24 {
            let q = v(3.0 + i as f64, 3.0 + j as f64);
            assert!(dist_to_cuts(&path, -5.0 + 1e-6, q) <= 3.0 + 0.01, "{q:?}");
        }
    }
    // A ring too short to ramp along is plunged into rather than refused.
    let tight = pocket(&[rect(0.0, 0.0, 6.004, 6.004)], &tool(6.0), &params, &cut(0.0, -5.0, 2.5)).unwrap();
    assert_eq!(tight.stats().lowest_z, Some(-5.0));
    assert!(steepest(&tight, 0.0).1);
    assert!(pocket(&regions, &tool(6.0), &PocketParams { ramp_angle: 2.0, ..params }, &c).is_err());
}

#[test]
fn pocket_with_passes_up_to_a_diameter_apart_still_clears_the_middle() {
    // A 40 × 30 pocket, a 6 mm tool, passes 5 apart: rings 3, 8 and 13 from the walls. The last
    // leaves a strip down the middle (15 from the long walls) more than a radius from it.
    let regions = [rect(0.0, 0.0, 40.0, 30.0)];
    for (stepover, name) in [(5.0, "wide"), (6.0, "a diameter"), (4.0, "moderate")] {
        let params = PocketParams { stepover, climb: true, stock_to_leave: 0.0, tolerance: 0.01, ramp_angle: 0.0, dogbones: false };
        let path = pocket(&regions, &tool(6.0), &params, &cut(0.0, -2.0, 2.0)).unwrap();
        for p in cutting_points(&path, 0.0) {
            assert!(p.x >= 3.0 - 1e-6 && p.x <= 37.0 + 1e-6 && p.y >= 3.0 - 1e-6 && p.y <= 27.0 + 1e-6, "{name}: gouge at {p:?}");
        }
        for i in 0..=68 {
            for j in 0..=48 {
                let q = v(3.0 + i as f64 * 0.5, 3.0 + j as f64 * 0.5);
                let d = dist_to_cuts(&path, -2.0 + 1e-6, q);
                assert!(d <= 3.0 + 0.01, "{name}: {q:?} is {d} from the nearest pass");
            }
        }
    }
    // Two rooms joined by a wide door, one room narrower: each is cleared to its own middle, round an island too.
    let rooms = Region2 {
        outer: polygon(&[(0.0, 0.0), (30.0, 0.0), (30.0, 8.0), (60.0, 8.0), (60.0, 26.0), (30.0, 26.0), (30.0, 40.0), (0.0, 40.0)]).outer,
        holes: vec![Loop2::circle(v(15.0, 20.0), 4.0)],
    };
    let params = PocketParams { stepover: 5.5, climb: true, stock_to_leave: 0.0, tolerance: 0.01, ramp_angle: 0.0, dogbones: false };
    let path = pocket(std::slice::from_ref(&rooms), &tool(6.0), &params, &cut(0.0, -2.0, 2.0)).unwrap();
    let shape = Shape::from_regions(std::slice::from_ref(&rooms), 0.01).unwrap();
    for p in cutting_points(&path, 0.0) {
        assert!(shape.clear(v(p.x, p.y), 3.0 - 1e-4), "gouge at {p:?}");
    }
    for i in 0..=120 {
        for j in 0..=80 {
            let q = v(i as f64 * 0.5, j as f64 * 0.5);
            if shape.clear(q, 3.0 + 1e-3) {
                let d = dist_to_cuts(&path, -2.0 + 1e-6, q);
                assert!(d <= 3.0 + 0.01, "{q:?} is {d} from the nearest pass");
            }
        }
    }
    let too_wide = PocketParams { stepover: 6.5, ..params };
    assert!(pocket(&regions, &tool(6.0), &too_wide, &cut(0.0, -2.0, 2.0)).is_err(), "passes more than a diameter apart");
}

/// Two holes plunged at 200 mm/min with a quarter-second dwell, written out move by move.
fn drill_program() -> Program {
    let t = Tool::new(3, ToolKind::Drill, 5.0, 40.0);
    let at = |x: f64, z: f64| Vec3::new(x, 10.0, z);
    let mut moves = Vec::new();
    for (i, x) in [10.0, 30.5].into_iter().enumerate() {
        if i > 0 {
            moves.push(Move::Rapid { to: at(10.0, 10.0) });
        }
        moves.extend([
            Move::Rapid { to: at(x, 10.0) },
            Move::Rapid { to: at(x, 2.0) },
            Move::Feed { to: at(x, -10.0), feed: 200.0 },
            Move::Dwell { seconds: 0.25 },
        ]);
    }
    moves.push(Move::Rapid { to: at(30.5, 10.0) });
    let path = Toolpath { moves };
    verify(&path, &cut(0.0, -10.0, 1.0).heights).unwrap();
    Program {
        work_offset: None,
        name: "Bracket (rev 2)".into(),
        operations: vec![Operation { name: "Drill".into(), tool: t, spindle_rpm: 3000.0, coolant: true, path }],
    }
}

/// A post for a controller that wants a decimal point on every number, two-digit motion codes,
/// dwells as `G04 X` in seconds and helices as lines.
fn padded() -> Post {
    Post {
        name: "padded".into(),
        trailing_decimal: true,
        two_digit_codes: true,
        helical_arcs: false,
        dwell: "G04 X{seconds}".into(),
        ..Post::rs274()
    }
}

#[test]
fn post_switches_change_how_numbers_and_codes_are_written() {
    let text = emit(&drill_program(), &padded()).unwrap();
    for line in ["G00 X10. Y10.", "G00 Z2.", "G01 Z-10. F200.", "G04 X0.25", "G00 X30.5"] {
        assert!(text.lines().any(|l| l == line), "no `{line}` in\n{text}");
    }
    let back = Toolpath { moves: parse(&text).unwrap() };
    assert_eq!(back.stats(), drill_program().operations[0].path.stats());
}

#[test]
fn rs274_program_text() {
    let text = emit(&drill_program(), &Post::rs274()).unwrap();
    let expected = "\
(Bracket rev 2)
G17 G21 G40 G49 G54 G80 G90 G94
(Drill: T3 D5 drill)
M5
T3 M6
G43 H3
S3000 M3
M8
G0 Z10
G0 X10 Y10
G0 Z2
G1 Z-10 F200
G4 P0.25
G0 Z10
G0 X30.5
G0 Z2
G1 Z-10
G4 P0.25
G0 Z10
M9
M5
M30
";
    assert_eq!(text, expected);
}

#[test]
fn written_program_reads_back_as_the_same_path() {
    let island = Loop2::circle(v(30.0, 20.0), 5.0);
    let regions = [Region2 { outer: rect(0.0, 0.0, 60.0, 40.0).outer, holes: vec![island] }];
    let params = PocketParams { stepover: 3.0, climb: true, stock_to_leave: 0.0, tolerance: 0.01, ramp_angle: 0.0, dogbones: false };
    let path = pocket(&regions, &tool(6.0), &params, &cut(0.0, -4.0, 2.0)).unwrap();
    let program = Program {
        work_offset: None,
        name: "p".into(),
        operations: vec![Operation { name: "Pocket".into(), tool: tool(6.0), spindle_rpm: 12000.0, coolant: false, path: path.clone() }],
    };
    for arcs in [true, false] {
        let mut post = Post::rs274();
        post.arcs = arcs;
        let text = emit(&program, &post).unwrap();
        assert_eq!(text.contains("G2 ") || text.contains("G3 "), arcs);
        let back = Toolpath { moves: parse(&text).unwrap() };
        verify(&back, &cut(0.0, -4.0, 2.0).heights).unwrap();
        let (a, b) = (path.stats(), back.stats());
        // Coordinates are written to 0.0001 mm.
        close(a.cut_length, b.cut_length, 0.2);
        close(a.rapid_length, b.rapid_length, 0.2);
        assert_eq!(a.lowest_z, b.lowest_z);
        // Every point of the path as read is on the path as planned.
        let planned: Vec<Vec3> = path.points(0.002).into_iter().map(|(p, _)| p).collect();
        for (p, _) in back.points(0.002) {
            let d = planned.windows(2).map(|w| p.dist_to_segment(w[0], w[1])).fold(f64::INFINITY, f64::min);
            assert!(d < 0.02, "{p:?} is {d} off the planned path");
        }
    }
}

#[test]
fn every_post_writes_within_its_own_precision() {
    let island = Loop2::circle(v(30.0, 20.0), 5.0);
    let regions = [Region2 { outer: rect(0.0, 0.0, 60.0, 40.0).outer, holes: vec![island] }];
    let params = PocketParams { stepover: 3.0, climb: true, stock_to_leave: 0.0, tolerance: 0.01, ramp_angle: 3.0f64.to_radians(), dogbones: false };
    let path = pocket(&regions, &tool(6.0), &params, &cut(0.0, -4.0, 2.0)).unwrap();
    let mut program = Program {
        work_offset: None,
        name: "p".into(),
        operations: vec![Operation { name: "Pocket".into(), tool: tool(6.0), spindle_rpm: 12000.0, coolant: false, path: path.clone() }],
    };
    let mut inch = Post::rs274();
    inch.units = Units::Inch;
    let mut chords = Post::rs274();
    chords.arcs = false;
    for post in [Post::rs274(), inch, chords, padded()] {
        let text = emit(&program, &post).unwrap();
        let r = readback(&text, &program, &post).unwrap();
        assert!(r.within(), "{}: {r:?}", post.name);
        close(r.cut_length, r.planned_cut_length, 0.05 * r.planned_cut_length / 100.0 + 0.5);
    }
    // The precision a post allows: four decimals in millimetres, 0.005 mm chords.
    close(allowed_deviation(&Post::rs274()), 3.0 * 0.00005 * 3f64.sqrt() + 0.01 + 1e-6, 1e-12);

    // A post that rounds to whole millimetres keeps to its own precision, but that precision
    // is coarse: the program strays tenths of a millimetre, which the read-back measures.
    let mut coarse = Post::rs274();
    coarse.decimals = 0;
    let r = readback(&emit(&program, &coarse).unwrap(), &program, &coarse).unwrap();
    assert!(r.within() && r.max_deviation > 0.3 && r.allowed > 2.0, "{r:?}");

    // A program that is not what was written: every move 0.5 mm higher.
    let text = emit(&program, &Post::rs274()).unwrap();
    for m in &mut program.operations[0].path.moves {
        if let Move::Rapid { to } | Move::Feed { to, .. } | Move::Arc { to, .. } = m {
            to.z += 0.5;
        }
    }
    let r = readback(&text, &program, &Post::rs274()).unwrap();
    assert!(!r.within(), "{r:?}");
    close(r.max_deviation, 0.5, 0.01);
    // A program with no cut against one with cuts: as far as the comparison looks.
    let empty = Program {
        work_offset: None,
        name: "e".into(),
        operations: vec![Operation {
            path: Toolpath { moves: vec![Move::Rapid { to: Vec3::new(0.0, 0.0, 10.0) }] },
            ..program.operations[0].clone()
        }],
    };
    let r = readback(&text, &empty, &Post::rs274()).unwrap();
    assert_eq!(r.max_deviation, r.reach);
}

#[test]
fn arc_too_short_to_write_is_never_a_full_circle() {
    // 0.00002 mm of arc: its end is written the same as its start.
    let moves = vec![
        Move::Rapid { to: Vec3::new(10.0, 0.0, 5.0) },
        Move::Feed { to: Vec3::new(10.0, 0.0, -1.0), feed: 100.0 },
        Move::Arc { to: Vec3::new(10.0, 0.00002, -1.0), center: v(0.0, 0.0), ccw: true, feed: 100.0 },
    ];
    let program = Program {
        work_offset: None,
        name: "p".into(),
        operations: vec![Operation { name: "o".into(), tool: tool(6.0), spindle_rpm: 1000.0, coolant: false, path: Toolpath { moves } }],
    };
    let text = emit(&program, &Post::rs274()).unwrap();
    assert!(!text.contains("G2 ") && !text.contains("G3 "), "{text}");
}

/// A helix of 270° counter-clockwise from -1 down to -2, then 300° clockwise at -2, both 20 mm
/// round the origin.
fn long_arcs_program() -> Program {
    let at = |deg: f64, z: f64| (Vec2::from_angle(deg.to_radians()) * 20.0).extend(z);
    let moves = vec![
        Move::Rapid { to: at(0.0, 10.0) },
        Move::Rapid { to: at(0.0, 2.0) },
        Move::Feed { to: at(0.0, -1.0), feed: 200.0 },
        Move::Arc { to: at(270.0, -2.0), center: v(0.0, 0.0), ccw: true, feed: 600.0 },
        Move::Arc { to: at(-30.0, -2.0), center: v(0.0, 0.0), ccw: false, feed: 600.0 },
        Move::Rapid { to: at(-30.0, 10.0) },
    ];
    let path = Toolpath { moves };
    verify(&path, &cut(0.0, -2.0, 1.0).heights).unwrap();
    Program {
        work_offset: None,
        name: "arcs".into(),
        operations: vec![Operation { name: "Arcs".into(), tool: tool(6.0), spindle_rpm: 12000.0, coolant: false, path }],
    }
}

/// The sweep of every arc among `moves` (degrees, counter-clockwise positive).
fn arc_sweeps(moves: &[Move]) -> Vec<f64> {
    let mut pos = None;
    let mut out = Vec::new();
    for m in moves {
        if let (Move::Arc { to, center, ccw, .. }, Some(from)) = (m, pos) {
            out.push(arc_sweep(from, *to, *center, *ccw).to_degrees());
        }
        pos = m.target().or(pos);
    }
    out
}

#[test]
fn arcs_are_split_for_controllers_that_limit_their_sweep() {
    let program = long_arcs_program();
    let whole = arc_sweeps(&parse(&emit(&program, &Post::rs274()).unwrap()).unwrap());
    assert_eq!(whole.len(), 2);
    close(whole[0], 270.0, 1e-3);
    close(whole[1], -300.0, 1e-3);
    // 270° in halves of 135°, 300° in halves of 150°; by quadrant 3 × 90° and 4 × 75°.
    for (limit, parts) in [(180.0, vec![135.0, 135.0, -150.0, -150.0]), (90.0, vec![90.0, 90.0, 90.0, -75.0, -75.0, -75.0, -75.0])] {
        let post = Post { max_arc_sweep: Some(limit), ..Post::rs274() };
        let text = emit(&program, &post).unwrap();
        let sweeps = arc_sweeps(&parse(&text).unwrap());
        assert_eq!(sweeps.len(), parts.len(), "{limit}°: {sweeps:?}");
        for (s, want) in sweeps.iter().zip(&parts) {
            close(*s, *want, 1e-3);
        }
        // The parts follow the helix down and end where the whole arcs end.
        let r = readback(&text, &program, &post).unwrap();
        assert!(r.within(), "{limit}°: {r:?}");
    }
    // A controller without arcs still gets chords, however the sweep is limited.
    let chords = Post { arcs: false, max_arc_sweep: Some(90.0), ..Post::rs274() };
    assert!(arc_sweeps(&parse(&emit(&program, &chords).unwrap()).unwrap()).is_empty());
}

#[test]
fn programs_the_machine_cannot_run_are_refused() {
    // One tool at 3000 rev/min.
    let mut post = Post { spindle_range: Some(SpindleRange { min: 3000.0, max: 24000.0 }), one_tool: true, ..Post::rs274() };
    assert!(emit(&drill_program(), &post).is_ok());
    post.spindle_range = Some(SpindleRange { min: 8000.0, max: 24000.0 });
    let refused = emit(&drill_program(), &post).unwrap_err().to_string();
    assert!(refused.contains("3000 rev/min") && refused.contains("8000 to 24000"), "{refused}");

    post.spindle_range = None;
    let mut two = drill_program();
    let mut second = two.operations[0].clone();
    second.tool.number = 4;
    two.operations.push(second);
    let refused = emit(&two, &post).unwrap_err().to_string();
    assert!(refused.contains("T3, T4"), "{refused}");
    assert!(emit(&two, &Post::rs274()).is_ok(), "a machine that changes tools takes both");
    // The same tool twice is still one tool.
    two.operations[1].tool.number = 3;
    assert!(emit(&two, &post).is_ok());

    // The holes are 20.5 mm apart and the tool goes from 10 above the top to 10 below it.
    let travel = |x, y, z| Post { travel: Some(Travel { x, y, z }), ..Post::rs274() };
    assert!(emit(&drill_program(), &travel(20.5, 0.1, 20.0)).is_ok(), "exactly the travel fits");
    let refused = emit(&drill_program(), &travel(20.0, 600.0, 200.0)).unwrap_err().to_string();
    assert!(refused.contains("20.5 mm in X") && refused.contains("20 mm"), "{refused}");
    let refused = emit(&drill_program(), &travel(900.0, 600.0, 15.0)).unwrap_err().to_string();
    assert!(refused.contains("20 mm in Z"), "{refused}");
}

#[test]
fn verify_refuses_unsafe_paths() {
    let h = cut(0.0, -5.0, 1.0).heights;
    let at = |x: f64, z: f64| Vec3::new(x, 0.0, z);
    let unsafe_paths = [
        // Starts below the clearance height.
        vec![Move::Rapid { to: at(0.0, 1.0) }],
        // Starts with a cut.
        vec![Move::Feed { to: at(0.0, 10.0), feed: 100.0 }],
        // Rapid down into uncut material.
        vec![Move::Rapid { to: at(0.0, 10.0) }, Move::Rapid { to: at(0.0, -1.0) }],
        // Sideways rapid below the retract height.
        vec![Move::Rapid { to: at(0.0, 10.0) }, Move::Feed { to: at(0.0, -1.0), feed: 100.0 }, Move::Rapid { to: at(5.0, -1.0) }],
        // Below the bottom.
        vec![Move::Rapid { to: at(0.0, 10.0) }, Move::Feed { to: at(0.0, -5.1), feed: 100.0 }],
        // No feed.
        vec![Move::Rapid { to: at(0.0, 10.0) }, Move::Feed { to: at(0.0, -1.0), feed: 0.0 }],
        // Not a number.
        vec![Move::Rapid { to: at(f64::NAN, 10.0) }],
        // An arc whose ends are at different radii.
        vec![Move::Rapid { to: at(5.0, 10.0) }, Move::Arc { to: at(-7.0, 10.0), center: v(0.0, 0.0), ccw: true, feed: 100.0 }],
    ];
    for moves in unsafe_paths {
        assert!(matches!(verify(&Toolpath { moves: moves.clone() }, &h), Err(CamError::Unsafe(_))), "{moves:?}");
    }
    // A drill returning into its own hole is fine.
    let ok = vec![
        Move::Rapid { to: at(0.0, 10.0) },
        Move::Feed { to: at(0.0, -3.0), feed: 100.0 },
        Move::Rapid { to: at(0.0, 2.0) },
        Move::Rapid { to: at(0.0, -2.5) },
        Move::Feed { to: at(0.0, -5.0), feed: 100.0 },
    ];
    verify(&Toolpath { moves: ok }, &h).unwrap();
}

#[test]
fn hostile_values_are_errors_never_panics() {
    let bad = [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, 0.0, -1.0, 1e300, -1e300, 1e-300, f64::MIN_POSITIVE];
    let regions = [rect(0.0, 0.0, 40.0, 30.0)];
    let good_cut = cut(0.0, -5.0, 2.0);
    for &b in &bad {
        let _ = offset_regions(&regions, b, 0.01);
        let _ = offset_regions(&regions, 1.0, b);
        let _ = offset_regions(&[rect(0.0, 0.0, b, b)], 1.0, 0.01);
        let _ = offset_regions(&[Region2 { outer: Loop2::circle(v(b, b), b), holes: vec![Loop2::circle(v(0.0, b), b)] }], b, 0.01);
        let t = Tool::new(1, ToolKind::FlatEndmill, b, b);
        let pocket_params = PocketParams { stepover: b, climb: true, stock_to_leave: b, tolerance: b, ramp_angle: b, dogbones: false };
        let contour_params =
            ContourParams { side: Side::Outside, climb: true, stock_to_leave: b, tolerance: b, ramp_angle: b, tabs: None, dogbones: false };
        let bad_cut = Cut { heights: Heights { clearance: b, retract: b, top: b, bottom: -b }, feeds: Feeds { cut: b, plunge: b }, stepdown: b };
        for c in [good_cut, bad_cut, Cut { stepdown: b, ..good_cut }] {
            for tl in [&t, &tool(6.0)] {
                let _ = pocket(&regions, tl, &pocket_params, &c);
                let _ = pocket(
                    &regions,
                    tl,
                    &PocketParams { stepover: 2.0, climb: true, stock_to_leave: 0.0, tolerance: 0.01, ramp_angle: 0.0, dogbones: false },
                    &c,
                );
                let _ = contour(&regions, tl, &contour_params, &c);
            }
        }
        let moves = vec![
            Move::Rapid { to: Vec3::new(b, b, b) },
            Move::Arc { to: Vec3::new(b, 0.0, b), center: v(b, b), ccw: true, feed: b },
            Move::Dwell { seconds: b },
        ];
        let path = Toolpath { moves };
        let _ = path.stats();
        let _ = path.points(b);
        let _ = verify(&path, &good_cut.heights);
        let program = Program {
            work_offset: None,
            name: "x".into(),
            operations: vec![Operation { name: "o".into(), tool: tool(6.0), spindle_rpm: b, coolant: false, path }],
        };
        let _ = emit(&program, &Post::rs274());
    }
    // Things that must be refused outright.
    assert!(
        pocket(
            &regions,
            &tool(6.0),
            &PocketParams { stepover: 1e-9, climb: true, stock_to_leave: 0.0, tolerance: 0.01, ramp_angle: 0.0, dogbones: false },
            &good_cut
        )
        .is_err()
    );
    assert!(
        pocket(
            &regions,
            &tool(60.0),
            &PocketParams { stepover: 2.0, climb: true, stock_to_leave: 0.0, tolerance: 0.01, ramp_angle: 0.0, dogbones: false },
            &good_cut
        )
        .is_err()
    );
    assert!(
        pocket(
            &[],
            &tool(6.0),
            &PocketParams { stepover: 2.0, climb: true, stock_to_leave: 0.0, tolerance: 0.01, ramp_angle: 0.0, dogbones: false },
            &good_cut
        )
        .is_err()
    );
    // A drill cuts only at its point: no contour or pocket with one.
    let drill = Tool::new(4, ToolKind::Drill, 6.0, 30.0);
    let square = [rect(0.0, 0.0, 20.0, 20.0)];
    let side = ContourParams { side: Side::Outside, climb: true, stock_to_leave: 0.0, tolerance: 0.01, ramp_angle: 0.0, tabs: None, dogbones: false };
    assert!(contour(&square, &drill, &side, &good_cut).unwrap_err().to_string().contains("a drill cannot mill"));
    let lanes = PocketParams { stepover: 2.0, climb: true, stock_to_leave: 0.0, tolerance: 0.01, ramp_angle: 0.0, dogbones: false };
    assert!(pocket(&square, &drill, &lanes, &good_cut).is_err());
    let open = Region2 { outer: Loop2 { segs: rect(0.0, 0.0, 10.0, 10.0).outer.segs.into_iter().take(3).collect() }, holes: vec![] };
    assert!(offset_regions(&[open], 1.0, 0.01).is_err());
    assert!(emit(&Program::default(), &Post::rs274()).is_err());
}

#[test]
fn posts_are_data_and_bad_ones_are_refused() {
    let mut own = Post::rs274();
    (own.name, own.units, own.line_numbers) = ("own".into(), Units::Inch, Some(LineNumbers { start: 10, step: 5 }));
    let json = serde_json::to_string(&own).unwrap();
    let post: Post = serde_json::from_str(&json).unwrap();
    assert_eq!(post, own);
    assert_eq!(Post::builtin("LinuxCNC"), Some(Post::rs274()));
    assert!(Post::BUILTIN.iter().all(|name| Post::builtin(name).is_some_and(|p| p.validate().is_ok())));
    let broken = |change: fn(&mut Post)| {
        let mut p = Post::rs274();
        change(&mut p);
        p.validate().is_err() && emit(&drill_program(), &p).is_err()
    };
    assert!(broken(|p| p.header = vec!["G21 {nonsense}".into()]));
    assert!(broken(|p| p.footer = vec!["M30 {tool".into()]));
    assert!(broken(|p| p.tool_change = vec!["M6\nG0 Z-100".into()]));
    assert!(broken(|p| p.decimals = 9));
    assert!(broken(|p| p.arc_tolerance = f64::NAN));
    assert!(broken(|p| p.line_numbers = Some(LineNumbers { start: 1, step: 0 })));
    assert!(broken(|p| p.extension = "../x".into()));
    assert!(broken(|p| p.spindle_on = vec!["M3".into(); 1000]));
    assert!(broken(|p| p.dwell = "G4 P{minutes}".into()));
    assert!(broken(|p| p.max_arc_sweep = Some(0.5)));
    assert!(broken(|p| p.max_arc_sweep = Some(360.0)));
    assert!(broken(|p| p.max_arc_sweep = Some(f64::NAN)));
    assert!(broken(|p| p.spindle_range = Some(SpindleRange { min: 24000.0, max: 3000.0 })));
    assert!(broken(|p| p.spindle_range = Some(SpindleRange { min: f64::NAN, max: 3000.0 })));
    assert!(broken(|p| p.travel = Some(Travel { x: 900.0, y: 0.0, z: 200.0 })));
    assert!(broken(|p| p.travel = Some(Travel { x: f64::INFINITY, y: 600.0, z: 200.0 })));
    // A post saved before these switches existed reads with what it meant.
    let mut old = serde_json::to_value(Post::rs274()).unwrap();
    for k in ["helical_arcs", "trailing_decimal", "two_digit_codes", "dwell", "max_arc_sweep", "spindle_range", "one_tool", "travel"] {
        old.as_object_mut().unwrap().remove(k);
    }
    assert_eq!(serde_json::from_value::<Post>(old).unwrap(), Post::rs274());
}

#[test]
fn reader_reports_what_it_cannot_read() {
    for text in ["G1 X1 Y1 Z1", "X1 Y1 Z1", "G0 X1 Y1 Z1\nG2 X2 Y2", "G91", "G0 X1..2", "G0 X", "G0 @", "G2 X1 Y1 Z1 I1 F10"] {
        assert!(matches!(parse(text), Err(CamError::Parse(..))), "{text}");
    }
    assert_eq!(parse("%\n(a comment) ; another\nN10 g0 x1 y2 (inline) z3\n").unwrap(), vec![Move::Rapid { to: Vec3::new(1.0, 2.0, 3.0) }]);
    assert_eq!(parse("G20 G0 X1 Y0 Z0\nG1 X2 F10").unwrap().last(), Some(&Move::Feed { to: Vec3::new(50.8, 0.0, 0.0), feed: 254.0 }));
}

#[test]
fn work_offsets_go_where_the_post_writes_them() {
    let mut program = drill_program();
    program.work_offset = Some(3);
    let text = emit(&program, &Post::rs274()).unwrap();
    assert!(text.contains("\nG17 G21 G40 G49 G56 G80 G90 G94\n"), "{text}");
    // A post that leaves the offset to the operator can only stand for the first one.
    let mut manual = Post::rs274();
    manual.header = vec!["G17 {units} G40 G49 G80 G90 G94".into()];
    let refused = emit(&program, &manual).unwrap_err().to_string();
    assert!(refused.contains("writes no work offset") && refused.contains("G56"), "{refused}");
    program.work_offset = Some(1);
    assert!(emit(&program, &manual).is_ok());
    for n in [0, 7, 255] {
        program.work_offset = Some(n);
        assert!(emit(&program, &Post::rs274()).is_err());
    }
    assert_eq!(work_offset_code(6).unwrap(), "G59");
}
