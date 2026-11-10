use std::f64::consts::PI;

use solidcore_record::{Loop2, Region2, Vec2, Vec3};
use solidcraft_geom::prelude::*;

use super::*;
use crate::offset::Shape;

fn v(x: f64, y: f64) -> Vec2 {
    Vec2::new(x, y)
}

fn v2(x: f64, y: f64) -> Vec2 {
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
    let params = PocketParams { stepover: 2.5, climb: true, stock_to_leave: 0.0, tolerance: 0.01, ramp_angle: 0.0, dogbones: false, rest: None };
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
    let params = PocketParams { stepover: 3.0, climb: false, stock_to_leave: 0.5, tolerance: 0.01, ramp_angle: 0.0, dogbones: false, rest: None };
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
        let params = ContourParams {
            side: Side::Outside,
            climb,
            stock_to_leave: 0.0,
            tolerance: 0.01,
            ramp_angle: 0.0,
            tabs: None,
            dogbones: false,
            rest: None,
        };
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
    let inside = ContourParams {
        side: Side::Inside,
        climb: true,
        stock_to_leave: 0.0,
        tolerance: 0.01,
        ramp_angle: 0.0,
        tabs: None,
        dogbones: false,
        rest: None,
    };
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
    let params = ContourParams {
        side: Side::Outside,
        climb: true,
        stock_to_leave: 0.0,
        tolerance: 0.01,
        ramp_angle: 0.0,
        tabs: None,
        dogbones: false,
        rest: None,
    };
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
        let params =
            ContourParams { side: Side::Outside, climb, stock_to_leave: 0.0, tolerance: 0.01, ramp_angle: 0.0, tabs: None, dogbones, rest: None };
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
    let params = ContourParams {
        side: Side::Inside,
        climb: true,
        stock_to_leave: 0.0,
        tolerance: 0.01,
        ramp_angle: 0.0,
        tabs: None,
        dogbones: true,
        rest: None,
    };
    let window = contour(&[rect(0.0, 0.0, 30.0, 20.0)], &tool(6.0), &params, &c).unwrap();
    for (x, y) in [(0.0, 0.0), (30.0, 0.0), (30.0, 20.0), (0.0, 20.0)] {
        close(dist_to_cuts(&window, -5.9, v(x, y)), 3.0, 1e-6);
    }

    // A pocket made for a square part, its corners notched along the walls.
    let housing = [rect(0.0, 0.0, 40.0, 18.0)];
    let pocket_params =
        PocketParams { stepover: 2.5, climb: true, stock_to_leave: 0.0, tolerance: 0.01, ramp_angle: 0.0, dogbones: true, rest: None };
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
    let params = ContourParams {
        side: Side::Outside,
        climb: true,
        stock_to_leave: 0.0,
        tolerance: 0.01,
        ramp_angle: 0.0,
        tabs: None,
        dogbones: true,
        rest: None,
    };
    let with = contour(&parts, &tool(6.0), &params, &c).unwrap();
    assert!(tight(&with, &shape, &[]) >= 3.0 - 0.006, "{}", tight(&with, &shape, &[]));
    let plain = contour(&parts, &tool(6.0), &ContourParams { dogbones: false, ..params }, &c).unwrap();
    close(with.stats().cut_length, plain.stats().cut_length, 1e-9);

    // A 50 × 30 pocket with an island 4 mm from its lower wall: the pocket's own corners are
    // notched, nothing cuts into the island or the wall beside it.
    let housing = [Region2 { outer: rect(0.0, 0.0, 50.0, 30.0).outer, holes: vec![rect(10.0, 4.0, 40.0, 20.0).outer.reversed()] }];
    let shape = Shape::from_regions(&housing, 0.01).unwrap();
    let pp = PocketParams { stepover: 2.5, climb: true, stock_to_leave: 0.0, tolerance: 0.01, ramp_angle: 0.0, dogbones: true, rest: None };
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
                rest: None,
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
    let cp = ContourParams {
        side: Side::Outside,
        climb: true,
        stock_to_leave: 0.0,
        tolerance: 0.01,
        ramp_angle: 0.0,
        tabs: None,
        dogbones: false,
        rest: None,
    };
    let pp = PocketParams { stepover: 1.5, climb: true, stock_to_leave: 0.0, tolerance: 0.01, ramp_angle: 0.0, dogbones: false, rest: None };
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
        let params = ContourParams {
            side: Side::Outside,
            climb: true,
            stock_to_leave: 0.0,
            tolerance: 0.01,
            ramp_angle: ramp,
            tabs,
            dogbones: false,
            rest: None,
        };
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
    let params = ContourParams {
        side: Side::Outside,
        climb: true,
        stock_to_leave: 0.0,
        tolerance: 0.01,
        ramp_angle: ramp,
        tabs: None,
        dogbones: false,
        rest: None,
    };
    let path = contour(&regions, &tool(6.0), &params, &c).unwrap();
    let lap = 140.0 + 6.0 * PI;
    close(path.stats().cut_length, 2.0 * (lap.hypot(3.0) + lap) + 2.0, 1e-6);
    let (angle, plunges) = steepest(&path, 0.0);
    assert!(!plunges && angle <= ramp + 1e-6 && angle > 0.01, "{angle} {plunges}");
    assert_eq!(path.stats().lowest_z, Some(-6.0));

    // In a pocket the first ring is short, so the ramp goes round it several times.
    let params = PocketParams { stepover: 2.5, climb: true, stock_to_leave: 0.0, tolerance: 0.01, ramp_angle: ramp, dogbones: false, rest: None };
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
        let params = PocketParams { stepover, climb: true, stock_to_leave: 0.0, tolerance: 0.01, ramp_angle: 0.0, dogbones: false, rest: None };
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
    let params = PocketParams { stepover: 5.5, climb: true, stock_to_leave: 0.0, tolerance: 0.01, ramp_angle: 0.0, dogbones: false, rest: None };
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

/// The lab's engraving tools: a V-bit, a chamfer mill and a tapered ball end mill.
fn vbit(angle: f64, diameter: f64) -> Tool {
    Tool { tip_angle: Some(angle), ..Tool::new(14, ToolKind::VBit, diameter, 4.5) }
}

/// A V-bit with cutting edges long enough for a deep carving.
fn long_vbit(angle: f64, diameter: f64) -> Tool {
    Tool { tip_angle: Some(angle), ..Tool::new(15, ToolKind::VBit, diameter, 20.0) }
}

fn tapered_ball() -> Tool {
    Tool { taper_angle: Some(5.05), tip_diameter: 0.5, ..Tool::new(10, ToolKind::TaperedBall, 3.175, 15.0) }
}

#[test]
fn vbit_and_tapered_ball_tools_validate_and_report_their_width() {
    // An 80° V-bit widens by 2·tan 40° per millimetre of depth, up to its 5 mm diameter.
    let t14 = vbit(80.0, 5.0);
    t14.validate().unwrap();
    close(t14.width_at_depth(0.0), 0.0, 1e-9);
    close(t14.width_at_depth(1.0), 2.0 * 40f64.to_radians().tan(), 1e-9);
    close(t14.width_at_depth(1.0), 1.678, 1e-3);
    close(t14.width_at_depth(10.0), 5.0, 1e-9);
    close(t14.radius_at(1.0), 0.839, 1e-3);
    close(t14.floor_diameter(), 0.0, 1e-9);
    assert_eq!(t14.describe(), "V-bit 80deg");
    // A 60° chamfer mill, capped at its 12.7 mm diameter: 11 mm down it is already full width.
    let t12 = vbit(60.0, 12.7);
    close(t12.width_at_depth(11.0), 12.7, 1e-9);
    close(t12.width_at_depth(2.0), 4.0 * 30f64.to_radians().tan(), 1e-9);
    // A tip flat adds to the width everywhere and is what clears a floor.
    let flat = Tool { tip_diameter: 0.5, ..vbit(80.0, 5.0) };
    close(flat.width_at_depth(1.0), 0.5 + 1.678, 1e-3);
    close(flat.floor_diameter(), 0.5, 1e-9);
    assert_eq!(flat.describe(), "V-bit 80deg tip 0.5");
    // The tapered ball end mill: its R0.25 ball to a quarter millimetre down, then the 5.05° cone.
    let t10 = tapered_ball();
    t10.validate().unwrap();
    close(t10.width_at_depth(0.25), 0.5, 1e-9);
    close(t10.width_at_depth(1.5), 0.5 + 2.0 * 1.25 * 5.05f64.to_radians().tan(), 1e-9);
    close(t10.width_at_depth(1.5), 0.721, 1e-3);
    close(t10.width_at_depth(100.0), 3.175, 1e-9);
    close(t10.floor_diameter(), 0.5, 1e-9);
    assert_eq!(t10.describe(), "tapered ball R0.25 5.05deg");
    // A ball end mill is narrower than its diameter above its equator, full width below it.
    let ball = Tool::new(2, ToolKind::BallEndmill, 6.0, 20.0);
    close(ball.width_at_depth(1.0), 2.0 * (9f64 - 4.0).sqrt(), 1e-9);
    close(ball.width_at_depth(3.0), 6.0, 1e-9);
    close(ball.width_at_depth(5.0), 6.0, 1e-9);
    close(tool(6.0).width_at_depth(0.1), 6.0, 1e-9);
    // Angles out of range, a missing angle, a tip wider than the tool, an angle on a flat end mill.
    for bad in [
        vbit(0.0, 5.0),
        vbit(180.0, 5.0),
        vbit(f64::NAN, 5.0),
        Tool { tip_angle: None, ..vbit(80.0, 5.0) },
        Tool { tip_diameter: 5.0, ..vbit(80.0, 5.0) },
        Tool { tip_diameter: -0.1, ..vbit(80.0, 5.0) },
        Tool { taper_angle: Some(90.0), ..tapered_ball() },
        Tool { taper_angle: None, ..tapered_ball() },
        Tool { tip_diameter: 0.0, ..tapered_ball() },
        Tool { tip_diameter: 4.0, ..tapered_ball() },
        Tool { tip_angle: Some(60.0), ..tool(6.0) },
        Tool { taper_angle: Some(5.0), ..tool(6.0) },
        Tool { tip_diameter: 1.0, ..tool(6.0) },
    ] {
        assert!(bad.validate().is_err(), "{bad:?}");
    }
}

#[test]
fn a_vbit_contour_offsets_by_its_width_at_depth_and_cannot_pocket_without_a_flat() {
    let regions = [rect(0.0, 0.0, 40.0, 30.0)];
    let t14 = vbit(80.0, 5.0);
    let c = cut(0.0, -2.0, 2.0);
    let params = ContourParams {
        side: Side::Outside,
        climb: true,
        stock_to_leave: 0.0,
        tolerance: 0.01,
        ramp_angle: 0.0,
        tabs: None,
        dogbones: false,
        rest: None,
    };
    let path = contour(&regions, &t14, &params, &c).unwrap();
    // Round the outside, the centre runs the tool's half-width at the top of the cut from the
    // wall: the top edge of the chamfer is on the outline.
    let away = t14.radius_at(2.0);
    close(away, 40f64.to_radians().tan() * 2.0, 1e-9);
    let shape = Shape::from_regions(&regions, 0.01).unwrap();
    for p in cutting_points(&path, -1e-9) {
        close(shape.dist(Vec2::new(p.x, p.y)), away, 1e-3);
    }
    // A sharp V-bit cannot clear a floor; with a tip flat it pockets, stepping at most the flat.
    let pocket_params =
        PocketParams { stepover: 0.3, climb: true, stock_to_leave: 0.0, tolerance: 0.01, ramp_angle: 0.0, dogbones: false, rest: None };
    let err = pocket(&regions, &t14, &pocket_params, &c).unwrap_err().to_string();
    assert!(err.contains("tip"), "{err}");
    let flat = Tool { tip_diameter: 1.0, ..vbit(80.0, 5.0) };
    let too_wide = PocketParams { stepover: 1.2, ..pocket_params };
    assert!(pocket(&[rect(0.0, 0.0, 10.0, 8.0)], &flat, &too_wide, &c).unwrap_err().to_string().contains("tip"));
    let ok = PocketParams { stepover: 0.8, ..pocket_params };
    let path = pocket(&[rect(0.0, 0.0, 10.0, 8.0)], &flat, &ok, &c).unwrap();
    // The wall ring runs the width at the top of the cut from the wall; the tip clears inside it.
    let wall = flat.radius_at(2.0);
    let shape = Shape::from_regions(&[rect(0.0, 0.0, 10.0, 8.0)], 0.01).unwrap();
    for p in cutting_points(&path, -1e-9) {
        assert!(shape.dist(Vec2::new(p.x, p.y)) >= wall - 1e-3, "{p:?} is inside the wall offset {wall}");
    }
}

#[test]
fn tool_comments_name_the_angle() {
    let mut program = drill_program();
    program.operations[0].tool = Tool { tip_angle: Some(90.0), ..Tool::new(5, ToolKind::VBit, 6.0, 4.0) };
    let text = emit(&program, &Post::rs274()).unwrap();
    assert!(text.contains("T5 D6 V-bit 90deg"), "{text}");
    program.operations[0].tool = tapered_ball();
    let text = emit(&program, &Post::rs274()).unwrap();
    assert!(text.contains("T10 D3.175 tapered ball R0.25 5.05deg"), "{text}");
}

fn chain(pts: &[(f64, f64)], closed: bool) -> Chain {
    Chain { points: pts.iter().map(|p| v(p.0, p.1)).collect(), closed }
}

/// The XY distance from `q` to the nearest point of any chain's segments.
fn dist_to_chains(chains: &[Chain], q: Vec2) -> f64 {
    let mut best = f64::INFINITY;
    for c in chains {
        let n = c.points.len();
        let pairs = if c.closed { n } else { n - 1 };
        for i in 0..pairs {
            let (a, b) = (c.points[i], c.points[(i + 1) % n]);
            best = best.min(q.extend(0.0).dist_to_segment(a.extend(0.0), b.extend(0.0)));
        }
    }
    best
}

#[test]
fn engrave_cuts_every_chain_at_every_level_and_reverses_open_ones() {
    let open = chain(&[(0.0, 0.0), (10.0, 0.0), (10.0, 5.0)], false);
    let square = chain(&[(20.0, 0.0), (30.0, 0.0), (30.0, 10.0), (20.0, 10.0)], true);
    let chains = [open.clone(), square.clone()];
    let c = cut(0.0, -1.0, 0.5);
    let path = engrave(&chains, &tool(2.0), &EngraveParams { tolerance: 0.01 }, &c).unwrap();
    // Every cutting point lies on a chain: nothing is offset.
    for p in cutting_points(&path, -1e-9) {
        close(dist_to_chains(&chains, Vec2::new(p.x, p.y)), 0.0, 1e-6);
    }
    // Two levels on each chain: four vertical feeds into the material, none of them a travel.
    let plunges: Vec<_> = path
        .moves
        .windows(2)
        .filter(|w| matches!((w[0].target(), w[1]), (Some(a), Move::Feed { to, .. }) if Vec2::new(a.x, a.y).dist(Vec2::new(to.x, to.y)) <= 1e-9 && to.z < a.z && to.z < 0.0))
        .collect();
    assert_eq!(plunges.len(), 4, "{plunges:?}");
    // The open chain's second level runs back from its far end: after reaching (10, 5) at −0.5
    // the tool goes down there and comes back to (0, 0) at −1, never travelling in between.
    let feeds: Vec<Vec3> = path
        .moves
        .iter()
        .filter_map(|m| match m {
            Move::Feed { to, .. } => Some(*to),
            _ => None,
        })
        .collect();
    let at_end = feeds.iter().position(|p| p.x == 10.0 && p.y == 5.0 && p.z == -0.5).unwrap();
    assert_eq!(feeds[at_end + 1], Vec3::new(10.0, 5.0, -1.0), "{feeds:?}");
    assert_eq!(feeds[at_end + 3], Vec3::new(0.0, 0.0, -1.0), "{feeds:?}");
    // The square comes back to its start on each level.
    let on_square: Vec<Vec3> = feeds.iter().copied().filter(|p| p.x >= 20.0).collect();
    assert_eq!(on_square.iter().filter(|p| p.x == 20.0 && p.y == 0.0 && p.z == -1.0).count(), 2, "{on_square:?}");
    // Rapids only at the clearance or the retract height, except straight down into the start.
    for m in &path.moves {
        if let Move::Rapid { to } = m {
            assert!(to.z >= 2.0 - 1e-9, "{to:?}");
        }
    }
    verify(&path, &c.heights).unwrap();
}

#[test]
fn engrave_joins_touching_pieces_and_starts_nearest() {
    // Three sides of a square, drawn apart within the tolerance, and the fourth: one closed chain.
    let pieces = [
        chain(&[(0.0, 0.0), (10.0, 0.0)], false),
        chain(&[(10.0, 10.0), (0.0, 10.0)], false),
        chain(&[(10.0, 0.005), (10.0, 10.0)], false),
        chain(&[(0.0, 10.0), (0.0, 0.0)], false),
        // A far stroke whose end is nearer to the square than its start: it is run backwards.
        chain(&[(80.0, 0.0), (40.0, 0.0)], false),
    ];
    let c = cut(0.0, -0.5, 1.0);
    let path = engrave(&pieces, &tool(1.0), &EngraveParams { tolerance: 0.01 }, &c).unwrap();
    // Exactly two travels across at the clearance height (one per chain), so the square was joined.
    let mut travels = 0;
    let mut last: Option<Vec3> = None;
    for m in &path.moves {
        if let Move::Rapid { to } = m
            && to.z == 10.0
            && last.is_none_or(|q| Vec2::new(q.x, q.y).dist(Vec2::new(to.x, to.y)) > 1e-9)
        {
            travels += 1;
        }
        last = m.target().or(last);
    }
    assert_eq!(travels, 2, "{:?}", path.moves);
    let feeds: Vec<Vec3> = path
        .moves
        .iter()
        .filter_map(|m| match m {
            Move::Feed { to, .. } => Some(*to),
            _ => None,
        })
        .collect();
    let far: Vec<&Vec3> = feeds.iter().filter(|p| p.x >= 40.0 && p.z < 0.0).collect();
    assert_eq!((far[0].x, far[far.len() - 1].x), (40.0, 80.0), "the far stroke runs from its near end: {far:?}");
    // Pieces too far apart stay apart: two chains, and the gap is never cut.
    let apart = [chain(&[(0.0, 0.0), (10.0, 0.0)], false), chain(&[(10.0, 1.0), (10.0, 10.0)], false)];
    let path = engrave(&apart, &tool(1.0), &EngraveParams { tolerance: 0.01 }, &c).unwrap();
    for p in cutting_points(&path, -1e-9) {
        close(dist_to_chains(&apart, Vec2::new(p.x, p.y)), 0.0, 1e-6);
    }
}

#[test]
fn engrave_refuses_drills_and_bad_chains() {
    let c = cut(0.0, -1.0, 1.0);
    let params = EngraveParams { tolerance: 0.01 };
    let line = [chain(&[(0.0, 0.0), (10.0, 0.0)], false)];
    let err = engrave(&line, &Tool::new(1, ToolKind::Drill, 3.0, 20.0), &params, &c).unwrap_err().to_string();
    assert!(err.contains("drill"), "{err}");
    assert!(engrave(&[chain(&[(0.0, 0.0)], false)], &tool(1.0), &params, &c).is_err(), "one point");
    assert!(engrave(&[chain(&[(0.0, 0.0), (1.0, 0.0)], true)], &tool(1.0), &params, &c).is_err(), "closed with two points");
    assert!(engrave(&[chain(&[(0.0, 0.0), (f64::NAN, 0.0)], false)], &tool(1.0), &params, &c).is_err(), "NaN");
    assert!(engrave(&[], &tool(1.0), &params, &c).is_err(), "nothing");
    assert!(engrave(&line, &tool(1.0), &EngraveParams { tolerance: 0.0 }, &c).is_err(), "no tolerance");
    assert!(engrave(&line, &Tool::new(1, ToolKind::FlatEndmill, 1.0, 0.5), &params, &c).is_err(), "deeper than the flutes");
    // A V-bit engraves: that is what it is for.
    engrave(&line, &vbit(60.0, 6.0), &params, &c).unwrap();
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

fn hole(x: f64, y: f64, bottom: f64) -> Hole {
    Hole { at: v(x, y), bottom }
}

#[test]
fn drill_writes_the_hand_built_program() {
    let holes = [hole(10.0, 10.0, -10.0), hole(30.5, 10.0, -10.0)];
    let t = Tool::new(3, ToolKind::Drill, 5.0, 40.0);
    let path = drill(&holes, &t, &DrillParams { peck: None, dwell: 0.25 }, &cut(0.0, -10.0, 1.0)).unwrap();
    assert_eq!(path.moves, drill_program().operations[0].path.moves);
}

#[test]
fn pecks_retract_and_re_enter_above_the_last_bottom() {
    let c = cut(0.0, -10.0, 10.0);
    let path = drill(&[hole(5.0, 5.0, -10.0)], &tool(3.0), &DrillParams { peck: Some(4.0), dwell: 0.5 }, &c).unwrap();
    let feeds: Vec<f64> = path
        .moves
        .iter()
        .filter_map(|m| match m {
            Move::Feed { to, .. } => Some(to.z),
            _ => None,
        })
        .collect();
    assert_eq!(feeds, vec![-4.0, -8.0, -10.0]);
    // Every rapid below the retract height goes straight down, a tenth above the last peck.
    let mut deepest = f64::INFINITY;
    let mut re_entries = Vec::new();
    for m in &path.moves {
        match m {
            Move::Feed { to, .. } => deepest = deepest.min(to.z),
            Move::Rapid { to } if to.z < 2.0 => {
                assert_eq!((to.x, to.y), (5.0, 5.0));
                re_entries.push(to.z);
                close(to.z, deepest + 0.1, 1e-9);
            }
            _ => {}
        }
    }
    assert_eq!(re_entries, vec![-3.9, -7.9]);
    assert!(path.moves.iter().any(|m| matches!(m, Move::Dwell { seconds } if *seconds == 0.5)));
    verify(&path, &c.heights).unwrap();
    // The time counts the dwell: 6 + 4.1 + 2.1 mm of feed at 200 mm/min, plus half a second.
    close(path.stats().cut_minutes, 12.2 / 200.0 + 0.5 / 60.0, 1e-9);
}

#[test]
fn drill_orders_holes_nearest_first_and_refuses_bad_ones() {
    let c = cut(0.0, -3.0, 3.0);
    let params = DrillParams { peck: None, dwell: 0.0 };
    // From the first hole, always the nearest next: 0, 10, 50, not 0, 50, 10.
    let holes = [hole(0.0, 0.0, -3.0), hole(50.0, 0.0, -3.0), hole(10.0, 0.0, -1.0)];
    let path = drill(&holes, &tool(3.0), &params, &c).unwrap();
    let plunged: Vec<(f64, f64)> = path
        .moves
        .iter()
        .filter_map(|m| match m {
            Move::Feed { to, .. } => Some((to.x, to.z)),
            _ => None,
        })
        .collect();
    assert_eq!(plunged, vec![(0.0, -3.0), (10.0, -1.0), (50.0, -3.0)]);
    // Many holes go row by row, every hole once.
    let grid: Vec<Hole> = (0..60).flat_map(|i| (0..60).map(move |j| hole(i as f64, j as f64, -0.5))).collect();
    let path = drill(&grid, &vbit(60.0, 6.0), &params, &c).unwrap();
    assert_eq!(path.moves.iter().filter(|m| matches!(m, Move::Feed { .. })).count(), 3600);
    verify(&path, &c.heights).unwrap();
    // A V-bit drills (halftone dots); a hole above the top or below the bottom, a peck of 0, a
    // negative dwell, a NaN and no holes at all are refused.
    drill(&[hole(0.0, 0.0, -0.3)], &vbit(60.0, 6.0), &params, &c).unwrap();
    assert!(drill(&[hole(0.0, 0.0, 0.0)], &tool(3.0), &params, &c).is_err(), "at the top");
    assert!(drill(&[hole(0.0, 0.0, -3.5)], &tool(3.0), &params, &c).is_err(), "below the bottom");
    assert!(drill(&[hole(0.0, 0.0, -3.0)], &tool(3.0), &DrillParams { peck: Some(0.0), dwell: 0.0 }, &c).is_err());
    assert!(drill(&[hole(0.0, 0.0, -3.0)], &tool(3.0), &DrillParams { peck: None, dwell: -1.0 }, &c).is_err());
    assert!(drill(&[hole(f64::NAN, 0.0, -3.0)], &tool(3.0), &params, &c).is_err());
    assert!(drill(&[], &tool(3.0), &params, &c).is_err());
    assert!(drill(&[hole(0.0, 0.0, -3.0)], &Tool::new(1, ToolKind::Drill, 3.0, 2.0), &params, &c).is_err(), "deeper than the flutes");
}

/// Every cutting point of a carving is as far from the boundary as the cone is wide at its depth:
/// the cone's edge touches the wall. Points at the floor (held at the bottom) are further in.
fn check_vcarve(regions: &[Region2], tool: &Tool, path: &Toolpath, top: f64, bottom: f64) {
    let shape = Shape::from_regions(regions, 0.01).unwrap();
    let tan_half = (tool.tip_angle.unwrap().to_radians() * 0.5).tan();
    let tip = tool.tip_diameter * 0.5;
    for p in cutting_points(path, top - 1e-9) {
        let d = shape.dist(Vec2::new(p.x, p.y));
        let want = tip + (top - p.z) * tan_half;
        if p.z > bottom + 1e-9 {
            close(d, want, 2e-3);
        } else {
            assert!(d >= want - 2e-3, "{p:?} is {d} in, the cone reaches {want}");
        }
    }
}

#[test]
fn vcarve_follows_the_inward_offsets_at_the_depth_the_cone_touches_the_wall() {
    let square = [rect(0.0, 0.0, 10.0, 10.0)];
    let bit = long_vbit(90.0, 12.0);
    let params = VCarveParams { stepover: 0.5, tolerance: 0.01 };
    let c = cut(0.0, -10.0, 10.0);
    let v = vcarve(&square, &bit, &params, &c).unwrap();
    // A 90° bit goes as deep as it is far from the wall: rings at 0.5 … 4.5 (5 is the middle).
    assert_eq!(v.rings, 9);
    close(v.deepest, -4.5, 1e-9);
    close(v.natural_depth, 4.5, 1e-9);
    assert!(!v.flat_floor);
    check_vcarve(&square, &bit, &v.path, 0.0, -10.0);
    verify(&v.path, &c.heights).unwrap();
    // Held at 2 mm: the inner rings run at the bottom and the floor is flat.
    let held = vcarve(&square, &bit, &params, &cut(0.0, -2.0, 2.0)).unwrap();
    assert_eq!(held.rings, 9);
    assert!(held.flat_floor);
    close(held.deepest, -2.0, 1e-9);
    close(held.natural_depth, 4.5, 1e-9);
    check_vcarve(&square, &bit, &held.path, 0.0, -2.0);
    // Nested rings are linked with a cut, not a travel: one travel for the whole square.
    let travels = v.path.moves.iter().filter(|m| matches!(m, Move::Rapid { to } if to.z == 10.0)).count();
    assert_eq!(travels, 2, "down to the start and back up: {:?}", v.path.moves.iter().take(6).collect::<Vec<_>>());
}

#[test]
fn vcarve_of_a_slot_a_letter_and_bad_tools() {
    // A 1 mm wide slot with a 60° bit: rings to 0.4 in (0.5 is the middle), 0.4 / tan 30° deep.
    let slot = [rect(0.0, 0.0, 20.0, 1.0)];
    let bit = long_vbit(60.0, 6.0);
    let params = VCarveParams { stepover: 0.1, tolerance: 0.01 };
    let v = vcarve(&slot, &bit, &params, &cut(0.0, -5.0, 5.0)).unwrap();
    assert_eq!(v.rings, 4);
    close(v.natural_depth, 0.4 / 30f64.to_radians().tan(), 1e-9);
    close(v.natural_depth, 0.693, 1e-3);
    check_vcarve(&slot, &bit, &v.path, 0.0, -5.0);
    // A letter O: an island inside; the rings run between the two walls.
    let o = [Region2 { outer: Loop2::circle(v2(20.0, 20.0), 10.0), holes: vec![Loop2::circle(v2(20.0, 20.0), 6.0).reversed()] }];
    let v = vcarve(&o, &bit, &VCarveParams { stepover: 0.25, tolerance: 0.01 }, &cut(0.0, -5.0, 5.0)).unwrap();
    assert!(v.rings >= 7, "{}", v.rings);
    close(v.natural_depth, 1.75 / 30f64.to_radians().tan(), 1e-9);
    check_vcarve(&o, &bit, &v.path, 0.0, -5.0);
    // A tip flat: the first ring starts half the flat further in, and the cone is wider.
    let flat = Tool { tip_diameter: 0.4, ..long_vbit(60.0, 6.0) };
    let v = vcarve(&slot, &flat, &params, &cut(0.0, -5.0, 5.0)).unwrap();
    assert_eq!(v.rings, 2, "0.3 and 0.4 in");
    check_vcarve(&slot, &flat, &v.path, 0.0, -5.0);
    // Not a V-bit, too fine a stepover, nothing wider than the tip.
    assert!(vcarve(&slot, &tool(6.0), &params, &cut(0.0, -5.0, 5.0)).unwrap_err().to_string().contains("V-bit"));
    assert!(vcarve(&slot, &bit, &VCarveParams { stepover: 0.001, tolerance: 0.01 }, &cut(0.0, -5.0, 5.0)).is_err());
    assert!(vcarve(&[rect(0.0, 0.0, 20.0, 0.05)], &bit, &params, &cut(0.0, -5.0, 5.0)).is_err());
}

#[test]
fn the_simulation_lowers_the_stock_where_the_tool_cuts_with_its_own_shape() {
    // A 10 mm slot with a 2 mm flat end mill, 1 mm deep: 10 × 2 plus two half discs.
    let c = cut(0.0, -1.0, 1.0);
    let slot = engrave(&[chain(&[(5.0, 10.0), (15.0, 10.0)], false)], &tool(2.0), &EngraveParams { tolerance: 0.01 }, &c).unwrap();
    let mut map = Heightmap::new(v(0.0, 0.0), v(20.0, 20.0), 0.0, 0.1).unwrap();
    assert_eq!((map.width, map.height), (200, 200));
    let removed = map.cut(&tool(2.0), &slot).unwrap();
    close(removed, 20.0 + PI, 0.5);
    close(map.removed(), removed, 1e-9);
    close(map.deepest(), -1.0, 1e-9);
    // Under the middle of the slot the floor is flat at −1; beside it the stock stands.
    let at = |x: f64, y: f64| f64::from(map.z[((y / 0.1) as usize) * map.width + (x / 0.1) as usize]);
    close(at(10.0, 10.0), -1.0, 1e-6);
    close(at(10.0, 10.9), -1.0, 1e-6);
    close(at(10.0, 12.0), 0.0, 1e-6);
    close(at(2.0, 10.0), 0.0, 1e-6);
    // A V-bit's groove is as wide at the top as the cone: 90° means as wide as deep on each side.
    let vee = engrave(&[chain(&[(5.0, 5.0), (15.0, 5.0)], false)], &long_vbit(90.0, 6.0), &EngraveParams { tolerance: 0.01 }, &c).unwrap();
    let mut map = Heightmap::new(v(0.0, 0.0), v(20.0, 20.0), 0.0, 0.05).unwrap();
    map.cut(&long_vbit(90.0, 6.0), &vee).unwrap();
    let at = |x: f64, y: f64| f64::from(map.z[((y / 0.05) as usize) * map.width + (x / 0.05) as usize]);
    close(at(10.0, 5.025), -1.0, 0.05);
    close(at(10.0, 5.525), -0.5, 0.05);
    close(at(10.0, 6.1), 0.0, 1e-6);
    // What stands in a region: the slot's own area is cleared, a wider pocket is not.
    let mut map = Heightmap::new(v(0.0, 0.0), v(20.0, 20.0), 0.0, 0.1).unwrap();
    map.cut(&tool(2.0), &slot).unwrap();
    let (standing, inside) = map.standing_in(&rect(5.0, 9.2, 15.0, 10.8), -1.0);
    assert!(inside > 1500 && standing == 0, "{standing} of {inside}");
    let (standing, inside) = map.standing_in(&rect(0.0, 0.0, 20.0, 20.0), -1.0);
    assert!(standing > inside * 9 / 10, "{standing} of {inside}");
    // Too fine a grid, and bad stock, are refused.
    assert!(Heightmap::new(v(0.0, 0.0), v(1000.0, 1000.0), 0.0, 0.1).is_err());
    assert!(Heightmap::new(v(0.0, 0.0), v(0.0, 20.0), 0.0, 0.1).is_err());
    assert!(Heightmap::new(v(0.0, 0.0), v(20.0, 20.0), 0.0, 0.0).is_err());
}

#[test]
fn rest_machining_cuts_only_where_the_larger_tool_could_not_reach() {
    // A 20 × 20 pocket cleared with a 6 mm cutter leaves its four corners round (R3). A 2 mm
    // cutter after it runs only there: every cut is within reach of a corner, and far shorter
    // than the whole wall ring.
    let regions = [rect(0.0, 0.0, 20.0, 20.0)];
    let c = cut(0.0, -2.0, 2.0);
    let big = tool(6.0);
    let rest = Rest { radius: big.radius_at(2.0), wall: big.radius_at(2.0) };
    let params =
        PocketParams { stepover: 1.0, climb: true, stock_to_leave: 0.0, tolerance: 0.01, ramp_angle: 0.0, dogbones: false, rest: Some(rest) };
    let path = pocket(&regions, &tool(2.0), &params, &c).unwrap();
    let corners = [v(0.0, 0.0), v(20.0, 0.0), v(20.0, 20.0), v(0.0, 20.0)];
    let pts = cutting_points(&path, -1e-9);
    assert!(!pts.is_empty());
    for p in &pts {
        let q = Vec2::new(p.x, p.y);
        let nearest = corners.iter().map(|c| c.dist(q)).fold(f64::INFINITY, f64::min);
        assert!(nearest <= 3.2, "{p:?} is {nearest} from the nearest corner");
        // Still a radius from the wall, as any pocket pass.
        assert!(q.x >= 1.0 - 1e-6 && q.y >= 1.0 - 1e-6 && q.x <= 19.0 + 1e-6 && q.y <= 19.0 + 1e-6, "{p:?}");
    }
    let full = PocketParams { rest: None, ..params };
    let whole = pocket(&regions, &tool(2.0), &full, &c).unwrap();
    assert!(path.stats().cut_length < whole.stats().cut_length / 3.0, "{} vs {}", path.stats().cut_length, whole.stats().cut_length);
    verify(&path, &c.heights).unwrap();
    // A tool as large as the earlier one has nothing left; a larger one is refused.
    let same = PocketParams { rest: Some(Rest { radius: 1.0, wall: 1.0 }), ..params };
    assert!(matches!(pocket(&regions, &tool(2.0), &same, &c), Err(CamError::Invalid(_))));
    let round = [Region2 { outer: Loop2::circle(v(10.0, 10.0), 10.0), holes: vec![] }];
    assert!(matches!(pocket(&round, &tool(2.0), &params, &c), Err(CamError::Nothing(_))), "a round pocket has no corners");
    // A contour inside an opening: the same corners, and nothing along the straight walls.
    let inside = ContourParams {
        side: Side::Inside,
        climb: true,
        stock_to_leave: 0.0,
        tolerance: 0.01,
        ramp_angle: 0.0,
        tabs: None,
        dogbones: false,
        rest: Some(rest),
    };
    let path = contour(&regions, &tool(2.0), &inside, &c).unwrap();
    for p in cutting_points(&path, -1e-9) {
        let q = Vec2::new(p.x, p.y);
        assert!(corners.iter().any(|c| c.dist(q) <= 3.2), "{p:?}");
    }
    // Where the earlier tool left stock on the wall, the band is wider: its path ran further in.
    let left = Rest { radius: 3.0, wall: 3.5 };
    let wider = PocketParams { rest: Some(left), ..params };
    let path = pocket(&regions, &tool(2.0), &wider, &c).unwrap();
    assert!(path.stats().cut_length > 0.0);
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
    let params = PocketParams { stepover: 3.0, climb: true, stock_to_leave: 0.0, tolerance: 0.01, ramp_angle: 0.0, dogbones: false, rest: None };
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
    let params = PocketParams {
        stepover: 3.0,
        climb: true,
        stock_to_leave: 0.0,
        tolerance: 0.01,
        ramp_angle: 3.0f64.to_radians(),
        dogbones: false,
        rest: None,
    };
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
        let pocket_params = PocketParams { stepover: b, climb: true, stock_to_leave: b, tolerance: b, ramp_angle: b, dogbones: false, rest: None };
        let contour_params = ContourParams {
            side: Side::Outside,
            climb: true,
            stock_to_leave: b,
            tolerance: b,
            ramp_angle: b,
            tabs: None,
            dogbones: false,
            rest: None,
        };
        let bad_cut = Cut { heights: Heights { clearance: b, retract: b, top: b, bottom: -b }, feeds: Feeds { cut: b, plunge: b }, stepdown: b };
        for c in [good_cut, bad_cut, Cut { stepdown: b, ..good_cut }] {
            for tl in [&t, &tool(6.0)] {
                let _ = pocket(&regions, tl, &pocket_params, &c);
                let _ = pocket(
                    &regions,
                    tl,
                    &PocketParams { stepover: 2.0, climb: true, stock_to_leave: 0.0, tolerance: 0.01, ramp_angle: 0.0, dogbones: false, rest: None },
                    &c,
                );
                let _ = contour(&regions, tl, &contour_params, &c);
                let _ = engrave(&[chain(&[(0.0, 0.0), (b, b), (1.0, b)], false)], tl, &EngraveParams { tolerance: b }, &c);
                let _ = drill(&[hole(b, b, -b), hole(0.0, 0.0, b)], tl, &DrillParams { peck: Some(b), dwell: b }, &c);
                let _ = vcarve(&regions, &vbit(b, 6.0), &VCarveParams { stepover: b, tolerance: b }, &c);
                let _ = pocket(&regions, tl, &PocketParams { rest: Some(Rest { radius: b, wall: b }), ..pocket_params }, &c);
                let _ = contour(&regions, tl, &ContourParams { rest: Some(Rest { radius: b, wall: b }), ..contour_params }, &c);
                let _ = vcarve(&regions, &vbit(60.0, 6.0), &VCarveParams { stepover: 0.5, tolerance: 0.01 }, &c);
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
            &PocketParams { stepover: 1e-9, climb: true, stock_to_leave: 0.0, tolerance: 0.01, ramp_angle: 0.0, dogbones: false, rest: None },
            &good_cut
        )
        .is_err()
    );
    assert!(
        pocket(
            &regions,
            &tool(60.0),
            &PocketParams { stepover: 2.0, climb: true, stock_to_leave: 0.0, tolerance: 0.01, ramp_angle: 0.0, dogbones: false, rest: None },
            &good_cut
        )
        .is_err()
    );
    assert!(
        pocket(
            &[],
            &tool(6.0),
            &PocketParams { stepover: 2.0, climb: true, stock_to_leave: 0.0, tolerance: 0.01, ramp_angle: 0.0, dogbones: false, rest: None },
            &good_cut
        )
        .is_err()
    );
    // A drill cuts only at its point: no contour or pocket with one.
    let drill = Tool::new(4, ToolKind::Drill, 6.0, 30.0);
    let square = [rect(0.0, 0.0, 20.0, 20.0)];
    let side = ContourParams {
        side: Side::Outside,
        climb: true,
        stock_to_leave: 0.0,
        tolerance: 0.01,
        ramp_angle: 0.0,
        tabs: None,
        dogbones: false,
        rest: None,
    };
    assert!(contour(&square, &drill, &side, &good_cut).unwrap_err().to_string().contains("a drill cannot mill"));
    let lanes = PocketParams { stepover: 2.0, climb: true, stock_to_leave: 0.0, tolerance: 0.01, ramp_angle: 0.0, dogbones: false, rest: None };
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
