//! Lofted round tubes through sections that are not evenly spaced.
//!
//! A loft's surface used to pass through its sections at uniform parameters, so a section close to
//! its neighbour (a bore that runs 5 mm past the end of a tube whose other sections are 12 mm
//! apart, or 60 mm in a snail) made the surface overshoot between the others, by several times the
//! tube's radius. The overshoot also made booleans with the tube fail for some section counts and
//! not for their neighbours.

use std::f64::consts::PI;

use super::*;

/// `sections` circles of radius `radius` centred on an arc of radius `path` turning `angle`
/// (radians), in planes normal to the arc, lofted into the body `body`; `over` extends the
/// sections past both ends along the tangent.
fn tube(s: &mut Session, body: &str, radius: f64, path: f64, angle: f64, sections: usize, over: f64) {
    let mut stations: Vec<[f64; 4]> = (0..sections)
        .map(|i| {
            let t = angle * i as f64 / (sections - 1) as f64;
            [path * t.cos(), path * t.sin(), -t.sin(), t.cos()]
        })
        .collect();
    if over > 0.0 {
        let (a, b) = (stations[0], stations[sections - 1]);
        stations.insert(0, [a[0] - over * a[2], a[1] - over * a[3], a[2], a[3]]);
        stations.push([b[0] + over * b[2], b[1] + over * b[3], b[2], b[3]]);
    }
    let mut list = Vec::new();
    for (i, [x, y, tx, ty]) in stations.iter().enumerate() {
        let name = format!("{body}{i}");
        run(s, "sketch.create", json!({"plane": {"origin": [x, y, 0], "x_dir": [0, 0, 1], "y_dir": [ty, -tx, 0]}, "name": name}));
        run(s, "sketch.circle.center", json!({"center": [0, 0], "radius": radius}));
        run(s, "sketch.finish", json!({}));
        list.push(json!({"sketch": name}));
    }
    run(s, "solid.loft", json!({"sections": list, "operation": "new", "name": format!("Loft{body}"), "body_name": body}));
}

fn bounds(s: &mut Session, body: &str) -> ([f64; 3], [f64; 3]) {
    let m = run(s, "inspect.measure", json!({}));
    let b = m["bodies"].as_array().unwrap().iter().find(|b| b["name"] == body).unwrap();
    let get = |k: &str| -> [f64; 3] {
        let v = b["bbox"][k].as_array().unwrap();
        [v[0].as_f64().unwrap(), v[1].as_f64().unwrap(), v[2].as_f64().unwrap()]
    };
    (get("min"), get("max"))
}

#[test]
fn a_bore_with_a_short_overrun_stays_inside_its_tube() {
    // 14 sections 12 mm apart, and one more 5 mm beyond each end: the bore is a tube of radius 18
    // around the arc (z within 18 of the arc's plane), and used to reach z = 88.
    let mut s = Session::default();
    tube(&mut s, "Bore", 18.0, 100.0, PI / 2.0, 14, 5.0);
    let (lo, hi) = bounds(&mut s, "Bore");
    assert!(hi[2] < 18.0 + 0.1 && lo[2] > -18.0 - 0.1, "z from {} to {}", lo[2], hi[2]);
    let want = PI * 18.0 * 18.0 * (100.0 * PI / 2.0 + 10.0);
    let got = volume(&mut s);
    assert!(rel(got, want) < 2e-3, "{got} vs {want}");
}

#[test]
fn a_hollow_elbow_is_cut_for_any_number_of_sections() {
    let (path, outer, inner) = (100.0, 30.0, 18.0);
    // Each of these gave "no result" (the first six) or failed in another way.
    for (degrees, sections) in [(60.0, 5), (60.0, 7), (90.0, 10), (90.0, 14), (120.0, 7), (120.0, 13)] {
        let angle = degrees * PI / 180.0;
        let mut s = Session::default();
        tube(&mut s, "Outer", outer, path, angle, sections, 0.0);
        tube(&mut s, "Inner", inner, path, angle, sections, 5.0);
        run(&mut s, "solid.combine", json!({"target": "Outer", "tools": ["Inner"], "operation": "cut", "name": "Hollow"}));
        assert_eq!(s.model.state().bodies.len(), 1, "{degrees} degrees, {sections} sections");
        let want = PI * (outer * outer - inner * inner) * path * angle;
        let got = volume(&mut s);
        assert!(rel(got, want) < 3e-3, "{degrees} degrees, {sections} sections: {got} vs {want}");
    }
}

/// #6158: an airfoil section, two fit-point splines of 41 points closed by two lines, lofts on a
/// sketch plane tilted 2.2° or 5° about X as it does on an axis-aligned one. The splines are
/// approximated coordinate by coordinate in 3D, so on a tilted plane the end caps' wires missed
/// their plane by more than truck's 1e-6 ("cap: cannot attach a plane to a wire that is not on
/// one plane"); on an axis-aligned plane one coordinate stays constant.
#[test]
fn airfoil_sections_on_tilted_planes_get_their_caps() {
    let section = |s: &mut Session, name: &str, y: f64, z: f64, tilt: f64, chord: f64, le: f64| {
        let n = 41;
        let round = |v: f64, places: i32| (v * 10f64.powi(places)).round() / 10f64.powi(places);
        let (mut top, mut bottom) = (Vec::new(), Vec::new());
        for i in 1..=n {
            let u = 0.5 * (1.0 - (PI * i as f64 / n as f64).cos());
            let half = 5.0 * 0.12 * (0.2969 * u.sqrt() - 0.126 * u - 0.3516 * u.powi(2) + 0.2843 * u.powi(3) - 0.1036 * u.powi(4)) * chord + 0.4 * u;
            top.push([round(le + chord * u, 4), round(half, 4)]);
            bottom.push([round(le + chord * u, 4), round(-half, 4)]);
        }
        bottom.reverse();
        let t = tilt.to_radians();
        let plane = json!({"origin": [0, y, z], "x_dir": [1, 0, 0], "y_dir": [0, round(-t.sin(), 6), round(t.cos(), 6)]});
        run(s, "sketch.create", json!({"plane": plane, "name": name}));
        run(s, "sketch.spline.fit_point", json!({"points": top}));
        run(s, "sketch.line", json!({"points": [top[n - 1], bottom[0]]}));
        run(s, "sketch.spline.fit_point", json!({"points": bottom}));
        run(s, "sketch.line", json!({"points": [bottom[n - 1], top[0]]}));
        run(s, "sketch.finish", json!({}));
    };
    let mut volumes = Vec::new();
    for tilt in [0.0, 2.2, 5.0] {
        let mut s = Session::default();
        section(&mut s, "S0", 130.0, 0.24, tilt, 185.0, 42.2);
        section(&mut s, "S1", 140.0, -0.12, tilt, 140.0, 46.4);
        run(&mut s, "solid.loft", json!({"sections": [{"sketch": "S0"}, {"sketch": "S1"}], "name": "loft", "body_name": "WING"}));
        volumes.push(run(&mut s, "inspect.measure", json!({}))["total"]["volume_mm3"].as_f64().unwrap_or(0.0));
    }
    // About 10 mm long between sections of 185 and 140 mm chord; tilting the planes by a few
    // degrees changes their distance a little, not the solid.
    assert!(volumes[0] > 1000.0, "{volumes:?}");
    for v in &volumes[1..] {
        assert!((v - volumes[0]).abs() < 0.02 * volumes[0], "{volumes:?}");
    }
}
