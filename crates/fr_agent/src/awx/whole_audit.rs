//! Geometric audit of whole-route lane geometry against source AWX audit bars.
use super::{
    corridor::{distance, point_segment_distance, Point},
    pairs,
    whole_context::{CopperSegment, LaneGeometry, WholeContext},
    whole_geometry::GeometryResult,
};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Finding {
    pub kind: String,
    pub lanes: Vec<String>,
    pub point: Point,
    pub layer: Option<String>,
    pub required_clearance: f64,
    /// Signed clearance surplus: negative means the source bar is violated.
    pub clearance_delta: f64,
    pub obstacle: Option<String>,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct AuditReport {
    pub findings: Vec<Finding>,
    pub counts: BTreeMap<String, usize>,
    pub planned_lanes: usize,
    pub planned_vias: usize,
    pub complete: bool,
    pub unavailable_checks: Vec<String>,
    pub band_outside_mm: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct GateReport {
    pub passed: bool,
    pub reasons: Vec<String>,
}

/// The source whole_gate contract: a missing/incomplete plan, failed maker,
/// folded approach, any measured audit finding, or any lane outside/broken in
/// its router band fails the gate.
pub fn gate(geometry: &GeometryResult, report: &AuditReport) -> GateReport {
    let mut reasons = Vec::new();
    if !report.complete {
        reasons.push("audit or geometry metadata is incomplete".into());
    }
    if geometry.failed {
        reasons.push("geometry producer marked plan failed".into());
    }
    if !geometry.folded.is_empty() {
        reasons.push(format!("folded approaches: {}", geometry.folded.join(", ")));
    }
    for kind in [
        "pitch", "dive", "static", "shape", "swim", "bands", "folded", "snap", "missing",
    ] {
        if report.counts.get(kind).copied().unwrap_or(0) > 0 {
            reasons.push(format!("{kind}: {} finding(s)", report.counts[kind]));
        }
    }
    for (kind, count) in &report.counts {
        if kind.starts_with("lint_") && *count > 0 {
            reasons.push(format!("{kind}: {count} invariant violation(s)"));
        }
    }
    if report.band_outside_mm > 1e-9 {
        reasons.push(format!(
            "band outside length {:.6} mm",
            report.band_outside_mm
        ));
    }
    for why in &report.unavailable_checks {
        reasons.push(why.clone());
    }
    GateReport {
        passed: reasons.is_empty(),
        reasons,
    }
}

fn lint_push(
    out: &mut AuditReport,
    kind: &str,
    lane: &str,
    p: Point,
    layer: Option<String>,
    message: String,
    delta: f64,
) {
    out.findings.push(Finding {
        kind: format!("lint_{kind}"),
        lanes: vec![lane.to_owned()],
        point: p,
        layer,
        required_clearance: 0.0,
        clearance_delta: delta,
        obstacle: Some(message),
    });
}

/// Check the snapped plan's own router invariants, corresponding to whole_lint.py.
pub fn lint(ctx: &WholeContext, geometry: &GeometryResult) -> Result<AuditReport, String> {
    ctx.validate()?;
    if !geometry.metadata_complete {
        return Err("AWX lint requires complete snapped geometry metadata".into());
    }
    let g = ctx.cfg.grid_step;
    let eps = 1e-6;
    let mut out = AuditReport {
        planned_lanes: geometry.lanes.len(),
        complete: true,
        ..AuditReport::default()
    };
    let on = |v: f64| (v / g - (v / g).round()).abs() < 1e-4;
    let oct = |dx: f64, dy: f64| {
        let a = dy.atan2(dx).to_degrees().rem_euclid(45.);
        a.min(45. - a) < 1e-3
    };
    for (n, r) in &geometry.lanes {
        if r.xy.len() < 2 || r.layers.len() + 1 != r.xy.len() {
            return Err(format!("AWX lint lane {n} has malformed pieces"));
        }
        let has_ends = !r.ends.is_empty();
        let lengths: Vec<_> = r.xy.windows(2).map(|w| distance(w[0], w[1])).collect();
        for i in 0..r.xy.len() - 1 {
            let a = r.xy[i];
            let b = r.xy[i + 1];
            let first = i == 0;
            let last = i + 1 == r.xy.len() - 1;
            let len = lengths[i];
            if len < eps {
                lint_push(
                    &mut out,
                    "zero",
                    n,
                    a,
                    Some(r.layers[i].clone()),
                    "zero length piece".into(),
                    -eps,
                );
                continue;
            }
            if !oct(b[0] - a[0], b[1] - a[1]) && !first && !last {
                lint_push(
                    &mut out,
                    "angle",
                    n,
                    a,
                    Some(r.layers[i].clone()),
                    "piece off router heading".into(),
                    -1.,
                );
            }
            let angle = (b[1] - a[1])
                .atan2(b[0] - a[0])
                .to_degrees()
                .rem_euclid(45.);
            if (first || last)
                && !has_ends
                && len > 2. * g * 2_f64.sqrt() + eps
                && angle.min(45. - angle) > 1e-3
            {
                lint_push(
                    &mut out,
                    "join",
                    n,
                    a,
                    Some(r.layers[i].clone()),
                    "terminal join too long and off heading".into(),
                    2. * g * 2_f64.sqrt() - len,
                );
            }
            if !(first && i == 0) && !(last && i + 1 == r.xy.len() - 1) && (!on(a[0]) || !on(a[1]))
            {
                lint_push(
                    &mut out,
                    "grid",
                    n,
                    a,
                    Some(r.layers[i].clone()),
                    "vertex off router grid".into(),
                    -1.,
                );
            }
            if i > 0 {
                let prev = r.xy[i - 1];
                let old = (a[1] - prev[1]).atan2(a[0] - prev[0]);
                let now = (b[1] - a[1]).atan2(b[0] - a[0]);
                let turn = (now - old + std::f64::consts::PI).rem_euclid(2. * std::f64::consts::PI)
                    - std::f64::consts::PI;
                if turn.abs() > std::f64::consts::FRAC_PI_2 + 1e-3 {
                    lint_push(
                        &mut out,
                        "reverse",
                        n,
                        a,
                        Some(r.layers[i].clone()),
                        format!("reverse turn {:.1} degrees", turn.to_degrees()),
                        std::f64::consts::FRAC_PI_2 - turn.abs(),
                    );
                }
            }
            if i > 0
                && r.layers[i] != r.layers[i - 1]
                && !r.vias.iter().any(|v| distance(*v, a) < 1e-4)
            {
                lint_push(
                    &mut out,
                    "via",
                    n,
                    a,
                    None,
                    "layer change without a via".into(),
                    -1.,
                );
            }
        }
        for &v in &r.vias {
            let change = (1..r.layers.len())
                .any(|i| r.layers[i] != r.layers[i - 1] && distance(v, r.xy[i]) < 1e-4);
            if !change {
                lint_push(
                    &mut out,
                    "via",
                    n,
                    v,
                    None,
                    "via without a layer change".into(),
                    -1.,
                );
            }
        }
        if !has_ends {
            let dirs = [
                ctx.tooth_dir.get(n).copied(),
                ctx.stub_dir.get(n).map(|u| [-u[0], -u[1]]),
            ];
            for (end, expected) in dirs.into_iter().enumerate() {
                let Some(e) = expected else {
                    continue;
                };
                let (start, forward) = if end == 0 {
                    (r.xy[0], true)
                } else {
                    (*r.xy.last().unwrap(), false)
                };
                let mut rem = ctx.cfg.track_width;
                let mut d = [0., 0.];
                if forward {
                    for (i, w) in r.xy.windows(2).enumerate() {
                        let len = distance(w[0], w[1]);
                        let take = rem.min(len);
                        d[0] += ((w[1][0] - w[0][0]) / len.max(1e-12)) * take;
                        d[1] += ((w[1][1] - w[0][1]) / len.max(1e-12)) * take;
                        rem -= take;
                        if rem <= 0. {
                            break;
                        }
                        let _ = i;
                    }
                } else {
                    for w in r.xy.windows(2).rev() {
                        let len = distance(w[0], w[1]);
                        let take = rem.min(len);
                        d[0] -= ((w[1][0] - w[0][0]) / len.max(1e-12)) * take;
                        d[1] -= ((w[1][1] - w[0][1]) / len.max(1e-12)) * take;
                        rem -= take;
                        if rem <= 0. {
                            break;
                        }
                    }
                    d = [-d[0], -d[1]];
                }
                let len = d[0].hypot(d[1]);
                if len > eps && (d[0] * e[0] + d[1] * e[1]) / len < -1e-6 {
                    lint_push(
                        &mut out,
                        "stub",
                        n,
                        start,
                        None,
                        format!(
                            "terminal direction folds {:.1} degrees",
                            ((d[0] * e[0] + d[1] * e[1]) / len)
                                .clamp(-1., 1.)
                                .acos()
                                .to_degrees()
                        ),
                        -1.,
                    );
                }
            }
        }
        if ctx.pairs.contains_key(n) {
            lint_pair(ctx, n, r, &mut out);
        }
    }
    for k in &geometry.conflicts {
        if k.frame == "snap" {
            lint_push(
                &mut out,
                "snap",
                k.lanes.first().map(String::as_str).unwrap_or(""),
                k.xy,
                None,
                k.kind.clone(),
                -1.,
            );
        }
    }
    for f in &out.findings {
        *out.counts.entry(f.kind.clone()).or_default() += 1;
    }
    out.complete = !geometry.failed && geometry.metadata_complete;
    Ok(out)
}

fn lint_pair(ctx: &WholeContext, n: &str, r: &LaneGeometry, out: &mut AuditReport) {
    let eps = 1e-6;
    let g = ctx.cfg.grid_step;
    let rt = pairs::turn_straight_steps(&ctx.cfg) as f64;
    let st = pairs::via_straight_steps(&ctx.cfg) as f64;
    let start = 1usize;
    let stop = r.xy.len().saturating_sub(1);
    let mut run = Vec::<(Point, f64, Vec<f64>)>::new();
    for i in start..stop.saturating_sub(1) {
        let a = r.xy[i];
        let b = r.xy[i + 1];
        let len = distance(a, b);
        if len < 1e-6 {
            continue;
        }
        let u = [
            ((b[0] - a[0]) / len * 1e6).round() / 1e6,
            ((b[1] - a[1]) / len * 1e6).round() / 1e6,
        ];
        let steps = len / (g / u[0].abs().max(u[1].abs()));
        let layer_change = i > 0 && r.layers[i] != r.layers[i - 1];
        if layer_change {
            if let Some(last) = run.last_mut() {
                last.2.push(last.1);
            }
        }
        if let Some(last) = run.last_mut() {
            if last.0 == u {
                last.1 += steps;
                continue;
            }
            if layer_change {
                lint_push(out, "pair", n, a, None, "pair turns at a via".into(), -1.);
            }
        }
        run.push((u, steps, Vec::new()));
    }
    for (k, (u, len, vias)) in run.iter().enumerate() {
        if k > 0 && k + 1 < run.len() && *len < rt - 1e-6 {
            lint_push(
                out,
                "pair",
                n,
                r.xy[start + k],
                None,
                format!("{len:.1} straight steps after turn"),
                len - rt,
            );
        }
        if k > 0 {
            let p = run[k - 1].0;
            if p[0] * u[0] + p[1] * u[1] < std::f64::consts::FRAC_1_SQRT_2 - 1e-6 {
                lint_push(
                    out,
                    "pair",
                    n,
                    r.xy[start + k],
                    None,
                    "pair turn exceeds 45 degrees".into(),
                    -1.,
                );
            }
        }
        for &v in vias {
            if (k > 0 && v < st - 1e-6) || (k + 1 < run.len() && len - v < st - 1e-6) {
                lint_push(
                    out,
                    "pair",
                    n,
                    r.xy[start + k],
                    None,
                    "pair via lacks straight approach".into(),
                    (v - st).min(len - v - st),
                );
            }
        }
    }
    for (end, e) in r.ends.iter().enumerate() {
        if !on_grid_point(e.pose, g) {
            lint_push(
                out,
                "ends",
                n,
                e.pose,
                Some(e.layer.clone()),
                "pair pose off grid".into(),
                -1.,
            );
        }
        if !oct(e.heading[0], e.heading[1]) {
            lint_push(
                out,
                "ends",
                n,
                e.pose,
                Some(e.layer.clone()),
                "pair pose heading off router direction".into(),
                -1.,
            );
        }
        let body =
            r.xy.iter()
                .min_by(|a, b| distance(**a, e.pose).total_cmp(&distance(**b, e.pose)))
                .copied();
        if body.is_none_or(|p| distance(p, e.pose) > 1e-6) {
            lint_push(
                out,
                "ends",
                n,
                e.pose,
                None,
                "pair body does not meet its pose".into(),
                -1.,
            );
        }
        let normal = [-e.heading[1], e.heading[0]];
        let offsets = [0, 1].map(|i| {
            let p = e.legs[i].last().copied().unwrap_or(e.pose);
            (p[0] - e.pose[0]) * normal[0] + (p[1] - e.pose[1]) * normal[1]
        });
        if (offsets[0].abs() - offsets[1].abs()).abs() > 1e-6 || offsets[0] * offsets[1] > 0. {
            lint_push(
                out,
                "ends",
                n,
                e.pose,
                Some(e.layer.clone()),
                "pair end legs are not on the pose's opposite track lines".into(),
                -1.,
            );
        }
        for leg in &e.legs {
            for w in leg.windows(3) {
                let t = signed_turn(w[0], w[1], w[2]).abs();
                if t > 45. + 1e-3 {
                    lint_push(
                        out,
                        "ends",
                        n,
                        w[1],
                        Some(e.layer.clone()),
                        format!("pair end leg turn {t:.1} degrees"),
                        45. - t,
                    );
                }
            }
        }
        if e.legs[0].len() > 1 && e.legs[1].len() > 1 {
            let mut nearest = f64::INFINITY;
            for a in e.legs[0].windows(2) {
                for b in e.legs[1].windows(2) {
                    nearest = nearest.min(seg_distance(a[0], a[1], b[0], b[1]).0);
                }
            }
            let need = ctx.cfg.track_width + ctx.cfg.clearance;
            if nearest < need - 1e-6 {
                lint_push(
                    out,
                    "ends",
                    n,
                    e.pose,
                    Some(e.layer.clone()),
                    format!("pair end legs {nearest:.3} apart"),
                    nearest - need,
                );
            }
        }
    }
    if let Some(c) = &r.cross {
        if !oct(c.heading[0], c.heading[1]) {
            lint_push(
                out,
                "cross",
                n,
                c.at,
                None,
                "crossover heading off router direction".into(),
                -1.,
            );
        }
        for p in c.poses {
            if !on_grid_point(p, g) {
                lint_push(
                    out,
                    "cross",
                    n,
                    p,
                    None,
                    "crossover pose off grid".into(),
                    -1.,
                );
            }
        }
        for p in c.poses.into_iter().chain([c.at]) {
            let on_body =
                r.xy.windows(2)
                    .any(|w| point_segment_distance(p, w[0], w[1]) < 1e-6);
            if !on_body {
                lint_push(
                    out,
                    "cross",
                    n,
                    p,
                    None,
                    "body misses crossover pose or center".into(),
                    -1.,
                );
            }
        }
        let mut sides = BTreeMap::new();
        for (which, pose) in [("entry", c.poses[0]), ("exit", c.poses[1])] {
            let mut offs = BTreeMap::new();
            for (k, p) in &c.entry {
                if which == "entry" {
                    offs.insert(
                        k.as_str(),
                        (p[0] - pose[0]) * -c.heading[1] + (p[1] - pose[1]) * c.heading[0],
                    );
                }
            }
            for (k, p) in &c.exit {
                if which == "exit" {
                    offs.insert(
                        k.as_str(),
                        (p[0] - pose[0]) * -c.heading[1] + (p[1] - pose[1]) * c.heading[0],
                    );
                }
            }
            let p = offs.get("P").copied().unwrap_or(0.);
            let q = offs.get("N").copied().unwrap_or(0.);
            if (p.abs() - q.abs()).abs() > 1e-6 || p * q > 0. {
                lint_push(
                    out,
                    "cross",
                    n,
                    pose,
                    None,
                    format!("{which} points not on opposite pose legs"),
                    -1.,
                );
            }
            sides.insert(which, p > 0.);
        }
        if sides.get("entry") == sides.get("exit") {
            lint_push(
                out,
                "cross",
                n,
                c.at,
                None,
                "crossover legs do not swap sides".into(),
                -1.,
            );
        }
        for (label, legs) in &c.legs {
            let Some(entry) = c.entry.get(label) else {
                continue;
            };
            let Some(exit) = c.exit.get(label) else {
                continue;
            };
            if let Some(first) = legs.first() {
                if first
                    .points
                    .first()
                    .is_none_or(|p| distance(*p, *entry) > 1e-6)
                {
                    lint_push(
                        out,
                        "cross",
                        n,
                        *entry,
                        None,
                        format!("{label} leg misses entry"),
                        -1.,
                    );
                }
            }
            if let Some(last) = legs.last() {
                if last
                    .points
                    .last()
                    .is_none_or(|p| distance(*p, *exit) > 1e-6)
                {
                    lint_push(
                        out,
                        "cross",
                        n,
                        *exit,
                        None,
                        format!("{label} leg misses exit"),
                        -1.,
                    );
                }
            }
            let barrels: Vec<_> = c.vias.iter().filter(|v| v.polarity == *label).collect();
            if barrels.len() != 1 {
                lint_push(
                    out,
                    "cross",
                    n,
                    c.at,
                    None,
                    format!("{label} crossover has {} barrels", barrels.len()),
                    -1.,
                );
            }
            for (i, leg) in legs.iter().enumerate() {
                if i >= c.layers.len() || leg.layer != c.layers[i] {
                    lint_push(
                        out,
                        "cross",
                        n,
                        c.at,
                        Some(leg.layer.clone()),
                        format!("{label} crossover layer mismatch"),
                        -1.,
                    );
                }
                if leg.points.windows(2).any(|w| distance(w[0], w[1]) < eps) {
                    lint_push(
                        out,
                        "cross",
                        n,
                        c.at,
                        Some(leg.layer.clone()),
                        format!("{label} crossover has zero leg"),
                        -1.,
                    );
                }
                for w in leg.points.windows(2) {
                    if !oct(w[1][0] - w[0][0], w[1][1] - w[0][1]) {
                        lint_push(
                            out,
                            "cross",
                            n,
                            w[0],
                            Some(leg.layer.clone()),
                            format!("{label} crossover piece off router direction"),
                            -1.,
                        );
                    }
                }
            }
            if barrels.len() == 1
                && legs.len() == 2
                && (legs[0]
                    .points
                    .last()
                    .is_none_or(|p| distance(*p, barrels[0].point) > 1e-6)
                    || legs[1]
                        .points
                        .first()
                        .is_none_or(|p| distance(*p, barrels[0].point) > 1e-6))
            {
                lint_push(
                    out,
                    "cross",
                    n,
                    barrels[0].point,
                    None,
                    format!("{label} leg does not change layer at its barrel"),
                    -1.,
                );
            }
            let barrel = c.vias.iter().find(|v| v.polarity == *label);
            if let Some(v) = barrel {
                if legs.len() != 2
                    || legs[0]
                        .points
                        .last()
                        .is_none_or(|p| distance(*p, v.point) > 1e-6)
                    || legs[1]
                        .points
                        .first()
                        .is_none_or(|p| distance(*p, v.point) > 1e-6)
                {
                    lint_push(
                        out,
                        "cross",
                        n,
                        v.point,
                        None,
                        format!("{label} legs do not change layer at their barrel"),
                        -1.,
                    );
                }
            }
        }
    } else if ctx.pair_ends.contains_key(n) && pair_handedness_crosses(ctx, n) {
        lint_push(
            out,
            "cross",
            n,
            r.xy[0],
            None,
            "opposite-hands pair has no crossover".into(),
            -1.,
        );
    }
}
fn on_grid_point(p: Point, g: f64) -> bool {
    (p[0] / g - (p[0] / g).round()).abs() < 1e-4 && (p[1] / g - (p[1] / g).round()).abs() < 1e-4
}
fn pair_handedness_crosses(ctx: &WholeContext, n: &str) -> bool {
    let Some(t) = ctx.pair_ends.get(n) else {
        return false;
    };
    let a = pairs::hand_vector(ctx.tooth_dir.get(n).copied(), t[0][0], t[0][1], false);
    let b = pairs::hand_vector(ctx.stub_dir.get(n).copied(), t[1][0], t[1][1], true);
    a != 0 && b != 0 && a != b
}
fn oct(dx: f64, dy: f64) -> bool {
    let a = dy.atan2(dx).to_degrees().rem_euclid(45.);
    a.min(45. - a) < 1e-3
}
fn signed_turn(a: Point, b: Point, c: Point) -> f64 {
    let u = [b[0] - a[0], b[1] - a[1]];
    let v = [c[0] - b[0], c[1] - b[1]];
    (u[0] * v[1] - u[1] * v[0])
        .atan2(u[0] * v[0] + u[1] * v[1])
        .to_degrees()
}

fn seg_distance(a: Point, b: Point, c: Point, d: Point) -> (f64, Point, Point) {
    let cross = |u: Point, v: Point| u[0] * v[1] - u[1] * v[0];
    let ab = [b[0] - a[0], b[1] - a[1]];
    let cd = [d[0] - c[0], d[1] - c[1]];
    let den = cross(ab, cd);
    if den.abs() > 1e-15 {
        let ac = [c[0] - a[0], c[1] - a[1]];
        let t = cross(ac, cd) / den;
        let u = cross(ac, ab) / den;
        if (0.0..=1.0).contains(&t) && (0.0..=1.0).contains(&u) {
            let p = [a[0] + ab[0] * t, a[1] + ab[1] * t];
            return (0.0, p, p);
        }
    }
    let candidates = [
        (point_segment_distance(a, c, d), a, project(a, c, d)),
        (point_segment_distance(b, c, d), b, project(b, c, d)),
        (point_segment_distance(c, a, b), project(c, a, b), c),
        (point_segment_distance(d, a, b), project(d, a, b), d),
    ];
    *candidates
        .iter()
        .min_by(|x, y| x.0.total_cmp(&y.0))
        .unwrap()
}

fn grid_piece(a: Point, b: Point, g: f64) -> bool {
    for v in [a[0], a[1], b[0], b[1]] {
        if (v / g - v.round()).abs() > 1e-3 {
            return false;
        }
    }
    let dx = (b[0] - a[0]).abs();
    let dy = (b[1] - a[1]).abs();
    dx < 1e-9 || dy < 1e-9 || (dx - dy).abs() < 1e-6
}
fn exact_end_leg(route: &super::whole_context::LaneGeometry, piece: &CopperSegment) -> bool {
    route.ends.iter().any(|e| {
        e.layer == piece.layer
            && e.legs.iter().any(|path| {
                path.windows(2).any(|w| {
                    (w[0] == piece.a && w[1] == piece.b) || (w[1] == piece.a && w[0] == piece.b)
                })
            })
    })
}
fn proj_t(p: Point, a: Point, b: Point) -> f64 {
    let d = [b[0] - a[0], b[1] - a[1]];
    let q = d[0] * d[0] + d[1] * d[1];
    if q < 1e-24 {
        0.
    } else {
        (((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / q).clamp(0., 1.)
    }
}
fn piece_allowance(
    ctx: &WholeContext,
    lane: &str,
    route: &super::whole_context::LaneGeometry,
    piece: &CopperSegment,
    nearest: Point,
    g: f64,
) -> f64 {
    if grid_piece(piece.a, piece.b, g) || exact_end_leg(route, piece) {
        return 0.;
    }
    let terminal = ctx.ends.get(lane).map(|e| [e.tooth, e.berth]);
    let near_a = terminal.is_some_and(|ps| ps.iter().any(|p| distance(*p, piece.a) < 1e-6));
    let near_b = terminal.is_some_and(|ps| ps.iter().any(|p| distance(*p, piece.b) < 1e-6));
    let on_grid = |p: Point| {
        (p[0] / g - (p[0] / g).round()).abs() <= 1e-3
            && (p[1] / g - (p[1] / g).round()).abs() <= 1e-3
    };
    let a = if near_a || on_grid(piece.a) {
        0.
    } else {
        g / 2.
    };
    let b = if near_b || on_grid(piece.b) {
        0.
    } else {
        g / 2.
    };
    let t = proj_t(nearest, piece.a, piece.b);
    a + (b - a) * t
}

fn project(p: Point, a: Point, b: Point) -> Point {
    let d = [b[0] - a[0], b[1] - a[1]];
    let n = d[0] * d[0] + d[1] * d[1];
    let t = if n <= 1e-24 {
        0.0
    } else {
        (((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / n).clamp(0.0, 1.0)
    };
    [a[0] + t * d[0], a[1] + t * d[1]]
}

fn straight_run(
    route: &super::whole_context::LaneGeometry,
    at: usize,
    forward: bool,
    heading: Point,
) -> f64 {
    let ulen = heading[0].hypot(heading[1]);
    if ulen < 1e-12 {
        return 0.;
    }
    let u = [heading[0] / ulen, heading[1] / ulen];
    let origin = route.xy[at];
    let mut total = 0.;
    let mut j = at;
    loop {
        let next = if forward { j + 1 } else { j };
        if next == 0 || next >= route.xy.len() {
            break;
        }
        let a = route.xy[next - 1];
        let b = route.xy[next];
        let delta = [b[0] - a[0], b[1] - a[1]];
        let len = delta[0].hypot(delta[1]);
        if len < 1e-9 {
            break;
        }
        let da = [a[0] - origin[0], a[1] - origin[1]];
        let db = [b[0] - origin[0], b[1] - origin[1]];
        let crossa = (da[0] * u[1] - da[1] * u[0]).abs();
        let crossb = (db[0] * u[1] - db[1] * u[0]).abs();
        let dot = (delta[0] * u[0] + delta[1] * u[1]) / len;
        if crossa > 1e-4 || crossb > 1e-4 || dot < 1. - 1e-6 {
            break;
        }
        total += len;
        if forward {
            j += 1;
            if j + 1 >= route.xy.len() {
                break;
            }
        } else {
            if j == 0 {
                break;
            }
            j -= 1;
        }
    }
    total
}

fn own_ids(ctx: &WholeContext, lane: &str) -> BTreeSet<i32> {
    let mut ids = BTreeSet::new();
    if let Some(id) = ctx.byname.get(lane) {
        ids.insert(*id);
    }
    if let Some(legs) = ctx.pairs.get(lane) {
        ids.extend(legs.iter().filter_map(|leg| ctx.byname.get(leg).copied()));
    }
    ids
}

fn cross_parameter(a: Point, b: Point, c: Point, d: Point) -> Option<f64> {
    let r = [b[0] - a[0], b[1] - a[1]];
    let s = [d[0] - c[0], d[1] - c[1]];
    let den = r[0] * s[1] - r[1] * s[0];
    if den.abs() < 1e-12 {
        return None;
    }
    let ac = [c[0] - a[0], c[1] - a[1]];
    let t = (ac[0] * s[1] - ac[1] * s[0]) / den;
    let u = (ac[0] * r[1] - ac[1] * r[0]) / den;
    ((0.0..=1.0).contains(&t) && (0.0..=1.0).contains(&u)).then_some(t)
}

fn band_audit(
    ctx: &WholeContext,
    lane: &str,
    route: &LaneGeometry,
    band: &super::whole_context::RouterBand,
) -> (f64, String) {
    let g = ctx.cfg.grid_step;
    let mut path = route.xy.clone();
    let mut tooth = ctx.ends[lane].tooth;
    let mut berth = ctx.ends[lane].berth;
    if route.ends.len() >= 2 {
        let ia = route
            .xy
            .iter()
            .enumerate()
            .min_by(|a, b| {
                distance(*a.1, route.ends[0].pose).total_cmp(&distance(*b.1, route.ends[0].pose))
            })
            .map(|x| x.0)
            .unwrap_or(0);
        let ib = route
            .xy
            .iter()
            .enumerate()
            .min_by(|a, b| {
                distance(*a.1, route.ends[1].pose).total_cmp(&distance(*b.1, route.ends[1].pose))
            })
            .map(|x| x.0)
            .unwrap_or(route.xy.len() - 1);
        if distance(route.ends[0].pose, route.xy[0]) <= distance(route.ends[1].pose, route.xy[0]) {
            tooth = route.ends[0].pose;
            berth = route.ends[1].pose;
            path = route.xy[ia.min(ib)..=ia.max(ib)].to_vec();
        } else {
            tooth = route.ends[1].pose;
            berth = route.ends[0].pose;
            path = route.xy[ia.min(ib)..=ia.max(ib)].to_vec();
        }
    } else if route.ends.len() == 1 {
        if let Some(tips) = ctx.pair_ends.get(lane) {
            let e = &route.ends[0];
            let s0 = (distance(e.tips[0], tips[0][0]) + distance(e.tips[1], tips[0][1]))
                .min(distance(e.tips[0], tips[0][1]) + distance(e.tips[1], tips[0][0]));
            let s1 = (distance(e.tips[0], tips[1][0]) + distance(e.tips[1], tips[1][1]))
                .min(distance(e.tips[0], tips[1][1]) + distance(e.tips[1], tips[1][0]));
            let i = route
                .xy
                .iter()
                .enumerate()
                .min_by(|a, b| distance(*a.1, e.pose).total_cmp(&distance(*b.1, e.pose)))
                .map(|x| x.0)
                .unwrap_or(0);
            if s0 <= s1 {
                tooth = e.pose;
                path = route.xy[i..].to_vec();
            } else {
                berth = e.pose;
                path = route.xy[..=i].to_vec();
            }
        }
    }
    let mut outside = 0.0;
    for w in path.windows(2) {
        let n = ((w[1][0] - w[0][0]).abs().max((w[1][1] - w[0][1]).abs()) / g - 1e-6)
            .ceil()
            .max(1.0) as usize;
        for k in 1..=n {
            let t = k as f64 / n as f64;
            let p = [
                w[0][0] + (w[1][0] - w[0][0]) * t,
                w[0][1] + (w[1][1] - w[0][1]) * t,
            ];
            let d = distance(w[0], w[1]) / n as f64;
            if !ctx.layers.iter().any(|layer| band.contains(p, layer)) {
                outside += d;
            }
        }
    }

    // Reproduce the router's grid flood: eight-connected cells can change layer at a shared open cell.
    let pad = 4.0 * ctx.cfg.lane_pitch;
    let bounds = [
        path.iter()
            .map(|p| p[0])
            .fold(tooth[0].min(berth[0]), f64::min)
            - pad,
        path.iter()
            .map(|p| p[1])
            .fold(tooth[1].min(berth[1]), f64::min)
            - pad,
        path.iter()
            .map(|p| p[0])
            .fold(tooth[0].max(berth[0]), f64::max)
            + pad,
        path.iter()
            .map(|p| p[1])
            .fold(tooth[1].max(berth[1]), f64::max)
            + pad,
    ];
    let ix0 = (bounds[0] / g).floor() as i64;
    let iy0 = (bounds[1] / g).floor() as i64;
    let nx = ((bounds[2] / g).ceil() as i64 - ix0 + 1).max(1) as usize;
    let ny = ((bounds[3] / g).ceil() as i64 - iy0 + 1).max(1) as usize;
    if nx.saturating_mul(ny).saturating_mul(ctx.layers.len()) > 4_000_000 {
        return (outside, "BAND-TOO-LARGE".into());
    }
    let mut open = vec![vec![vec![false; ctx.layers.len()]; ny]; nx];
    for i in 0..nx {
        for j in 0..ny {
            let p = [(ix0 + i as i64) as f64 * g, (iy0 + j as i64) as f64 * g];
            for (k, l) in ctx.layers.iter().enumerate() {
                open[i][j][k] = band.contains(p, l);
            }
        }
    }
    let index = |p: Point| {
        (
            (p[0] / g).round() as i64 - ix0,
            (p[1] / g).round() as i64 - iy0,
        )
    };
    let seed = |p: Point, layer: &str| -> Option<(usize, usize, usize)> {
        let (x, y) = index(p);
        let k = ctx.layers.iter().position(|l| l == layer)?;
        let r = (ctx.cfg.lane_min / g).ceil() as i64;
        let mut best = None;
        for dx in -r..=r {
            for dy in -r..=r {
                let i = x + dx;
                let j = y + dy;
                if i >= 0
                    && j >= 0
                    && (i as usize) < nx
                    && (j as usize) < ny
                    && open[i as usize][j as usize][k]
                {
                    let ds = dx * dx + dy * dy;
                    if best.is_none_or(|(old, _, _, _)| ds < old) {
                        best = Some((ds, i as usize, j as usize, k));
                    }
                }
            }
        }
        best.map(|(_, i, j, k)| (i, j, k))
    };
    let Some(start) = seed(tooth, &ctx.tooth_layer[lane]) else {
        return (outside, "TOOTH-OUT".into());
    };
    let Some(goal) = seed(berth, &ctx.dest_layer[lane]) else {
        return (outside, "BERTH-OUT".into());
    };
    let mut seen = BTreeSet::from([start]);
    let mut q = std::collections::VecDeque::from([start]);
    while let Some((i, j, k)) = q.pop_front() {
        if (i, j, k) == goal {
            return (outside, "connected".into());
        }
        for di in -1i32..=1 {
            for dj in -1i32..=1 {
                if di == 0 && dj == 0 {
                    continue;
                }
                let x = i as i32 + di;
                let y = j as i32 + dj;
                if x >= 0 && y >= 0 && (x as usize) < nx && (y as usize) < ny {
                    let v = (x as usize, y as usize, k);
                    if open[v.0][v.1][v.2] && seen.insert(v) {
                        q.push_back(v);
                    }
                }
            }
        }
        for kk in 0..ctx.layers.len() {
            let v = (i, j, kk);
            if open[i][j][kk] && seen.insert(v) {
                q.push_back(v);
            }
        }
    }
    (outside, "BROKEN".into())
}

fn pieces(
    ctx: &WholeContext,
    lane: &str,
    g: &super::whole_context::LaneGeometry,
) -> Result<Vec<CopperSegment>, String> {
    let pair_legs = if !g.paired_legs.is_empty() {
        g.paired_legs.clone()
    } else if ctx.pairs.contains_key(lane) {
        super::whole_polish::pair_legs_for_audit(ctx, lane, g)?
    } else {
        BTreeMap::new()
    };
    if !pair_legs.is_empty() {
        return Ok(pair_legs
            .iter()
            .flat_map(|(net, legs)| {
                legs.iter().map(move |leg| CopperSegment {
                    net: net.clone(),
                    net_id: *ctx.byname.get(net).unwrap_or(&0),
                    a: leg.a,
                    b: leg.b,
                    width: ctx.cfg.track_width,
                    layer: leg.layer.clone(),
                })
            })
            .collect());
    }
    Ok(g.xy
        .windows(2)
        .zip(&g.layers)
        .map(|(ab, layer)| CopperSegment {
            net: lane.to_owned(),
            net_id: *ctx.byname.get(lane).unwrap_or(&0),
            a: ab[0],
            b: ab[1],
            width: ctx.cfg.track_width,
            layer: layer.clone(),
        })
        .collect())
}

/// Measure lane pitch, static copper, shape, inter-layer swim spacing, and
/// barrel clearances. Source reports a plan incomplete when an admitted lane
/// or its exact physical pair barrels are missing.
pub fn audit(ctx: &WholeContext, geometry: &GeometryResult) -> Result<AuditReport, String> {
    ctx.validate()?;
    let cfg = &ctx.cfg;
    let gstep = cfg.grid_step;
    let half_step = gstep / 2.0;
    let mut report = AuditReport {
        planned_lanes: geometry.lanes.len(),
        complete: true,
        ..AuditReport::default()
    };
    for p in &ctx.pads {
        if !p.non_plated_hole && !p.layers.is_empty() && p.copper_shapes.is_empty() {
            report.complete = false;
            report.unavailable_checks.push(format!(
                "static: missing native copper shape for {}.{}",
                p.component_ref, p.pad_number
            ));
        }
    }
    for lane in &ctx.lanes {
        if !geometry.lanes.contains_key(lane) {
            report.complete = false;
            report.findings.push(Finding {
                kind: "missing".into(),
                lanes: vec![lane.clone()],
                point: [0.0, 0.0],
                layer: None,
                required_clearance: 0.0,
                clearance_delta: f64::NEG_INFINITY,
                obstacle: None,
            });
        }
    }
    let mut lane_pieces = BTreeMap::new();
    for (lane, route) in &geometry.lanes {
        if route.xy.len() < 2 || route.layers.len() != route.xy.len() - 1 {
            return Err(format!("AWX malformed lane geometry for {lane}"));
        }
        let p = pieces(ctx, lane, route)?;
        lane_pieces.insert(lane.clone(), p);
    }
    let mut crossover_candidates = BTreeMap::new();
    for lane in &ctx.lanes {
        if ctx.pairs.contains_key(lane) {
            if let Some(c) = super::whole_polish::planned_crossover_for_audit(ctx, lane, geometry)?
            {
                crossover_candidates.insert(lane.clone(), c);
            }
        }
    }
    report.planned_vias = 0;

    // PITCH: pairwise same-layer centerline/barrel-leg distance.
    let names: Vec<_> = lane_pieces.keys().cloned().collect();
    for (i, a) in names.iter().enumerate() {
        for b in &names[i + 1..] {
            for pa in &lane_pieces[a] {
                for pb in &lane_pieces[b] {
                    if pa.layer != pb.layer {
                        continue;
                    }
                    let (d, x, y) = seg_distance(pa.a, pa.b, pb.a, pb.b);
                    let pair_reach =
                        if ctx.pairs.contains_key(a) && geometry.lanes[a].paired_legs.is_empty() {
                            pairs::pitch(cfg) / 2.
                        } else {
                            0.
                        } + if ctx.pairs.contains_key(b) && geometry.lanes[b].paired_legs.is_empty()
                        {
                            pairs::pitch(cfg) / 2.
                        } else {
                            0.
                        };
                    let allow_a = piece_allowance(ctx, a, &geometry.lanes[a], pa, x, gstep);
                    let allow_b = piece_allowance(ctx, b, &geometry.lanes[b], pb, y, gstep);
                    let required = cfg.track_width + cfg.clearance + pair_reach + allow_a + allow_b;
                    if d < required - half_step.min(allow_a + allow_b) - 1e-6 {
                        report.findings.push(Finding {
                            kind: "pitch".into(),
                            lanes: vec![a.clone(), b.clone()],
                            point: x,
                            layer: Some(pa.layer.clone()),
                            required_clearance: required,
                            clearance_delta: d - required,
                            obstacle: None,
                        });
                    }
                }
            }
        }
    }

    // SWIM: crossings separated by too little distance to make the required layer transition.
    for (lane, route) in &geometry.lanes {
        let mut arc = vec![0.0; route.xy.len()];
        for i in 1..route.xy.len() {
            arc[i] = arc[i - 1] + distance(route.xy[i - 1], route.xy[i]);
        }
        let mut hits = Vec::<(f64, String, String, Point)>::new();
        for other in geometry.lanes.keys().filter(|n| *n != lane) {
            for (i, w) in route.xy.windows(2).enumerate() {
                for seg in &lane_pieces[other] {
                    if let Some(t) = cross_parameter(w[0], w[1], seg.a, seg.b) {
                        let p = [
                            w[0][0] + (w[1][0] - w[0][0]) * t,
                            w[0][1] + (w[1][1] - w[0][1]) * t,
                        ];
                        hits.push((
                            arc[i] + t * (arc[i + 1] - arc[i]),
                            seg.layer.clone(),
                            other.clone(),
                            p,
                        ));
                    }
                }
            }
        }
        hits.sort_by(|a, b| a.0.total_cmp(&b.0));
        let pair_width = if ctx.pairs.contains_key(lane) {
            pairs::pitch(cfg)
        } else {
            0.0
        };
        let mut merged = Vec::new();
        for h in hits {
            if merged
                .last()
                .is_some_and(|p: &(f64, String, String, Point)| {
                    p.1 == h.1 && p.2 == h.2 && h.0 - p.0 < 1.5 * pair_width.max(cfg.track_width)
                })
            {
                continue;
            }
            merged.push(h);
        }
        let need = 2.0 * (cfg.via_size + cfg.clearance) + pair_width;
        for w in merged.windows(2) {
            if w[0].1 != w[1].1 && w[1].0 - w[0].0 < need {
                report.findings.push(Finding {
                    kind: "swim".into(),
                    lanes: vec![lane.clone(), w[0].2.clone(), w[1].2.clone()],
                    point: w[0].3,
                    layer: Some(w[0].1.clone()),
                    required_clearance: 0.0,
                    clearance_delta: w[1].0 - w[0].0 - need,
                    obstacle: Some(format!("layer crossings {} then {}", w[0].1, w[1].1)),
                });
            }
        }
    }

    // BAND: exact router-band occupancy plus its source grid flood connectivity check.
    for (lane, route) in &geometry.lanes {
        let Some(band) = geometry.bands.get(lane) else {
            report.complete = false;
            report
                .unavailable_checks
                .push(format!("bands: missing router band for {lane}"));
            continue;
        };
        let (outside, status) = band_audit(ctx, lane, route, band);
        report.band_outside_mm += outside;
        if status != "connected" {
            report.findings.push(Finding {
                kind: "bands".into(),
                lanes: vec![lane.clone()],
                point: route.xy[0],
                layer: None,
                required_clearance: 0.0,
                clearance_delta: -1.0,
                obstacle: Some(status),
            });
        }
    }
    for lane in &geometry.folded {
        report.findings.push(Finding {
            kind: "folded".into(),
            lanes: vec![lane.clone()],
            point: geometry
                .lanes
                .get(lane)
                .and_then(|r| r.xy.first().copied())
                .unwrap_or([0.0, 0.0]),
            layer: None,
            required_clearance: 0.0,
            clearance_delta: -1.0,
            obstacle: Some("approach outside source stub arc".into()),
        });
    }
    for c in &geometry.conflicts {
        if c.frame == "snap" {
            report.findings.push(Finding {
                kind: "snap".into(),
                lanes: c.lanes.clone(),
                point: c.xy,
                layer: None,
                required_clearance: 0.0,
                clearance_delta: -1.0,
                obstacle: Some(c.kind.clone()),
            });
        }
    }
    if geometry.failed {
        report.complete = false;
        report
            .unavailable_checks
            .push("geometry producer marked the plan failed".into());
    }
    if !geometry.metadata_complete {
        report.complete = false;
        report
            .unavailable_checks
            .push("geometry metadata is incomplete".into());
    }

    // STATIC: route tracks against other nets' pad copper, base tracks and vias.
    for (lane, route) in &geometry.lanes {
        let own = own_ids(ctx, lane);
        for seg in &lane_pieces[lane] {
            let need = cfg.track_width / 2. + cfg.clearance + half_step;
            for pad in &ctx.pads {
                if own.contains(&pad.net_id)
                    || (!pad.layers.is_empty() && !pad.layers.contains(&seg.layer))
                {
                    continue;
                }
                let n = ((distance(seg.a, seg.b) / gstep).ceil() as usize).max(1);
                let mut edge = f64::INFINITY;
                for i in 0..=n {
                    let t = i as f64 / n as f64;
                    let p = [
                        seg.a[0] + (seg.b[0] - seg.a[0]) * t,
                        seg.a[1] + (seg.b[1] - seg.a[1]) * t,
                    ];
                    let mut d = if pad.non_plated_hole {
                        distance(pad.point, p) - pad.drill / 2.
                    } else {
                        pad.copper_shapes
                            .iter()
                            .filter(|s| s.layer == seg.layer)
                            .map(|s| s.shape.signed_distance(p))
                            .fold(f64::INFINITY, f64::min)
                    };
                    if !ctx.pairs.contains_key(lane) && !pad.non_plated_hole {
                        d -= pairs::pad_corner_buffer(
                            p[0] - pad.point[0],
                            p[1] - pad.point[1],
                            pad.size[0] / 2.,
                            pad.size[1] / 2.,
                            pad.corner_radius,
                            gstep,
                        );
                    }
                    edge = edge.min(d);
                }
                if edge < need - 1e-6 {
                    report.findings.push(Finding {
                        kind: "static".into(),
                        lanes: vec![lane.clone()],
                        point: project(pad.point, seg.a, seg.b),
                        layer: Some(seg.layer.clone()),
                        required_clearance: need,
                        clearance_delta: edge - need,
                        obstacle: Some(format!("pad {}.{}", pad.component_ref, pad.pad_number)),
                    });
                }
            }
            for obs in &ctx.base_segments {
                if obs.net_id == 0 || own.contains(&obs.net_id) || obs.layer != seg.layer {
                    continue;
                }
                let (d, x, _) = seg_distance(seg.a, seg.b, obs.a, obs.b);
                let delta = d - obs.width / 2. - need;
                if delta < -1e-6 {
                    report.findings.push(Finding {
                        kind: "static".into(),
                        lanes: vec![lane.clone()],
                        point: x,
                        layer: Some(seg.layer.clone()),
                        required_clearance: need + obs.width / 2.,
                        clearance_delta: delta,
                        obstacle: Some(format!("copper {}", obs.net)),
                    });
                }
            }
            for via in &ctx.base_vias {
                if own.contains(&via.net_id) || !via.layers.contains(&seg.layer) {
                    continue;
                }
                let d = point_segment_distance(via.point, seg.a, seg.b) - via.diameter / 2. - need;
                if d < -1e-6 {
                    report.findings.push(Finding {
                        kind: "static".into(),
                        lanes: vec![lane.clone()],
                        point: project(via.point, seg.a, seg.b),
                        layer: Some(seg.layer.clone()),
                        required_clearance: need,
                        clearance_delta: d,
                        obstacle: Some(format!("via {}", via.net)),
                    });
                }
            }
        }
        // Shape measures on each same-layer run, matching plan_audit's 100° local and scale-turn bars.
        let scale = cfg.track_width + cfg.clearance;
        let mut arc = vec![0.0; route.xy.len()];
        for k in 1..route.xy.len() {
            arc[k] = arc[k - 1] + distance(route.xy[k - 1], route.xy[k]);
        }
        let mut local_turns = Vec::new();
        for i in 1..route.xy.len() - 1 {
            if route.layers[i - 1] != route.layers[i] {
                continue;
            }
            let u = [
                route.xy[i][0] - route.xy[i - 1][0],
                route.xy[i][1] - route.xy[i - 1][1],
            ];
            let v = [
                route.xy[i + 1][0] - route.xy[i][0],
                route.xy[i + 1][1] - route.xy[i][1],
            ];
            let angle = (u[0] * v[1] - u[1] * v[0])
                .atan2(u[0] * v[0] + u[1] * v[1])
                .to_degrees();
            local_turns.push((i, angle, distance(route.xy[i], route.xy[i + 1])));
            if angle.abs() > 100.0 {
                report.findings.push(Finding {
                    kind: "shape".into(),
                    lanes: vec![lane.clone()],
                    point: route.xy[i],
                    layer: Some(route.layers[i].clone()),
                    required_clearance: 0.0,
                    clearance_delta: 100.0 - angle.abs(),
                    obstacle: Some("vertex turn".into()),
                });
            }
            let before = (0..i)
                .rev()
                .find(|&j| arc[i] - arc[j] >= scale)
                .unwrap_or(0);
            let after = (i + 1..route.xy.len())
                .find(|&j| arc[j] - arc[i] >= scale)
                .unwrap_or(route.xy.len() - 1);
            let d1 = [
                route.xy[i][0] - route.xy[before][0],
                route.xy[i][1] - route.xy[before][1],
            ];
            let d2 = [
                route.xy[after][0] - route.xy[i][0],
                route.xy[after][1] - route.xy[i][1],
            ];
            let turn = (d1[0] * d2[1] - d1[1] * d2[0])
                .atan2(d1[0] * d2[0] + d1[1] * d2[1])
                .to_degrees();
            if turn.abs() > 100.0 {
                report.findings.push(Finding {
                    kind: "shape".into(),
                    lanes: vec![lane.clone()],
                    point: route.xy[i],
                    layer: Some(route.layers[i].clone()),
                    required_clearance: 0.0,
                    clearance_delta: 100.0 - turn.abs(),
                    obstacle: Some("scale turn".into()),
                });
            }
        }
        for w in local_turns.windows(2) {
            if w[0].1.abs() > 60.
                && w[1].1.abs() > 60.
                && w[0].1 * w[1].1 < 0.
                && w[0].2 < cfg.track_width + gstep
            {
                report.findings.push(Finding {
                    kind: "shape".into(),
                    lanes: vec![lane.clone()],
                    point: route.xy[w[0].0],
                    layer: Some(route.layers[w[0].0].clone()),
                    required_clearance: 0.0,
                    clearance_delta: w[0].2 - (cfg.track_width + gstep),
                    obstacle: Some("notch".into()),
                });
            }
        }
    }

    // DIVE: each physical barrel must clear other lanes' tracks and barrels, fab drill spacing and static copper.
    for (lane, route) in &geometry.lanes {
        let own = own_ids(ctx, lane);
        let barrel_rows: Vec<(String, Point)> = if let Some(pair) = ctx.pairs.get(lane) {
            if route.vias.is_empty() {
                Vec::new()
            } else {
                if pair.iter().any(|n| !route.paired_vias.contains_key(n)) {
                    report.complete = false;
                    report.findings.push(Finding {
                        kind: "dive".into(),
                        lanes: vec![lane.clone()],
                        point: route.vias[0],
                        layer: None,
                        required_clearance: 0.0,
                        clearance_delta: f64::NEG_INFINITY,
                        obstacle: Some("missing physical P/N barrel geometry".into()),
                    });
                    Vec::new()
                } else {
                    pair.iter()
                        .flat_map(|n| route.paired_vias[n].iter().map(|&p| (n.clone(), p)))
                        .collect()
                }
            }
        } else {
            route.vias.iter().map(|&p| (lane.clone(), p)).collect()
        };
        report.planned_vias += barrel_rows.len();
        for (net, point) in &barrel_rows {
            for (other, other_pieces) in &lane_pieces {
                if other == lane {
                    continue;
                }
                let other_pair = ctx.pairs.contains_key(other);
                let ring = if other_pair {
                    pairs::via_ring(cfg, pairs::pitch(cfg) / 2., None)
                } else {
                    pairs::via_ring(cfg, 0., None)
                };
                for s in other_pieces {
                    let d = point_segment_distance(*point, s.a, s.b) - s.width / 2.;
                    let required = ring + half_step;
                    if d < required - 1e-6 {
                        report.findings.push(Finding {
                            kind: "dive".into(),
                            lanes: vec![lane.clone(), other.clone()],
                            point: *point,
                            layer: Some(s.layer.clone()),
                            required_clearance: required,
                            clearance_delta: d - required,
                            obstacle: Some("planned lane track".into()),
                        });
                    }
                }
            }
            for obs in &ctx.base_segments {
                if own.contains(&obs.net_id) {
                    continue;
                }
                let required = cfg.via_size / 2. + cfg.clearance + obs.width / 2. + half_step;
                let d = point_segment_distance(*point, obs.a, obs.b);
                if d < required - 1e-6 {
                    report.findings.push(Finding {
                        kind: "dive".into(),
                        lanes: vec![lane.clone()],
                        point: *point,
                        layer: None,
                        required_clearance: required,
                        clearance_delta: d - required,
                        obstacle: Some(format!("copper {}", obs.net)),
                    });
                }
            }
            for other in &ctx.base_vias {
                if own.contains(&other.net_id) {
                    continue;
                }
                let copper = cfg.via_size / 2. + cfg.clearance + other.diameter / 2.;
                let drill = cfg.via_drill / 2. + other.drill / 2. + cfg.hole_to_hole_clearance;
                let required = copper.max(drill) + half_step;
                let d = distance(*point, other.point);
                if d < required - 1e-6 {
                    report.findings.push(Finding {
                        kind: "dive".into(),
                        lanes: vec![lane.clone()],
                        point: *point,
                        layer: None,
                        required_clearance: required,
                        clearance_delta: d - required,
                        obstacle: Some(format!("via {}", other.net)),
                    });
                }
            }
            for (other, other_route) in &geometry.lanes {
                if other == lane {
                    continue;
                }
                let points: Vec<Point> = if let Some(legs) = ctx.pairs.get(other) {
                    legs.iter()
                        .flat_map(|leg| {
                            other_route
                                .paired_vias
                                .get(leg)
                                .into_iter()
                                .flatten()
                                .copied()
                        })
                        .collect()
                } else {
                    other_route.vias.clone()
                };
                for q in points {
                    let required = (cfg.via_size + cfg.clearance)
                        .max(cfg.via_drill + cfg.hole_to_hole_clearance)
                        + half_step;
                    let d = distance(*point, q);
                    if d < required - 1e-6 {
                        report.findings.push(Finding {
                            kind: "dive".into(),
                            lanes: vec![lane.clone(), other.clone()],
                            point: *point,
                            layer: None,
                            required_clearance: required,
                            clearance_delta: d - required,
                            obstacle: Some("planned via".into()),
                        });
                    }
                }
            }
            for pad in &ctx.pads {
                let (d, required) = if pad.non_plated_hole {
                    (
                        distance(*point, pad.point),
                        cfg.via_drill / 2.
                            + pad.drill / 2.
                            + cfg.hole_to_hole_clearance
                            + half_step,
                    )
                } else {
                    let edge = pad
                        .copper_shapes
                        .iter()
                        .map(|s| s.shape.signed_distance(*point))
                        .fold(f64::INFINITY, f64::min);
                    (edge, cfg.via_size / 2. + cfg.clearance + half_step)
                };
                if own.contains(&pad.net_id) || d < required - 1e-6 {
                    if own.contains(&pad.net_id) {
                        continue;
                    }
                    report.findings.push(Finding {
                        kind: "dive".into(),
                        lanes: vec![lane.clone()],
                        point: *point,
                        layer: None,
                        required_clearance: required,
                        clearance_delta: d - required,
                        obstacle: Some(format!(
                            "{} {}.{}",
                            if pad.non_plated_hole { "hole" } else { "pad" },
                            pad.component_ref,
                            pad.pad_number
                        )),
                    });
                }
            }
            if ctx.pairs.contains_key(lane) {
                for s in &lane_pieces[lane] {
                    let d = point_segment_distance(*point, s.a, s.b) - cfg.track_width / 2.;
                    let need = pairs::via_ring(cfg, 0., None) + half_step;
                    if d < need - 1e-6 {
                        report.findings.push(Finding {
                            kind: "dive".into(),
                            lanes: vec![lane.clone()],
                            point: *point,
                            layer: Some(s.layer.clone()),
                            required_clearance: need,
                            clearance_delta: d - need,
                            obstacle: Some("pair leg".into()),
                        });
                    }
                }
            }
            let _ = net;
        }
        if ctx.pairs.contains_key(lane) {
            let transitions: Vec<_> = (1..route.layers.len())
                .filter(|&i| route.layers[i] != route.layers[i - 1])
                .collect();
            let arc = {
                let mut s = vec![0.; route.xy.len()];
                for i in 1..route.xy.len() {
                    s[i] = s[i - 1] + distance(route.xy[i - 1], route.xy[i]);
                }
                s
            };
            let total = *arc.last().unwrap_or(&0.);
            for &i in &transitions {
                let before = [
                    route.xy[i][0] - route.xy[i - 1][0],
                    route.xy[i][1] - route.xy[i - 1][1],
                ];
                let after = [
                    route.xy[i + 1][0] - route.xy[i][0],
                    route.xy[i + 1][1] - route.xy[i][1],
                ];
                let l0 = straight_run(route, i, false, before);
                let l1 = straight_run(route, i, true, after);
                let n0 = before[0].hypot(before[1]).max(1e-12);
                let n1 = after[0].hypot(after[1]).max(1e-12);
                let u0 = [before[0] / n0, before[1] / n0];
                let u1 = [after[0] / n1, after[1] / n1];
                let need0 = pairs::via_straight(cfg, u0);
                let need1 = pairs::via_straight(cfg, u1);
                let turning = u0[0] * u1[0] + u0[1] * u1[1] < 1. - 1e-6;
                if turning || l0 < need0 - 1e-6 || l1 < need1 - 1e-6 {
                    report.findings.push(Finding {
                        kind: "dive".into(),
                        lanes: vec![lane.clone()],
                        point: route.xy[i],
                        layer: None,
                        required_clearance: 0.0,
                        clearance_delta: (l0 - need0).min(l1 - need1).min(if turning {
                            -1e-6
                        } else {
                            0.
                        }),
                        obstacle: Some(format!(
                            "pair straight {l0:.3}/{need0:.3} before, {l1:.3}/{need1:.3} after{}",
                            if turning { " with turn" } else { "" }
                        )),
                    });
                }
                if ctx.pair_ends.contains_key(lane) {
                    let n = transitions.len();
                    if n > 0 && i == transitions[0] {
                        let need = pairs::dive_room(
                            cfg,
                            ctx.pair_ends[lane][0],
                            ctx.tooth_dir.get(lane).copied(),
                        );
                        if arc[i] < need - 1e-6 {
                            report.findings.push(Finding {
                                kind: "dive".into(),
                                lanes: vec![lane.clone()],
                                point: route.xy[i],
                                layer: None,
                                required_clearance: 0.0,
                                clearance_delta: arc[i] - need,
                                obstacle: Some("dive too close to tooth".into()),
                            });
                        }
                    }
                    if n > 0 && i == *transitions.last().unwrap() {
                        let need = pairs::dive_room(
                            cfg,
                            ctx.pair_ends[lane][1],
                            ctx.stub_dir.get(lane).map(|u| [-u[0], -u[1]]),
                        );
                        if total - arc[i] < need - 1e-6 {
                            report.findings.push(Finding {
                                kind: "dive".into(),
                                lanes: vec![lane.clone()],
                                point: route.xy[i],
                                layer: None,
                                required_clearance: 0.0,
                                clearance_delta: total - arc[i] - need,
                                obstacle: Some("dive too close to berth".into()),
                            });
                        }
                    }
                }
            }
            for e in &route.ends {
                let i = route
                    .xy
                    .iter()
                    .enumerate()
                    .min_by(|a, b| distance(*a.1, e.pose).total_cmp(&distance(*b.1, e.pose)))
                    .map(|v| v.0)
                    .unwrap_or(0);
                let d = if i == 0 {
                    e.heading
                } else {
                    [-e.heading[0], -e.heading[1]]
                };
                let run = straight_run(route, i, i == 0, d);
                let need = pairs::pose_probe_steps(cfg) as f64 * cfg.grid_step
                    / d[0].abs().max(d[1].abs()).max(1e-12);
                if run < need - 1e-6 {
                    report.findings.push(Finding {
                        kind: "dive".into(),
                        lanes: vec![lane.clone()],
                        point: e.pose,
                        layer: Some(e.layer.clone()),
                        required_clearance: 0.0,
                        clearance_delta: run - need,
                        obstacle: Some("pair pose probe".into()),
                    });
                }
            }
            if let Some(c) = &route.cross {
                if let Some(i) = route
                    .xy
                    .iter()
                    .enumerate()
                    .min_by(|a, b| distance(*a.1, c.at).total_cmp(&distance(*b.1, c.at)))
                    .map(|v| v.0)
                {
                    let step =
                        cfg.grid_step / c.heading[0].abs().max(c.heading[1].abs()).max(1e-12);
                    let b = straight_run(route, i, false, c.heading);
                    let a = straight_run(route, i, true, c.heading);
                    let nb = -c.span[0] + pairs::pose_probe_steps(cfg) as f64 * step;
                    let na = c.span[1] + pairs::pose_probe_steps(cfg) as f64 * step;
                    if b < nb - 1e-6 || a < na - 1e-6 {
                        report.findings.push(Finding {
                            kind: "dive".into(),
                            lanes: vec![lane.clone()],
                            point: c.at,
                            layer: None,
                            required_clearance: 0.0,
                            clearance_delta: (b - nb).min(a - na),
                            obstacle: Some("crossover straight run".into()),
                        });
                    }
                }
            }
            if let Some(c) = crossover_candidates.get(lane) {
                if route.cross.is_none() {
                    if let Some(i) = route
                        .xy
                        .iter()
                        .enumerate()
                        .min_by(|a, b| distance(*a.1, c.at).total_cmp(&distance(*b.1, c.at)))
                        .map(|v| v.0)
                    {
                        let step =
                            cfg.grid_step / c.heading[0].abs().max(c.heading[1].abs()).max(1e-12);
                        let b = straight_run(route, i, false, c.heading);
                        let a = straight_run(route, i, true, c.heading);
                        let nb = -c.span[0] + pairs::pose_probe_steps(cfg) as f64 * step;
                        let na = c.span[1] + pairs::pose_probe_steps(cfg) as f64 * step;
                        if b < nb - 1e-6 || a < na - 1e-6 {
                            report.findings.push(Finding {
                                kind: "dive".into(),
                                lanes: vec![lane.clone()],
                                point: c.at,
                                layer: None,
                                required_clearance: 0.0,
                                clearance_delta: (b - nb).min(a - na),
                                obstacle: Some("planned crossover straight run".into()),
                            });
                        }
                    }
                }
            }
        }
    }

    // Shape summaries are per finding; retain stable ordering and per-kind counts for gate callers.
    report.findings.sort_by(|a, b| {
        a.kind
            .cmp(&b.kind)
            .then_with(|| a.lanes.cmp(&b.lanes))
            .then_with(|| a.point[0].total_cmp(&b.point[0]))
            .then_with(|| a.point[1].total_cmp(&b.point[1]))
    });
    for f in &report.findings {
        *report.counts.entry(f.kind.clone()).or_default() += 1;
    }
    for key in [
        "pitch", "dive", "static", "shape", "bands", "swim", "folded",
    ] {
        report.counts.entry(key.into()).or_default();
    }
    report.complete &= report.findings.iter().all(|f| f.clearance_delta >= -1e-6);
    Ok(report)
}
