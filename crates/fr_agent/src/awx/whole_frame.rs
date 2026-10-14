//! AWX whole_frame.py caad45cd: canonical whole-route frame from actual terminal geometry.
use super::{
    corridor::{self, add, distance, dot, length, mul, sub, Point, Spine},
    numeric, pairs,
    whole_context::WholeContext,
};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq, PartialOrd, Ord)]
pub enum Face {
    E,
    N,
    W,
    S,
}
impl Face {
    pub fn tag(self) -> &'static str {
        match self {
            Self::E => "E",
            Self::N => "N",
            Self::W => "W",
            Self::S => "S",
        }
    }
}
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct PerimeterCut {
    pub face: Face,
    pub coordinate: f64,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PairSplit {
    pub pair: String,
    pub end: usize,
    pub between: Vec<String>,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum FrameError {
    FarFace(Vec<String>),
    PairSplit(Vec<PairSplit>),
    Invalid(String),
}
impl From<String> for FrameError {
    fn from(s: String) -> Self {
        Self::Invalid(s)
    }
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct TipBetween {
    pub lane: String,
    pub end: usize,
    pub members: Vec<String>,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct WholeFrame {
    pub m: Vec<String>,
    pub src: String,
    pub dst: String,
    pub tooth: BTreeMap<String, Point>,
    pub bend: BTreeMap<String, Point>,
    pub sb: [f64; 4],
    pub db: [f64; 4],
    pub spine: Spine,
    pub mid: BTreeMap<String, Vec<Point>>,
    pub dcut: PerimeterCut,
    pub cls: BTreeMap<String, String>,
    pub se: BTreeMap<String, Point>,
    pub h0: f64,
    pub p: BTreeMap<String, f64>,
    pub tip_between: Vec<TipBetween>,
    pub land: BTreeMap<String, Point>,
    pub start: BTreeMap<String, Point>,
    pub st: BTreeMap<String, Point>,
    pub launch: Vec<String>,
    pub final_order: Vec<String>,
    pub o_h: BTreeMap<String, f64>,
    pub rings: BTreeMap<String, Spine>,
    pub hk: BTreeMap<String, f64>,
}
pub fn grown(b: [f64; 4], points: &[Point]) -> [f64; 4] {
    let mut d: Vec<_> = points
        .iter()
        .map(|p| {
            (b[0] - p[0])
                .max(p[0] - b[2])
                .max(b[1] - p[1])
                .max(p[1] - b[3])
        })
        .filter(|v| *v > 0.)
        .collect();
    d.sort_by(f64::total_cmp);
    let n = d.len();
    let m = if n == 0 {
        0.
    } else if n % 2 == 1 {
        d[n / 2]
    } else {
        (d[n / 2 - 1] + d[n / 2]) / 2.
    };
    [b[0] - m, b[1] - m, b[2] + m, b[3] + m]
}
pub fn face(p: Point, b: [f64; 4]) -> Face {
    [
        (Face::E, (p[0] - b[2]).abs()),
        (Face::N, (p[1] - b[1]).abs()),
        (Face::W, (p[0] - b[0]).abs()),
        (Face::S, (p[1] - b[3]).abs()),
    ]
    .into_iter()
    .min_by(|a, b| a.1.total_cmp(&b.1))
    .unwrap()
    .0
}
pub fn cut(ys: &[f64], lo: f64, hi: f64) -> f64 {
    let mut pts = ys.to_vec();
    pts.sort_by(f64::total_cmp);
    pts.insert(0, lo);
    pts.push(hi);
    let mut best = 0;
    for i in 1..pts.len() - 1 {
        if pts[i + 1] - pts[i] > pts[best + 1] - pts[best] {
            best = i;
        }
    }
    (pts[best] + pts[best + 1]) / 2.
}
pub fn cuts_tied(ys: &[f64], lo: f64, hi: f64, tol: f64) -> Vec<f64> {
    let mut ps = ys.to_vec();
    ps.sort_by(f64::total_cmp);
    ps.insert(0, lo);
    ps.push(hi);
    let gaps: Vec<_> = ps
        .windows(2)
        .map(|w| (w[1] - w[0], (w[0] + w[1]) / 2.))
        .collect();
    let mut i0 = 0;
    for i in 1..gaps.len() {
        if gaps[i].0 > gaps[i0].0 {
            i0 = i;
        }
    }
    let mut out = vec![gaps[i0].1];
    out.extend(
        gaps.iter()
            .enumerate()
            .filter(|(i, g)| *i != i0 && g.0 > gaps[i0].0 - tol)
            .map(|(_, g)| g.1),
    );
    out
}
pub fn in_frame(sp: &Spine, path: &[Point]) -> Vec<Point> {
    let mut out = Vec::new();
    for &p in path {
        let so = sp.project_pt(p);
        if out.last().is_none_or(|q: &Point| so[0] > q[0] + 1e-9) {
            out.push(so);
        }
    }
    out
}
pub fn dense(path: &[Point], step: f64) -> Vec<Point> {
    if path.is_empty() {
        return vec![];
    }
    let mut out = vec![path[0]];
    for w in path.windows(2) {
        let n = (distance(w[0], w[1]) / step).ceil().max(1.) as usize;
        for i in 1..=n {
            out.push(add(w[0], mul(sub(w[1], w[0]), i as f64 / n as f64)));
        }
    }
    out
}
pub fn perim_d(p: Point, b: [f64; 4], ycut: f64) -> f64 {
    let (w, h) = (b[2] - b[0], b[3] - b[1]);
    match face(p, b) {
        Face::E if p[1] <= ycut => ycut - p[1],
        Face::N => ycut - b[1] + b[2] - p[0],
        Face::W => ycut - b[1] + w + p[1] - b[1],
        Face::S => ycut - b[1] + w + h + p[0] - b[0],
        _ => ycut - b[1] + 2. * w + h + b[3] - p[1],
    }
}
pub fn cut_point(c: PerimeterCut, b: [f64; 4]) -> Point {
    match c.face {
        Face::E => [b[2], c.coordinate],
        Face::N => [c.coordinate, b[1]],
        Face::S => [c.coordinate, b[3]],
        Face::W => [b[0], c.coordinate],
    }
}
pub fn perim_c(p: Point, b: [f64; 4], c: PerimeterCut) -> f64 {
    if c.face == Face::E {
        perim_d(p, b, c.coordinate)
    } else {
        (perim_d(p, b, b[1]) - perim_d(cut_point(c, b), b, b[1]))
            .rem_euclid(2. * (b[2] - b[0] + b[3] - b[1]))
    }
}
pub fn ring_side(p: Point, b: [f64; 4], c: PerimeterCut) -> Option<Face> {
    let f = face(p, b);
    if f == Face::W {
        return None;
    }
    if c.face == Face::E {
        Some(if f == Face::E {
            if p[1] <= c.coordinate {
                Face::N
            } else {
                Face::S
            }
        } else {
            f
        })
    } else {
        Some(if perim_c(p, b, c) < perim_c([b[0], b[1]], b, c) {
            Face::N
        } else {
            Face::S
        })
    }
}
pub fn cut_gaps(
    points: &[Point],
    b: [f64; 4],
    avoid: &[Point],
) -> Result<Vec<PerimeterCut>, String> {
    let (w, h) = (b[2] - b[0], b[3] - b[1]);
    let p = 2. * (w + h);
    if p <= 0. {
        return Err("degenerate perimeter cut box".into());
    }
    let rounded = |pts: &[Point]| -> Result<Vec<f64>, String> {
        let mut vals: Vec<_> = pts
            .iter()
            .map(|&q| numeric::round_digits(perim_d(q, b, b[1]), 6))
            .collect::<Result<_, _>>()?;
        vals.sort_by(f64::total_cmp);
        vals.dedup();
        Ok(vals)
    };
    let us = rounded(points)?;
    let av = rounded(avoid)?;
    let mut out = Vec::new();
    for (i, &a) in us.iter().enumerate() {
        let end = us[(i + 1) % us.len()] + if i + 1 == us.len() { p } else { 0. };
        if end - a < 1e-6 {
            continue;
        }
        let mut inner: Vec<_> = av
            .iter()
            .filter(|&&v| (a < v && v < end) || (a < v + p && v + p < end))
            .map(|&v| v + if v < a { p } else { 0. })
            .collect();
        inner.sort_by(f64::total_cmp);
        inner.insert(0, a);
        inner.push(end);
        let mut k = 0;
        for j in 1..inner.len() - 1 {
            if inner[j + 1] - inner[j] > inner[k + 1] - inner[k] {
                k = j;
            }
        }
        let m = ((inner[k] + inner[k + 1]) / 2.).rem_euclid(p);
        let c = if m < w {
            PerimeterCut {
                face: Face::N,
                coordinate: b[2] - m,
            }
        } else if m < w + h {
            continue;
        } else if m < 2. * w + h {
            PerimeterCut {
                face: Face::S,
                coordinate: b[0] + m - w - h,
            }
        } else {
            PerimeterCut {
                face: Face::E,
                coordinate: b[3] - (m - 2. * w - h),
            }
        };
        out.push(c);
    }
    Ok(out)
}
pub fn cut_sig(points: &[Point], b: [f64; 4], c: PerimeterCut) -> Vec<usize> {
    let mut is: Vec<_> = (0..points.len()).collect();
    is.sort_by(|&i, &j| {
        perim_c(points[i], b, c)
            .total_cmp(&perim_c(points[j], b, c))
            .then(i.cmp(&j))
    });
    is
}
pub fn perim_s(p: Point, b: [f64; 4]) -> f64 {
    let (w, h) = (b[2] - b[0], b[3] - b[1]);
    let cut = (b[1] + b[3]) / 2.;
    match face(p, b) {
        Face::W if p[1] <= cut => cut - p[1],
        Face::N => cut - b[1] + p[0] - b[0],
        Face::E => cut - b[1] + w + p[1] - b[1],
        Face::S => cut - b[1] + w + h + b[2] - p[0],
        _ => cut - b[1] + 2. * w + h + b[3] - p[1],
    }
}
pub fn between(lo: f64, hi: f64, v: f64, whole: f64) -> bool {
    if hi - lo <= whole / 2. {
        lo < v && v < hi
    } else {
        v > hi || v < lo
    }
}
pub fn inside_octohull(p: Point, points: &[Point], margin: f64) -> bool {
    (0..8).all(|k| {
        let a = k as f64 * std::f64::consts::FRAC_PI_4;
        let d = [a.cos(), a.sin()];
        dot(p, d)
            <= points
                .iter()
                .map(|&q| dot(q, d))
                .fold(f64::NEG_INFINITY, f64::max)
                + margin
    })
}
pub fn room_inside(
    edge: f64,
    near: f64,
    width: f64,
    inner: f64,
    track: f64,
    clear: f64,
    lane_min: f64,
) -> Option<f64> {
    let edge = near.max(edge + clear + track / 2. - lane_min);
    if edge + width + track / 2. + clear <= inner + 1e-9 {
        Some(edge)
    } else {
        None
    }
}
pub fn segment_rectangle_distance(a: Point, b: Point, r: [f64; 4]) -> f64 {
    let inside = |p: Point| r[0] <= p[0] && p[0] <= r[2] && r[1] <= p[1] && p[1] <= r[3];
    if inside(a) || inside(b) {
        return 0.;
    }
    let cs = [[r[0], r[1]], [r[2], r[1]], [r[2], r[3]], [r[0], r[3]]];
    (0..4)
        .map(|i| pairs::segment_distance(a, b, cs[i], cs[(i + 1) % 4]))
        .fold(f64::INFINITY, f64::min)
}
pub fn build(ctx: &WholeContext, dest: &str) -> Result<WholeFrame, FrameError> {
    build_with_trunk(ctx, dest, &BTreeSet::new())
}
fn build_with_trunk(
    ctx: &WholeContext,
    dest: &str,
    trunk: &BTreeSet<String>,
) -> Result<WholeFrame, FrameError> {
    ctx.validate()?;
    let lanes: Vec<_> = ctx
        .lanes
        .iter()
        .filter(|n| {
            ctx.paths.contains_key(*n) && ctx.ends.get(*n).is_some_and(|e| e.destination == dest)
        })
        .cloned()
        .collect();
    let mut sources: BTreeMap<String, usize> = BTreeMap::new();
    for n in &lanes {
        *sources.entry(ctx.src_ref[n].clone()).or_default() += 1;
    }
    let src = lanes
        .iter()
        .map(|n| ctx.src_ref[n].clone())
        .max_by_key(|s| sources[s])
        .ok_or_else(|| FrameError::Invalid("no whole route source".into()))?;
    let m: Vec<_> = lanes
        .into_iter()
        .filter(|n| ctx.src_ref[n] == src)
        .collect();
    let tooth: BTreeMap<_, _> = m.iter().map(|n| (n.clone(), ctx.ends[n].tooth)).collect();
    let bend: BTreeMap<_, _> = m.iter().map(|n| (n.clone(), ctx.ends[n].berth)).collect();
    let sb = grown(
        ctx.box_of(&src)?,
        &tooth.values().copied().collect::<Vec<_>>(),
    );
    let db = grown(
        ctx.box_of(dest)?,
        &bend.values().copied().collect::<Vec<_>>(),
    );
    let tf: BTreeMap<_, _> = m.iter().map(|n| (n.clone(), face(tooth[n], sb))).collect();
    let far: Vec<_> = m.iter().filter(|n| tf[*n] == Face::W).cloned().collect();
    if !far.is_empty() {
        return Err(FrameError::FarFace(far));
    }
    let ca = [(sb[0] + sb[2]) / 2., (sb[1] + sb[3]) / 2.];
    let cb = [(db[0] + db[2]) / 2., (db[1] + db[3]) / 2.];
    let d = corridor::unit(sub(cb, ca))?;
    let back = (sb[2] - sb[0]).hypot(sb[3] - sb[1]) / 2. + ctx.cfg.lane_pitch;
    let fwd = (db[2] - db[0]).hypot(db[3] - db[1]) / 2. + ctx.cfg.lane_pitch;
    let spine = Spine::new(&[sub(ca, mul(d, back)), add(cb, mul(d, fwd))])?;
    let mut mid: BTreeMap<_, _> = m
        .iter()
        .map(|n| (n.clone(), in_frame(&spine, &ctx.paths[n])))
        .collect();
    let bf: BTreeMap<_, _> = m.iter().map(|n| (n.clone(), face(bend[n], db))).collect();
    let ys: Vec<_> = m
        .iter()
        .filter(|n| bf[*n] == Face::E)
        .map(|n| bend[n][1])
        .collect();
    let dcut = ctx.dest_cut.unwrap_or(PerimeterCut {
        face: Face::E,
        coordinate: cut(&ys, db[1], db[3]),
    });
    let mut cls = BTreeMap::new();
    for n in &m {
        if let Some(f) = ring_side(bend[n], db, dcut) {
            if !trunk.contains(n) && matches!(f, Face::N | Face::S) {
                cls.insert(n.clone(), f.tag().into());
            }
        }
    }
    let near: Vec<_> = m
        .iter()
        .filter(|n| !cls.contains_key(*n))
        .cloned()
        .collect();
    for n in &m {
        let mut pts = Vec::new();
        for mut p in dense(&ctx.paths[n], ctx.cfg.lane_pitch / 2.) {
            for (kind, b) in [("src", sb), ("dst", db)] {
                let side = if kind == "src" {
                    match tf[n] {
                        Face::N => Some(Face::N),
                        Face::S => Some(Face::S),
                        _ => None,
                    }
                } else {
                    cls.get(n).map(|s| if s == "N" { Face::N } else { Face::S })
                };
                if let Some(side) = side {
                    if b[0] < p[0] && p[0] < b[2] && b[1] < p[1] && p[1] < b[3] {
                        p[1] = if side == Face::N {
                            b[1] - ctx.cfg.lane_pitch
                        } else {
                            b[3] + ctx.cfg.lane_pitch
                        };
                    }
                }
            }
            pts.push(p);
        }
        mid.insert(n.clone(), in_frame(&spine, &pts));
    }
    let se: BTreeMap<_, _> = near
        .iter()
        .map(|n| (n.clone(), spine.project_pt(bend[n])))
        .collect();
    let h0 = if near.is_empty() {
        spine.project_pt([db[0], (db[1] + db[3]) / 2.])[0]
    } else {
        near.iter().map(|n| se[n][0]).fold(f64::INFINITY, f64::min)
    };
    let mut tip_between = Vec::new();
    let mut splits = Vec::new();
    for end in 0..2 {
        let whole = if end == 0 {
            2. * (sb[2] - sb[0] + sb[3] - sb[1])
        } else {
            2. * (db[2] - db[0] + db[3] - db[1])
        };
        let mut at = Vec::new();
        for n in &m {
            let tips = ctx
                .pair_ends
                .get(n)
                .map(|ps| ps[end].to_vec())
                .unwrap_or_else(|| vec![if end == 0 { tooth[n] } else { bend[n] }]);
            let layer = if end == 0 {
                &ctx.tooth_layer[n]
            } else {
                &ctx.dest_layer[n]
            };
            for p in tips {
                at.push((
                    if end == 0 {
                        perim_s(p, sb)
                    } else {
                        perim_c(p, db, dcut)
                    },
                    n.clone(),
                    layer.clone(),
                ));
            }
        }
        for pair in ctx.pairs.keys().filter(|n| m.contains(*n)) {
            let mut vs: Vec<_> = at.iter().filter(|r| &r.1 == pair).map(|r| r.0).collect();
            vs.sort_by(f64::total_cmp);
            if vs.len() < 2 {
                continue;
            }
            let members: BTreeSet<_> = at
                .iter()
                .filter(|r| &r.1 != pair && between(vs[0], *vs.last().unwrap(), r.0, whole))
                .map(|r| r.1.clone())
                .collect();
            tip_between.push(TipBetween {
                lane: pair.clone(),
                end,
                members: members.iter().cloned().collect(),
            });
            let lay = if end == 0 {
                &ctx.tooth_layer[pair]
            } else {
                &ctx.dest_layer[pair]
            };
            let via_end = |n: &String| {
                if ctx.layers.len() == 2 {
                    return false;
                }
                let b = if end == 0 { sb } else { db };
                let legs = ctx
                    .pairs
                    .get(n)
                    .map(|p| p.to_vec())
                    .unwrap_or_else(|| vec![n.clone()]);
                legs.iter().all(|leg| {
                    ctx.byname.get(leg).is_some_and(|&id| {
                        ctx.base_vias.iter().any(|v| {
                            v.net_id == id
                                && b[0] - 1e-6 <= v.point[0]
                                && v.point[0] <= b[2] + 1e-6
                                && b[1] - 1e-6 <= v.point[1]
                                && v.point[1] <= b[3] + 1e-6
                        })
                    })
                })
            };
            let inside: BTreeSet<_> = at
                .iter()
                .filter(|r| {
                    &r.1 != pair
                        && &r.2 == lay
                        && between(vs[0], *vs.last().unwrap(), r.0, whole)
                        && !(via_end(pair) || via_end(&r.1))
                })
                .map(|r| r.1.clone())
                .collect();
            if !inside.is_empty() {
                splits.push(PairSplit {
                    pair: pair.clone(),
                    end,
                    between: inside.into_iter().collect(),
                });
            }
        }
    }
    if !splits.is_empty() {
        return Err(FrameError::PairSplit(splits));
    }
    let p: BTreeMap<_, _> = m
        .iter()
        .map(|n| (n.clone(), perim_c(bend[n], db, dcut)))
        .collect();
    let mut land = bend.clone();
    let td = *spine.d.last().unwrap();
    for n in ctx.pairs.keys().filter(|n| m.contains(*n)) {
        if let Some(&e) = ctx.stub_dir.get(n) {
            if cls.contains_key(n) || dot(e, td).abs() < 0.5_f64.sqrt() * length(e) {
                let tips = ctx.pair_ends[n][1];
                let r = pairs::end_run(&ctx.cfg, tips, Some(e))
                    + (pairs::turn_straight_steps(&ctx.cfg) + 1) as f64 * ctx.cfg.grid_step;
                land.insert(n.clone(), add(pairs::mid(tips[0], tips[1]), mul(e, r)));
            }
        }
    }
    let mut start = tooth.clone();
    for n in ctx.pairs.keys().filter(|n| m.contains(*n)) {
        if let Some(&e) = ctx.tooth_dir.get(n) {
            let el = length(e);
            if el > 1e-9 && dot(e, spine.d[0]).abs() < 0.5_f64.sqrt() * el {
                let tips = ctx.pair_ends[n][0];
                let r = pairs::end_run(&ctx.cfg, tips, Some(e))
                    + (pairs::turn_straight_steps(&ctx.cfg) + 1) as f64 * ctx.cfg.grid_step;
                start.insert(n.clone(), add(pairs::mid(tips[0], tips[1]), mul(e, r / el)));
            }
        }
    }
    let st = m
        .iter()
        .map(|n| (n.clone(), spine.project_pt(start[n])))
        .collect();
    let mut launch = m.clone();
    launch.sort_by(|a, b| perim_s(tooth[a], sb).total_cmp(&perim_s(tooth[b], sb)));
    let mut final_order = m.clone();
    final_order.sort_by(|a, b| p[a].total_cmp(&p[b]));
    let o_h = near.iter().map(|n| (n.clone(), se[n][1])).collect();
    let mut f = WholeFrame {
        m,
        src,
        dst: dest.into(),
        tooth,
        bend,
        sb,
        db,
        spine,
        mid,
        dcut,
        cls,
        se,
        h0,
        p,
        tip_between,
        land,
        start,
        st,
        launch,
        final_order,
        o_h,
        rings: BTreeMap::new(),
        hk: BTreeMap::new(),
    };
    let dpads: Vec<_> = ctx.pads_of(dest).iter().map(|p| p.point).collect();
    let cen = mul(
        dpads.iter().copied().fold([0., 0.], add),
        1. / dpads.len() as f64,
    );
    let north = if f.spine.xy(h0, 1.)[1] < f.spine.xy(h0, 0.)[1] {
        1.
    } else {
        -1.
    };
    let mut tails: Vec<_> = near
        .iter()
        .map(|n| f.spine.xy(h0, f.o_h[n]))
        .chain(near.iter().map(|n| f.bend[n]))
        .collect();
    let base: Vec<_> = dpads
        .iter()
        .copied()
        .chain(f.bend.values().copied())
        .chain(tails.iter().copied())
        .collect();
    let rings_round = |p: &super::whole_context::ContextPad| {
        ctx.layers.len() == 2 || p.drill > 0. || ctx.layers.iter().all(|l| p.layers.contains(l))
    };
    for pad in &ctx.pads {
        if pad.component_ref == f.src || pad.component_ref == dest || !rings_round(pad) {
            continue;
        }
        if inside_octohull(pad.point, &base, ctx.cfg.lane_pitch + 2. * ctx.cfg.lane_min) {
            for sx in [-1., 1.] {
                for sy in [-1., 1.] {
                    tails.push([
                        pad.point[0] + sx * pad.size[0] / 2.,
                        pad.point[1] + sy * pad.size[1] / 2.,
                    ]);
                }
            }
        }
    }
    let mut in_hull = BTreeSet::new();
    let mut packed = BTreeMap::new();
    let parts: BTreeSet<_> = ctx.pads.iter().map(|p| p.component_ref.clone()).collect();
    for _ in 0..2 * parts.len() + 2 {
        let (hits, room) = stack_rings(
            &mut f, ctx, &dpads, &near, &tails, north, cen, &in_hull, &packed,
        )?;
        let mut acted = false;
        for (k, mut hs) in hits {
            hs.sort_by(|a, b| a.1.total_cmp(&b.1).then(a.0.cmp(&b.0)));
            let (ref_, inner) = &hs[0];
            let (e, ne, w) = room[&k];
            if let Some(edge) = room_inside(
                e,
                ne,
                w,
                *inner,
                ctx.cfg.track_width,
                ctx.cfg.clearance,
                ctx.cfg.lane_min,
            ) {
                if packed.get(&k) != Some(&edge) {
                    packed.insert(k, edge);
                    acted = true;
                }
            } else {
                in_hull.insert(ref_.clone());
                for p in ctx.pads_of(ref_) {
                    for sx in [-1., 1.] {
                        for sy in [-1., 1.] {
                            tails.push([
                                p.point[0] + sx * p.size[0] / 2.,
                                p.point[1] + sy * p.size[1] / 2.,
                            ]);
                        }
                    }
                }
                acted = true;
            }
        }
        if !acted {
            break;
        }
    }
    let mut short = BTreeSet::new();
    for (k, ring) in &f.rings {
        let mem: Vec<_> = f.m.iter().filter(|n| f.cls.get(*n) == Some(k)).collect();
        let entry = mem
            .iter()
            .map(|n| ring.project_pt(f.spine.xy(f.hk[k], f.o_h[*n]))[0])
            .fold(f64::NEG_INFINITY, f64::max)
            + 2. * ctx.cfg.lane_min;
        for n in mem {
            if ring.project_pt(f.land[n])[0] < entry {
                short.insert(n.clone());
            }
        }
    }
    if !short.is_empty() {
        let mut next = trunk.clone();
        next.extend(short);
        return build_with_trunk(ctx, dest, &next);
    }
    Ok(f)
}
fn stack_rings(
    f: &mut WholeFrame,
    ctx: &WholeContext,
    dpads: &[Point],
    near: &[String],
    tails: &[Point],
    north: f64,
    cen: Point,
    in_hull: &BTreeSet<String>,
    packed: &BTreeMap<String, f64>,
) -> Result<
    (
        BTreeMap<String, Vec<(String, f64)>>,
        BTreeMap<String, (f64, f64, f64)>,
    ),
    FrameError,
> {
    let hull: Vec<_> = dpads
        .iter()
        .copied()
        .chain(f.bend.values().copied())
        .chain(tails.iter().copied())
        .collect();
    let rpad = ctx
        .pads_of(&f.dst)
        .iter()
        .map(|p| p.size[0].max(p.size[1]) / 2.)
        .fold(0., f64::max);
    let width = |n: &String| {
        ctx.cfg.lane_min
            + if ctx.pairs.contains_key(n) {
                pairs::pitch(&ctx.cfg)
            } else {
                0.
            }
    };
    for k in ["N", "S"] {
        let sg = if k == "N" { north } else { -north };
        let ne = near
            .iter()
            .map(|n| sg * f.se[n][1] + (width(n) - ctx.cfg.lane_min) / 2.)
            .fold(-ctx.cfg.lane_min / 2., f64::max);
        let he = hull
            .iter()
            .map(|&p| sg * f.spine.project_pt(p)[1])
            .fold(f64::NEG_INFINITY, f64::max)
            + 2. * ctx.cfg.lane_pitch
            - ctx.cfg.lane_min;
        let mut edge = ne.max(he);
        if let Some(&p) = packed.get(k) {
            edge = edge.min(p);
        }
        let mut mem: Vec<_> = f
            .final_order
            .iter()
            .filter(|n| f.cls.get(*n).is_some_and(|c| c == k))
            .cloned()
            .collect();
        if k == "N" {
            mem.reverse();
        }
        for n in mem {
            f.o_h.insert(
                n.clone(),
                sg * (edge + width(&n) / 2. + ctx.cfg.lane_min / 2.),
            );
            edge += width(&n);
        }
    }
    f.rings.clear();
    f.hk.clear();
    let classes: BTreeSet<_> = f.cls.values().cloned().collect();
    for k in classes {
        let mem: Vec<_> =
            f.m.iter()
                .filter(|n| f.cls.get(*n) == Some(&k))
                .cloned()
                .collect();
        let sg = if k == "N" { north } else { -north };
        let stubs: Vec<_> = mem.iter().map(|n| f.bend[n]).collect();
        let inner = mem
            .iter()
            .min_by(|a, b| (sg * f.o_h[*a]).total_cmp(&(sg * f.o_h[*b])))
            .unwrap();
        let o_start = f.o_h[inner] - sg * ctx.cfg.lane_pitch;
        let mut s_start = f.h0;
        let rhull: Vec<_> = dpads
            .iter()
            .copied()
            .chain(stubs.iter().copied())
            .chain(tails.iter().copied())
            .collect();
        while inside_octohull(
            f.spine.xy(s_start, o_start),
            &rhull,
            2. * ctx.cfg.lane_pitch,
        ) && s_start > 0.
        {
            s_start -= ctx.cfg.lane_min / 5.;
        }
        let mut ccw = mem
            .iter()
            .map(|n| corridor::sweep_round(&ctx.paths[n], cen))
            .sum::<f64>()
            > 0.;
        if f.dcut.face != Face::E {
            let b = f.db;
            let wn = [
                [b[0], (b[1] + b[3]) / 2.],
                [(b[0] + b[2]) / 2., b[1]],
                [b[2], (b[1] + b[3]) / 2.],
            ];
            ccw = (corridor::sweep_round(&wn, cen) > 0.) == (k == "N");
        }
        f.hk.insert(k.clone(), s_start);
        let handoffs: Vec<_> = mem.iter().map(|n| f.spine.xy(s_start, f.o_h[n])).collect();
        let (_, ring) = corridor::ring_spine(
            dpads,
            &handoffs,
            &stubs,
            f.spine.xy(s_start, o_start),
            ccw,
            *f.spine.d.last().unwrap(),
            tails,
            ctx.cfg.lane_pitch,
            f.dcut.face != Face::E,
        )?;
        f.rings.insert(k, ring);
    }
    let mut hits: BTreeMap<String, Vec<(String, f64)>> = BTreeMap::new();
    let mut room = BTreeMap::new();
    let parts: BTreeSet<_> = ctx.pads.iter().map(|p| p.component_ref.clone()).collect();
    for k in f.rings.keys() {
        let mem: Vec<_> = f.m.iter().filter(|n| f.cls.get(*n) == Some(k)).collect();
        let sg = if k == "N" { north } else { -north };
        let offset = |p| sg * f.spine.project_pt(p)[1];
        let e = dpads
            .iter()
            .map(|&p| offset(p) + rpad)
            .chain(
                f.m.iter()
                    .map(|n| offset(f.bend[n]) + ctx.cfg.track_width / 2.),
            )
            .fold(f64::NEG_INFINITY, f64::max);
        let ne = near
            .iter()
            .map(|n| sg * f.se[n][1] + (width(n) - ctx.cfg.lane_min) / 2.)
            .fold(-ctx.cfg.lane_min / 2., f64::max);
        room.insert(k.clone(), (e, ne, mem.iter().map(|n| width(n)).sum()));
        let outer = mem
            .iter()
            .max_by(|a, b| {
                (sg * *f.o_h.get(**a).expect("frame lane lacks outer offset"))
                    .total_cmp(&(sg * *f.o_h.get(**b).expect("frame lane lacks outer offset")))
            })
            .unwrap();
        let min = mem
            .iter()
            .map(|n| sg * f.o_h[*n])
            .fold(f64::INFINITY, f64::min);
        let a = f.spine.xy(f.hk[k], sg * (min - ctx.cfg.lane_pitch));
        let b = f.spine.xy(f.hk[k], f.o_h[*outer]);
        let reach =
            (width(outer) - ctx.cfg.lane_min) / 2. + ctx.cfg.track_width / 2. + ctx.cfg.clearance;
        for part in &parts {
            if part == &f.src || part == &f.dst || in_hull.contains(part) {
                continue;
            }
            let rects: Vec<_> = ctx
                .pads_of(part)
                .iter()
                .filter(|p| {
                    ctx.layers.len() == 2
                        || p.drill > 0.
                        || ctx.layers.iter().all(|l| p.layers.contains(l))
                })
                .map(|p| {
                    [
                        p.point[0] - p.size[0] / 2.,
                        p.point[1] - p.size[1] / 2.,
                        p.point[0] + p.size[0] / 2.,
                        p.point[1] + p.size[1] / 2.,
                    ]
                })
                .collect();
            if rects
                .iter()
                .any(|&r| segment_rectangle_distance(a, b, r) < reach - 1e-9)
            {
                let inner = rects
                    .iter()
                    .flat_map(|r| [[r[0], r[1]], [r[0], r[3]], [r[2], r[1]], [r[2], r[3]]])
                    .map(offset)
                    .fold(f64::INFINITY, f64::min);
                hits.entry(k.clone())
                    .or_default()
                    .push((part.clone(), inner));
            }
        }
    }
    Ok((hits, room))
}
impl WholeFrame {
    pub fn ref_offset(&self, lane: &str, s: f64) -> Result<f64, String> {
        let rows = self.mid.get(lane).ok_or("unknown frame lane")?;
        if rows.is_empty() {
            return Err("empty reference path".into());
        }
        if s <= rows[0][0] {
            return Ok(rows[0][1]);
        }
        for w in rows.windows(2) {
            if s <= w[1][0] {
                let t = (s - w[0][0]) / (w[1][0] - w[0][0]);
                return Ok(w[0][1] + t * (w[1][1] - w[0][1]));
            }
        }
        Ok(rows.last().unwrap()[1])
    }
}
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct FlowFrame {
    #[serde(default)]
    pub mirror_axis_y: Option<f64>,
    pub quarter_turns: i32,
    pub center: Point,
}
impl FlowFrame {
    pub fn choose(
        source: &[Point],
        destination: &[Point],
        destination_center: Point,
    ) -> Result<Self, String> {
        if source.is_empty() || destination.is_empty() {
            return Err("flow frame has no source/destination pads".into());
        }
        let mean = |ps: &[Point]| mul(ps.iter().copied().fold([0., 0.], add), 1. / ps.len() as f64);
        let d = sub(mean(destination), mean(source));
        let mut best = 0;
        let mut x = f64::NEG_INFINITY;
        for k in 0..4 {
            let value = numeric::round_digits(
                Self {
                    mirror_axis_y: None,
                    quarter_turns: k,
                    center: [0., 0.],
                }
                .to_frame(d)[0],
                6,
            )?;
            if value > x {
                x = value;
                best = k;
            }
        }
        Ok(Self {
            mirror_axis_y: None,
            quarter_turns: best,
            center: [
                numeric::checked_round_i64(destination_center[0] / 0.1)? as f64 * 0.1,
                numeric::checked_round_i64(destination_center[1] / 0.1)? as f64 * 0.1,
            ],
        })
    }
    pub fn to_frame(self, mut p: Point) -> Point {
        if let Some(y) = self.mirror_axis_y {
            p[1] = 2. * y - p[1];
        }
        let mut d = sub(p, self.center);
        for _ in 0..self.quarter_turns.rem_euclid(4) {
            d = [-d[1], d[0]];
        }
        add(d, self.center)
    }
    pub fn to_board(self, p: Point) -> Point {
        let mut d = sub(p, self.center);
        for _ in 0..(-self.quarter_turns).rem_euclid(4) {
            d = [-d[1], d[0]];
        }
        let mut p = add(d, self.center);
        if let Some(y) = self.mirror_axis_y {
            p[1] = 2. * y - p[1];
        }
        p
    }
    pub fn to_frame_layer(self, name: &str, physical_stack: &[String]) -> Result<String, String> {
        if self.mirror_axis_y.is_none() {
            return Ok(name.into());
        }
        let i = physical_stack
            .iter()
            .position(|s| s == name)
            .ok_or_else(|| format!("unknown mirror layer {name}"))?;
        Ok(physical_stack[physical_stack.len() - 1 - i].clone())
    }
    pub fn to_board_layer(self, name: &str, physical_stack: &[String]) -> Result<String, String> {
        self.to_frame_layer(name, physical_stack)
    }
}
