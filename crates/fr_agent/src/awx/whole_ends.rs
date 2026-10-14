//! AWX caad45cd whole_ends.py: deterministic whole-route tooth/berth option search and exact route scoring.
use super::{
    board_snapshot::BoardSnapshot,
    corridor::{self, Point},
    escape_moves::{Move, MoveKind},
    joint_escape::{JointInput, JointMenus, JointPlan},
    pairs,
    whole_context::{RoutingRules, WholeContext},
    whole_frame::{self, FlowFrame, PerimeterCut},
};
use serde::{Deserialize, Serialize};
use std::{
    collections::{BTreeMap, BTreeSet, VecDeque},
    time::Instant,
};
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct FeedbackEnd {
    pub lane: String,
    pub end: usize,
    pub points: Vec<Point>,
    pub layer: String,
    pub times: u32,
    pub round: Option<usize>,
}
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct EndsFeedback {
    pub avoid: Vec<FeedbackEnd>,
    pub pairs: Vec<[FeedbackEnd; 2]>,
    pub over: BTreeMap<String, i64>,
    pub named: Vec<String>,
    /// Endpoint points/layers are already in the WholeContext/FlowFrame coordinate system.
    #[serde(default)]
    pub points_in_flow_frame: bool,
}
pub struct EndsProblem<'a> {
    pub snapshot: &'a BoardSnapshot,
    pub source_input: &'a JointInput,
    pub destination_input: &'a JointInput,
    pub source_menus: &'a JointMenus,
    pub destination_menus: &'a JointMenus,
    pub requested: &'a BTreeSet<String>,
    pub pairs: &'a [pairs::Pair],
    pub feedback: &'a EndsFeedback,
    pub laid_source: Option<&'a JointPlan>,
    pub laid_destination: Option<&'a JointPlan>,
    pub source_free: bool,
    pub free_teeth: Option<&'a BTreeSet<String>>,
    pub fixed: &'a BTreeMap<String, String>,
    pub seed: &'a BTreeMap<String, Move>,
    pub learned: &'a BTreeSet<(String, String)>,
    pub deadline: Instant,
    pub rules: &'a RoutingRules,
}
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct EndsModel {
    pub objective: f64,
    pub laid_objective: f64,
    pub free_objective: Option<f64>,
    pub moved: bool,
    pub fan: i64,
    pub route: i64,
    pub route_est: i64,
    pub over: i64,
    pub parity: i64,
    pub crossover: i64,
    pub cover: i64,
    pub settle: i64,
    pub couple: i64,
    pub congestion: f64,
    pub feedback: f64,
    pub ride: f64,
    pub changes: BTreeMap<String, i64>,
    pub crossings: usize,
    pub same: usize,
    pub conflicts: usize,
    pub splits: usize,
    pub refused: usize,
    pub lane_over: BTreeMap<String, i64>,
    pub lane_load: BTreeMap<String, f64>,
    pub lane_x: BTreeMap<String, usize>,
    pub stacks: usize,
    pub exact_failed: bool,
    pub cut: Option<PerimeterCut>,
    pub front_cost: f64,
    pub front: BTreeMap<String, BTreeMap<usize, [f64; 4]>>,
    pub walled: Vec<String>,
    pub conflict_lanes: Vec<String>,
}
pub struct EndsChoice {
    pub source: JointPlan,
    pub destination: JointPlan,
    pub model: EndsModel,
}

/// Remove menu escapes whose measured terminal face cannot equal the move's
/// declared face. This mirrors the source terminal readback contract and
/// prevents zero-pitch array axes from producing unmeasurable stubs.
pub fn retain_measurable_menu_moves(
    input: &JointInput,
    menus: &mut JointMenus,
) -> Result<(), String> {
    let mut grids = BTreeMap::new();
    for component in input
        .pads
        .iter()
        .map(|pad| pad.component_ref.as_str())
        .collect::<BTreeSet<_>>()
    {
        let pads: Vec<_> = input
            .pads
            .iter()
            .filter(|pad| pad.component_ref == component)
            .cloned()
            .collect();
        grids.insert(component.to_owned(), super::escape_moves::grid_of(&pads)?);
    }
    for (key, moves) in &mut menus.menus {
        let pad = menus
            .items
            .get(key)
            .ok_or_else(|| format!("AWX menu has no physical pad {key}"))?;
        let grid = grids
            .get(&pad.component_ref)
            .ok_or_else(|| format!("AWX menu has no array grid for {}", pad.component_ref))?;
        moves.retain(|movement| {
            terminal_direction(movement, grid).is_some_and(|face| face == movement.direction)
        });
    }
    Ok(())
}

fn terminal_direction(
    movement: &Move,
    grid: &super::escape_moves::Grid,
) -> Option<super::escape_moves::Direction> {
    let segment_direction = movement.legs.iter().rev().find_map(|leg| {
        let vector = corridor::sub(leg.b, leg.a);
        (vector[0].abs() + vector[1].abs() > 1e-9).then(|| snap_direction(vector))
    });
    let (hx, hy) = (grid.pitch_x / 2.0, grid.pitch_y / 2.0);
    let outside = [
        (
            super::escape_moves::Direction::Right,
            movement.exit_pt[0] > grid.bbox[2] + hx * 0.5,
        ),
        (
            super::escape_moves::Direction::Left,
            movement.exit_pt[0] < grid.bbox[0] - hx * 0.5,
        ),
        (
            super::escape_moves::Direction::Down,
            movement.exit_pt[1] > grid.bbox[3] + hy * 0.5,
        ),
        (
            super::escape_moves::Direction::Up,
            movement.exit_pt[1] < grid.bbox[1] - hy * 0.5,
        ),
    ];
    let outside: Vec<_> = outside
        .iter()
        .filter(|(_, yes)| *yes)
        .map(|(dir, _)| *dir)
        .collect();
    if outside.len() == 1 {
        return Some(outside[0]);
    }
    segment_direction
        .filter(|direction| outside.is_empty() || outside.contains(direction))
        .or_else(|| outside.first().copied())
}
#[derive(Clone, Debug)]
pub struct EndOption {
    pub moves: Vec<Move>,
    pub point: Point,
    pub layer: String,
    pub vias: i64,
    pub refused: bool,
}
type State = Vec<[usize; 2]>;
const W_OVER: f64 = 5.;
const BIG: f64 = 100000.;
const VIA_MM: f64 = 7.5;
const EPS_X: f64 = 0.01;
/// CPython's random.Random integer-seed MT19937 plus getrandbits/randbelow/sample behavior.
struct PythonRandom {
    mt: [u32; 624],
    index: usize,
}
impl PythonRandom {
    fn new(seed: u32) -> Self {
        let mut r = Self {
            mt: [0; 624],
            index: 624,
        };
        r.mt[0] = 19650218;
        for i in 1..624 {
            r.mt[i] = 1812433253u32
                .wrapping_mul(r.mt[i - 1] ^ (r.mt[i - 1] >> 30))
                .wrapping_add(i as u32);
        }
        let key = [seed];
        let (mut i, mut j) = (1usize, 0usize);
        for _ in 0..624.max(key.len()) {
            r.mt[i] = (r.mt[i] ^ ((r.mt[i - 1] ^ (r.mt[i - 1] >> 30)).wrapping_mul(1664525)))
                .wrapping_add(key[j])
                .wrapping_add(j as u32);
            i += 1;
            j += 1;
            if i >= 624 {
                r.mt[0] = r.mt[623];
                i = 1;
            }
            if j >= key.len() {
                j = 0;
            }
        }
        for _ in 0..623 {
            r.mt[i] = (r.mt[i] ^ ((r.mt[i - 1] ^ (r.mt[i - 1] >> 30)).wrapping_mul(1566083941)))
                .wrapping_sub(i as u32);
            i += 1;
            if i >= 624 {
                r.mt[0] = r.mt[623];
                i = 1;
            }
        }
        r.mt[0] = 0x80000000;
        r.index = 624;
        r
    }
    fn word(&mut self) -> u32 {
        if self.index >= 624 {
            for k in 0..624 {
                let y = (self.mt[k] & 0x80000000) | (self.mt[(k + 1) % 624] & 0x7fffffff);
                self.mt[k] =
                    self.mt[(k + 397) % 624] ^ (y >> 1) ^ if y & 1 != 0 { 0x9908b0df } else { 0 };
            }
            self.index = 0;
        }
        let mut y = self.mt[self.index];
        self.index += 1;
        y ^= y >> 11;
        y ^= (y << 7) & 0x9d2c5680;
        y ^= (y << 15) & 0xefc60000;
        y ^= y >> 18;
        y
    }
    fn getrandbits(&mut self, k: u32) -> u32 {
        if k == 0 {
            0
        } else {
            self.word() >> (32 - k)
        }
    }
    fn randbelow(&mut self, n: usize) -> usize {
        let bits = (usize::BITS - (n as usize).leading_zeros()) as u32;
        loop {
            let j = self.getrandbits(bits) as usize;
            if j < n {
                return j;
            }
        }
    }
    fn sample_indices(&mut self, n: usize, k: usize) -> Vec<usize> {
        let setsize = 21usize.saturating_add(if k > 5 {
            4usize.pow(((k * 3) as f64).log(4.).ceil() as u32)
        } else {
            0
        });
        let mut out = Vec::with_capacity(k);
        if n <= setsize {
            let mut pool: Vec<_> = (0..n).collect();
            for i in 0..k {
                let j = self.randbelow(n - i);
                out.push(pool[j]);
                pool[j] = pool[n - i - 1];
            }
        } else {
            let mut selected = BTreeSet::new();
            while out.len() < k {
                let j = self.randbelow(n);
                if selected.insert(j) {
                    out.push(j);
                }
            }
        }
        out
    }
}
fn dup_tolerance(rules: &RoutingRules) -> f64 {
    (rules.track_width + rules.clearance + 0.005) / 2.
}
pub fn move_signature(m: &Move) -> Result<String, String> {
    let kind = match m.kind {
        MoveKind::Surface => "surface",
        MoveKind::Dogbone => "dogbone",
        MoveKind::ViaInPad => "via_in_pad",
    };
    let direction = match m.direction {
        super::escape_moves::Direction::Left => "left",
        super::escape_moves::Direction::Right => "right",
        super::escape_moves::Direction::Up => "up",
        super::escape_moves::Direction::Down => "down",
    };
    let r = |v| super::numeric::round_digits(v, 2);
    let site = m
        .site
        .map(|p| Ok::<_, String>([r(p[0])?, r(p[1])?]))
        .transpose()?;
    let legs: Vec<_> = m
        .legs
        .iter()
        .map(|p| {
            Ok::<_, String>((
                r(p.a[0])?,
                r(p.a[1])?,
                r(p.b[0])?,
                r(p.b[1])?,
                p.layer.clone(),
            ))
        })
        .collect::<Result<_, _>>()?;
    serde_json::to_string(&(
        kind,
        direction,
        &m.layer,
        r(m.exit_pt[0])?,
        r(m.exit_pt[1])?,
        site,
        legs,
    ))
    .map_err(|e| e.to_string())
}
fn via_end(o: &EndOption) -> bool {
    !o.moves.is_empty() && o.moves.iter().all(|m| m.kind != MoveKind::Surface)
}
fn combinations(
    menus: &[Vec<Move>],
    reach: f64,
    forced: bool,
    dup: f64,
) -> Result<Vec<EndOption>, String> {
    match menus {
        [menu] => Ok(menu
            .iter()
            .map(|m| EndOption {
                moves: vec![m.clone()],
                point: m.exit_pt,
                layer: m.layer.clone(),
                vias: m.vias as i64,
                refused: false,
            })
            .collect()),
        [left, right] => {
            let mut out = vec![];
            for a in left {
                for b in right {
                    let distance = corridor::distance(a.exit_pt, b.exit_pt);
                    let ok = a.direction == b.direction
                        && a.layer == b.layer
                        && distance > dup
                        && distance <= reach;
                    if ok || forced {
                        out.push(EndOption {
                            moves: vec![a.clone(), b.clone()],
                            point: pairs::mid(a.exit_pt, b.exit_pt),
                            layer: a.layer.clone(),
                            vias: i64::from(a.vias) + i64::from(b.vias),
                            refused: !ok,
                        });
                    }
                }
            }
            Ok(out)
        }
        _ => Err("whole_ends requires one single or two physical pair legs".into()),
    }
}
fn feedback_weights(
    opts: &[EndOption],
    item: &FeedbackEnd,
    nlayers: usize,
    dup: f64,
) -> Result<BTreeMap<usize, f64>, String> {
    let escalate = 2_f64
        .powi(i32::try_from(item.times.max(1) - 1).map_err(|_| "feedback escalation overflow")?);
    if !escalate.is_finite() {
        return Err("feedback escalation is not finite".into());
    }
    let mut out = BTreeMap::new();
    for (i, o) in opts.iter().enumerate() {
        let d = o
            .moves
            .iter()
            .flat_map(|m| {
                item.points
                    .iter()
                    .map(move |p| corridor::distance(m.exit_pt, *p))
            })
            .fold(f64::INFINITY, f64::min);
        let place = if d <= dup {
            1.
        } else {
            0.5 * (1. - (d - dup) / 0.76).max(0.)
        };
        if place > 0. {
            let other_ok = !(nlayers > 2 && via_end(o));
            out.insert(
                i,
                escalate
                    * place
                    * if o.layer == item.layer || !other_ok {
                        1.
                    } else {
                        0.5
                    },
            );
        }
    }
    Ok(out)
}
/// Source min_cut_cover: exact weighted bipartite vertex cover through residual max-flow.
fn min_cut_cover(
    edges: &[(usize, usize)],
    weights: &BTreeMap<usize, f64>,
) -> Result<Vec<usize>, String> {
    let left: BTreeSet<_> = edges.iter().map(|p| p.0).collect();
    let right: BTreeSet<_> = edges.iter().map(|p| p.1).collect();
    let ls: Vec<_> = left.iter().copied().collect();
    let rs: Vec<_> = right.iter().copied().collect();
    let sink = 1 + ls.len() + rs.len();
    let mut adj = vec![vec![]; sink + 1];
    let mut cap: BTreeMap<(usize, usize), f64> = BTreeMap::new();
    let mut add = |u: usize, v: usize, c: f64| {
        adj[u].push(v);
        adj[v].push(u);
        cap.insert((u, v), c);
        cap.entry((v, u)).or_insert(0.);
    };
    for (i, n) in ls.iter().enumerate() {
        add(0, i + 1, *weights.get(n).ok_or("missing min-cut weight")?)
    }
    for (i, n) in rs.iter().enumerate() {
        add(
            1 + ls.len() + i,
            sink,
            *weights.get(n).ok_or("missing min-cut weight")?,
        )
    }
    for (a, b) in edges {
        let i = ls.iter().position(|n| n == a).unwrap() + 1;
        let j = rs.iter().position(|n| n == b).unwrap() + 1 + ls.len();
        add(i, j, f64::INFINITY)
    }
    let final_seen = loop {
        let mut prev = vec![None; sink + 1];
        let mut seen = vec![false; sink + 1];
        seen[0] = true;
        let mut queue = VecDeque::from([0]);
        while let Some(u) = queue.pop_front() {
            for &v in &adj[u] {
                if !seen[v] && cap[&(u, v)] > 1e-9 {
                    seen[v] = true;
                    prev[v] = Some(u);
                    queue.push_back(v)
                }
            }
        }
        if !seen[sink] {
            break seen;
        }
        let mut f = f64::INFINITY;
        let mut v = sink;
        while let Some(u) = prev[v] {
            f = f.min(cap[&(u, v)]);
            v = u
        }
        if !f.is_finite() || f <= 0. {
            return Err("invalid weighted min-cut flow".into());
        }
        let mut v = sink;
        while let Some(u) = prev[v] {
            *cap.get_mut(&(u, v)).unwrap() -= f;
            *cap.get_mut(&(v, u)).unwrap() += f;
            v = u
        }
    };
    Ok(ls
        .into_iter()
        .enumerate()
        .filter(|(i, _)| !final_seen[i + 1])
        .map(|(_, n)| n)
        .chain(
            rs.into_iter()
                .enumerate()
                .filter(|(i, _)| final_seen[1 + left.len() + i])
                .map(|(_, n)| n),
        )
        .collect())
}

#[derive(Clone)]
struct Span {
    row: bool,
    coordinate: f64,
    layer: String,
    a: f64,
    b: f64,
}
fn spans(m: &Move) -> Result<Vec<Span>, String> {
    if m.legs.is_empty() && m.site.is_none() {
        return Ok(vec![]);
    }
    let plain = || {
        let src = m
            .site
            .or_else(|| m.legs.first().map(|p| p.a))
            .ok_or("move has neither site nor legs")?;
        let row = matches!(
            m.direction,
            super::escape_moves::Direction::Left | super::escape_moves::Direction::Right
        );
        let axis = usize::from(!row);
        let keyaxis = 1 - axis;
        Ok::<_, String>(Span {
            row,
            coordinate: super::numeric::round_digits(m.exit_pt[keyaxis], 3)?,
            layer: m.layer.clone(),
            a: src[axis].min(m.exit_pt[axis]),
            b: src[axis].max(m.exit_pt[axis]),
        })
    };
    if m.climb == 0 && !m.street {
        return Ok(vec![plain()?]);
    }
    let mut out = vec![];
    for p in &m.legs {
        if p.layer != m.layer && !m.street {
            continue;
        }
        let dx = (p.a[0] - p.b[0]).abs();
        let dy = (p.a[1] - p.b[1]).abs();
        if dx < 1e-6 && dy > 1e-6 {
            out.push(Span {
                row: false,
                coordinate: super::numeric::round_digits(p.a[0], 3)?,
                layer: p.layer.clone(),
                a: p.a[1].min(p.b[1]),
                b: p.a[1].max(p.b[1]),
            })
        } else if dy < 1e-6 && dx > 1e-6 {
            out.push(Span {
                row: true,
                coordinate: super::numeric::round_digits(p.a[1], 3)?,
                layer: p.layer.clone(),
                a: p.a[0].min(p.b[0]),
                b: p.a[0].max(p.b[0]),
            })
        }
    }
    if out.is_empty() {
        out.push(plain()?)
    }
    Ok(out)
}
fn site_in_lane(m: &Move, s: &Span) -> bool {
    let Some(p) = m.site else { return false };
    let axis = usize::from(!s.row);
    (p[1 - axis] - s.coordinate).abs() < 0.30 && s.a - 0.30 < p[axis] && p[axis] < s.b + 0.30
}
fn moves_conflict(m: &Move, other: &Move, strict: bool, stack_pitch: f64) -> Result<bool, String> {
    let a = spans(m)?;
    let b = spans(other)?;
    for p in &a {
        for q in &b {
            let same = p.row == q.row
                && p.layer == q.layer
                && if strict {
                    (p.coordinate - q.coordinate).abs() < 0.16
                } else {
                    p.coordinate == q.coordinate
                };
            if same && p.a < q.b + 1e-6 && q.a < p.b + 1e-6 {
                return Ok(true);
            }
            if p.row != q.row && p.layer == q.layer {
                let (r, c) = if p.row { (p, q) } else { (q, p) };
                if r.a - 0.16 < c.coordinate
                    && c.coordinate < r.b + 0.16
                    && c.a - 0.16 < r.coordinate
                    && r.coordinate < c.b + 0.16
                {
                    return Ok(true);
                }
            }
        }
    }
    if let (Some(a), Some(b)) = (m.site, other.site) {
        if super::numeric::round_digits(a[0], 3)? == super::numeric::round_digits(b[0], 3)?
            && super::numeric::round_digits(a[1], 3)? == super::numeric::round_digits(b[1], 3)?
        {
            return Ok(true);
        }
    }
    if a.iter().any(|s| site_in_lane(other, s)) || b.iter().any(|s| site_in_lane(m, s)) {
        return Ok(true);
    }
    if m.layer == other.layer
        && ((m.exit_pt[0] - other.exit_pt[0]).abs() < 0.16
            && (m.exit_pt[1] - other.exit_pt[1]).abs() < 0.16
            || corridor::distance(m.exit_pt, other.exit_pt) < stack_pitch)
    {
        return Ok(true);
    }
    Ok(false)
}
fn segment_hits_box(a: Point, b: Point, bounds: [f64; 4]) -> bool {
    let d = corridor::sub(b, a);
    let mut lo = 0_f64;
    let mut hi = 1_f64;
    for (p, q) in [
        (-d[0], a[0] - bounds[0]),
        (d[0], bounds[2] - a[0]),
        (-d[1], a[1] - bounds[1]),
        (d[1], bounds[3] - a[1]),
    ] {
        if p.abs() < 1e-12 {
            if q < 0. {
                return false;
            }
            continue;
        }
        let r = q / p;
        if p < 0. {
            if r > hi {
                return false;
            }
            lo = lo.max(r)
        } else {
            if r < lo {
                return false;
            }
            hi = hi.min(r)
        }
    }
    lo < hi
}
fn around_box(a: Point, b: Point, bounds: [f64; 4], pad: f64) -> f64 {
    let r = [
        bounds[0] - pad,
        bounds[1] - pad,
        bounds[2] + pad,
        bounds[3] + pad,
    ];
    let inner = [r[0] + 0.05, r[1] + 0.05, r[2] - 0.05, r[3] - 0.05];
    if !segment_hits_box(a, b, inner) {
        return corridor::distance(a, b);
    }
    let corners = [[r[0], r[1]], [r[2], r[1]], [r[0], r[3]], [r[2], r[3]]];
    let mut best = f64::INFINITY;
    for c in corners {
        for d in corners {
            if segment_hits_box(a, c, inner)
                || segment_hits_box(d, b, inner)
                || c != d && segment_hits_box(c, d, inner)
            {
                continue;
            }
            let len =
                corridor::distance(a, c) + corridor::distance(c, d) + corridor::distance(d, b);
            if len < best {
                best = len
            }
        }
    }
    if best.is_finite() {
        best
    } else {
        corridor::distance(a, b)
    }
}
fn wound_ride(a: Point, b: Point, bounds: [f64; 4], north: bool, pad: f64) -> (f64, usize) {
    let r = [
        bounds[0] - pad,
        bounds[1] - pad,
        bounds[2] + pad,
        bounds[3] + pad,
    ];
    let seq = if north {
        [[r[0], r[1]], [r[2], r[1]], [r[2], r[3]], [r[0], r[3]]]
    } else {
        [[r[0], r[3]], [r[2], r[3]], [r[2], r[1]], [r[0], r[1]]]
    };
    let faces = if north {
        [
            whole_frame::Face::N,
            whole_frame::Face::E,
            whole_frame::Face::S,
            whole_frame::Face::W,
        ]
    } else {
        [
            whole_frame::Face::S,
            whole_frame::Face::E,
            whole_frame::Face::N,
            whole_frame::Face::W,
        ]
    };
    let face = whole_frame::face(b, r);
    let mut last = a;
    let mut length = 0.;
    let mut turns = 0;
    for (c, f) in seq.into_iter().zip(faces) {
        length += corridor::distance(last, c);
        last = c;
        turns += 1;
        if f == face {
            break;
        }
    }
    (length + corridor::distance(last, b), turns)
}
fn cover(
    flat: &[usize],
    layer: &[String],
    kp: &[(f64, usize)],
    kd: &[(f64, usize)],
    weights: &[f64],
) -> BTreeSet<usize> {
    let mut out = BTreeSet::new();
    for l in flat
        .iter()
        .map(|i| layer[*i].clone())
        .collect::<BTreeSet<_>>()
    {
        let mut ls: Vec<_> = flat.iter().copied().filter(|i| layer[*i] == l).collect();
        ls.sort_by(|a, b| kp[*a].partial_cmp(&kp[*b]).unwrap());
        let mut best = BTreeMap::new();
        let mut prev: BTreeMap<usize, Option<usize>> = BTreeMap::new();
        for (i, a) in ls.iter().enumerate() {
            best.insert(*a, weights[*a]);
            prev.insert(*a, None);
            for b in &ls[..i] {
                if kd[*b] < kd[*a] && best[b] + weights[*a] > best[a] {
                    best.insert(*a, best[b] + weights[*a]);
                    prev.insert(*a, Some(*b));
                }
            }
        }
        if ls.is_empty() {
            continue;
        }
        let mut x = Some(
            *ls.iter()
                .max_by(|a, b| best[a].total_cmp(&best[b]).then(kp[**b].1.cmp(&kp[**a].1)))
                .unwrap(),
        );
        let mut chain = BTreeSet::new();
        while let Some(n) = x {
            chain.insert(n);
            x = prev[&n]
        }
        out.extend(ls.into_iter().filter(|n| !chain.contains(n)));
    }
    out
}

#[derive(Clone, Debug)]
struct EndDomain {
    fixed: Option<usize>,
    neck: Option<usize>,
    allowed: BTreeSet<usize>,
}
#[derive(Clone)]
struct ExactRoute {
    route: i64,
    over: i64,
    changes: Vec<i64>,
    dropped: Vec<i64>,
}
fn exact_route(
    kp: &[(f64, usize)],
    kd: &[(f64, usize)],
    layers: &[String],
    domains: &[[EndDomain; 2]],
    stub_vias: &[i64],
    opposite: &[bool],
    multiplicity: &[i64],
    separation: &[((usize, usize), (usize, usize))],
    deadline: Instant,
) -> Result<Option<ExactRoute>, String> {
    use super::cp_model::{BoolVar, CpSolverStatus, IntVar, Model, SatParameters};
    let nl = layers.len();
    let n = kp.len();
    let mut launch: Vec<_> = (0..n).collect();
    launch.sort_by(|a, b| kp[*a].partial_cmp(&kp[*b]).unwrap());
    let mut final_order: Vec<_> = (0..n).collect();
    final_order.sort_by(|a, b| kd[*a].partial_cmp(&kd[*b]).unwrap());
    let mut rl = vec![0; n];
    let mut rf = vec![0; n];
    for (i, a) in launch.iter().enumerate() {
        rl[*a] = i
    }
    for (i, a) in final_order.iter().enumerate() {
        rf[*a] = i
    }
    let qkp: Vec<_> = kp
        .iter()
        .map(|k| super::numeric::round_digits(k.0, 3))
        .collect::<Result<_, _>>()?;
    let qkd: Vec<_> = kd
        .iter()
        .map(|k| super::numeric::round_digits(k.0, 3))
        .collect::<Result<_, _>>()?;
    launch.sort_by(|a, b| {
        qkp[*a]
            .total_cmp(&qkp[*b])
            .then(rf[*a].cmp(&rf[*b]))
            .then(a.cmp(b))
    });
    final_order.sort_by(|a, b| {
        qkd[*a]
            .total_cmp(&qkd[*b])
            .then(rl[*a].cmp(&rl[*b]))
            .then(a.cmp(b))
    });
    for (i, a) in launch.iter().enumerate() {
        rl[*a] = i
    }
    for (i, a) in final_order.iter().enumerate() {
        rf[*a] = i
    }
    let mut x = Vec::new();
    for (i, a) in launch.iter().enumerate() {
        for b in launch.iter().skip(i + 1) {
            if rf[*a] > rf[*b] {
                x.push((*a, *b))
            }
        }
    }
    let h = i64::try_from(x.len())
        .map_err(|_| "exact order size overflow")?
        .checked_mul(2)
        .and_then(|h| h.checked_add(2))
        .ok_or("exact horizon overflow")?;
    let mut m = Model::new();
    let mut t = BTreeMap::new();
    for key in &x {
        t.insert(*key, m.int(1, h, "")?);
    }
    for (ii, i) in launch.iter().enumerate() {
        for (jj, j) in launch.iter().enumerate().skip(ii + 1) {
            for k in launch.iter().skip(jj + 1) {
                let ij = t.get(&(*i, *j));
                let ik = t.get(&(*i, *k));
                let jk = t.get(&(*j, *k));
                if let (Some(ij), Some(ik), Some(jk)) = (ij, ik, jk) {
                    let b = m.bool("");
                    m.le(&[(*ij, 1), (*ik, -1)], -1, &[b])?;
                    m.le(&[(*ik, 1), (*jk, -1)], -1, &[b])?;
                    m.le(&[(*jk, 1), (*ik, -1)], -1, &[!b])?;
                    m.le(&[(*ik, 1), (*ij, -1)], -1, &[!b])?;
                } else if let (Some(ij), Some(ik)) = (ij, ik) {
                    m.le(&[(*ij, 1), (*ik, -1)], -1, &[])?;
                } else if let (Some(ik), Some(jk)) = (ik, jk) {
                    m.le(&[(*jk, 1), (*ik, -1)], -1, &[])?;
                }
            }
        }
    }
    let events: Vec<Vec<_>> = (0..n)
        .map(|a| {
            x.iter()
                .copied()
                .filter(|key| key.0 == a || key.1 == a)
                .collect()
        })
        .collect();
    for ev in &events {
        if ev.len() > 1 {
            m.inner
                .add_all_different(&ev.iter().map(|k| t[k]).collect::<Vec<_>>());
        }
    }
    let mut acts: Vec<Vec<BoolVar>> = vec![];
    let mut ys: Vec<Vec<IntVar>> = vec![];
    let mut drops: Vec<Vec<BoolVar>> = vec![];
    let mut before: BTreeMap<(usize, (usize, usize)), Vec<BoolVar>> = BTreeMap::new();
    let mut cost = vec![];
    for lane in 0..n {
        let cs = (0..3)
            .map(|_| m.int(0, h + 1, ""))
            .collect::<Result<Vec<_>, _>>()?;
        let act: Vec<_> = (0..3).map(|_| m.bool("")).collect();
        for k in 0..3 {
            m.le(&[(cs[k], 1)], h, &[act[k]])?;
            m.eq(&[(cs[k], 1)], h + 1, &[!act[k]])?;
            if k > 0 {
                m.ge(&[(cs[k], 1), (cs[k - 1], -1)], 1, &[act[k]])?;
                m.implication(act[k], act[k - 1]);
            }
        }
        for key in &events[lane] {
            let mut bits = vec![];
            for k in 0..3 {
                let b = m.bool("");
                m.le(&[(cs[k], 1), (t[key], -1)], -1, &[b])?;
                m.ge(&[(cs[k], 1), (t[key], -1)], 1, &[!b, act[k]])?;
                m.implication(b, act[k]);
                bits.push(b)
            }
            before.insert((lane, *key), bits);
        }
        let mut dropped = vec![];
        if nl == 2 {
            let tl = domains[lane][0]
                .fixed
                .ok_or("two-layer exact tooth must be fixed")?;
            let dl = domains[lane][1]
                .fixed
                .ok_or("two-layer exact berth must be fixed")?;
            m.xor_with_one(&act, tl == dl)?;
            ys.push(vec![]);
        } else {
            let y = (0..4)
                .map(|_| m.int(0, nl as i64 - 1, ""))
                .collect::<Result<Vec<_>, _>>()?;
            for (index, domain) in [(0, &domains[lane][0]), (3, &domains[lane][1])] {
                if let Some(f) = domain.fixed {
                    m.eq(&[(y[index], 1)], f as i64, &[])?;
                } else {
                    for q in 0..nl {
                        if !domain.allowed.contains(&q) {
                            m.ne(&[(y[index], 1)], q as i64, &[])?;
                        }
                    }
                    if let Some(neck) = domain.neck.filter(|q| domain.allowed.contains(q)) {
                        let d = m.bool("");
                        m.eq(&[(y[index], 1)], neck as i64, &[d])?;
                        m.ne(&[(y[index], 1)], neck as i64, &[!d])?;
                        dropped.push(d)
                    }
                }
            }
            for k in 0..3 {
                m.ne(&[(y[k + 1], 1), (y[k], -1)], 0, &[act[k]])?;
                m.eq(&[(y[k + 1], 1), (y[k], -1)], 0, &[!act[k]])?;
            }
            ys.push(y);
        }
        if opposite[lane] {
            m.eq(&[(act[0].as_int_var(), 1)], 1, &[])?;
        }
        let ov = m.int(0, 11, "")?;
        let mut terms = vec![(ov, 1)];
        terms.extend(act.iter().map(|b| (b.as_int_var(), -1)));
        terms.extend(dropped.iter().map(|b| (b.as_int_var(), 1)));
        m.ge(&terms, stub_vias[lane] - 2, &[])?;
        cost.push((ov, 5));
        cost.extend(act.iter().map(|b| (b.as_int_var(), multiplicity[lane])));
        cost.extend(
            dropped
                .iter()
                .map(|b| (b.as_int_var(), -multiplicity[lane])),
        );
        acts.push(act);
        drops.push(dropped);
    }
    if nl > 2 {
        for &((a, ka), (b, kb)) in separation {
            m.ne(
                &[
                    (ys[a][if ka == 0 { 0 } else { 3 }], 1),
                    (ys[b][if kb == 0 { 0 } else { 3 }], -1),
                ],
                0,
                &[],
            )?;
        }
    }
    for key in &x {
        let (a, b) = *key;
        if nl == 2 {
            let mut bits = before[&(a, *key)].clone();
            bits.extend(&before[&(b, *key)]);
            m.xor_with_one(&bits, domains[a][0].fixed != domains[b][0].fixed)?;
        } else {
            let mut at = vec![];
            for lane in [a, b] {
                let index = m.int(0, 3, "")?;
                let mut terms = vec![(index, 1)];
                terms.extend(before[&(lane, *key)].iter().map(|b| (b.as_int_var(), -1)));
                m.eq(&terms, 0, &[])?;
                let l = m.int(0, nl as i64 - 1, "")?;
                m.element(index, &ys[lane], l)?;
                at.push(l)
            }
            m.ne(&[(at[0], 1), (at[1], -1)], 0, &[])?;
        }
    }
    m.minimize(&cost);
    let mut params = SatParameters::default();
    params.num_workers = Some(1);
    params.max_deterministic_time = Some(60.);
    params.max_time_in_seconds = Some(
        deadline
            .saturating_duration_since(Instant::now())
            .as_secs_f64(),
    );
    let response = super::cp_sat_bridge::solve(&m.inner, &params, None)?;
    if !matches!(
        response.status(),
        CpSolverStatus::Feasible | CpSolverStatus::Optimal
    ) {
        return Ok(None);
    }
    let value = |b: BoolVar| {
        response
            .solution
            .get(b.index() as usize)
            .copied()
            .ok_or_else(|| "exact route incomplete solution".to_string())
    };
    let changes: Vec<i64> = acts
        .iter()
        .map(|a| {
            a.iter()
                .try_fold(0_i64, |sum, b| Ok::<_, String>(sum + value(*b)?))
        })
        .collect::<Result<_, _>>()?;
    let dropped: Vec<i64> = drops
        .iter()
        .map(|a| {
            a.iter()
                .try_fold(0_i64, |sum, b| Ok::<_, String>(sum + value(*b)?))
        })
        .collect::<Result<_, _>>()?;
    let route = (0..n).try_fold(0_i64, |sum, i| {
        super::numeric::checked_add(
            sum,
            super::numeric::checked_mul(multiplicity[i], changes[i])?,
        )
    })?;
    let over = (0..n).try_fold(0_i64, |sum, i| {
        super::numeric::checked_add(sum, (stub_vias[i] + changes[i] - dropped[i] - 2).max(0))
    })?;
    Ok(Some(ExactRoute {
        route,
        over,
        changes,
        dropped,
    }))
}

fn transform_move(mut m: Move, flow: FlowFrame, physical: &[String]) -> Result<Move, String> {
    let (dx, dy) = m.direction.delta();
    let mut vector = [dx as f64, dy as f64];
    if flow.mirror_axis_y.is_some() {
        vector[1] = -vector[1]
    }
    for _ in 0..flow.quarter_turns.rem_euclid(4) {
        vector = [-vector[1], vector[0]]
    }
    m.direction = if vector[0] < -0.5 {
        super::escape_moves::Direction::Left
    } else if vector[0] > 0.5 {
        super::escape_moves::Direction::Right
    } else if vector[1] < -0.5 {
        super::escape_moves::Direction::Up
    } else {
        super::escape_moves::Direction::Down
    };
    m.exit_pt = flow.to_frame(m.exit_pt);
    m.site = m.site.map(|p| flow.to_frame(p));
    m.layer = flow.to_frame_layer(&m.layer, physical)?;
    for leg in &mut m.legs {
        leg.a = flow.to_frame(leg.a);
        leg.b = flow.to_frame(leg.b);
        leg.layer = flow.to_frame_layer(&leg.layer, physical)?
    }
    for l in &mut m.runs_on {
        *l = flow.to_frame_layer(l, physical)?
    }
    Ok(m)
}
fn plain_context(problem: &EndsProblem<'_>, flow: FlowFrame) -> Result<WholeContext, String> {
    let s = problem.snapshot;
    let mut ctx = WholeContext {
        cfg: problem.rules.clone(),
        layers: problem.source_input.routing_layers.clone(),
        copper_layers: s.layers.clone(),
        board_bounds: s.board_bounds,
        board_outline: s.board_outline.clone(),
        pads: s.pads.clone(),
        source: problem.source_input.reference.clone(),
        destination: problem.destination_input.reference.clone(),
        lanes: vec![],
        byname: s.byname.clone(),
        pairs: BTreeMap::new(),
        ends: BTreeMap::new(),
        src_ref: BTreeMap::new(),
        tooth_layer: BTreeMap::new(),
        dest_layer: BTreeMap::new(),
        tooth_dir: BTreeMap::new(),
        stub_dir: BTreeMap::new(),
        pair_ends: BTreeMap::new(),
        pair_layers: BTreeMap::new(),
        paths: BTreeMap::new(),
        base_segments: s.segments.clone(),
        base_vias: s.vias.clone(),
        dest_cut: None,
        plane_nets: BTreeSet::new(),
        native_to_logical_components: s
            .pads
            .iter()
            .map(|p| (p.key.clone(), p.component_ref.clone()))
            .collect(),
        chirality_mirrored: flow.mirror_axis_y.is_some(),
    };
    ctx = ctx.transformed(flow);
    for l in &mut ctx.copper_layers {
        *l = flow.to_frame_layer(l, &s.layers)?
    }
    for p in &mut ctx.pads {
        for l in &mut p.layers {
            *l = flow.to_frame_layer(l, &s.layers)?
        }
        for shape in &mut p.copper_shapes {
            shape.layer = flow.to_frame_layer(&shape.layer, &s.layers)?
        }
    }
    for segment in &mut ctx.base_segments {
        segment.layer = flow.to_frame_layer(&segment.layer, &s.layers)?
    }
    for via in &mut ctx.base_vias {
        for l in &mut via.layers {
            *l = flow.to_frame_layer(l, &s.layers)?
        }
    }
    Ok(ctx)
}
fn grid_for(ctx: &WholeContext, part: &str) -> Result<super::escape_moves::Grid, String> {
    let pads: Vec<_> = ctx
        .pads
        .iter()
        .filter(|p| p.component_ref == part)
        .map(|p| super::escape_moves::Pad {
            key: p.key.clone(),
            net: p.net.clone(),
            net_id: p.net_id,
            component_ref: p.component_ref.clone(),
            pad_number: p.pad_number.clone(),
            point: p.point,
            layers: p.layers.clone(),
            size: p.size,
        })
        .collect();
    super::escape_moves::grid_of(&pads)
}
fn frame_for(problem: &EndsProblem<'_>) -> Result<FlowFrame, String> {
    let s = problem.snapshot;
    let src = &problem.source_input.reference;
    let dst = &problem.destination_input.reference;
    let box_for = |part: &str| -> Result<[f64; 4], String> {
        let p: Vec<_> = s
            .pads
            .iter()
            .filter(|p| p.component_ref == part)
            .map(|p| p.point)
            .collect();
        if p.is_empty() {
            return Err(format!("whole_ends missing actual array {part}"));
        }
        Ok([
            p.iter().map(|p| p[0]).fold(f64::INFINITY, f64::min),
            p.iter().map(|p| p[1]).fold(f64::INFINITY, f64::min),
            p.iter().map(|p| p[0]).fold(f64::NEG_INFINITY, f64::max),
            p.iter().map(|p| p[1]).fold(f64::NEG_INFINITY, f64::max),
        ])
    };
    let sb = box_for(src)?;
    let db = box_for(dst)?;
    let sc = [(sb[0] + sb[2]) / 2., (sb[1] + sb[3]) / 2.];
    let dc = [(db[0] + db[2]) / 2., (db[1] + db[3]) / 2.];
    let axis = corridor::sub(dc, sc);
    let mut sp = vec![];
    let mut dp = vec![];
    let mut moment = 0.;
    let names: Vec<_> = problem.requested.iter().cloned().collect();
    let detected = if problem.pairs.is_empty() {
        pairs::pair_names(&names, true)
    } else {
        problem.pairs.to_vec()
    };
    let (lanes, prs) = pairs::members(&names, &detected);
    for n in &lanes {
        let physical = prs.get(n).map(|p| &p[0]).unwrap_or(n);
        let id = *s
            .byname
            .get(physical)
            .ok_or("whole_ends unknown requested net")?;
        let a = s
            .pads
            .iter()
            .find(|p| p.component_ref == *src && p.net_id == id)
            .ok_or("whole_ends missing source ball")?
            .point;
        let b = s
            .pads
            .iter()
            .find(|p| p.component_ref == *dst && p.net_id == id)
            .ok_or("whole_ends missing destination ball")?
            .point;
        moment += corridor::cross(axis, corridor::sub(a, sc))
            + corridor::cross(axis, corridor::sub(b, sc));
        sp.push(a);
        dp.push(b);
    }
    let mirror = if moment < 0. {
        Some(
            super::numeric::checked_round_i64((s.board_bounds[1] + s.board_bounds[3]) / 2. / 0.05)?
                as f64
                * 0.05,
        )
    } else {
        None
    };
    let map = |mut p: Point| {
        if let Some(y) = mirror {
            p[1] = 2. * y - p[1]
        }
        p
    };
    let mut flow = FlowFrame::choose(
        &sp.into_iter().map(map).collect::<Vec<_>>(),
        &dp.into_iter().map(map).collect::<Vec<_>>(),
        map(dc),
    )?;
    flow.mirror_axis_y = mirror;
    Ok(flow)
}
struct Ends<'a> {
    problem: &'a EndsProblem<'a>,
    ctx: WholeContext,
    flow: FlowFrame,
    lanes: Vec<(String, Vec<String>)>,
    t: Vec<Vec<EndOption>>,
    b: Vec<Vec<EndOption>>,
    cur: BTreeMap<String, Move>,
    sbox: [f64; 4],
    dbox: [f64; 4],
    ring_d0: f64,
    run_ids: BTreeSet<i32>,
    oconf: BTreeMap<(usize, usize, usize), BTreeSet<(usize, usize)>>,
    front: BTreeMap<(usize, usize, usize), (u8, Option<[f64; 4]>)>,
    tie: BTreeMap<String, i64>,
    fb_avoid: BTreeMap<(usize, usize), BTreeMap<usize, f64>>,
    fb_pairs: Vec<(
        usize,
        usize,
        usize,
        BTreeMap<usize, f64>,
        BTreeMap<usize, f64>,
    )>,
    pw: Option<super::ends_geometry::PlaneWays>,
    pool: BTreeMap<State, f64>,
    memo: BTreeMap<(State, bool, String), (f64, EndsModel)>,
    xroute: BTreeMap<(State, String), Option<ExactRoute>>,
    wcut: Option<PerimeterCut>,
    tabu: BTreeSet<(usize, usize, usize)>,
}
impl<'a> Ends<'a> {
    fn new(problem: &'a EndsProblem<'a>, source_free: bool) -> Result<Self, String> {
        problem.rules.validate()?;
        let flow = frame_for(problem)?;
        let mut ctx = plain_context(problem, flow)?;
        let sg = grid_for(&ctx, &ctx.source)?;
        let dg = grid_for(&ctx, &ctx.destination)?;
        let cs = [
            (sg.bbox[0] + sg.bbox[2]) / 2.,
            (sg.bbox[1] + sg.bbox[3]) / 2.,
        ];
        let cd = [
            (dg.bbox[0] + dg.bbox[2]) / 2.,
            (dg.bbox[1] + dg.bbox[3]) / 2.,
        ];
        if cd[0] <= cs[0] || (cd[1] - cs[1]).abs() > cd[0] - cs[0] {
            return Err("whole_ends actual board is not in canonical flow".into());
        }
        let mut names = Vec::new();
        for net in problem.requested.iter() {
            let keys = problem.destination_menus.physical_keys_for_net(net)?;
            if keys.iter().any(|key| {
                problem
                    .destination_menus
                    .menus
                    .get(*key)
                    .is_some_and(|menu| !menu.is_empty())
            }) {
                names.push(net.clone());
            }
        }
        if names.len() != problem.requested.len() {
            return Err("whole_ends admitted net has no real destination menu".into());
        }
        let detected = if problem.pairs.is_empty() {
            pairs::pair_names(&names, true)
        } else {
            problem.pairs.to_vec()
        };
        let prs: BTreeMap<_, _> = detected
            .iter()
            .map(|p| (p.base.clone(), vec![p.p.clone(), p.n.clone()]))
            .collect();
        let legs: BTreeSet<_> = prs.values().flatten().cloned().collect();
        let mut lanes: Vec<_> = names
            .iter()
            .filter(|n| !legs.contains(*n))
            .map(|n| (n.clone(), vec![n.clone()]))
            .collect();
        lanes.extend(prs.iter().map(|(b, p)| (b.clone(), p.clone())));
        for (base, p) in &prs {
            if p.iter().any(|n| !problem.requested.contains(n)) || ctx.byname.contains_key(base) {
                return Err(format!("whole_ends invalid physical pair {base}"));
            }
            ctx.byname.insert(base.clone(), ctx.byname[&p[0]]);
            ctx.pairs.insert(base.clone(), [p[0].clone(), p[1].clone()]);
        }
        ctx.lanes = lanes.iter().map(|(n, _)| n.clone()).collect();
        let run_ids = lanes
            .iter()
            .flat_map(|(_, lg)| lg.iter())
            .map(|n| ctx.byname[n])
            .collect::<BTreeSet<_>>();
        let mut cur = BTreeMap::new();
        if let Some(laid) = problem.laid_source {
            for n in &names {
                let Some(physical_key) = selected_key_for_net(laid, problem.source_menus, n)?
                else {
                    continue;
                };
                let Some(super::joint_escape::OptionMove::Escape { movement }) =
                    laid.chosen.get(physical_key)
                else {
                    continue;
                };
                // `laid_source` is a proposal, not proof that copper exists.
                // Read the native board just as source `current_tooth` does;
                // absent endpoints (including bare pads) stay absent.
                let menu_pad = &problem.source_menus.items[physical_key];
                let Some(pad) = ctx.pads.iter().find(|pad| {
                    pad.component_ref == ctx.source
                        && pad.net_id == ctx.byname[n]
                        && pad.pad_number == menu_pad.pad_number
                }) else {
                    continue;
                };
                let Some(piece) = laid.pieces.get(physical_key) else {
                    return Err(format!(
                        "whole_ends laid source lacks physical piece for {physical_key}"
                    ));
                };
                if let Some(current) = source_current_tooth(
                    problem.snapshot,
                    &problem.source_input.routing_layers,
                    physical_key,
                    movement,
                    piece,
                )? {
                    cur.insert(
                        n.clone(),
                        transform_move(current, flow, &problem.snapshot.layers)?,
                    );
                }
            }
        }
        let far = super::escape_moves::Direction::ALL
            .into_iter()
            .min_by(|a, b| {
                let (ax, ay) = a.delta();
                let (bx, by) = b.delta();
                (ax as f64 * (cd[0] - cs[0]) + ay as f64 * (cd[1] - cs[1]))
                    .total_cmp(&(bx as f64 * (cd[0] - cs[0]) + by as f64 * (cd[1] - cs[1])))
            })
            .unwrap();
        let dup = dup_tolerance(problem.rules);
        let mut t = vec![];
        let mut b = vec![];
        for (lane, lg) in &lanes {
            let held = !source_free || problem.free_teeth.is_some_and(|free| !free.contains(lane));
            let current: Vec<_> = lg
                .iter()
                .map(|n| cur.get(n).cloned().into_iter().collect())
                .collect();
            let reach = 1.3 * sg.pitch_x.max(sg.pitch_y);
            let mut to;
            if held && lg.iter().all(|n| cur.contains_key(n)) {
                to = combinations(&current, reach, true, dup)?;
                for o in &mut to {
                    o.refused |= o.moves[0].direction == far
                }
            } else {
                let mut menus = vec![];
                for n in lg {
                    let current = cur.get(n);
                    let mut ms: Vec<_> = current.cloned().into_iter().collect();
                    for key in problem.source_menus.physical_keys_for_net(n)? {
                        for m in problem.source_menus.menus.get(key).into_iter().flatten() {
                            // Joint escape's menu includes straight moves on
                            // degenerate one-column grids. The source whole-end
                            // menu enables straight only under PLAN_JUDGE=ends;
                            // a zero-length surface stub is never a laid tooth.
                            if m.kind == MoveKind::Surface
                                && m.legs
                                    .iter()
                                    .all(|leg| corridor::distance(leg.a, leg.b) <= 1e-9)
                            {
                                continue;
                            }
                            let m = transform_move(m.clone(), flow, &problem.snapshot.layers)?;
                            if current.is_some_and(|c| {
                                c.kind == m.kind
                                    && c.direction == m.direction
                                    && c.layer == m.layer
                                    && corridor::distance(c.exit_pt, m.exit_pt) <= dup
                            }) {
                                continue;
                            }
                            ms.push(m)
                        }
                    }
                    ms.retain(|m| m.direction != far);
                    menus.push(ms)
                }
                to = combinations(&menus, reach, false, dup)?;
                if to.is_empty() {
                    to = if menus.iter().all(|m| !m.is_empty()) {
                        combinations(&menus, reach, true, dup)?
                    } else {
                        vec![]
                    };
                    if to.is_empty() && current.iter().all(|m| !m.is_empty()) {
                        to = combinations(&current, reach, true, dup)?;
                        for o in &mut to {
                            o.refused = true
                        }
                    }
                }
            }
            let mut dm = vec![];
            for n in lg {
                let keys = problem.destination_menus.physical_keys_for_net(n)?;
                let native: Vec<_> = keys
                    .iter()
                    .flat_map(|key| {
                        problem
                            .destination_menus
                            .menus
                            .get(*key)
                            .into_iter()
                            .flatten()
                    })
                    .collect();
                if native.is_empty() {
                    return Err(format!("whole_ends missing berth menu for {n}"));
                }
                let filtered: Vec<_> = native
                    .iter()
                    .copied()
                    .filter(|m| {
                        problem
                            .fixed
                            .get(n)
                            .is_none_or(|sig| move_signature(m).is_ok_and(|s| &s == sig))
                    })
                    .collect();
                let ms = if filtered.is_empty() {
                    native
                } else {
                    filtered
                };
                dm.push(
                    ms.into_iter()
                        .map(|m| transform_move(m.clone(), flow, &problem.snapshot.layers))
                        .collect::<Result<Vec<_>, _>>()?,
                )
            }
            let forced = lg.iter().all(|n| problem.fixed.contains_key(n));
            let mut bo = combinations(&dm, 1.3 * dg.pitch_x.max(dg.pitch_y), forced, dup)?;
            if bo.is_empty() {
                bo = combinations(&dm, 1.3 * dg.pitch_x.max(dg.pitch_y), true, dup)?
            }
            if lg.len() == 2 {
                for o in to.iter_mut().chain(&mut bo) {
                    let a = o.moves[0].exit_pt;
                    let b = o.moves[1].exit_pt;
                    o.refused |= ctx
                        .base_segments
                        .iter()
                        .filter(|s| !run_ids.contains(&s.net_id) && s.layer == o.layer)
                        .any(|s| {
                            pairs::segment_distance(a, b, s.a, s.b)
                                < problem.rules.track_width / 2.
                                    + problem.rules.clearance
                                    + s.width / 2.
                        })
                        || ctx
                            .base_vias
                            .iter()
                            .filter(|v| !run_ids.contains(&v.net_id))
                            .any(|v| {
                                corridor::point_segment_distance(v.point, a, b)
                                    < problem.rules.track_width / 2.
                                        + problem.rules.clearance
                                        + v.diameter / 2.
                            });
                }
            }
            if to.is_empty() || bo.is_empty() {
                return Err(format!("whole_ends no tooth/berth option for {lane}"));
            }
            t.push(to);
            b.push(bo);
        }
        let mut oconf: BTreeMap<_, BTreeSet<_>> = BTreeMap::new();
        let stack = problem.rules.track_width + problem.rules.clearance + 0.005;
        for end in 0..2 {
            let menus = if end == 0 { &t } else { &b };
            for a in 0..lanes.len() {
                for i in 0..menus[a].len() {
                    oconf.entry((a, end, i)).or_default();
                    for other in a + 1..lanes.len() {
                        for j in 0..menus[other].len() {
                            let hit = menus[a][i].moves.iter().any(|m| {
                                menus[other][j]
                                    .moves
                                    .iter()
                                    .any(|o| moves_conflict(m, o, true, stack).unwrap_or(true))
                            });
                            if hit {
                                oconf.entry((a, end, i)).or_default().insert((other, j));
                                oconf.entry((other, end, j)).or_default().insert((a, i));
                            }
                        }
                    }
                }
            }
        }
        for (a, sa) in &problem.learned.iter().cloned().collect::<Vec<_>>() {
            for i in 0..lanes.len() {
                for oi in 0..b[i].len() {
                    if !b[i][oi]
                        .moves
                        .iter()
                        .any(|m| move_signature(m).is_ok_and(|s| &s == a))
                    {
                        continue;
                    }
                    for j in 0..lanes.len() {
                        if i == j {
                            continue;
                        }
                        for oj in 0..b[j].len() {
                            if b[j][oj]
                                .moves
                                .iter()
                                .any(|m| move_signature(m).is_ok_and(|s| &s == sa))
                            {
                                oconf.entry((i, 1, oi)).or_default().insert((j, oj));
                                oconf.entry((j, 1, oj)).or_default().insert((i, oi));
                            }
                        }
                    }
                }
            }
        }
        let mut front = BTreeMap::new();
        for (i, (_, lg)) in lanes.iter().enumerate() {
            if lg.len() != 1 {
                continue;
            }
            for end in 0..2 {
                for (index, o) in (if end == 0 { &t[i] } else { &b[i] }).iter().enumerate() {
                    let f = super::ends_geometry::front(
                        &ctx,
                        ctx.byname[&lg[0]],
                        &o.moves[0],
                        &run_ids,
                    )?;
                    if f.0 > 0 {
                        front.insert((i, end, index), f);
                    }
                }
            }
        }
        let mut tie = BTreeMap::new();
        for (_, lg) in &lanes {
            for n in lg {
                let pad = ctx
                    .pads
                    .iter()
                    .find(|p| p.component_ref == ctx.destination && p.net_id == ctx.byname[n])
                    .ok_or("whole_ends missing destination tie pad")?;
                tie.insert(
                    n.clone(),
                    i64::from(
                        ctx.pads
                            .iter()
                            .filter(|q| q.net_id == pad.net_id)
                            .any(|q| pairs::under_pad(pad, q, problem.rules.via_size)),
                    ),
                );
            }
        }
        let sbox = sg.bbox;
        let dbox = dg.bbox;
        let mut outside: Vec<_> = b
            .iter()
            .flatten()
            .map(|o| {
                (dbox[0] - o.point[0])
                    .max(o.point[0] - dbox[2])
                    .max(dbox[1] - o.point[1])
                    .max(o.point[1] - dbox[3])
            })
            .filter(|v| *v > 0.)
            .collect();
        outside.sort_by(f64::total_cmp);
        let ring_d0 =
            outside.get(outside.len() / 2).copied().unwrap_or(0.) + problem.rules.lane_pitch;
        let mut fb_avoid: BTreeMap<(usize, usize), BTreeMap<usize, f64>> = BTreeMap::new();
        let mut fb_pairs = vec![];
        let index_of = |name: &str| lanes.iter().position(|(n, _)| n == name);
        for item in &problem.feedback.avoid {
            if item.end > 1 {
                return Err("whole_ends invalid feedback end".into());
            }
            if let Some(l) = index_of(&item.lane) {
                let mut item = item.clone();
                if !problem.feedback.points_in_flow_frame {
                    for p in &mut item.points {
                        *p = flow.to_frame(*p)
                    }
                    item.layer = flow.to_frame_layer(&item.layer, &problem.snapshot.layers)?;
                }
                let weights = feedback_weights(
                    if item.end == 0 { &t[l] } else { &b[l] },
                    &item,
                    ctx.layers.len(),
                    dup,
                )?;
                let target = fb_avoid.entry((l, item.end)).or_default();
                for (i, w) in weights {
                    *target.entry(i).or_default() += w
                }
            }
        }
        for [a, c] in &problem.feedback.pairs {
            if a.end != c.end || a.end > 1 {
                continue;
            }
            if let (Some(la), Some(lb)) = (index_of(&a.lane), index_of(&c.lane)) {
                let mut a = a.clone();
                let mut c = c.clone();
                if !problem.feedback.points_in_flow_frame {
                    for item in [&mut a, &mut c] {
                        for p in &mut item.points {
                            *p = flow.to_frame(*p)
                        }
                        item.layer = flow.to_frame_layer(&item.layer, &problem.snapshot.layers)?
                    }
                }
                let wa = feedback_weights(
                    if a.end == 0 { &t[la] } else { &b[la] },
                    &a,
                    ctx.layers.len(),
                    dup,
                )?;
                let wb = feedback_weights(
                    if c.end == 0 { &t[lb] } else { &b[lb] },
                    &c,
                    ctx.layers.len(),
                    dup,
                )?;
                if !wa.is_empty() && !wb.is_empty() {
                    fb_pairs.push((la, lb, a.end, wa, wb));
                }
            }
        }
        let tm = lanes
            .iter()
            .enumerate()
            .map(|(i, (n, _))| (n.clone(), t[i].clone()))
            .collect();
        let bm = lanes
            .iter()
            .enumerate()
            .map(|(i, (n, _))| (n.clone(), b[i].clone()))
            .collect();
        let pw = super::ends_geometry::plane_ways(problem, &ctx, flow, &lanes, &tm, &bm, &cur)?;
        Ok(Self {
            problem,
            ctx,
            flow,
            lanes,
            t,
            b,
            cur,
            sbox,
            dbox,
            ring_d0,
            run_ids,
            oconf,
            front,
            tie,
            fb_avoid,
            fb_pairs,
            pw,
            pool: BTreeMap::new(),
            memo: BTreeMap::new(),
            xroute: BTreeMap::new(),
            wcut: None,
            tabu: BTreeSet::new(),
        })
    }
}

impl<'a> Ends<'a> {
    fn option(&self, state: &State, lane: usize, end: usize) -> &EndOption {
        if end == 0 {
            &self.t[lane][state[lane][0]]
        } else {
            &self.b[lane][state[lane][1]]
        }
    }
    fn hits(&self, state: &State, lane: usize, end: usize, index: usize) -> BTreeSet<usize> {
        self.oconf
            .get(&(lane, end, index))
            .into_iter()
            .flatten()
            .filter(|(o, j)| state[*o][end] == *j)
            .map(|(o, _)| *o)
            .collect()
    }
    fn end_domain(&self, o: &EndOption, end: usize, legs: &[String]) -> Result<EndDomain, String> {
        let own = self
            .ctx
            .layers
            .iter()
            .position(|l| l == &o.layer)
            .ok_or("end layer outside route stack")?;
        if !via_end(o) {
            return Ok(EndDomain {
                fixed: Some(own),
                neck: None,
                allowed: BTreeSet::new(),
            });
        }
        let mut necks = BTreeSet::new();
        for leg in legs {
            let component = if end == 0 {
                &self.ctx.source
            } else {
                &self.ctx.destination
            };
            let pad = self
                .ctx
                .pads
                .iter()
                .find(|p| &p.component_ref == component && p.net_id == self.ctx.byname[leg])
                .ok_or("missing end-domain ball")?;
            let copper: Vec<_> = self
                .ctx
                .layers
                .iter()
                .enumerate()
                .filter(|(_, l)| pad.layers.contains(l))
                .map(|(i, _)| i)
                .collect();
            necks.insert(if copper.len() == 1 {
                Some(copper[0])
            } else {
                None
            });
        }
        let neck = if necks.len() == 1 {
            *necks.iter().next().unwrap()
        } else {
            None
        };
        let mut allowed = BTreeSet::new();
        for (q, layer) in self.ctx.layers.iter().enumerate() {
            let mut fine = true;
            for (leg, m) in legs.iter().zip(&o.moves) {
                if layer == &m.layer {
                    continue;
                }
                let obstacle = super::taut_paths::build_obstacles(
                    &self.ctx,
                    self.ctx.byname[leg],
                    &self.run_ids,
                    layer,
                    None,
                )?;
                if m.legs
                    .iter()
                    .filter(|s| s.layer == m.layer)
                    .any(|s| !obstacle.seg_clear(s.a, s.b))
                {
                    fine = false;
                    break;
                }
            }
            if fine {
                allowed.insert(q);
            }
        }
        Ok(EndDomain {
            fixed: None,
            neck,
            allowed,
        })
    }
    fn runs_meet(&self, a: &EndOption, b: &EndOption, layer: Option<&str>) -> bool {
        let bar = self.problem.rules.track_width + self.problem.rules.clearance - 0.01;
        a.moves
            .iter()
            .flat_map(|m| m.legs.iter().filter(|s| s.layer == m.layer))
            .any(|s| {
                b.moves.iter().any(|m| {
                    m.legs
                        .iter()
                        .filter(|t| t.layer == layer.unwrap_or(&m.layer))
                        .any(|t| pairs::segment_distance(s.a, s.b, t.a, t.b) < bar)
                })
            })
    }
    fn domains(
        &self,
        state: &State,
    ) -> Result<(Vec<[EndDomain; 2]>, Vec<((usize, usize), (usize, usize))>), String> {
        let mut ends = vec![];
        for (l, (_, legs)) in self.lanes.iter().enumerate() {
            ends.push([
                self.end_domain(self.option(state, l, 0), 0, legs)?,
                self.end_domain(self.option(state, l, 1), 1, legs)?,
            ]);
        }
        let original = ends.clone();
        let mut separation = vec![];
        let free: Vec<_> = (0..ends.len())
            .flat_map(|l| (0..2).map(move |e| (l, e)))
            .filter(|(l, e)| ends[*l][*e].fixed.is_none())
            .collect();
        for &(l, e) in &free {
            let own = self
                .ctx
                .layers
                .iter()
                .position(|q| q == &self.option(state, l, e).layer);
            let mut lose = BTreeSet::new();
            for o in 0..ends.len() {
                if o == l {
                    continue;
                }
                if let Some(q) = original[o][e].fixed {
                    if Some(q) != own
                        && original[l][e].allowed.contains(&q)
                        && self.runs_meet(
                            self.option(state, l, e),
                            self.option(state, o, e),
                            Some(&self.ctx.layers[q]),
                        )
                    {
                        lose.insert(q);
                    }
                }
            }
            ends[l][e].allowed.retain(|q| !lose.contains(q));
        }
        for (i, &(l, e)) in free.iter().enumerate() {
            for &(o, k) in &free[i + 1..] {
                if l != o
                    && e == k
                    && self.option(state, l, e).layer != self.option(state, o, k).layer
                    && self.runs_meet(self.option(state, l, e), self.option(state, o, k), None)
                {
                    separation.push(((l, e), (o, k)));
                }
            }
        }
        Ok((ends, separation))
    }
    fn score(&mut self, state: &State, exact: bool) -> Result<(f64, EndsModel), String> {
        let cut_key = serde_json::to_string(&self.wcut).map_err(|e| e.to_string())?;
        let key = (state.clone(), exact, cut_key);
        if let Some(v) = self.memo.get(&key) {
            return Ok(v.clone());
        }
        let got = if let Some(cut) = self.wcut {
            self.score_at(state, exact, Some(cut))?
        } else {
            let pts: Vec<_> = (0..self.lanes.len())
                .map(|l| self.option(state, l, 1).point)
                .collect();
            let db = whole_frame::grown(self.dbox, &pts);
            let ys: Vec<_> = pts
                .iter()
                .filter(|p| whole_frame::face(**p, db) == whole_frame::Face::E)
                .map(|p| p[1])
                .collect();
            let cuts = whole_frame::cuts_tied(&ys, db[1], db[3], self.problem.rules.lane_pitch);
            let mut best = None;
            for c in cuts {
                let got = self.score_at(
                    state,
                    exact,
                    Some(PerimeterCut {
                        face: whole_frame::Face::E,
                        coordinate: c,
                    }),
                )?;
                if best.as_ref().is_none_or(|(v, _)| got.0 < *v - 1e-9) {
                    best = Some(got);
                }
            }
            best.ok_or("whole ends has no destination cut")?
        };
        self.memo.insert(key, got.clone());
        Ok(got)
    }
    fn score_at(
        &mut self,
        state: &State,
        exact: bool,
        cut_at: Option<PerimeterCut>,
    ) -> Result<(f64, EndsModel), String> {
        use whole_frame::Face;
        let n = self.lanes.len();
        if state.len() != n
            || (0..n).any(|i| state[i][0] >= self.t[i].len() || state[i][1] >= self.b[i].len())
        {
            return Err("invalid whole ends state".into());
        }
        let to: Vec<_> = (0..n).map(|i| self.option(state, i, 0).clone()).collect();
        let bo: Vec<_> = (0..n).map(|i| self.option(state, i, 1).clone()).collect();
        let mult: Vec<i64> = self.lanes.iter().map(|(_, lg)| lg.len() as i64).collect();
        let sb = whole_frame::grown(self.sbox, &to.iter().map(|o| o.point).collect::<Vec<_>>());
        let db = whole_frame::grown(self.dbox, &bo.iter().map(|o| o.point).collect::<Vec<_>>());
        let ys: Vec<_> = bo
            .iter()
            .filter(|o| whole_frame::face(o.point, db) == Face::E)
            .map(|o| o.point[1])
            .collect();
        let cut = cut_at.unwrap_or(PerimeterCut {
            face: Face::E,
            coordinate: whole_frame::cut(&ys, db[1], db[3]),
        });
        let kp: Vec<_> = to
            .iter()
            .enumerate()
            .map(|(i, o)| (whole_frame::perim_s(o.point, sb), i))
            .collect();
        let kd: Vec<_> = bo
            .iter()
            .enumerate()
            .map(|(i, o)| (whole_frame::perim_c(o.point, db, cut), i))
            .collect();
        let inv: Vec<_> = (0..n)
            .flat_map(|a| (a + 1..n).map(move |b| (a, b)))
            .filter(|(a, b)| (kp[*a] < kp[*b]) != (kd[*a] < kd[*b]))
            .collect();
        let flat: Vec<_> = (0..n).map(|i| to[i].layer == bo[i].layer).collect();
        let opposite: Vec<_> = (0..n)
            .map(|l| {
                if mult[l] != 2 {
                    return false;
                }
                let a = pairs::hand(
                    to[l].moves[0].direction,
                    to[l].moves[0].exit_pt,
                    to[l].moves[1].exit_pt,
                    false,
                );
                let b = pairs::hand(
                    bo[l].moves[0].direction,
                    bo[l].moves[0].exit_pt,
                    bo[l].moves[1].exit_pt,
                    true,
                );
                a != 0 && b != 0 && a != b
            })
            .collect();
        let stub: Vec<i64> = (0..n)
            .map(|l| {
                to[l]
                    .moves
                    .iter()
                    .zip(&bo[l].moves)
                    .map(|(a, b)| i64::from(a.vias) + i64::from(b.vias))
                    .max()
                    .unwrap_or(0)
            })
            .collect();
        let mut model = EndsModel {
            cut: Some(cut),
            crossings: inv.len(),
            ..EndsModel::default()
        };
        model.fan = (0..n)
            .map(|l| {
                to[l].vias
                    + bo[l].vias
                    + self.lanes[l]
                        .1
                        .iter()
                        .map(|g| self.tie.get(g).copied().unwrap_or(0))
                        .sum::<i64>()
            })
            .sum();
        let mut conflict_lanes = BTreeSet::new();
        for l in 0..n {
            for e in 0..2 {
                for o in self.hits(state, l, e, state[l][e]) {
                    model.conflicts += 1;
                    conflict_lanes.insert(self.lanes[l].0.clone());
                    conflict_lanes.insert(self.lanes[o].0.clone());
                }
            }
        }
        model.conflicts /= 2;
        model.conflict_lanes = conflict_lanes.into_iter().collect();
        model.refused = (0..n).filter(|l| to[*l].refused || bo[*l].refused).count();
        for l in 0..n {
            if mult[l] < 2 {
                continue;
            }
            for e in 0..2 {
                let opts = if e == 0 { &to } else { &bo };
                let box_ = if e == 0 { sb } else { db };
                let per = |p| {
                    if e == 0 {
                        whole_frame::perim_s(p, sb)
                    } else {
                        whole_frame::perim_c(p, db, cut)
                    }
                };
                let mut tips: Vec<_> = opts[l].moves.iter().map(|m| per(m.exit_pt)).collect();
                tips.sort_by(f64::total_cmp);
                let whole = 2. * (box_[2] - box_[0] + box_[3] - box_[1]);
                for o in 0..n {
                    if o == l
                        || opts[o].layer != opts[l].layer
                        || self.ctx.layers.len() > 2 && (via_end(&opts[o]) || via_end(&opts[l]))
                    {
                        continue;
                    }
                    if opts[o].moves.iter().any(|m| {
                        whole_frame::between(tips[0], *tips.last().unwrap(), per(m.exit_pt), whole)
                    }) {
                        model.splits += 1;
                    }
                }
            }
        }
        let mut changes = vec![0_i64; n];
        let mut dropped = vec![0_i64; n];
        let mut xo = vec![0_i64; n];
        let mut sv = stub.clone();
        let (domains, separation) = if self.ctx.layers.len() > 2 {
            self.domains(state)?
        } else {
            (
                (0..n)
                    .map(|l| {
                        [
                            EndDomain {
                                fixed: self.ctx.layers.iter().position(|q| q == &to[l].layer),
                                neck: None,
                                allowed: BTreeSet::new(),
                            },
                            EndDomain {
                                fixed: self.ctx.layers.iter().position(|q| q == &bo[l].layer),
                                neck: None,
                                allowed: BTreeSet::new(),
                            },
                        ]
                    })
                    .collect(),
                vec![],
            )
        };
        if self.ctx.layers.len() > 2 {
            let mut nb = vec![BTreeSet::new(); n];
            for &(a, b) in &inv {
                nb[a].insert(b);
                nb[b].insert(a);
            }
            let mut order: Vec<_> = (0..n).collect();
            order.sort_by_key(|l| (std::cmp::Reverse(nb[*l].len()), *l));
            let mut col = vec![None; n];
            for l in order {
                let used: BTreeSet<_> = nb[l].iter().filter_map(|o| col[*o]).collect();
                let free: Vec<_> = (0..self.ctx.layers.len())
                    .filter(|q| !used.contains(q))
                    .collect();
                let price = |c| {
                    let mut ch = 0_i64;
                    let mut dr = 0_i64;
                    for end in &domains[l] {
                        if let Some(f) = end.fixed {
                            ch += i64::from(c != f);
                        } else {
                            ch += i64::from(!end.allowed.contains(&c));
                            dr += i64::from(end.neck == Some(c) && end.allowed.contains(&c));
                        }
                    }
                    (ch, dr)
                };
                let candidates = if free.is_empty() {
                    (0..self.ctx.layers.len()).collect()
                } else {
                    free.clone()
                };
                let pick = *candidates
                    .iter()
                    .min_by_key(|c| {
                        let (ch, dr) = price(**c);
                        (ch - dr, **c)
                    })
                    .unwrap();
                let (ch, dr) = price(pick);
                col[l] = if free.is_empty() { None } else { Some(pick) };
                changes[l] = ch + if free.is_empty() { 2 } else { 0 };
                dropped[l] = dr;
            }
            model.same = inv
                .iter()
                .filter(|(a, b)| col[*a].is_some() && col[*a] == col[*b])
                .count();
            for l in 0..n {
                if opposite[l] && changes[l] == 0 {
                    xo[l] = if domains[l].iter().all(|e| e.fixed.is_some()) {
                        2
                    } else {
                        1
                    };
                    changes[l] += xo[l];
                }
                sv[l] = (stub[l] - dropped[l]).max(0);
                model.fan -= mult[l] * dropped[l];
                if domains[l][0].fixed.is_some()
                    && domains[l][1].fixed.is_some()
                    && domains[l][0].fixed != domains[l][1].fixed
                {
                    model.parity += mult[l];
                }
            }
        } else {
            for l in 0..n {
                xo[l] = if opposite[l] && flat[l] { 2 } else { 0 };
                changes[l] = i64::from(!flat[l]) + xo[l];
                model.parity += mult[l] * i64::from(!flat[l]);
            }
            model.same = inv
                .iter()
                .filter(|(a, b)| {
                    flat[*a]
                        && flat[*b]
                        && to[*a].layer == to[*b].layer
                        && xo[*a] == 0
                        && xo[*b] == 0
                })
                .count();
            let ov = |l: usize, k: i64| (stub[l] + changes[l] + k - 2).max(0);
            let weights: Vec<_> = (0..n)
                .map(|l| 2. * mult[l] as f64 + W_OVER * (ov(l, 2) - ov(l, 0)) as f64)
                .collect();
            let cov = cover(
                &(0..n)
                    .filter(|l| flat[*l] && xo[*l] == 0)
                    .collect::<Vec<_>>(),
                &to.iter().map(|o| o.layer.clone()).collect::<Vec<_>>(),
                &kp,
                &kd,
                &weights,
            );
            for l in cov {
                changes[l] += 2;
                model.cover += mult[l];
            }
            let mut launch: Vec<_> = (0..n).collect();
            launch.sort_by(|a, b| kp[*a].partial_cmp(&kp[*b]).unwrap());
            let mut finish: Vec<_> = (0..n).collect();
            finish.sort_by(|a, b| kd[*a].partial_cmp(&kd[*b]).unwrap());
            let mut rl = vec![0_i64; n];
            let mut rf = vec![0_i64; n];
            for (i, l) in launch.iter().enumerate() {
                rl[*l] = i as i64;
            }
            for (i, l) in finish.iter().enumerate() {
                rf[*l] = i as i64;
            }
            let mut xs = vec![BTreeSet::new(); n];
            let mut cset = BTreeSet::new();
            for &(a, b) in &inv {
                xs[a].insert(b);
                xs[b].insert(a);
                cset.insert((a.min(b), a.max(b)));
            }
            let crosses = |a: usize, b: usize| cset.contains(&(a.min(b), a.max(b)));
            let tcr = |a: usize, b: usize| {
                (rl[b] - rl[a]) as f64 / ((rl[b] - rl[a]) - (rf[b] - rf[a])) as f64
            };
            let ovk = |l: usize, k: i64| (stub[l] + k - 2).max(0);
            for &l in &launch {
                if flat[l] {
                    continue;
                }
                let mut partners: Vec<_> = xs[l]
                    .iter()
                    .copied()
                    .filter(|m| flat[*m] && changes[*m] == 0)
                    .collect();
                partners.sort_by(|a, b| tcr(l, *a).total_cmp(&tcr(l, *b)));
                let mut bad = vec![];
                for (i, &a) in partners.iter().enumerate() {
                    for &b in &partners[i + 1..] {
                        if to[a].layer != bo[l].layer
                            && to[b].layer != to[l].layer
                            && !crosses(a, b)
                        {
                            bad.push((a, b));
                        }
                    }
                }
                if bad.is_empty() {
                    continue;
                }
                let wt: BTreeMap<_, _> = bad
                    .iter()
                    .flat_map(|(a, b)| [*a, *b])
                    .map(|m| {
                        (
                            m,
                            W_OVER * (ovk(m, 2) - ovk(m, 0)) as f64 + 2. * mult[m] as f64,
                        )
                    })
                    .collect();
                let div = min_cut_cover(&bad, &wt)?;
                let cost = W_OVER * (ovk(l, changes[l] + 2) - ovk(l, changes[l])) as f64
                    + 2. * mult[l] as f64;
                if cost <= div.iter().map(|m| wt[m]).sum() {
                    changes[l] += 2;
                    model.settle += 2 * mult[l];
                } else {
                    for m in div {
                        changes[m] = 2;
                        model.settle += 2 * mult[m];
                    }
                }
            }
            let tie = |a: usize, b: usize| {
                (kp[a].0 - kp[b].0).abs() < 1e-3 || (kd[a].0 - kd[b].0).abs() < 1e-3
            };
            let mut conflicts = vec![];
            for &(a, b) in &inv {
                if !(flat[a]
                    && flat[b]
                    && changes[a] == 0
                    && changes[b] == 0
                    && to[a].layer != to[b].layer)
                    || tie(a, b)
                {
                    continue;
                }
                let mut ns: Vec<_> = xs[a]
                    .intersection(&xs[b])
                    .copied()
                    .filter(|l| !flat[*l] && changes[*l] == 1 && !tie(*l, a) && !tie(*l, b))
                    .collect();
                let mut want = BTreeMap::new();
                for &l in &ns {
                    let first = if to[l].layer != to[a].layer { a } else { b };
                    let pre = if (rl[a] - rl[l]) * (rl[b] - rl[l]) > 0 {
                        if (rl[a] - rl[l]).abs() < (rl[b] - rl[l]).abs() {
                            a
                        } else {
                            b
                        }
                    } else {
                        a
                    };
                    want.insert(l, first == pre);
                }
                ns.sort_by(|x, y| tcr(a, *x).total_cmp(&tcr(a, *y)));
                for (i, &x) in ns.iter().enumerate() {
                    for &y in &ns[i + 1..] {
                        if !crosses(x, y) && !tie(x, y) && !want[&x] && want[&y] {
                            conflicts.push([a, b, x, y]);
                        }
                    }
                }
            }
            while !conflicts.is_empty() {
                let cand: BTreeSet<_> = conflicts.iter().flatten().copied().collect();
                let cost = |x: usize| {
                    let c = if flat[x] { 0 } else { changes[x] };
                    (W_OVER * (ovk(x, c + 2) - ovk(x, c)) as f64 + 2. * mult[x] as f64)
                        / conflicts.iter().filter(|c| c.contains(&x)).count() as f64
                };
                let x = *cand
                    .iter()
                    .min_by(|a, b| cost(**a).total_cmp(&cost(**b)).then(a.cmp(b)))
                    .unwrap();
                changes[x] = if flat[x] { 2 } else { changes[x] + 2 };
                model.couple += 2 * mult[x];
                conflicts.retain(|c| !c.contains(&x));
            }
        }
        model.crossover = (0..n).map(|l| mult[l] * xo[l]).sum();
        model.route = (0..n).map(|l| mult[l] * changes[l]).sum();
        model.route_est = model.route;
        model.over = (0..n).map(|l| (sv[l] + changes[l] - 2).max(0)).sum();
        let dcls: Vec<_> = bo
            .iter()
            .map(|o| {
                let face = whole_frame::face(o.point, db);
                if cut.face == Face::E {
                    if face == Face::E {
                        if o.point[1] < cut.coordinate {
                            Face::N
                        } else {
                            Face::S
                        }
                    } else {
                        face
                    }
                } else {
                    whole_frame::ring_side(o.point, db, cut).unwrap_or(Face::W)
                }
            })
            .collect();
        let pitch = self.problem.rules.lane_pitch;
        let gap = (db[0] - sb[2]).max(pitch);
        let mut trunk = vec![0; n];
        let mut lane_x = vec![0; n];
        for &(a, b) in &inv {
            lane_x[a] += 1;
            lane_x[b] += 1;
            if !(dcls[a] == dcls[b] && dcls[a] != Face::W) {
                trunk[a] += 1;
                trunk[b] += 1;
            }
        }
        model.congestion = 0.2 * trunk.iter().sum::<usize>() as f64 / 2.;
        for l in 0..n {
            let load = (trunk[l] as f64 * pitch / (self.ctx.layers.len() - 1) as f64
                + if dcls[l] == Face::W {
                    changes[l] as f64 * 2. * 1.1 * self.problem.rules.via_need()
                } else {
                    0.
                })
                / gap;
            model.congestion += mult[l] as f64 * 20. * (load - 0.5).max(0.).powi(2);
            model.lane_load.insert(self.lanes[l].0.clone(), load);
            if lane_x[l] > 0 {
                model.lane_x.insert(self.lanes[l].0.clone(), lane_x[l]);
            }
        }
        if exact {
            let key = (
                state.clone(),
                serde_json::to_string(&cut).map_err(|e| e.to_string())?,
            );
            let xr = if let Some(x) = self.xroute.get(&key) {
                x.clone()
            } else {
                let x = exact_route(
                    &kp,
                    &kd,
                    &self.ctx.layers,
                    &domains,
                    &sv,
                    &opposite,
                    &mult,
                    &separation,
                    self.problem.deadline,
                )?;
                self.xroute.insert(key, x.clone());
                x
            };
            if let Some(x) = xr {
                model.route = x.route;
                model.over = x.over;
                changes = x.changes;
                if self.ctx.layers.len() > 2 {
                    model.fan += (0..n)
                        .map(|l| mult[l] * (dropped[l] - x.dropped[l]))
                        .sum::<i64>();
                }
            } else {
                model.exact_failed = true;
            }
        }
        if !self.problem.feedback.over.is_empty() {
            model.over = (0..n)
                .map(|l| {
                    (sv[l] + changes[l] - 2).max(0).max(
                        self.problem
                            .feedback
                            .over
                            .get(&self.lanes[l].0)
                            .copied()
                            .unwrap_or(0),
                    )
                })
                .sum();
        }
        let far = PerimeterCut {
            face: Face::E,
            coordinate: whole_frame::cut(&ys, db[1], db[3]),
        };
        let wound: Vec<_> = (0..n)
            .map(|l| {
                if self.wcut.is_none() {
                    None
                } else {
                    whole_frame::ring_side(bo[l].point, db, cut)
                        .filter(|s| Some(*s) != whole_frame::ring_side(bo[l].point, db, far))
                }
            })
            .collect();
        for l in 0..n {
            let a = to[l].point;
            let b = bo[l].point;
            let (dest, turns) = if let Some(side) = wound[l] {
                wound_ride(a, b, self.dbox, side == Face::N, self.ring_d0)
            } else {
                (around_box(a, b, self.dbox, 0.3), 0)
            };
            let mut ride = mult[l] as f64
                * (dest + around_box(a, b, self.sbox, 0.3) - corridor::distance(a, b));
            for ((leg, tm), bm) in self.lanes[l].1.iter().zip(&to[l].moves).zip(&bo[l].moves) {
                for (component, m) in [(&self.ctx.source, tm), (&self.ctx.destination, bm)] {
                    let pad = self
                        .ctx
                        .pads
                        .iter()
                        .find(|p| &p.component_ref == component && p.net_id == self.ctx.byname[leg])
                        .ok_or("missing ride ball")?;
                    ride += corridor::distance(pad.point, m.exit_pt);
                }
            }
            if let Some(side) = wound[l] {
                let inside = (0..n)
                    .filter(|o| {
                        *o != l
                            && dcls[*o] == side
                            && if side == Face::N {
                                kd[*o] > kd[l]
                            } else {
                                kd[*o] < kd[l]
                            }
                    })
                    .count();
                ride += mult[l] as f64
                    * turns as f64
                    * std::f64::consts::FRAC_PI_2
                    * (inside as f64 / 2.)
                    * pitch;
            }
            model.ride += ride;
            let over = (sv[l] + changes[l] - 2).max(0).max(
                self.problem
                    .feedback
                    .over
                    .get(&self.lanes[l].0)
                    .copied()
                    .unwrap_or(0),
            );
            if over > 0 {
                model.lane_over.insert(self.lanes[l].0.clone(), over);
            }
            model.changes.insert(self.lanes[l].0.clone(), changes[l]);
        }
        for e in 0..2 {
            let opts = if e == 0 { &to } else { &bo };
            for a in 0..n {
                for b in a + 1..n {
                    if changes[a] == 0 && changes[b] == 0 {
                        continue;
                    }
                    for ma in &opts[a].moves {
                        for mb in &opts[b].moves {
                            if ma.layer != mb.layer
                                && corridor::distance(ma.exit_pt, mb.exit_pt)
                                    <= dup_tolerance(self.problem.rules)
                            {
                                model.stacks += if mult[a] > 1 || mult[b] > 1 { 2 } else { 1 };
                            }
                        }
                    }
                }
            }
        }
        model.feedback = 5.
            * self
                .fb_avoid
                .iter()
                .map(|((l, e), ix)| ix.get(&state[*l][*e]).copied().unwrap_or(0.))
                .sum::<f64>()
            + 10.
                * self
                    .fb_pairs
                    .iter()
                    .map(|(a, b, e, wa, wb)| {
                        wa.get(&state[*a][*e]).copied().unwrap_or(0.)
                            * wb.get(&state[*b][*e]).copied().unwrap_or(0.)
                    })
                    .sum::<f64>();
        for l in 0..n {
            for e in 0..2 {
                if let Some(&(value, span)) = self.front.get(&(l, e, state[l][e])) {
                    model.front_cost += if value == 1 { 1. } else { 1000. };
                    if value == 1 && self.fb_avoid.contains_key(&(l, e)) {
                        if let Some(span) = span {
                            model
                                .front
                                .entry(self.lanes[l].0.clone())
                                .or_default()
                                .insert(e, span);
                        }
                    }
                }
            }
        }
        if let Some(pw) = &self.pw {
            let state = self
                .lanes
                .iter()
                .enumerate()
                .map(|(i, (name, _))| (name.clone(), state[i]))
                .collect();
            model.walled = super::ends_geometry::walled_balls(pw, &self.lanes, &state);
        }
        let value = W_OVER * model.over as f64
            + model.fan as f64
            + model.route as f64
            + model.ride / VIA_MM
            + model.congestion
            + model.feedback
            + EPS_X * inv.len() as f64
            + BIG * (model.conflicts + model.splits + model.refused) as f64
            + 5. * model.stacks as f64
            + model.front_cost
            + 1000. * model.walled.len() as f64;
        model.objective = value;
        Ok((value, model))
    }

    fn start(&self) -> Result<State, String> {
        let mut state = vec![[0usize; 2]; self.lanes.len()];
        for (lane, (name, legs)) in self.lanes.iter().enumerate() {
            let src_plan = self
                .problem
                .laid_source
                .ok_or("whole_ends requires the real laid source plan")?;
            let dst_plan = self
                .problem
                .laid_destination
                .ok_or("whole_ends requires the real laid destination plan")?;
            let source_match = |o: &EndOption| {
                legs.iter().zip(&o.moves).all(|(net, candidate)| {
                matches!(self.cur.get(net), Some(current) if current.kind==candidate.kind
                    && current.direction==candidate.direction && current.layer==candidate.layer
                    && corridor::distance(current.exit_pt,candidate.exit_pt)<=dup_tolerance(self.problem.rules))
            })
            };
            let ti = self.t[lane]
                .iter()
                .position(source_match)
                .unwrap_or_else(|| {
                    self.t[lane]
                        .iter()
                        .enumerate()
                        .min_by_key(|(_, o)| o.vias)
                        .map(|(i, _)| i)
                        .unwrap_or(0)
                });
            let mut wanted = Vec::new();
            for n in legs {
                let planned = if let Some(movement) = self.problem.seed.get(n) {
                    Some(movement)
                } else {
                    match selected_move_for_net(dst_plan, self.problem.destination_menus, n)? {
                        Some(super::joint_escape::OptionMove::Escape { movement }) => {
                            Some(movement)
                        }
                        _ => None,
                    }
                };
                let movement = planned;
                if let Some(movement) = movement {
                    wanted.push(transform_move(
                        movement.clone(),
                        self.flow,
                        &self.problem.snapshot.layers,
                    )?);
                }
            }
            let bi = if wanted.len() == legs.len() {
                self.b[lane]
                    .iter()
                    .enumerate()
                    .min_by(|(_, a), (_, b)| {
                        let cost = |o: &EndOption| {
                            o.moves
                                .iter()
                                .zip(&wanted)
                                .map(|(m, w)| {
                                    corridor::distance(m.exit_pt, w.exit_pt)
                                        + if move_signature(m).ok() == move_signature(w).ok() {
                                            0.
                                        } else {
                                            0.001
                                        }
                                })
                                .sum::<f64>()
                        };
                        cost(a).total_cmp(&cost(b))
                    })
                    .map(|(i, _)| i)
                    .unwrap_or(0)
            } else {
                self.b[lane]
                    .iter()
                    .enumerate()
                    .min_by_key(|(_, o)| o.vias)
                    .map(|(i, _)| i)
                    .unwrap_or(0)
            };
            state[lane] = [ti, bi];
            let _ = (name, src_plan);
        }
        Ok(state)
    }

    fn objective(&mut self, state: &State) -> Result<f64, String> {
        Ok(self.score(state, false)?.0)
    }

    fn search(&mut self, mut state: State, sweeps: usize) -> Result<State, String> {
        if state.len() != self.lanes.len() {
            return Err("invalid whole_ends start state".into());
        }
        let mut best = self.objective(&state)?;
        for _ in 0..sweeps {
            let mut improved = false;
            for lane in 0..self.lanes.len() {
                for end in [1usize, 0usize] {
                    let count = if end == 0 {
                        self.t[lane].len()
                    } else {
                        self.b[lane].len()
                    };
                    for index in 0..count {
                        if index == state[lane][end] || self.tabu.contains(&(lane, end, index)) {
                            continue;
                        }
                        let hits = self.hits(&state, lane, end, index);
                        if hits.len() > 1 {
                            continue;
                        }
                        let mut trial = state.clone();
                        trial[lane][end] = index;
                        if let Some(other) = hits.iter().next().copied() {
                            let n = if end == 0 {
                                self.t[other].len()
                            } else {
                                self.b[other].len()
                            };
                            let mut best_eject = None;
                            for j in 0..n {
                                if self.hits(&trial, other, end, j).is_empty()
                                    && !self.tabu.contains(&(other, end, j))
                                {
                                    let mut candidate = trial.clone();
                                    candidate[other][end] = j;
                                    let value = self.objective(&candidate)?;
                                    if best_eject.as_ref().is_none_or(|(v, _)| value < *v) {
                                        best_eject = Some((value, candidate));
                                    }
                                }
                            }
                            let Some((_, candidate)) = best_eject else {
                                continue;
                            };
                            trial = candidate;
                        }
                        let value = self.objective(&trial)?;
                        if value < best - 1e-9 {
                            state = trial;
                            best = value;
                            improved = true;
                        }
                    }
                }
            }
            let key = state.clone();
            self.pool
                .entry(key)
                .and_modify(|v| *v = v.min(best))
                .or_insert(best);
            if !improved {
                break;
            }
        }
        Ok(state)
    }

    fn best_exact(&mut self, state: State) -> Result<(State, (f64, EndsModel)), String> {
        let mut ranked: Vec<_> = self.pool.iter().map(|(s, v)| (s.clone(), *v)).collect();
        ranked.sort_by(|a, b| a.1.total_cmp(&b.1).then(a.0.cmp(&b.0)));
        let threshold = ranked.first().map(|x| x.1 + 4.).unwrap_or(f64::INFINITY);
        let mut candidates: Vec<_> = ranked
            .into_iter()
            .take(6)
            .filter(|x| x.1 <= threshold)
            .map(|x| x.0)
            .collect();
        if !candidates.contains(&state) {
            candidates.push(state.clone());
        }
        let mut best: Option<(State, (f64, EndsModel))> = None;
        for candidate in candidates {
            let got = self.score(&candidate, true)?;
            if best
                .as_ref()
                .is_none_or(|(_, (v, m))| (got.1.exact_failed, got.0) < (m.exact_failed, *v - 1e-9))
            {
                best = Some((candidate, got));
            }
        }
        best.ok_or_else(|| "whole_ends exact ranking has no candidates".into())
    }

    fn iterate(&mut self, mut state: State) -> Result<State, String> {
        let mut rng = PythonRandom::new(622);
        let mut best = state.clone();
        let mut best_v = self.objective(&best)?;
        let mut idle = 0;
        let moving: Vec<_> = (0..self.lanes.len())
            .filter(|l| self.t[*l].len() > 1 || self.b[*l].len() > 1)
            .collect();
        for _round in 0..12 {
            if idle >= 4 || Instant::now() >= self.problem.deadline {
                break;
            }
            let mut candidate = best.clone();
            let mut order = moving.clone();
            let kicks = rng.sample_indices(order.len(), order.len().min(3));
            for lane in kicks.into_iter().map(|i| order[i]) {
                let end = if self.t[lane].len() > 1 {
                    rng.randbelow(2)
                } else {
                    1
                };
                let count = if end == 0 {
                    self.t[lane].len()
                } else {
                    self.b[lane].len()
                };
                if count > 0 {
                    candidate[lane][end] = rng.randbelow(count);
                }
            }
            state = self.search(candidate, 20)?;
            let value = self.objective(&state)?;
            if value < best_v - 1e-9 {
                best = state.clone();
                best_v = value;
                idle = 0;
            } else {
                idle += 1;
            }
        }
        Ok(best)
    }

    fn ban_kicks(&mut self, state: State) -> Result<State, String> {
        let mut best = state;
        let (_, mut model) = self.score(&best, false)?;
        let mut best_v = model.objective;
        let mut kicked = BTreeSet::new();
        let mut idle = 0;
        for _ in 0..8 {
            if idle >= 4 || Instant::now() >= self.problem.deadline {
                break;
            }
            let mut group: Vec<_> = (0..self.lanes.len())
                .filter(|l| !kicked.contains(l) && (self.t[*l].len() > 1 || self.b[*l].len() > 1))
                .collect();
            group.sort_by(|a, b| {
                model
                    .lane_x
                    .get(&self.lanes[*b].0)
                    .copied()
                    .unwrap_or(0)
                    .cmp(&model.lane_x.get(&self.lanes[*a].0).copied().unwrap_or(0))
                    .then(self.lanes[*a].0.cmp(&self.lanes[*b].0))
            });
            group.truncate(3);
            if group.is_empty() {
                break;
            }
            kicked.extend(group.iter().copied());
            let mut candidate = best.clone();
            self.tabu.clear();
            for lane in &group {
                for end in 0..2 {
                    let count = if end == 0 {
                        self.t[*lane].len()
                    } else {
                        self.b[*lane].len()
                    };
                    if count <= 1 {
                        continue;
                    }
                    let current = candidate[*lane][end];
                    self.tabu.insert((*lane, end, current));
                    let mut picks: Vec<_> = (0..count).filter(|i| *i != current).collect();
                    picks.sort_by_key(|i| {
                        let mut trial = candidate.clone();
                        trial[*lane][end] = *i;
                        let hits = self.hits(&trial, *lane, end, *i).len();
                        let o = if end == 0 {
                            &self.t[*lane][*i]
                        } else {
                            &self.b[*lane][*i]
                        };
                        (hits > 0, o.vias, *i)
                    });
                    if let Some(i) = picks.first() {
                        candidate[*lane][end] = *i;
                    }
                }
            }
            candidate = self.search(candidate, 20)?;
            self.tabu.clear();
            candidate = self.search(candidate, 20)?;
            let (value, parts) = self.score(&candidate, false)?;
            if value < best_v - 1e-9 {
                best = candidate;
                best_v = value;
                model = parts;
                idle = 0;
            } else {
                idle += 1;
            }
        }
        self.tabu.clear();
        Ok(best)
    }
}

/// Jointly choose source teeth and destination berths from real joint-router plans.
pub fn choose(problem: &EndsProblem<'_>) -> Result<EndsChoice, String> {
    let laid_source = problem
        .laid_source
        .ok_or("whole_ends requires a real laid source plan")?;
    let laid_destination = problem
        .laid_destination
        .ok_or("whole_ends requires a real laid destination plan")?;
    let mut ends = Ends::new(problem, false)?;
    let start = ends.start()?;
    let mut state = ends.search(start, 20)?;
    state = ends.iterate(state)?;
    state = ends.ban_kicks(state)?;
    let (mut state, mut laid_score) = ends.best_exact(state)?;
    (state, laid_score) = cut_phase(problem, &state, laid_score)?;
    let laid_objective = laid_score.0;
    let mut final_source = laid_source.clone();
    let mut final_destination = laid_destination.clone();
    apply_choice(&ends, &state, &mut final_source, &mut final_destination)?;
    let mut objective = laid_objective;
    let mut model = laid_score.1;
    if problem.source_free {
        let mut free = Ends::new(problem, true)?;
        let initial = free.start()?;
        state = free.search(initial, 20)?;
        state = free.iterate(state)?;
        state = free.ban_kicks(state)?;
        let (best, scored) = free.best_exact(state)?;
        let (best, scored) = cut_phase(problem, &best, scored)?;
        if scored.0 < objective - 0.5 {
            apply_choice(&free, &best, &mut final_source, &mut final_destination)?;
            objective = scored.0;
            model = scored.1;
        }
    }
    Ok(EndsChoice {
        source: final_source,
        destination: final_destination,
        model: EndsModel {
            objective,
            laid_objective,
            ..model
        },
    })
}

/// Re-evaluate the real laid plans on exact route orders; callers use this after a materialized route audit.
pub fn judge(problem: &EndsProblem<'_>) -> Result<(f64, EndsModel), String> {
    let mut ends = Ends::new(problem, false)?;
    let state = ends.start()?;
    ends.score(&state, true)
}

/// Try perimeter cuts away from the far face and retain one only when exact scoring improves the held state.
pub fn cut_phase(
    problem: &EndsProblem<'_>,
    state: &State,
    current: (f64, EndsModel),
) -> Result<(State, (f64, EndsModel)), String> {
    use whole_frame::Face;
    let mut ends = Ends::new(problem, false)?;
    let best_state = state.clone();
    let mut best = (best_state, current);
    let pts: Vec<_> = (0..ends.lanes.len())
        .map(|l| ends.option(state, l, 1).point)
        .collect();
    if pts.is_empty() {
        return Ok(best);
    }
    let db = whole_frame::grown(ends.dbox, &pts);
    let avoid: Vec<_> = (0..ends.lanes.len())
        .flat_map(|l| {
            ends.b[l]
                .iter()
                .flat_map(|o| o.moves.iter().map(|m| m.exit_pt))
        })
        .collect();
    let candidates = whole_frame::cut_gaps(&pts, db, &avoid)?;
    for cut in candidates.into_iter().filter(|c| c.face != Face::E).take(3) {
        if Instant::now() >= problem.deadline {
            break;
        }
        ends.wcut = Some(cut);
        ends.pool.clear();
        let trial = ends.search(state.clone(), 20)?;
        let trial = ends.iterate(trial)?;
        let got = ends.best_exact(trial)?;
        let better = got.1 .0 < best.1 .0 - 1.0
            && (got.1 .1.fan + got.1 .1.route < best.1 .1.fan + best.1 .1.route
                || best.1 .1.exact_failed && !got.1 .1.exact_failed);
        if better {
            best = got;
        }
    }
    Ok(best)
}

fn apply_choice(
    ends: &Ends<'_>,
    state: &State,
    source: &mut JointPlan,
    destination: &mut JointPlan,
) -> Result<(), String> {
    for (lane, (name, legs)) in ends.lanes.iter().enumerate() {
        let _ = name;
        for end in 0..2 {
            let option = if end == 0 {
                &ends.t[lane][state[lane][0]]
            } else {
                &ends.b[lane][state[lane][1]]
            };
            let menus = if end == 0 {
                ends.problem.source_menus
            } else {
                ends.problem.destination_menus
            };
            for (net, movement) in legs.iter().zip(&option.moves) {
                if end == 0
                    && matches!(selected_move_for_net(source,menus,net)?,Some(super::joint_escape::OptionMove::Escape{movement:old})
                    if transform_move(old.clone(),ends.flow,&ends.problem.snapshot.layers).is_ok_and(|old|
                        old.kind==movement.kind && old.direction==movement.direction && old.layer==movement.layer
                            && corridor::distance(old.exit_pt,movement.exit_pt)<=dup_tolerance(ends.problem.rules)))
                {
                    continue;
                }
                let mut matches = Vec::new();
                for key in menus.physical_keys_for_net(net)? {
                    for candidate in menus.menus.get(key).into_iter().flatten() {
                        if transform_move(
                            candidate.clone(),
                            ends.flow,
                            &ends.problem.snapshot.layers,
                        )
                        .is_ok_and(|m| move_signature(&m).ok() == move_signature(movement).ok())
                        {
                            matches.push((key, candidate));
                        }
                    }
                }
                if matches.len() > 1 {
                    return Err(format!(
                        "whole_ends selected move is ambiguous across physical pads for {net}"
                    ));
                }
                let (physical_key, native) = matches.pop().ok_or_else(|| {
                    format!("whole_ends selected move has no native menu entry for {net}")
                })?;
                let joint=menus.options.get(physical_key).into_iter().flatten().find(|o|matches!(&o.option,super::joint_escape::OptionMove::Escape{movement:m} if move_signature(m).ok()==move_signature(native).ok()))
                    .ok_or_else(||format!("whole_ends selected move has no real joint piece for {net}"))?;
                let plan = if end == 0 {
                    &mut *source
                } else {
                    &mut *destination
                };
                plan.chosen.insert(
                    physical_key.to_string(),
                    super::joint_escape::OptionMove::Escape {
                        movement: native.clone(),
                    },
                );
                plan.pieces
                    .insert(physical_key.to_string(), joint.piece.clone());
                if let Some(index)=menus.options.get(physical_key).and_then(|opts|opts.iter().position(|o|matches!(&o.option,super::joint_escape::OptionMove::Escape{movement:m} if move_signature(m).ok()==move_signature(native).ok()))){plan.selected_indices.insert(physical_key.to_string(),index);}
            }
        }
    }
    Ok(())
}

fn selected_key_for_net<'a>(
    plan: &'a JointPlan,
    menus: &'a JointMenus,
    net: &str,
) -> Result<Option<&'a str>, String> {
    let mut selected = menus
        .physical_keys_for_net(net)?
        .into_iter()
        .filter(|key| plan.chosen.contains_key(*key));
    let first = selected.next();
    if selected.next().is_some() {
        return Err(format!(
            "whole_ends plan has multiple selected physical pads for {net}"
        ));
    }
    Ok(first)
}

fn selected_move_for_net<'a>(
    plan: &'a JointPlan,
    menus: &'a JointMenus,
    net: &str,
) -> Result<Option<&'a super::joint_escape::OptionMove>, String> {
    Ok(selected_key_for_net(plan, menus, net)?.and_then(|key| plan.chosen.get(key)))
}

/// Source `pages_first.current_tooth` readback for one exact physical source
/// pad. The plan supplies only a proximity hint; the returned tooth, direction,
/// layer, kind, and vias come from copper actually present in this snapshot.
fn source_current_tooth(
    snapshot: &BoardSnapshot,
    routing_layers: &[String],
    physical_key: &str,
    hint: &Move,
    piece: &super::joint_geometry::Piece,
) -> Result<Option<Move>, String> {
    let asked = super::destination_realize::asked_berth_for_piece(
        &snapshot.native,
        physical_key.to_owned(),
        hint.clone(),
        None,
        piece,
        false,
    )?;
    let Some(achieved) =
        super::destination_realize::measure_tooth(&snapshot.native, &asked.pad, None)?
    else {
        return Ok(None);
    };
    if !routing_layers.contains(&achieved.layer) {
        return Ok(None);
    }
    Ok(Some(Move {
        net: super::escape_moves::short_name(&asked.pad.net).to_owned(),
        kind: achieved.kind,
        direction: achieved.direction,
        layer: achieved.layer,
        exit_pt: achieved.tooth,
        vias: u32::try_from(achieved.vias).map_err(|_| "whole_ends current via count overflow")?,
        legs: Vec::new(),
        site: achieved.site,
        climb: 0,
        street: false,
        runs_on: Vec::new(),
    }))
}

fn snap_direction(vector: Point) -> super::escape_moves::Direction {
    let length = vector[0].hypot(vector[1]).max(1e-12);
    super::escape_moves::Direction::ALL
        .into_iter()
        .min_by(|a, b| {
            let score = |direction: super::escape_moves::Direction| {
                let (x, y) = direction.delta();
                (x as f64 - vector[0] / length).powi(2) + (y as f64 - vector[1] / length).powi(2)
            };
            score(*a).total_cmp(&score(*b))
        })
        .unwrap_or(super::escape_moves::Direction::Up)
}
