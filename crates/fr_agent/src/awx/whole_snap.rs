//! Complete octilinear snap port of AWX whole_snap.py at caad45cd252f9a50f1fb4b3b15461bdf1b9457a1.
use super::{
    corridor::{self, Point},
    joint_geometry::JointGeometry,
    numeric, pairs,
    whole_context::{LaneGeometry, LayerPiece, PairEnd, WholeContext},
    whole_frame::WholeFrame,
    whole_geometry::GeometryResult,
};
use std::{
    cmp::Ordering,
    collections::{BTreeMap, BTreeSet, BinaryHeap, HashMap},
    time::{Duration, Instant},
};
const DIRS: [[i64; 2]; 8] = [
    [1, 0],
    [1, 1],
    [0, 1],
    [-1, 1],
    [-1, 0],
    [-1, -1],
    [0, -1],
    [1, -1],
];
#[derive(Clone)]
struct SmoothLane {
    points: Vec<Point>,
    layers: Vec<String>,
    runs: Vec<String>,
    vias: Vec<Point>,
    barrels: Vec<Point>,
}
#[derive(Clone)]
struct Window {
    origin: [i64; 2],
    ni: usize,
    nj: usize,
    band: Vec<bool>,
    distance: Vec<f64>,
    arc: Vec<f64>,
    tolerance: Option<Vec<f64>>,
    bad: BTreeMap<String, Vec<bool>>,
    via_bad: [Vec<bool>; 4],
    soft: Option<BTreeMap<String, Vec<bool>>>,
    cross_room: Option<Vec<bool>>,
}
impl Window {
    fn index(&self, i: usize, j: usize) -> usize {
        i * self.nj + j
    }
    fn valid(&self, i: i64, j: i64) -> bool {
        i >= 0 && j >= 0 && (i as usize) < self.ni && (j as usize) < self.nj
    }
    fn point(&self, i: usize, j: usize, g: f64) -> Point {
        [
            (self.origin[0] + i as i64) as f64 * g,
            (self.origin[1] + j as i64) as f64 * g,
        ]
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct State {
    cell: [usize; 2],
    heading: usize,
    via: usize,
    owed: usize,
    taken: usize,
}
#[derive(Clone, Copy)]
struct QueueItem {
    estimate: f64,
    cost: f64,
    state: u64,
}
impl PartialEq for QueueItem {
    fn eq(&self, b: &Self) -> bool {
        self.estimate == b.estimate && self.cost == b.cost && self.state == b.state
    }
}
impl Eq for QueueItem {}
impl PartialOrd for QueueItem {
    fn partial_cmp(&self, b: &Self) -> Option<Ordering> {
        Some(self.cmp(b))
    }
}
impl Ord for QueueItem {
    fn cmp(&self, b: &Self) -> Ordering {
        b.estimate
            .total_cmp(&self.estimate)
            .then(b.cost.total_cmp(&self.cost))
            .then(b.state.cmp(&self.state))
    }
}
struct Radix {
    ma: u64,
    mk: u64,
    md: u64,
    mj: u64,
    mi: u64,
}
impl Radix {
    fn new(nj: usize, via_count: usize, cap: usize, owed_cap: usize) -> Result<Self, String> {
        let ma = u64::try_from(cap)
            .map_err(|_| "snap count overflow")?
            .checked_add(1)
            .ok_or("snap count overflow")?;
        let c0 = u64::try_from(owed_cap)
            .map_err(|_| "snap count overflow")?
            .checked_add(1)
            .ok_or("snap count overflow")?;
        let mk = c0.checked_mul(ma).ok_or("snap radix overflow")?;
        let md = u64::try_from(via_count)
            .map_err(|_| "snap via count overflow")?
            .checked_add(1)
            .and_then(|v| v.checked_mul(mk))
            .ok_or("snap radix overflow")?;
        let mj = 8u64.checked_mul(md).ok_or("snap radix overflow")?;
        let mi = u64::try_from(nj)
            .map_err(|_| "snap window overflow")?
            .checked_mul(mj)
            .ok_or("snap radix overflow")?;
        Ok(Self { ma, mk, md, mj, mi })
    }
    fn pack(&self, s: State) -> Result<u64, String> {
        let terms = [
            (s.cell[0], self.mi),
            (s.cell[1], self.mj),
            (s.heading, self.md),
            (s.via, self.mk),
            (s.owed, self.ma),
            (s.taken, 1),
        ];
        let mut value = 0u64;
        for (n, scale) in terms {
            value = value
                .checked_add((n as u64).checked_mul(scale).ok_or("snap state overflow")?)
                .ok_or("snap state overflow")?;
        }
        Ok(value)
    }
    fn unpack(&self, value: u64) -> State {
        let i = value / self.mi;
        let r = value % self.mi;
        let j = r / self.mj;
        let r = r % self.mj;
        let d = r / self.md;
        let r = r % self.md;
        let k = r / self.mk;
        let r = r % self.mk;
        State {
            cell: [i as usize, j as usize],
            heading: d as usize,
            via: k as usize,
            owed: (r / self.ma) as usize,
            taken: (r % self.ma) as usize,
        }
    }
}
struct SearchInput<'a> {
    window: &'a Window,
    grid: f64,
    track: f64,
    lane_pitch: f64,
    runs: &'a [String],
    via_points: &'a [Point],
    gates: Vec<[f64; 2]>,
    room_start: [f64; 8],
    room_end: [f64; 8],
    total: f64,
    start: [usize; 2],
    end: [usize; 2],
    start_heading: usize,
    end_heading: usize,
    join_start: Point,
    join_end: Point,
    outward: Point,
    inward: Point,
    paired: bool,
    crossed: bool,
    straight: usize,
    turn_straight: usize,
    cap: usize,
    probe: usize,
    cross_steps: [[usize; 2]; 8],
    poses: bool,
    strict: bool,
    keep_weight: f64,
    deadline: Instant,
}
struct SearchResult {
    path: Option<Vec<State>>,
    popped: usize,
    farthest: Option<(f64, State)>,
}
fn search<F>(
    input: &SearchInput<'_>,
    multi: Option<(
        &[([usize; 2], usize)],
        &BTreeMap<[usize; 2], BTreeSet<usize>>,
    )>,
    mut crossover: F,
) -> Result<SearchResult, String>
where
    F: FnMut(usize, usize, usize) -> Result<bool, String>,
{
    let w = input.window;
    let via_count = input.via_points.len();
    let pr = if input.poses { input.probe } else { 0 };
    let owed_cap = input
        .cap
        .max(input.turn_straight)
        .max(input.straight)
        .max(pr)
        .max(if input.crossed {
            input.cross_steps.iter().map(|x| x[1]).max().unwrap_or(0)
        } else {
            0
        });
    let radix = Radix::new(w.nj, via_count, input.cap, owed_cap)?;
    let h = |cell: [usize; 2]| {
        if multi.is_some() {
            0.0
        } else {
            ((cell[0] as f64 - input.end[0] as f64).hypot(cell[1] as f64 - input.end[1] as f64))
                * input.grid
        }
    };
    let mut best: HashMap<u64, f64> = HashMap::new();
    let mut queue = BinaryHeap::new();
    let starts = multi
        .map(|m| m.0.to_vec())
        .unwrap_or_else(|| vec![(input.start, input.start_heading)]);
    for (cell, heading) in starts {
        let state = radix.pack(State {
            cell,
            heading,
            via: 0,
            owed: pr,
            taken: 0,
        })?;
        best.insert(state, 0.0);
        queue.push(QueueItem {
            estimate: h(cell),
            cost: 0.0,
            state,
        });
    }
    let mut previous = HashMap::new();
    let mut popped = 0usize;
    let mut goal = None;
    let mut farthest = None;
    while let Some(item) = queue.pop() {
        if Instant::now() >= input.deadline {
            return Err("AWX snap budget exhausted".into());
        }
        if best
            .get(&item.state)
            .is_some_and(|&cost| cost < item.cost - 1e-12)
        {
            continue;
        }
        popped = popped.checked_add(1).ok_or("snap state count overflow")?;
        let state = radix.unpack(item.state);
        let [i, j] = state.cell;
        let cell = w.index(i, j);
        if multi.is_some() && farthest.as_ref().is_none_or(|(arc, _)| w.arc[cell] > *arc) {
            farthest = Some((w.arc[cell], state));
        }
        let at_end = if let Some((_, ends)) = multi {
            ends.get(&state.cell)
                .is_some_and(|headings| headings.contains(&state.heading))
        } else {
            state.cell == input.end
        };
        if at_end && state.via == via_count && (!input.poses || state.taken >= pr) {
            goal = Some(item.state);
            break;
        }
        let layer = &input.runs[state.via];
        let mut via_allowed = false;
        let mut owed_after = input.straight;
        if input.crossed && state.via == 0 && via_count > 0 {
            via_allowed = state.owed == 0
                && state.taken >= input.cross_steps[state.heading][0]
                && input.room_start[state.heading] <= w.arc[cell]
                && w.arc[cell] <= input.total - input.room_end[state.heading]
                && !w.bad[&input.runs[1]][cell]
                && crossover(i, j, state.heading)?;
            owed_after = input.cross_steps[state.heading][1];
        } else if state.via < via_count {
            via_allowed = !w.via_bad[state.heading % 4][cell]
                && state.owed == 0
                && state.taken >= input.straight
                && input.room_start[state.heading] <= w.arc[cell]
                && w.arc[cell] <= input.total - input.room_end[state.heading]
                && !w.bad[&input.runs[state.via + 1]][cell];
        }
        if via_allowed {
            let point = w.point(i, j, input.grid);
            let deviation = corridor::distance(point, input.via_points[state.via]);
            if deviation <= 2.0 * input.lane_pitch {
                let next = State {
                    via: state.via + 1,
                    owed: owed_after,
                    taken: 0,
                    ..state
                };
                let packed = radix.pack(next)?;
                let cost = item.cost + deviation;
                if cost < best.get(&packed).copied().unwrap_or(f64::INFINITY) - 1e-12 {
                    best.insert(packed, cost);
                    previous.insert(packed, item.state);
                    queue.push(QueueItem {
                        estimate: cost + h(next.cell),
                        cost,
                        state: packed,
                    });
                }
            }
        }
        for heading in 0usize..8 {
            let difference = heading.abs_diff(state.heading);
            let bend = difference.min(8 - difference);
            if bend >= 3 || (input.paired && bend >= 2) || (bend > 0 && state.owed > 0) {
                continue;
            }
            let dir = DIRS[heading];
            let ni = i as i64 + dir[0];
            let nj = j as i64 + dir[1];
            if !w.valid(ni, nj) {
                continue;
            }
            let next_cell = [ni as usize, nj as usize];
            let index = w.index(next_cell[0], next_cell[1]);
            if !w.band[index] {
                continue;
            }
            let along = |v: Point| dir[0] as f64 * v[0] + dir[1] as f64 * v[1];
            if !input.poses {
                if state.cell == input.start && state.via == 0 && along(input.join_start) < -1e-9 {
                    continue;
                }
                if next_cell == input.end && state.via == via_count && along(input.join_end) < -1e-9
                {
                    continue;
                }
                if input.strict
                    && state.via == 0
                    && (i as f64 - input.start[0] as f64).hypot(j as f64 - input.start[1] as f64)
                        * input.grid
                        <= input.track
                    && along(input.outward) < -1e-9
                {
                    continue;
                }
                if input.strict
                    && state.via == via_count
                    && (ni as f64 - input.end[0] as f64).hypot(nj as f64 - input.end[1] as f64)
                        * input.grid
                        <= input.track
                    && along(input.inward) < -1e-9
                {
                    continue;
                }
            }
            if (next_cell != input.end || input.poses) && w.bad[layer][index] {
                continue;
            }
            if !(input.gates[state.via][0] <= w.arc[index]
                && w.arc[index] <= input.gates[state.via][1])
            {
                continue;
            }
            let step = input.grid
                * if dir[0] != 0 && dir[1] != 0 {
                    2.0_f64.sqrt()
                } else {
                    1.0
                };
            let mut cost =
                item.cost + step * (1.0 + 0.5 * w.distance[index]) + 4.0 * input.grid * bend as f64;
            if let Some(tol) = &w.tolerance {
                if w.distance[index] > tol[index] {
                    cost += step * input.keep_weight * (w.distance[index] - tol[index]);
                }
            }
            if w.soft.as_ref().is_some_and(|soft| soft[layer][index]) {
                cost += step;
            }
            if multi.is_none() && next_cell == input.end {
                let difference = heading.abs_diff(input.end_heading);
                let end_bend = difference.min(8 - difference);
                if (input.paired && end_bend >= 2) || (input.poses && end_bend != 0) {
                    continue;
                }
                cost += 4.0 * input.grid * end_bend as f64;
            }
            let (owed, taken) = if input.paired {
                if bend == 0 {
                    (
                        state.owed.saturating_sub(1),
                        (state.taken + 1).min(input.cap),
                    )
                } else {
                    (input.turn_straight, 1)
                }
            } else {
                (0, 0)
            };
            let next = State {
                cell: next_cell,
                heading,
                via: state.via,
                owed,
                taken,
            };
            let packed = radix.pack(next)?;
            if cost < best.get(&packed).copied().unwrap_or(f64::INFINITY) - 1e-12 {
                best.insert(packed, cost);
                previous.insert(packed, item.state);
                queue.push(QueueItem {
                    estimate: cost + h(next_cell),
                    cost,
                    state: packed,
                });
            }
        }
    }
    let path = if let Some(goal) = goal {
        let mut path = vec![goal];
        while let Some(&parent) = previous.get(path.last().unwrap()) {
            path.push(parent);
        }
        path.reverse();
        Some(path.into_iter().map(|v| radix.unpack(v)).collect())
    } else {
        None
    };
    Ok(SearchResult {
        path,
        popped,
        farthest,
    })
}
#[derive(Clone)]
struct PlacedSegment {
    lane: String,
    layer: String,
    a: Point,
    b: Point,
    exact: bool,
}
#[derive(Clone)]
struct PlacedVia {
    lane: String,
    point: Point,
    exact: bool,
}
#[derive(Clone)]
struct Attempt {
    lane: Option<LaneGeometry>,
    reason: String,
    stuck: Option<Point>,
    stuck_layer: Option<String>,
}
struct Snapper<'a, G: JointGeometry + ?Sized> {
    ctx: &'a WholeContext,
    frame: &'a WholeFrame,
    oracle: &'a G,
    input: &'a GeometryResult,
    smooth: BTreeMap<String, SmoothLane>,
    order: Vec<String>,
    result: GeometryResult,
    placed: Vec<PlacedSegment>,
    legs: Vec<PlacedSegment>,
    vias: Vec<PlacedVia>,
    dive_tests: Vec<(String, Point)>,
    memo: HashMap<String, Attempt>,
    static_cache:
        BTreeMap<(String, i64, i64, usize, usize), (BTreeMap<String, Vec<bool>>, [Vec<bool>; 4])>,
    pairs_only: bool,
    keep: Vec<Point>,
    deadline: Instant,
    keep_weight: f64,
}
impl<'a, G: JointGeometry + ?Sized> Snapper<'a, G> {
    fn check_budget(&self) -> Result<(), String> {
        if Instant::now() >= self.deadline {
            Err("AWX snap budget exhausted".into())
        } else {
            Ok(())
        }
    }
    fn pair(&self, n: &str) -> bool {
        self.ctx.pairs.contains_key(n)
    }
    fn half(&self) -> f64 {
        pairs::pitch(&self.ctx.cfg) / 2.0
    }
    fn snap_half(&self, n: &str) -> f64 {
        if self.pair(n) {
            self.half() / std::f64::consts::FRAC_PI_8.cos()
        } else {
            0.0
        }
    }
    fn barrel_offset(&self, n: &str) -> f64 {
        if self.pair(n) {
            pairs::dive_offset(&self.ctx.cfg, self.half())
        } else {
            0.0
        }
    }
    fn lane_bar(&self, a: &str, b: &str) -> f64 {
        self.ctx.cfg.track_width
            + self.ctx.cfg.clearance
            + self.snap_half(a)
            + self.snap_half(b)
            + self.ctx.cfg.grid_step / 2.0
                * (u8::from(self.pair(a)) as f64 + u8::from(self.pair(b)) as f64)
    }
    fn ring(&self, n: &str) -> f64 {
        pairs::via_ring(
            &self.ctx.cfg,
            if self.pair(n) { self.half() } else { 0.0 },
            None,
        )
    }
    fn own(&self, n: &str) -> Result<(i32, Option<i32>, BTreeSet<i32>), String> {
        if let Some(legs) = self.ctx.pairs.get(n) {
            let a = *self
                .ctx
                .byname
                .get(&legs[0])
                .ok_or("snap physical P net missing")?;
            let b = *self
                .ctx
                .byname
                .get(&legs[1])
                .ok_or("snap physical N net missing")?;
            Ok((a, Some(b), [a, b].into_iter().collect()))
        } else {
            let a = *self.ctx.byname.get(n).ok_or("snap net id missing")?;
            Ok((a, None, [a].into_iter().collect()))
        }
    }
    fn make_smooth(
        ctx: &WholeContext,
        geometry: &GeometryResult,
    ) -> Result<BTreeMap<String, SmoothLane>, String> {
        let mut out = BTreeMap::new();
        for (n, lane) in &geometry.lanes {
            if lane.xy.len() < 2 || lane.layers.len() + 1 != lane.xy.len() {
                return Err(format!("snap malformed smooth pieces for {n}"));
            }
            let pieces: Vec<_> = lane
                .xy
                .windows(2)
                .zip(&lane.layers)
                .map(|(p, l)| LayerPiece {
                    a: p[0],
                    b: p[1],
                    layer: l.clone(),
                })
                .collect();
            let mut vias = vec![];
            let mut runs = vec![lane.layers[0].clone()];
            for k in 1..lane.layers.len() {
                if lane.layers[k] != lane.layers[k - 1] {
                    vias.push(lane.xy[k]);
                    runs.push(lane.layers[k].clone());
                }
            }
            let barrels = if ctx.pairs.contains_key(n) {
                vias.iter()
                    .flat_map(|&v| {
                        pairs::dive_barrels(
                            v,
                            &pieces,
                            pairs::dive_offset(&ctx.cfg, pairs::pitch(&ctx.cfg) / 2.0),
                        )
                    })
                    .collect()
            } else {
                vias.clone()
            };
            out.insert(
                n.clone(),
                SmoothLane {
                    points: lane.xy.clone(),
                    layers: lane.layers.clone(),
                    runs,
                    vias,
                    barrels,
                },
            );
        }
        Ok(out)
    }
    fn end_dirs(&self, n: &str) -> Result<[Point; 2], String> {
        let lane = &self.smooth[n];
        let first = lane.points[0];
        let last = *lane.points.last().unwrap();
        let a = if self.pair(n) {
            self.ctx.tooth_dir.get(n).copied()
        } else {
            self.ctx
                .stub_direction(n, self.ctx.ends[n].tooth, &lane.layers[0])
        };
        let b = if self.pair(n) {
            self.ctx.stub_dir.get(n).copied()
        } else {
            self.ctx
                .stub_direction(n, self.ctx.ends[n].berth, lane.layers.last().unwrap())
        };
        Ok([
            a.unwrap_or(corridor::unit(corridor::sub(lane.points[1], first))?),
            b.map(|v| [-v[0], -v[1]])
                .unwrap_or(corridor::unit(corridor::sub(
                    last,
                    lane.points[lane.points.len() - 2],
                ))?),
        ])
    }
    fn own_via_blocked(&self, point: Point, own: &BTreeSet<i32>) -> Result<bool, String> {
        let cfg = &self.ctx.cfg;
        let g = cfg.grid_step;
        let cell = [
            numeric::quantize(point[0], g)?,
            numeric::quantize(point[1], g)?,
        ];
        for via in &self.ctx.base_vias {
            if !own.contains(&via.net_id) {
                continue;
            }
            let center = [
                numeric::quantize(via.point[0], g)?,
                numeric::quantize(via.point[1], g)?,
            ];
            let ring = (cfg.via_size + cfg.clearance).max(
                if cfg.hole_to_hole_clearance > 0.0 && via.drill > 0.0 {
                    (via.drill
                        + cfg
                            .net_via_sizes
                            .get(&via.net_id)
                            .map(|v| v[1])
                            .unwrap_or(cfg.via_drill)
                            .max(cfg.via_drill))
                        / 2.0
                        + cfg.hole_to_hole_clearance
                } else {
                    0.0
                },
            );
            let radius = (ring / g).max(1.0)
                + corridor::distance(via.point, [center[0] as f64 * g, center[1] as f64 * g]) / g;
            if (cell[0] - center[0]) as f64 * ((cell[0] - center[0]) as f64)
                + (cell[1] - center[1]) as f64 * ((cell[1] - center[1]) as f64)
                <= radius * radius
            {
                return Ok(true);
            }
        }
        for pad in &self.ctx.pads {
            if !own.contains(&pad.net_id) || pad.drill <= 0.0 || cfg.hole_to_hole_clearance <= 0.0 {
                continue;
            }
            let center = [
                numeric::quantize(pad.point[0], g)?,
                numeric::quantize(pad.point[1], g)?,
            ];
            if cell == center {
                continue;
            }
            if {
                let axis = pad.drill_axis.unwrap_or([pad.point; 2]);
                corridor::point_segment_distance(point, axis[0], axis[1])
            } < pad.drill / 2.0
                + cfg
                    .net_via_sizes
                    .get(&pad.net_id)
                    .map(|v| v[1])
                    .unwrap_or(cfg.via_drill)
                    .max(cfg.via_drill)
                    / 2.0
                + cfg.hole_to_hole_clearance
            {
                return Ok(true);
            }
        }
        for &net in own {
            if self.oracle.same_net_pad_via_blocked(net, point, cfg)? {
                return Ok(true);
            }
        }
        Ok(false)
    }
    fn static_fields(
        &mut self,
        n: &str,
        w: &Window,
    ) -> Result<(BTreeMap<String, Vec<bool>>, [Vec<bool>; 4]), String> {
        let key = (n.to_owned(), w.origin[0], w.origin[1], w.ni, w.nj);
        if let Some(value) = self.static_cache.get(&key) {
            return Ok(value.clone());
        }
        let (net, partner, own) = self.own(n)?;
        let cfg = &self.ctx.cfg;
        let count = w.ni.checked_mul(w.nj).ok_or("snap mask size overflow")?;
        let mut tracks: BTreeMap<String, Vec<bool>> = self
            .ctx
            .layers
            .iter()
            .map(|l| (l.clone(), vec![true; count]))
            .collect();
        let mut vias: [Vec<bool>; 4] = std::array::from_fn(|_| vec![true; count]);
        let spc = if self.pair(n) {
            pairs::pose_via_cells(cfg, self.half())
        } else {
            0
        };
        for i in 0..w.ni {
            self.check_budget()?;
            for j in 0..w.nj {
                let index = w.index(i, j);
                if !w.band[index] {
                    continue;
                }
                let point = w.point(i, j, cfg.grid_step);
                for layer in &self.ctx.layers {
                    tracks.get_mut(layer).unwrap()[index] = self.oracle.snap_cell_blocked(
                        net,
                        partner,
                        Some(layer),
                        point,
                        if self.pair(n) { self.half() } else { 0.0 },
                        cfg,
                    )?;
                }
                for axis in 0..4 {
                    let perpendicular = [-DIRS[axis][1] * spc, DIRS[axis][0] * spc];
                    let mut blocked = false;
                    for sign in [0, 1, -1] {
                        let probe = [
                            point[0] + (sign * perpendicular[0]) as f64 * cfg.grid_step,
                            point[1] + (sign * perpendicular[1]) as f64 * cfg.grid_step,
                        ];
                        blocked |= self.oracle.snap_cell_blocked(
                            net,
                            partner,
                            None,
                            probe,
                            if self.pair(n) { self.half() } else { 0.0 },
                            cfg,
                        )? || self.own_via_blocked(probe, &own)?;
                    }
                    vias[axis][index] = blocked;
                }
            }
        }
        self.static_cache
            .insert(key, (tracks.clone(), vias.clone()));
        Ok((tracks, vias))
    }
}
fn box_near(point: Point, a: Point, b: Point, reach: f64) -> bool {
    point[0] >= a[0].min(b[0]) - reach
        && point[0] <= a[0].max(b[0]) + reach
        && point[1] >= a[1].min(b[1]) - reach
        && point[1] <= a[1].max(b[1]) + reach
}
fn direction_index(v: Point) -> Result<usize, String> {
    let q = numeric::checked_round_i64(v[1].atan2(v[0]) / std::f64::consts::FRAC_PI_4)?;
    Ok(q.rem_euclid(8) as usize)
}
fn on_grid(a: Point, b: Point, g: f64) -> bool {
    let grid = |v: f64| (v / g - (v / g).round_ties_even()).abs() < 1e-3;
    (grid(a[0]) && grid(a[1]) && grid(b[0]) && grid(b[1]))
        && ((a[0] - b[0]).abs() < 1e-6
            || (a[1] - b[1]).abs() < 1e-6
            || ((a[0] - b[0]).abs() - (a[1] - b[1]).abs()).abs() < 1e-6)
}
fn stamp<F>(
    window: &Window,
    mask: &mut [bool],
    a: Point,
    b: Point,
    bar: f64,
    offset: Point,
    g: f64,
    share: bool,
    band_width: f64,
    mut distance: F,
) where
    F: FnMut(Point) -> f64,
{
    let reach = bar + if share { band_width } else { 0.0 };
    let lo = window.point(0, 0, g);
    let i0 = ((a[0].min(b[0]) - reach - offset[0] - lo[0]) / g)
        .floor()
        .max(0.0) as usize;
    let j0 = ((a[1].min(b[1]) - reach - offset[1] - lo[1]) / g)
        .floor()
        .max(0.0) as usize;
    let i1 = ((a[0].max(b[0]) + reach - offset[0] - lo[0]) / g)
        .ceil()
        .min(window.ni as f64 - 1.0);
    let j1 = ((a[1].max(b[1]) + reach - offset[1] - lo[1]) / g)
        .ceil()
        .min(window.nj as f64 - 1.0);
    if i1 < 0.0 || j1 < 0.0 || i0 as f64 > i1 || j0 as f64 > j1 {
        return;
    }
    for i in i0..=i1 as usize {
        for j in j0..=j1 as usize {
            let idx = window.index(i, j);
            let p = corridor::add(window.point(i, j, g), offset);
            let d = distance(p);
            mask[idx] |= if share {
                d - window.distance[idx] < bar - 1e-3
            } else {
                d < bar + 1e-3
            };
        }
    }
}
impl<'a, G: JointGeometry + ?Sized> Snapper<'a, G> {
    fn build_window(&mut self, n: &str) -> Result<Window, String> {
        self.check_budget()?;
        let cfg = &self.ctx.cfg;
        let g = cfg.grid_step;
        let band_width = 2.0 * cfg.lane_min;
        let smooth = self.smooth[n].clone();
        let mut min = [f64::INFINITY; 2];
        let mut max = [f64::NEG_INFINITY; 2];
        for p in &smooth.points {
            for k in 0..2 {
                min[k] = min[k].min(p[k]);
                max[k] = max[k].max(p[k]);
            }
        }
        let origin = [
            numeric::checked_round_i64(((min[0] - band_width - g) / g).floor())?,
            numeric::checked_round_i64(((min[1] - band_width - g) / g).floor())?,
        ];
        let finish = [
            numeric::checked_round_i64(((max[0] + band_width + g) / g).ceil())?,
            numeric::checked_round_i64(((max[1] + band_width + g) / g).ceil())?,
        ];
        let ni = usize::try_from(
            finish[0]
                .checked_sub(origin[0])
                .and_then(|v| v.checked_add(1))
                .ok_or("snap window overflow")?,
        )
        .map_err(|_| "snap negative window")?;
        let nj = usize::try_from(
            finish[1]
                .checked_sub(origin[1])
                .and_then(|v| v.checked_add(1))
                .ok_or("snap window overflow")?,
        )
        .map_err(|_| "snap negative window")?;
        let count = ni
            .checked_mul(nj)
            .ok_or("snap window allocation overflow")?;
        let mut w = Window {
            origin,
            ni,
            nj,
            band: vec![false; count],
            distance: vec![f64::INFINITY; count],
            arc: vec![0.0; count],
            tolerance: self.pair(n).then(|| vec![0.0; count]),
            bad: BTreeMap::new(),
            via_bad: std::array::from_fn(|_| vec![false; count]),
            soft: None,
            cross_room: None,
        };
        let mut arc0 = 0.0;
        for segment in smooth.points.windows(2) {
            let a = segment[0];
            let b = segment[1];
            let d = corridor::sub(b, a);
            let length = corridor::length(d);
            let length2 = (length * length).max(1e-18);
            for i in 0..ni {
                self.check_budget()?;
                for j in 0..nj {
                    let point = w.point(i, j, g);
                    let t = (corridor::dot(corridor::sub(point, a), d) / length2).clamp(0.0, 1.0);
                    let closest = corridor::add(a, corridor::mul(d, t));
                    let distance = corridor::distance(point, closest);
                    let idx = w.index(i, j);
                    if distance < w.distance[idx] {
                        w.distance[idx] = distance;
                        w.arc[idx] = arc0 + t * length;
                        if let Some(tol) = &mut w.tolerance {
                            tol[idx] = pairs::stair_spread(cfg, d) / 2.0 + g;
                        }
                    }
                }
            }
            arc0 += length;
        }
        for k in 0..count {
            w.band[k] = w.distance[k] <= band_width;
        }
        let (mut bad, static_via) = self.static_fields(n, &w)?;
        let unplaced: Vec<_> = self
            .order
            .iter()
            .filter(|m| {
                m.as_str() != n
                    && !self.result.lanes.contains_key(*m)
                    && (self.pair(m) || !self.pairs_only)
            })
            .cloned()
            .collect();
        let half = self.snap_half(n);
        let offg = if self.pair(n) { 1.0 } else { 0.0 };
        let via_offset = self.barrel_offset(n);
        let ring = self.ring(n);
        let via_via =
            (cfg.via_size + cfg.clearance).max(cfg.via_drill + cfg.hole_to_hole_clearance);
        // 500–532: pair body clearance as drawn against foreign pads/stubs/vias.
        if self.pair(n) {
            let (_, _, own) = self.own(n)?;
            let bar = half + cfg.track_width / 2.0 + cfg.clearance + g / 2.0;
            for layer in &self.ctx.layers {
                let mask = bad.get_mut(layer).unwrap();
                for pad in &self.ctx.pads {
                    if own.contains(&pad.net_id) {
                        continue;
                    }
                    let on = pad.pad_type == "np_thru_hole"
                        || pad.drill > 0.0
                        || pad.layers.iter().any(|l| l == "*.Cu" || l == layer);
                    if !on {
                        continue;
                    }
                    let p = pad.point;
                    if pad.pad_type == "np_thru_hole" {
                        stamp(
                            &w,
                            mask,
                            p,
                            p,
                            bar + pad.drill / 2.0,
                            [0.0; 2],
                            g,
                            false,
                            band_width,
                            |q| corridor::distance(q, p),
                        );
                    } else {
                        let h = [pad.size[0] / 2.0, pad.size[1] / 2.0];
                        stamp(
                            &w,
                            mask,
                            corridor::sub(p, h),
                            corridor::add(p, h),
                            bar,
                            [0.0; 2],
                            g,
                            false,
                            band_width,
                            |q| {
                                pairs::pad_distance(
                                    q[0] - p[0],
                                    q[1] - p[1],
                                    h[0],
                                    h[1],
                                    pad.corner_radius,
                                )
                            },
                        );
                    }
                }
                for segment in &self.ctx.base_segments {
                    if segment.layer == *layer && !own.contains(&segment.net_id) {
                        stamp(
                            &w,
                            mask,
                            segment.a,
                            segment.b,
                            bar + segment.width / 2.0,
                            [0.0; 2],
                            g,
                            false,
                            band_width,
                            |q| corridor::point_segment_distance(q, segment.a, segment.b),
                        );
                    }
                }
                for via in &self.ctx.base_vias {
                    if !own.contains(&via.net_id) {
                        stamp(
                            &w,
                            mask,
                            via.point,
                            via.point,
                            bar + via.diameter / 2.0,
                            [0.0; 2],
                            g,
                            false,
                            band_width,
                            |q| corridor::distance(q, via.point),
                        );
                    }
                }
            }
        }
        // Placed singles' centerlines and pairs' actual mitred/exact legs.
        for segment in self
            .placed
            .iter()
            .filter(|s| !self.pair(&s.lane))
            .chain(self.legs.iter())
        {
            if segment.lane == n {
                continue;
            }
            let hm = 0.0;
            let om = if self.legs.iter().any(|s| std::ptr::eq(s, segment)) {
                if segment.exact { 0.0 } else { 1.0 }
            } else {
                0.0
            };
            if let Some(mask) = bad.get_mut(&segment.layer) {
                let bar = cfg.track_width + cfg.clearance + half + hm + g / 2.0 * (offg + om);
                stamp(
                    &w,
                    mask,
                    segment.a,
                    segment.b,
                    bar,
                    [0.0; 2],
                    g,
                    false,
                    band_width,
                    |p| corridor::point_segment_distance(p, segment.a, segment.b),
                );
            }
        }
        if !self.pair(n) {
            for (_, point) in &self.dive_tests {
                for mask in bad.values_mut() {
                    stamp(
                        &w,
                        mask,
                        *point,
                        *point,
                        cfg.via_size / 2.0
                            + cfg.track_width / 2.0
                            + cfg.clearance
                            + self.half()
                            + g / 2.0,
                        [0.0; 2],
                        g,
                        false,
                        band_width,
                        |p| corridor::distance(p, *point),
                    );
                }
            }
        }
        for via in &self.vias {
            let rounded = [
                numeric::quantize(via.point[0], g)? as f64 * g,
                numeric::quantize(via.point[1], g)? as f64 * g,
            ];
            let bar = ring + corridor::distance(via.point, rounded);
            for mask in bad.values_mut() {
                stamp(
                    &w,
                    mask,
                    rounded,
                    rounded,
                    bar,
                    [0.0; 2],
                    g,
                    false,
                    band_width,
                    |p| corridor::distance(p, rounded),
                );
                stamp(
                    &w,
                    mask,
                    via.point,
                    via.point,
                    bar,
                    [0.0; 2],
                    g,
                    false,
                    band_width,
                    |p| corridor::distance(p, via.point),
                );
            }
        }
        for m in &unplaced {
            let other = &self.smooth[m];
            for (segment, layer) in other.points.windows(2).zip(&other.layers) {
                stamp(
                    &w,
                    bad.get_mut(layer)
                        .ok_or("snap smooth layer outside stack")?,
                    segment[0],
                    segment[1],
                    self.lane_bar(n, m),
                    [0.0; 2],
                    g,
                    true,
                    band_width,
                    |p| corridor::point_segment_distance(p, segment[0], segment[1]),
                );
            }
            for &barrel in &other.barrels {
                for mask in bad.values_mut() {
                    stamp(
                        &w,
                        mask,
                        barrel,
                        barrel,
                        ring + g / 2.0 * if self.pair(m) { 1.0 } else { 0.0 },
                        [0.0; 2],
                        g,
                        true,
                        band_width,
                        |p| corridor::distance(p, barrel),
                    );
                }
            }
        }
        let vfield = |offset: Point| -> Result<Vec<bool>, String> {
            let mut blocked = vec![false; count];
            let rounded = [
                numeric::quantize(offset[0], g)? as f64 * g,
                numeric::quantize(offset[1], g)? as f64 * g,
            ];
            let boff = corridor::distance(offset, rounded);
            for segment in &self.placed {
                let bar = self.ring(&segment.lane)
                    + boff
                    + if on_grid(segment.a, segment.b, g) {
                        0.0
                    } else {
                        g / 2.0
                    };
                stamp(
                    &w,
                    &mut blocked,
                    segment.a,
                    segment.b,
                    bar,
                    offset,
                    g,
                    false,
                    band_width,
                    |p| corridor::point_segment_distance(p, segment.a, segment.b),
                );
            }
            for segment in &self.legs {
                let bar = pairs::via_ring(cfg, 0.0, None)
                    + boff
                    + if on_grid(segment.a, segment.b, g) {
                        0.0
                    } else {
                        g / 2.0
                    };
                stamp(
                    &w,
                    &mut blocked,
                    segment.a,
                    segment.b,
                    bar,
                    offset,
                    g,
                    false,
                    band_width,
                    |p| corridor::point_segment_distance(p, segment.a, segment.b),
                );
            }
            for via in &self.vias {
                let bar = via_via
                    + g / 2.0
                        * (offg
                            + if via.exact {
                                0.0
                            } else if self.pair(&via.lane) {
                                1.0
                            } else {
                                0.0
                            });
                stamp(
                    &w,
                    &mut blocked,
                    via.point,
                    via.point,
                    bar,
                    offset,
                    g,
                    false,
                    band_width,
                    |p| corridor::distance(p, via.point),
                );
            }
            if !self.pair(n) {
                for (_, point) in &self.dive_tests {
                    stamp(
                        &w,
                        &mut blocked,
                        *point,
                        *point,
                        cfg.via_size + cfg.clearance + g / 2.0,
                        offset,
                        g,
                        false,
                        band_width,
                        |p| corridor::distance(p, *point),
                    );
                }
            }
            for m in &unplaced {
                let other = &self.smooth[m];
                for segment in other.points.windows(2) {
                    stamp(
                        &w,
                        &mut blocked,
                        segment[0],
                        segment[1],
                        self.ring(m) + boff,
                        offset,
                        g,
                        true,
                        band_width,
                        |p| corridor::point_segment_distance(p, segment[0], segment[1]),
                    );
                }
                for &barrel in &other.barrels {
                    stamp(
                        &w,
                        &mut blocked,
                        barrel,
                        barrel,
                        via_via + g / 2.0 * (offg + if self.pair(m) { 1.0 } else { 0.0 }),
                        offset,
                        g,
                        true,
                        band_width,
                        |p| corridor::distance(p, barrel),
                    );
                }
            }
            Ok(blocked)
        };
        let pfield = |offset: Point| {
            let mut blocked = vec![false; count];
            let line = cfg.via_size / 2.0 + cfg.track_width / 2.0 + cfg.clearance + self.half();
            let via = cfg.via_size + cfg.clearance;
            for segment in self
                .placed
                .iter()
                .filter(|s| !self.pair(&s.lane))
                .chain(self.legs.iter())
            {
                stamp(
                    &w,
                    &mut blocked,
                    segment.a,
                    segment.b,
                    line,
                    offset,
                    g,
                    false,
                    band_width,
                    |p| corridor::point_segment_distance(p, segment.a, segment.b),
                );
            }
            for lane in self.result.lanes.values() {
                for &point in &lane.vias {
                    let rounded = [
                        (point[0] / g).round_ties_even() * g,
                        (point[1] / g).round_ties_even() * g,
                    ];
                    stamp(
                        &w,
                        &mut blocked,
                        point,
                        point,
                        via + corridor::distance(point, rounded),
                        offset,
                        g,
                        false,
                        band_width,
                        |p| corridor::distance(p, point),
                    );
                }
            }
            for m in &unplaced {
                let lane = &self.smooth[m];
                for segment in lane.points.windows(2) {
                    stamp(
                        &w,
                        &mut blocked,
                        segment[0],
                        segment[1],
                        line + self.snap_half(m) + g / 2.0 * if self.pair(m) { 1.0 } else { 0.0 },
                        offset,
                        g,
                        true,
                        band_width,
                        |p| corridor::point_segment_distance(p, segment[0], segment[1]),
                    );
                }
                for &point in &lane.vias {
                    stamp(
                        &w,
                        &mut blocked,
                        point,
                        point,
                        via + g / 2.0,
                        offset,
                        g,
                        true,
                        band_width,
                        |p| corridor::distance(p, point),
                    );
                }
            }
            blocked
        };
        if self.pair(n) {
            let spc = pairs::pose_via_cells(cfg, self.half());
            let center = pfield([0.0; 2]);
            let (_, _, own) = self.own(n)?;
            let sfield = |offset: Point| {
                let mut blocked = vec![false; count];
                let need = cfg.via_size / 2.0 + cfg.clearance + g / 2.0;
                for pad in &self.ctx.pads {
                    if own.contains(&pad.net_id) {
                        continue;
                    }
                    if pad.pad_type == "np_thru_hole" {
                        stamp(
                            &w,
                            &mut blocked,
                            pad.point,
                            pad.point,
                            cfg.via_drill / 2.0 + pad.drill / 2.0 + cfg.hole_to_hole_clearance,
                            offset,
                            g,
                            false,
                            band_width,
                            |p| corridor::distance(p, pad.point),
                        );
                    } else {
                        let h = [pad.size[0] / 2.0, pad.size[1] / 2.0];
                        stamp(
                            &w,
                            &mut blocked,
                            corridor::sub(pad.point, h),
                            corridor::add(pad.point, h),
                            need,
                            offset,
                            g,
                            false,
                            band_width,
                            |p| {
                                pairs::pad_distance(
                                    p[0] - pad.point[0],
                                    p[1] - pad.point[1],
                                    h[0],
                                    h[1],
                                    pad.corner_radius,
                                )
                            },
                        );
                    }
                }
                for segment in &self.ctx.base_segments {
                    if !own.contains(&segment.net_id) {
                        stamp(
                            &w,
                            &mut blocked,
                            segment.a,
                            segment.b,
                            need + segment.width / 2.0,
                            offset,
                            g,
                            false,
                            band_width,
                            |p| corridor::point_segment_distance(p, segment.a, segment.b),
                        );
                    }
                }
                for via in &self.ctx.base_vias {
                    if !own.contains(&via.net_id) {
                        let bar = (cfg.via_size / 2.0 + via.diameter / 2.0 + cfg.clearance).max(
                            cfg.via_drill / 2.0 + via.drill / 2.0 + cfg.hole_to_hole_clearance,
                        ) + g / 2.0;
                        stamp(
                            &w,
                            &mut blocked,
                            via.point,
                            via.point,
                            bar,
                            offset,
                            g,
                            false,
                            band_width,
                            |p| corridor::distance(p, via.point),
                        );
                    }
                }
                blocked
            };
            let mut combined_via = [
                vec![false; count],
                vec![false; count],
                vec![false; count],
                vec![false; count],
            ];
            for axis in 0..4 {
                let perpendicular = [-DIRS[axis][1] as f64, DIRS[axis][0] as f64];
                let len = corridor::length(perpendicular);
                let offset = corridor::mul(perpendicular, via_offset / len);
                let probe = corridor::mul(perpendicular, spc as f64 * g);
                let fields = [
                    vfield(offset)?,
                    vfield(corridor::mul(offset, -1.0))?,
                    center.clone(),
                    pfield(probe),
                    pfield(corridor::mul(probe, -1.0)),
                    sfield(offset),
                    sfield(corridor::mul(offset, -1.0)),
                ];
                for idx in 0..count {
                    combined_via[axis][idx] =
                        static_via[axis][idx] || fields.iter().any(|f| f[idx]);
                }
            }
            w.via_bad = combined_via;
            let mut room = vec![false; count];
            for m in self
                .order
                .iter()
                .filter(|m| !self.pair(m) && !self.result.lanes.contains_key(*m))
            {
                for segment in self.smooth[m].points.windows(2) {
                    stamp(
                        &w,
                        &mut room,
                        segment[0],
                        segment[1],
                        pairs::via_ring(cfg, 0.0, None) + g,
                        [0.0; 2],
                        g,
                        false,
                        band_width,
                        |p| corridor::point_segment_distance(p, segment[0], segment[1]),
                    );
                }
            }
            w.cross_room = Some(room);
        } else {
            let field = vfield([0.0; 2])?;
            for axis in 0..4 {
                for idx in 0..count {
                    w.via_bad[axis][idx] = field[idx] || static_via[0][idx];
                }
            }
        }
        if self.pair(n) && self.pairs_only {
            let mut soft: BTreeMap<_, _> = self
                .ctx
                .layers
                .iter()
                .map(|l| (l.clone(), vec![false; count]))
                .collect();
            for m in self
                .order
                .iter()
                .filter(|m| m.as_str() != n && !self.pair(m) && !self.result.lanes.contains_key(*m))
            {
                let lane = &self.smooth[m];
                for (segment, layer) in lane.points.windows(2).zip(&lane.layers) {
                    stamp(
                        &w,
                        soft.get_mut(layer).unwrap(),
                        segment[0],
                        segment[1],
                        self.lane_bar(n, m),
                        [0.0; 2],
                        g,
                        true,
                        band_width,
                        |p| corridor::point_segment_distance(p, segment[0], segment[1]),
                    );
                    if !self.keep.is_empty()
                        && self.keep.iter().any(|&p| {
                            corridor::distance(p, segment[0]).min(corridor::distance(p, segment[1]))
                                < 3.0 * cfg.lane_min + corridor::distance(segment[0], segment[1])
                        })
                    {
                        stamp(
                            &w,
                            bad.get_mut(layer).unwrap(),
                            segment[0],
                            segment[1],
                            self.lane_bar(n, m),
                            [0.0; 2],
                            g,
                            false,
                            band_width,
                            |p| corridor::point_segment_distance(p, segment[0], segment[1]),
                        );
                    }
                }
            }
            w.soft = Some(soft);
        }
        w.bad = bad;
        Ok(w)
    }
}
#[derive(Clone)]
struct EndCandidate {
    length: f64,
    cell: [i64; 2],
    heading: usize,
    end: PairEnd,
    rank: f64,
}
impl<'a, G: JointGeometry + ?Sized> Snapper<'a, G> {
    fn near_static(
        &self,
        center: Point,
        radius: f64,
    ) -> (
        Vec<&super::whole_context::ContextPad>,
        Vec<&super::whole_context::CopperSegment>,
        Vec<&super::whole_context::CopperVia>,
    ) {
        let near = |point: Point| corridor::distance(point, center) < radius;
        (
            self.ctx.pads.iter().filter(|p| near(p.point)).collect(),
            self.ctx
                .base_segments
                .iter()
                .filter(|s| near(s.a) || near(s.b))
                .collect(),
            self.ctx
                .base_vias
                .iter()
                .filter(|v| near(v.point))
                .collect(),
        )
    }
    fn leg_clear(
        &self,
        points: &[Point],
        layer: &str,
        net: i32,
        lane: &str,
        center: Point,
        radius: f64,
    ) -> bool {
        let cfg = &self.ctx.cfg;
        let bar = cfg.track_width / 2.0 + cfg.clearance;
        let step = cfg.grid_step / 4.0;
        let (pads, segments, vias) = self.near_static(center, radius);
        let mut samples = vec![];
        for segment in points.windows(2) {
            let count = (corridor::distance(segment[0], segment[1]) / step)
                .ceil()
                .max(1.0) as usize;
            for k in 0..=count {
                samples.push(corridor::add(
                    segment[0],
                    corridor::mul(
                        corridor::sub(segment[1], segment[0]),
                        k as f64 / count as f64,
                    ),
                ));
            }
        }
        if samples.is_empty() {
            return false;
        }
        for pad in pads {
            let on = pad.pad_type == "np_thru_hole"
                || pad.drill > 0.0
                || pad.layers.iter().any(|l| l == "*.Cu" || l == layer);
            if pad.net_id == net || !on {
                continue;
            }
            let minimum = samples
                .iter()
                .map(|p| {
                    if pad.pad_type == "np_thru_hole" {
                        corridor::distance(*p, pad.point) - pad.drill / 2.0
                    } else {
                        pairs::pad_distance(
                            p[0] - pad.point[0],
                            p[1] - pad.point[1],
                            pad.size[0] / 2.0,
                            pad.size[1] / 2.0,
                            pad.corner_radius,
                        )
                    }
                })
                .fold(f64::INFINITY, f64::min);
            if minimum < bar + step / 2.0 {
                return false;
            }
        }
        for segment in segments {
            if segment.net_id != net
                && segment.layer == layer
                && pairs::poly_dist(points, &[segment.a, segment.b])
                    < bar + segment.width / 2.0 - 1e-9
            {
                return false;
            }
        }
        for via in vias {
            if via.net_id != net
                && points
                    .windows(2)
                    .map(|s| corridor::point_segment_distance(via.point, s[0], s[1]))
                    .fold(f64::INFINITY, f64::min)
                    < bar + via.diameter / 2.0 - 1e-9
            {
                return false;
            }
        }
        for segment in &self.legs {
            if segment.lane != lane
                && segment.layer == layer
                && pairs::poly_dist(points, &[segment.a, segment.b])
                    < cfg.track_width + cfg.clearance + cfg.grid_step / 2.0 - 1e-9
            {
                return false;
            }
        }
        for segment in &self.placed {
            if segment.lane != lane
                && !self.pair(&segment.lane)
                && segment.layer == layer
                && pairs::poly_dist(points, &[segment.a, segment.b])
                    < cfg.track_width + cfg.clearance - 1e-9
            {
                return false;
            }
        }
        for via in &self.vias {
            if via.lane != lane
                && points
                    .windows(2)
                    .map(|s| corridor::point_segment_distance(via.point, s[0], s[1]))
                    .fold(f64::INFINITY, f64::min)
                    < cfg.via_size / 2.0 + cfg.track_width / 2.0 + cfg.clearance - 1e-9
            {
                return false;
            }
        }
        true
    }
    fn barrel_clear(&self, point: Point, net: i32, lane: &str, center: Point, radius: f64) -> bool {
        let cfg = &self.ctx.cfg;
        let (pads, segments, vias) = self.near_static(center, radius);
        for pad in pads {
            if pad.net_id == net {
                continue;
            }
            if pad.pad_type == "np_thru_hole" {
                if corridor::distance(point, pad.point)
                    < cfg.via_drill / 2.0 + pad.drill / 2.0 + cfg.hole_to_hole_clearance - 1e-9
                {
                    return false;
                }
                continue;
            }
            if pairs::pad_distance(
                point[0] - pad.point[0],
                point[1] - pad.point[1],
                pad.size[0] / 2.0,
                pad.size[1] / 2.0,
                pad.corner_radius,
            ) < cfg.via_size / 2.0 + cfg.clearance - 1e-9
            {
                return false;
            }
        }
        for segment in segments {
            if segment.net_id != net
                && corridor::point_segment_distance(point, segment.a, segment.b)
                    < cfg.via_size / 2.0 + segment.width / 2.0 + cfg.clearance - 1e-9
            {
                return false;
            }
        }
        for via in vias {
            if via.net_id != net
                && corridor::distance(point, via.point)
                    < (cfg.via_size / 2.0 + via.diameter / 2.0 + cfg.clearance)
                        .max(cfg.via_drill / 2.0 + via.drill / 2.0 + cfg.hole_to_hole_clearance)
                        - 1e-9
            {
                return false;
            }
        }
        for segment in &self.legs {
            if segment.lane != lane
                && corridor::point_segment_distance(point, segment.a, segment.b)
                    < cfg.via_size / 2.0
                        + cfg.track_width / 2.0
                        + cfg.clearance
                        + cfg.grid_step / 2.0
                        - 1e-9
            {
                return false;
            }
        }
        for segment in &self.placed {
            if segment.lane != lane
                && !self.pair(&segment.lane)
                && corridor::point_segment_distance(point, segment.a, segment.b)
                    < cfg.via_size / 2.0 + cfg.track_width / 2.0 + cfg.clearance - 1e-9
            {
                return false;
            }
        }
        let via_distance =
            (cfg.via_size + cfg.clearance).max(cfg.via_drill + cfg.hole_to_hole_clearance);
        !self
            .vias
            .iter()
            .any(|v| v.lane != lane && corridor::distance(point, v.point) < via_distance - 1e-9)
    }
    fn pair_end_candidates(
        &self,
        n: &str,
        end: usize,
        w: &Window,
        escape: Point,
    ) -> Result<Vec<EndCandidate>, String> {
        let cfg = &self.ctx.cfg;
        let tips = self.ctx.pair_ends[n][end];
        let names = &self.ctx.pairs[n];
        let ids = [self.ctx.byname[&names[0]], self.ctx.byname[&names[1]]];
        let middle = pairs::mid(tips[0], tips[1]);
        let direction = corridor::unit(escape)?;
        let floor = pairs::handover_setback(cfg);
        let reach = pairs::end_run(cfg, tips, None) + 2.0 * pairs::pitch(cfg);
        let layer = if end == 0 {
            self.smooth[n].layers[0].clone()
        } else {
            self.smooth[n].layers.last().unwrap().clone()
        };
        let mut out = vec![];
        for i in 0..w.ni {
            self.check_budget()?;
            for j in 0..w.nj {
                let idx = w.index(i, j);
                if !w.band[idx] {
                    continue;
                }
                let pose = w.point(i, j, cfg.grid_step);
                let ahead = corridor::dot(corridor::sub(pose, middle), direction);
                if ahead < floor.max(cfg.grid_step)
                    || corridor::distance(pose, middle) > reach
                    || w.bad[&layer][idx]
                {
                    continue;
                }
                for heading in 0usize..8 {
                    let u = corridor::unit([DIRS[heading][0] as f64, DIRS[heading][1] as f64])?;
                    if corridor::dot(u, direction) < -1e-9 {
                        continue;
                    }
                    let q = corridor::sub(pose, corridor::mul(u, floor));
                    let Some(legs) = pairs::end_legs(
                        tips,
                        direction,
                        q,
                        u,
                        self.half(),
                        cfg.track_width + cfg.clearance,
                        cfg.grid_step,
                        reach,
                    ) else {
                        continue;
                    };
                    let full: Vec<_> = legs
                        .iter()
                        .map(|points| {
                            let mut p = points.clone();
                            if floor > 0.0 {
                                let end = *p.last().unwrap();
                                p.push(corridor::add(end, corridor::mul(u, floor)));
                            }
                            p
                        })
                        .collect();
                    let static_radius = reach + 4.0 * cfg.lane_min;
                    if !full.iter().zip(ids).all(|(points, net)| {
                        self.leg_clear(points, &layer, net, n, middle, static_radius)
                    }) {
                        continue;
                    }
                    let length = full
                        .iter()
                        .flat_map(|p| p.windows(2))
                        .map(|s| corridor::distance(s[0], s[1]))
                        .sum();
                    let rank =
                        length + 2.0 * pairs::poly_dist(&[pose, pose], &self.smooth[n].points);
                    out.push(EndCandidate {
                        length,
                        cell: [w.origin[0] + i as i64, w.origin[1] + j as i64],
                        heading,
                        end: PairEnd {
                            layer: layer.clone(),
                            tips,
                            pose,
                            heading: u,
                            handover: [*legs[0].last().unwrap(), *legs[1].last().unwrap()],
                            legs,
                        },
                        rank,
                    });
                }
            }
        }
        out.sort_by(|a, b| {
            a.length
                .total_cmp(&b.length)
                .then(a.cell.cmp(&b.cell))
                .then(a.heading.cmp(&b.heading))
        });
        Ok(out)
    }
    fn crossed(&self, n: &str) -> bool {
        let Some(tips) = self.ctx.pair_ends.get(n) else {
            return false;
        };
        let a = pairs::hand_vector(
            self.ctx.tooth_dir.get(n).copied(),
            tips[0][0],
            tips[0][1],
            false,
        );
        let b = pairs::hand_vector(
            self.ctx.stub_dir.get(n).copied(),
            tips[1][0],
            tips[1][1],
            true,
        );
        a != 0 && b != 0 && a != b
    }
    fn crossover_at(
        &self,
        n: &str,
        w: &Window,
        i: usize,
        j: usize,
        heading: usize,
        hand: i32,
    ) -> Result<Option<pairs::Crossover>, String> {
        let cfg = &self.ctx.cfg;
        let names = &self.ctx.pairs[n];
        let point = w.point(i, j, cfg.grid_step);
        let direction = [DIRS[heading][0] as f64, DIRS[heading][1] as f64];
        let mut best: Option<(usize, pairs::Crossover)> = None;
        for first in ["P", "N"] {
            let Some(cross) = pairs::crossover(
                point,
                direction,
                hand,
                self.half(),
                cfg.via_size,
                self.barrel_offset(n),
                cfg.track_width,
                cfg.clearance,
                cfg.grid_step,
                &self.smooth[n].runs[0],
                &self.smooth[n].runs[1],
                first,
                pairs::handover_setback(cfg),
            ) else {
                continue;
            };
            let radius = 4.0 * cfg.lane_min;
            let legs_clear = cross.legs.iter().all(|(polarity, pieces)| {
                let net = self.ctx.byname[&names[usize::from(polarity == "N")]];
                pieces
                    .iter()
                    .all(|p| self.leg_clear(&p.points, &p.layer, net, n, point, radius))
            });
            let barrels_clear = cross.vias.iter().all(|v| {
                self.barrel_clear(
                    v.point,
                    self.ctx.byname[&names[usize::from(v.polarity == "N")]],
                    n,
                    point,
                    radius,
                )
            });
            if !legs_clear || !barrels_clear {
                continue;
            }
            let room = if let Some(mask) = &w.cross_room {
                let mut room = 0;
                for via in &cross.vias {
                    let cell = [
                        numeric::quantize(via.point[0], cfg.grid_step)? - w.origin[0],
                        numeric::quantize(via.point[1], cfg.grid_step)? - w.origin[1],
                    ];
                    if !w.valid(cell[0], cell[1])
                        || !mask[w.index(cell[0] as usize, cell[1] as usize)]
                    {
                        room += 1
                    }
                }
                room
            } else {
                0
            };
            if best.as_ref().is_none_or(|(old, _)| room > *old) {
                best = Some((room, cross));
            }
        }
        Ok(best.map(|(_, cross)| cross))
    }
}

impl<'a, G: JointGeometry + ?Sized> Snapper<'a, G> {
    fn terminal_cell(
        &self,
        n: &str,
        w: &Window,
        p: Point,
        e: Point,
        way: Point,
        start: bool,
        layer: &str,
    ) -> Result<[i64; 2], String> {
        let g = self.ctx.cfg.grid_step;
        let free = |i: i64, j: i64| {
            let c = [i - w.origin[0], j - w.origin[1]];
            if !w.valid(c[0], c[1]) || w.bad[layer][w.index(c[0] as usize, c[1] as usize)] {
                return false;
            }
            let q = [i as f64 * g, j as f64 * g];
            on_grid(p, q, g)
                || self.vias.iter().filter(|v| v.lane != n).all(|v| {
                    let rounded = [
                        numeric::quantize(v.point[0], g).unwrap_or(0) as f64 * g,
                        numeric::quantize(v.point[1], g).unwrap_or(0) as f64 * g,
                    ];
                    corridor::point_segment_distance(v.point, p, q)
                        >= self.ring(n) + corridor::distance(v.point, rounded) + g / 2. - 1e-9
                })
        };
        let nearest = [numeric::quantize(p[0], g)?, numeric::quantize(p[1], g)?];
        if !self.pair(n) && free(nearest[0], nearest[1]) {
            return Ok(nearest);
        }
        let mut best: Option<((bool, bool, bool, f64), [i64; 2])> = None;
        for i in (p[0] / g).floor() as i64 - 2..=(p[0] / g).ceil() as i64 + 2 {
            for j in (p[1] / g).floor() as i64 - 2..=(p[1] / g).ceil() as i64 + 2 {
                let q = [i as f64 * g, j as f64 * g];
                let mut v = corridor::sub(q, p);
                if !start {
                    v = corridor::mul(v, -1.)
                }
                let len = corridor::length(v);
                let dot = corridor::dot(v, e);
                if len > 1e-9
                    && dot < len * std::f64::consts::FRAC_1_SQRT_2 - 1e-9
                    && (self.pair(n) || dot < -1e-9)
                {
                    continue;
                }
                let closer = self
                    .smooth
                    .iter()
                    .filter(|(m, _)| m.as_str() != n)
                    .any(|(m, s)| {
                        [0, s.points.len() - 1].into_iter().any(|k| {
                            s.layers[k.min(s.layers.len() - 1)] == layer
                                && corridor::distance(q, s.points[k])
                                    < (self.ctx.cfg.track_width
                                        + self.ctx.cfg.clearance
                                        + self.snap_half(n)
                                        + self.snap_half(m)
                                        + g)
                                        .min(corridor::distance(p, s.points[k]))
                                        - 1e-9
                        })
                    });
                let key = (
                    !free(i, j),
                    closer,
                    len > 1e-9 && corridor::dot(v, way) < -1e-9,
                    len,
                );
                if best
                    .as_ref()
                    .is_none_or(|(old, _)| key.partial_cmp(old) == Some(Ordering::Less))
                {
                    best = Some((key, [i, j]))
                }
            }
        }
        best.map(|(_, q)| q)
            .ok_or_else(|| format!("{n}: no terminal grid direction"))
    }
    fn turned_over(&self, path: &[State], w: &Window) -> Option<Point> {
        let moves: Vec<_> = path
            .windows(2)
            .map(|s| {
                if s[0].cell == s[1].cell {
                    None
                } else {
                    Some((s[1].heading as i32 - s[0].heading as i32 + 4).rem_euclid(8) - 4)
                }
            })
            .collect();
        pairs::pose_turn_over(&moves, pairs::pose_turn_units(&self.ctx.cfg)).map(|k| {
            w.point(
                path[k + 1].cell[0],
                path[k + 1].cell[1],
                self.ctx.cfg.grid_step,
            )
        })
    }
    fn route(&mut self, n: &str, strict: bool) -> Result<Attempt, String> {
        self.check_budget()?;
        let w = self.build_window(n)?;
        let s = self.smooth[n].clone();
        let cfg = &self.ctx.cfg;
        let g = cfg.grid_step;
        let p = &s.points;
        let [outward, inward] = self.end_dirs(n)?;
        let mut arcs = vec![0.];
        for q in p.windows(2) {
            arcs.push(arcs.last().copied().unwrap() + corridor::distance(q[0], q[1]))
        }
        let total = *arcs.last().unwrap();
        let scale = cfg.track_width + cfg.clearance;
        let k0 = arcs.partition_point(|x| *x < scale).min(p.len() - 1);
        let k1 = arcs
            .partition_point(|x| *x < total - scale)
            .saturating_sub(1);
        let a = self.terminal_cell(
            n,
            &w,
            p[0],
            outward,
            corridor::sub(p[k0], p[0]),
            true,
            &s.layers[0],
        )?;
        let b = self.terminal_cell(
            n,
            &w,
            *p.last().unwrap(),
            inward,
            corridor::sub(*p.last().unwrap(), p[k1]),
            false,
            s.layers.last().unwrap(),
        )?;
        let a = [a[0] - w.origin[0], a[1] - w.origin[1]];
        let b = [b[0] - w.origin[0], b[1] - w.origin[1]];
        if !w.valid(a[0], a[1]) || !w.valid(b[0], b[1]) {
            return Ok(Attempt {
                lane: None,
                reason: "terminal outside the window".into(),
                stuck: None,
                stuck_layer: None,
            });
        }
        let mut join_start = corridor::sub(w.point(a[0] as usize, a[1] as usize, g), p[0]);
        let mut join_end =
            corridor::sub(*p.last().unwrap(), w.point(b[0] as usize, b[1] as usize, g));
        if corridor::length(join_start) <= g / std::f64::consts::SQRT_2 + 1e-9 {
            join_start = outward
        }
        if corridor::length(join_end) <= g / std::f64::consts::SQRT_2 + 1e-9 {
            join_end = inward
        }
        let varc: Vec<_> = s
            .vias
            .iter()
            .map(|v| {
                let k = p
                    .iter()
                    .enumerate()
                    .min_by(|(_, a), (_, b)| {
                        corridor::distance(**a, *v).total_cmp(&corridor::distance(**b, *v))
                    })
                    .unwrap()
                    .0;
                arcs[k]
            })
            .collect();
        let paired = self.pair(n);
        let crossed = paired && self.crossed(n);
        let straight = if paired {
            pairs::via_straight_steps(cfg) as usize
        } else {
            0
        };
        let probe = if paired {
            pairs::pose_probe_steps(cfg) as usize
        } else {
            0
        };
        let mut cap = straight.max(probe);
        let mut cross_steps = [[0; 2]; 8];
        let hand = if crossed {
            let tips = self.ctx.pair_ends[n][0];
            pairs::hand_vector(self.ctx.tooth_dir.get(n).copied(), tips[0], tips[1], false)
        } else {
            0
        };
        if crossed {
            for (d, steps) in cross_steps.iter_mut().enumerate() {
                let u = [DIRS[d][0] as f64, DIRS[d][1] as f64];
                let c = pairs::crossover(
                    [0., 0.],
                    u,
                    hand,
                    self.half(),
                    cfg.via_size,
                    self.barrel_offset(n),
                    cfg.track_width,
                    cfg.clearance,
                    g,
                    "F.Cu",
                    "B.Cu",
                    "P",
                    pairs::handover_setback(cfg),
                )
                .ok_or("snap crossover shape unavailable")?;
                let step = g * corridor::length(u);
                *steps = [
                    numeric::checked_round_i64(-c.span[0] / step)? as usize + probe,
                    numeric::checked_round_i64(c.span[1] / step)? as usize + probe,
                ];
                cap = cap.max(steps[0])
            }
        }
        let mut input = SearchInput {
            window: &w,
            grid: g,
            track: cfg.track_width,
            lane_pitch: cfg.lane_min,
            runs: &s.runs,
            via_points: &s.vias,
            gates: (0..=s.vias.len())
                .map(|k| {
                    [
                        if k == 0 {
                            f64::NEG_INFINITY
                        } else {
                            varc[k - 1] - 3. * cfg.lane_min
                        },
                        if k == s.vias.len() {
                            f64::INFINITY
                        } else {
                            varc[k] + 3. * cfg.lane_min
                        },
                    ]
                })
                .collect(),
            room_start: [0.; 8],
            room_end: [0.; 8],
            total,
            start: [a[0] as usize, a[1] as usize],
            end: [b[0] as usize, b[1] as usize],
            start_heading: direction_index(outward)?,
            end_heading: direction_index(inward)?,
            join_start,
            join_end,
            outward,
            inward,
            paired,
            crossed,
            straight,
            turn_straight: if paired {
                pairs::turn_straight_steps(cfg) as usize
            } else {
                0
            },
            cap,
            probe,
            cross_steps,
            poses: paired,
            strict,
            keep_weight: self.keep_weight,
            deadline: self.deadline,
        };
        if paired {
            for d in 0..8 {
                let u = corridor::unit([DIRS[d][0] as f64, DIRS[d][1] as f64])?;
                input.room_start[d] = pairs::dive_room(cfg, self.ctx.pair_ends[n][0], Some(u));
                input.room_end[d] = pairs::dive_room(cfg, self.ctx.pair_ends[n][1], Some(u));
            }
        }
        let mut xo: BTreeMap<(usize, usize, usize), Option<pairs::Crossover>> = BTreeMap::new();
        let mut callback = |i, j, d| {
            let key = (i, j, d);
            if !xo.contains_key(&key) {
                xo.insert(key, self.crossover_at(n, &w, i, j, d, hand)?);
            }
            Ok(xo[&key].is_some())
        };
        let mut ends = vec![];
        let mut stuck = None;
        let mut stuck_layer = None;
        let mut popped = 0;
        let mut path = None;
        if paired {
            let mut c0 = self.pair_end_candidates(n, 0, &w, outward)?;
            let mut c1 = self.pair_end_candidates(n, 1, &w, corridor::mul(inward, -1.))?;
            c0.sort_by(|a, b| a.rank.total_cmp(&b.rank));
            c1.sort_by(|a, b| a.rank.total_cmp(&b.rank));
            c0.truncate(12);
            c1.truncate(12);
            let mut combos: Vec<_> = c0
                .iter()
                .enumerate()
                .flat_map(|(i, a)| {
                    c1.iter()
                        .enumerate()
                        .map(move |(j, b)| (a.rank + b.rank, i, j))
                })
                .collect();
            combos.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)).then(a.2.cmp(&b.2)));
            for (tried, (_, i, j)) in combos.iter().enumerate() {
                input.start = [
                    (c0[*i].cell[0] - w.origin[0]) as usize,
                    (c0[*i].cell[1] - w.origin[1]) as usize,
                ];
                input.end = [
                    (c1[*j].cell[0] - w.origin[0]) as usize,
                    (c1[*j].cell[1] - w.origin[1]) as usize,
                ];
                input.start_heading = c0[*i].heading;
                input.end_heading = (c1[*j].heading + 4) % 8;
                let r = search(&input, None, &mut callback)?;
                popped = r.popped;
                if let Some(q) = r.path {
                    if let Some(at) = self.turned_over(&q, &w) {
                        stuck = Some(at)
                    } else {
                        path = Some(q);
                        ends = vec![c0[*i].end.clone(), c1[*j].end.clone()];
                        break;
                    }
                }
                if tried + 1 == 4 {
                    let starts: Vec<_> = c0
                        .iter()
                        .map(|c| {
                            (
                                [
                                    (c.cell[0] - w.origin[0]) as usize,
                                    (c.cell[1] - w.origin[1]) as usize,
                                ],
                                c.heading,
                            )
                        })
                        .collect();
                    let mut goals: BTreeMap<_, BTreeSet<_>> = BTreeMap::new();
                    for c in &c1 {
                        goals
                            .entry([
                                (c.cell[0] - w.origin[0]) as usize,
                                (c.cell[1] - w.origin[1]) as usize,
                            ])
                            .or_default()
                            .insert((c.heading + 4) % 8);
                    }
                    let r = search(&input, Some((&starts, &goals)), &mut callback)?;
                    if let Some(q) = r.path {
                        if let Some(at) = self.turned_over(&q, &w) {
                            stuck = Some(at);
                            break;
                        }
                    } else {
                        if let Some((_, q)) = r.farthest {
                            stuck = Some(w.point(q.cell[0], q.cell[1], g));
                            stuck_layer = Some(s.runs[q.via].clone());
                        }
                        break;
                    }
                }
            }
        } else {
            let r = search(&input, None, &mut callback)?;
            popped = r.popped;
            path = r.path;
            if path.is_none() {
                let starts = [(input.start, input.start_heading)];
                let goals = BTreeMap::from([(input.end, (0..8).collect())]);
                let r = search(&input, Some((&starts, &goals)), &mut callback)?;
                if r.path.is_none() {
                    if let Some((_, q)) = r.farthest {
                        stuck = Some(w.point(q.cell[0], q.cell[1], g));
                        stuck_layer = Some(s.runs[q.via].clone());
                    }
                }
            }
        }
        let Some(path) = path else {
            return Ok(Attempt {
                lane: None,
                reason: format!("no path in its band ({popped} states searched)"),
                stuck,
                stuck_layer,
            });
        };
        let cross = if crossed {
            path.windows(2)
                .find(|q| q[0].cell == q[1].cell && q[1].via == q[0].via + 1)
                .and_then(|q| {
                    xo.get(&(q[0].cell[0], q[0].cell[1], q[0].heading))
                        .cloned()
                        .flatten()
                })
        } else {
            None
        };
        let pts: Vec<_> = path
            .iter()
            .map(|q| (w.point(q.cell[0], q.cell[1], g), q.via))
            .collect();
        let mut vias = vec![];
        for q in pts.windows(2) {
            if q[0].0 == q[1].0 && q[0].1 != q[1].1 {
                vias.push(q[1].0)
            }
        }
        let mut pieces = vec![LayerPiece {
            a: p[0],
            b: pts[0].0,
            layer: s.runs[0].clone(),
        }];
        for q in pts.windows(2) {
            if q[0].0 != q[1].0 {
                pieces.push(LayerPiece {
                    a: q[0].0,
                    b: q[1].0,
                    layer: s.runs[q[1].1].clone(),
                })
            }
        }
        pieces.push(LayerPiece {
            a: pts.last().unwrap().0,
            b: *p.last().unwrap(),
            layer: s.runs.last().unwrap().clone(),
        });
        if !paired {
            if pieces.len() > 2
                && corridor::distance(p[0], pts[0].0) <= g / std::f64::consts::SQRT_2 + 1e-9
                && pieces[1].a == pts[0].0
                && pieces[1].layer == s.runs[0]
            {
                let b = pieces[1].b;
                pieces.splice(
                    0..2,
                    [LayerPiece {
                        a: p[0],
                        b,
                        layer: s.runs[0].clone(),
                    }],
                );
            }
            let k = pieces.len();
            if k > 2
                && corridor::distance(*p.last().unwrap(), pts.last().unwrap().0)
                    <= g / std::f64::consts::SQRT_2 + 1e-9
                && pieces[k - 2].b == pts.last().unwrap().0
                && pieces[k - 2].layer == *s.runs.last().unwrap()
            {
                let a = pieces[k - 2].a;
                pieces.splice(
                    k - 2..k,
                    [LayerPiece {
                        a,
                        b: *p.last().unwrap(),
                        layer: s.runs.last().unwrap().clone(),
                    }],
                );
            }
        }
        let mut merged: Vec<LayerPiece> = vec![];
        for (k, q) in pieces.iter().enumerate() {
            if corridor::distance(q.a, q.b) < 1e-12 {
                continue;
            }
            if merged.len() > 1 && k < pieces.len() - 1 {
                let old = merged.last_mut().unwrap();
                let v = corridor::sub(old.b, old.a);
                let delta = corridor::sub(q.b, old.a);
                if old.layer == q.layer
                    && old.b == q.a
                    && (v[0] * delta[1] - v[1] * delta[0]).abs() < 1e-12
                    && corridor::dot(v, corridor::sub(q.b, old.b)) > 0.
                {
                    old.b = q.b;
                    continue;
                }
            }
            merged.push(q.clone())
        }
        let mut lane = LaneGeometry {
            xy: vec![],
            layers: vec![],
            vias,
            uxy: vec![],
            paired_legs: BTreeMap::new(),
            paired_vias: BTreeMap::new(),
            snapped: true,
            ends,
            cross,
        };
        for q in merged {
            if lane.xy.is_empty() {
                lane.xy.push(q.a)
            }
            lane.xy.push(q.b);
            lane.layers.push(q.layer)
        }
        Ok(Attempt {
            lane: Some(lane),
            reason: format!("{popped} states"),
            stuck: None,
            stuck_layer: None,
        })
    }
}

fn lane_pieces(lane: &LaneGeometry) -> Vec<LayerPiece> {
    lane.xy
        .windows(2)
        .zip(&lane.layers)
        .map(|(p, l)| LayerPiece {
            a: p[0],
            b: p[1],
            layer: l.clone(),
        })
        .collect()
}
fn cut_span(pieces: &[LayerPiece], entry: Point, exit: Point) -> [Vec<LayerPiece>; 2] {
    let mut before = vec![];
    let mut after = vec![];
    let mut state = 0;
    for q in pieces {
        if state == 0 {
            if corridor::point_segment_distance(entry, q.a, q.b) < 1e-6 {
                if corridor::distance(entry, q.a) > 1e-9 {
                    before.push(LayerPiece {
                        a: q.a,
                        b: entry,
                        layer: q.layer.clone(),
                    })
                }
                state = 1;
                if corridor::point_segment_distance(exit, q.a, q.b) < 1e-6 {
                    if corridor::distance(q.b, exit) > 1e-9 {
                        after.push(LayerPiece {
                            a: exit,
                            b: q.b,
                            layer: q.layer.clone(),
                        })
                    }
                    state = 2
                }
                continue;
            }
            before.push(q.clone())
        } else if state == 1 {
            if corridor::point_segment_distance(exit, q.a, q.b) < 1e-6 {
                if corridor::distance(q.b, exit) > 1e-9 {
                    after.push(LayerPiece {
                        a: exit,
                        b: q.b,
                        layer: q.layer.clone(),
                    })
                }
                state = 2
            }
        } else {
            after.push(q.clone())
        }
    }
    [before, after]
}
fn chain_runs(pieces: &[LayerPiece], layer: &str) -> Vec<Vec<Point>> {
    let mut runs: Vec<Vec<Point>> = vec![];
    for q in pieces.iter().filter(|q| q.layer == layer) {
        let mut added = false;
        for r in &mut runs {
            if corridor::distance(q.a, *r.last().unwrap()) < 1e-4 {
                r.push(q.b);
                added = true;
                break;
            }
            if corridor::distance(q.b, r[0]) < 1e-4 {
                r.insert(0, q.a);
                added = true;
                break;
            }
        }
        if !added {
            runs.push(vec![q.a, q.b])
        }
    }
    loop {
        let mut join = None;
        for (i, a) in runs.iter().enumerate() {
            for (j, b) in runs.iter().enumerate() {
                if i != j && corridor::distance(*a.last().unwrap(), b[0]) < 1e-4 {
                    join = Some((i, j));
                    break;
                }
            }
            if join.is_some() {
                break;
            }
        }
        let Some((i, j)) = join else { break };
        let b = runs[j][1..].to_vec();
        runs[i].extend(b);
        runs.remove(j);
    }
    runs
}
fn simplify(run: &[Point]) -> Vec<Point> {
    if run.len() < 3 {
        return run.to_vec();
    }
    let mut out = vec![run[0]];
    for (i, p) in run.iter().enumerate().skip(1) {
        if corridor::distance(*p, *out.last().unwrap()) < 0.02 {
            if i == run.len() - 1 && out.len() == 1 {
                out[0] = *p
            }
            continue;
        }
        out.push(*p)
    }
    if corridor::distance(*run.last().unwrap(), *out.last().unwrap()) > 1e-9 {
        *out.last_mut().unwrap() = *run.last().unwrap()
    }
    let mut res = vec![out[0]];
    for i in 1..out.len().saturating_sub(1) {
        let a = corridor::unit(corridor::sub(out[i], *res.last().unwrap()));
        let b = corridor::unit(corridor::sub(out[i + 1], out[i]));
        if let (Ok(a), Ok(b)) = (a, b) {
            if corridor::cross(a, b).abs() < 1e-6 && corridor::dot(a, b) > 0. {
                continue;
            }
        }
        res.push(out[i])
    }
    if out.len() > 1 {
        res.push(*out.last().unwrap())
    }
    res
}
fn offset_side(run: &[Point], h: f64) -> Vec<Point> {
    let mut pts = run.to_vec();
    loop {
        let off = pairs::offset_polyline(&pts, h);
        if pts.len() <= 2 {
            return off;
        }
        let back = (0..pts.len() - 1).find(|i| {
            corridor::dot(
                corridor::sub(pts[i + 1], pts[*i]),
                corridor::sub(off[i + 1], off[*i]),
            ) <= 0.
        });
        let Some(i) = back else { return off };
        let k = if i + 1 < pts.len() - 1 { i + 1 } else { i };
        pts.remove(k);
    }
}
impl<'a, G: JointGeometry + ?Sized> Snapper<'a, G> {
    fn place(&mut self, n: &str, mut lane: LaneGeometry) -> Result<(), String> {
        let pieces = lane_pieces(&lane);
        if pieces.is_empty() {
            return Err(format!("snap {n}: no pieces to place"));
        }
        let body = if lane.ends.is_empty() {
            pieces.as_slice()
        } else {
            if pieces.len() < 2 {
                return Err(format!("snap {n}: end connectors without body"));
            }
            &pieces[1..pieces.len() - 1]
        };
        self.placed.extend(body.iter().map(|q| PlacedSegment {
            lane: n.into(),
            layer: q.layer.clone(),
            a: q.a,
            b: q.b,
            exact: false,
        }));
        if self.pair(n) {
            let names = self.ctx.pairs[n].clone();
            lane.paired_legs.clear();
            lane.paired_vias.clear();
            for name in &names {
                lane.paired_legs.insert(name.clone(), vec![]);
                lane.paired_vias.insert(name.clone(), vec![]);
            }
            let tips = self.ctx.pair_ends[n][0];
            let hand =
                pairs::hand_vector(self.ctx.tooth_dir.get(n).copied(), tips[0], tips[1], false);
            if hand == 0 {
                return Err(format!("snap {n}: unknown physical pair hand"));
            }
            let parts = lane
                .cross
                .as_ref()
                .map(|c| {
                    cut_span(body, c.poses[0], c.poses[1])
                        .into_iter()
                        .collect::<Vec<_>>()
                })
                .unwrap_or_else(|| vec![body.to_vec()]);
            for (partno, part) in parts.iter().enumerate() {
                let mut layers = vec!["F.Cu".to_string(), "B.Cu".to_string()];
                layers.extend(
                    part.iter()
                        .map(|q| q.layer.clone())
                        .filter(|l| l != "F.Cu" && l != "B.Cu")
                        .collect::<BTreeSet<_>>(),
                );
                for l in layers {
                    for run in chain_runs(part, &l) {
                        let run = simplify(&run);
                        if run.len() < 2 {
                            continue;
                        }
                        for leg in 0..2 {
                            let side = hand as f64
                                * if leg == 0 { 1. } else { -1. }
                                * if partno == 1 { -1. } else { 1. };
                            let off = offset_side(&run, side * self.half());
                            for q in off.windows(2) {
                                lane.paired_legs
                                    .get_mut(&names[leg])
                                    .unwrap()
                                    .push(LayerPiece {
                                        a: q[0],
                                        b: q[1],
                                        layer: l.clone(),
                                    });
                                self.legs.push(PlacedSegment {
                                    lane: n.into(),
                                    layer: l.clone(),
                                    a: q[0],
                                    b: q[1],
                                    exact: false,
                                })
                            }
                        }
                    }
                }
            }
            for end in &lane.ends {
                for (leg, points) in end.legs.iter().enumerate() {
                    for q in points.windows(2) {
                        lane.paired_legs
                            .get_mut(&names[leg])
                            .unwrap()
                            .push(LayerPiece {
                                a: q[0],
                                b: q[1],
                                layer: end.layer.clone(),
                            });
                        self.legs.push(PlacedSegment {
                            lane: n.into(),
                            layer: end.layer.clone(),
                            a: q[0],
                            b: q[1],
                            exact: true,
                        })
                    }
                }
            }
            if let Some(c) = &lane.cross {
                for (leg, polarity) in ["P", "N"].iter().enumerate() {
                    if let Some(parts) = c.legs.get(*polarity) {
                        for part in parts {
                            for q in part.points.windows(2) {
                                lane.paired_legs
                                    .get_mut(&names[leg])
                                    .unwrap()
                                    .push(LayerPiece {
                                        a: q[0],
                                        b: q[1],
                                        layer: part.layer.clone(),
                                    });
                                self.legs.push(PlacedSegment {
                                    lane: n.into(),
                                    layer: part.layer.clone(),
                                    a: q[0],
                                    b: q[1],
                                    exact: true,
                                })
                            }
                        }
                    }
                }
                for b in &c.vias {
                    let leg = match b.polarity.as_str() {
                        "P" => 0,
                        "N" => 1,
                        _ => return Err("unknown crossover barrel polarity".into()),
                    };
                    lane.paired_vias.get_mut(&names[leg]).unwrap().push(b.point);
                    self.vias.push(PlacedVia {
                        lane: n.into(),
                        point: b.point,
                        exact: true,
                    })
                }
            }
            for v in &lane.vias {
                if lane
                    .cross
                    .as_ref()
                    .is_some_and(|c| corridor::distance(*v, c.at) <= 1e-6)
                {
                    continue;
                }
                let barrels = pairs::dive_barrels(*v, &pieces, self.barrel_offset(n));
                if barrels.len() != 2 {
                    return Err(format!("snap {n}: missing actual dive heading"));
                }
                let after = lane.cross.as_ref().is_some_and(|c| {
                    pieces
                        .iter()
                        .position(|q| corridor::distance(q.b, *v) < 1e-6)
                        .unwrap_or(0)
                        > pieces
                            .iter()
                            .position(|q| corridor::point_segment_distance(c.at, q.a, q.b) < 1e-6)
                            .unwrap_or(usize::MAX)
                });
                let sign = hand * if after { -1 } else { 1 };
                for (k, b) in barrels.iter().enumerate() {
                    let leg = if sign > 0 { k } else { 1 - k };
                    lane.paired_vias.get_mut(&names[leg]).unwrap().push(*b);
                    self.vias.push(PlacedVia {
                        lane: n.into(),
                        point: *b,
                        exact: false,
                    })
                }
                if let Some(q) = pieces.iter().find(|q| {
                    corridor::distance(q.b, *v) < 1e-6 && corridor::distance(q.a, q.b) > 1e-9
                }) {
                    let hk = direction_index(corridor::sub(q.b, q.a))?;
                    let sp = pairs::pose_via_cells(&self.ctx.cfg, self.half()) as f64
                        * self.ctx.cfg.grid_step;
                    let perp = [-DIRS[hk][1] as f64 * sp, DIRS[hk][0] as f64 * sp];
                    for sg in [0., 1., -1.] {
                        self.dive_tests
                            .push((n.into(), corridor::add(*v, corridor::mul(perp, sg))))
                    }
                }
            }
        } else {
            for v in &lane.vias {
                self.vias.push(PlacedVia {
                    lane: n.into(),
                    point: *v,
                    exact: false,
                })
            }
        }
        lane.uxy = self
            .input
            .lanes
            .get(n)
            .map(|g| g.uxy.clone())
            .unwrap_or_default();
        self.result.lanes.insert(n.into(), lane);
        Ok(())
    }
    fn lift(&mut self, n: &str) {
        self.placed.retain(|q| q.lane != n);
        self.legs.retain(|q| q.lane != n);
        self.vias.retain(|q| q.lane != n);
        self.dive_tests.retain(|(m, _)| m != n);
        self.result.lanes.remove(n);
    }
    fn route_memo(&mut self, n: &str, strict: bool) -> Result<Attempt, String> {
        let board = serde_json::to_string(&self.result.lanes).map_err(|e| e.to_string())?;
        let key = format!("{n}:{}:{board}", strict || self.pair(n));
        if let Some(a) = self.memo.get(&key) {
            return Ok(a.clone());
        }
        let a = self.route(n, strict)?;
        self.memo.insert(key, a.clone());
        Ok(a)
    }
    fn attempt(&mut self, n: &str) -> Result<(Attempt, bool), String> {
        let a = self.route_memo(n, true)?;
        if a.lane.is_some() {
            return Ok((a, false));
        }
        Ok((self.route_memo(n, false)?, true))
    }
}

pub fn snap<G: JointGeometry + ?Sized>(
    ctx: &WholeContext,
    frame: &WholeFrame,
    geometry: &GeometryResult,
    oracle: &G,
    pairs_only: bool,
    keep: Option<&super::whole_types::HistoryAudit>,
    budget_ms: u64,
) -> Result<GeometryResult, String> {
    if !geometry.metadata_complete {
        return Err("snap requires complete native geometry metadata".into());
    }
    let deadline = Instant::now()
        .checked_add(Duration::from_millis(budget_ms))
        .ok_or("snap deadline overflow")?;
    let smooth = Snapper::<G>::make_smooth(ctx, geometry)?;
    let order: Vec<_> = frame
        .m
        .iter()
        .filter(|n| smooth.contains_key(*n) && ctx.pairs.contains_key(*n))
        .chain(
            frame
                .m
                .iter()
                .filter(|n| smooth.contains_key(*n) && !ctx.pairs.contains_key(*n)),
        )
        .cloned()
        .collect();
    if order.len() != smooth.len() {
        return Err("snap lane/frame membership differs".into());
    }
    let mut result = geometry.clone();
    result.lanes.clear();
    result.conflicts.clear();
    result.folded.clear();
    result.failed = false;
    let mut snap = Snapper {
        ctx,
        frame,
        oracle,
        input: geometry,
        smooth,
        order: order.clone(),
        result,
        placed: vec![],
        legs: vec![],
        vias: vec![],
        dive_tests: vec![],
        memo: HashMap::new(),
        static_cache: BTreeMap::new(),
        pairs_only,
        keep: keep
            .map(|k| k.hot.iter().map(|h| h.point).collect())
            .unwrap_or_default(),
        deadline,
        keep_weight: 20.,
    };
    for n in order.iter().filter(|n| geometry.held.contains(*n)) {
        snap.place(n, geometry.lanes[n].clone())?
    }
    let lay: Vec<_> = order
        .iter()
        .filter(|n| !geometry.held.contains(*n) && (ctx.pairs.contains_key(*n) || !pairs_only))
        .cloned()
        .collect();
    let mut failed: BTreeMap<String, Attempt> = BTreeMap::new();
    let mut folded = BTreeSet::new();
    for n in &lay {
        let (a, fold) = snap.attempt(n)?;
        if let Some(lane) = a.lane {
            if fold {
                folded.insert(n.clone());
            }
            snap.place(n, lane)?
        } else {
            failed.insert(n.clone(), a);
        }
    }
    for _ in 0..2 {
        let mut relaid = false;
        for n in lay
            .iter()
            .filter(|n| failed.contains_key(*n))
            .cloned()
            .collect::<Vec<_>>()
        {
            let (a, fold) = snap.attempt(&n)?;
            if let Some(lane) = a.lane {
                if fold {
                    folded.insert(n.clone());
                }
                snap.place(&n, lane)?;
                failed.remove(&n);
                relaid = true
            } else {
                failed.insert(n, a);
            }
        }
        let nb0: usize = snap.result.lanes.values().map(|l| l.layers.len()).sum();
        for n in &lay {
            if failed.contains_key(n) {
                continue;
            }
            let Some(old) = snap.result.lanes.get(n).cloned() else {
                continue;
            };
            snap.lift(n);
            let a = snap.route_memo(n, !folded.contains(n))?;
            snap.place(n, a.lane.unwrap_or(old))?
        }
        let nb1: usize = snap.result.lanes.values().map(|l| l.layers.len()).sum();
        if nb1 >= nb0 && !relaid {
            break;
        }
    }
    for (n, a) in failed {
        let at = a.stuck.unwrap_or(snap.smooth[&n].points[0]);
        snap.result
            .conflicts
            .push(super::whole_geometry::GeometryConflict {
                frame: "snap".into(),
                xy: at,
                lanes: vec![n.clone()],
                kind: format!("snap: {}", a.reason),
            });
        if let Some(layer) = a
            .stuck_layer
            .as_ref()
            .and_then(|layer| ctx.layers.iter().position(|l| l == layer))
        {
            snap.result
                .cuts
                .lcuts
                .extend(super::feedback::snap_layer_cuts(
                    &n,
                    at,
                    layer,
                    &geometry.island_boxes,
                    ctx.layers.len(),
                    2.,
                ));
        }
        if ctx.pairs.contains_key(&n) {
            let vs = &snap.smooth[&n].vias;
            if let Some(ch) = geometry.changes.get(&n) {
                if !vs.is_empty() && ch.len() == vs.len() {
                    let q = vs
                        .iter()
                        .enumerate()
                        .min_by(|(_, a), (_, b)| {
                            corridor::distance(**a, at).total_cmp(&corridor::distance(**b, at))
                        })
                        .unwrap()
                        .0;
                    snap.result.cuts.vcuts.push(super::whole_types::ViaCut {
                        lane: n.clone(),
                        u: ch[q],
                        w: pairs::jog_room(&ctx.cfg),
                        built: false,
                    })
                }
            }
        }
    }
    if pairs_only {
        for n in &order {
            if !snap.result.lanes.contains_key(n)
                && !snap.result.conflicts.iter().any(|c| c.lanes.contains(n))
            {
                snap.result
                    .lanes
                    .insert(n.clone(), geometry.lanes[n].clone());
            }
        }
        snap.result.held = snap
            .result
            .lanes
            .keys()
            .filter(|n| ctx.pairs.contains_key(*n))
            .cloned()
            .collect();
    }
    snap.result.failed = !snap.result.conflicts.is_empty();
    snap.result.folded = folded.into_iter().collect();
    Ok(snap.result)
}
