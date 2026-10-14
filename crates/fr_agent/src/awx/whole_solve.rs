//! Direct constraint translation of AWX whole_solve.py, fixed caad45cd252f9a50f1fb4b3b15461bdf1b9457a1.
use super::{
    cp_model::{BoolVar, IntVar, Model},
    numeric::{checked_add, quantize, quantize_up, round_digits},
    whole_types::{ViaCut, WholeProblem},
};
use std::collections::{BTreeMap, BTreeSet};
const KMAX: usize = 3;
type CrossingKey = (String, String);
#[derive(Clone, Copy, Debug)]
struct OrderedScalar(f64);
impl PartialEq for OrderedScalar {
    fn eq(&self, other: &Self) -> bool {
        self.0 == other.0
    }
}
impl Eq for OrderedScalar {}
impl PartialOrd for OrderedScalar {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}
impl Ord for OrderedScalar {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.0.total_cmp(&other.0)
    }
}
type CutKey = (String, String, String, OrderedScalar, OrderedScalar);
fn rounded_scalar(value: f64) -> Result<OrderedScalar, String> {
    let value = round_digits(value, 3)?;
    Ok(OrderedScalar(if value == 0.0 { 0.0 } else { value }))
}
fn cut_json(key: &CutKey) -> serde_json::Value {
    if key.2.is_empty() {
        serde_json::json!([key.0, key.1, key.3 .0, key.4 .0])
    } else {
        serde_json::json!([key.0, key.1, key.2, key.3 .0])
    }
}
#[derive(Clone)]
struct LayerSpan {
    lane: String,
    tag: String,
    target: i64,
    blocked: Vec<i64>,
    route: [f64; 2],
    change: [f64; 2],
    dogbone: Option<[f64; 2]>,
}
#[derive(Clone)]
struct Changes {
    positions: Vec<IntVar>,
    active: Vec<BoolVar>,
    before: BTreeMap<CrossingKey, Vec<BoolVar>>,
    layers: Vec<IntVar>,
}
struct RouteDomain {
    grid: f64,
    entry: BTreeMap<String, f64>,
    end: BTreeMap<String, f64>,
    trunk_end: BTreeMap<String, f64>,
    handoff: BTreeMap<String, f64>,
    ring_start: BTreeMap<String, f64>,
    branch: BTreeMap<String, String>,
    tooth_layer: BTreeMap<String, i64>,
    berth_layer: BTreeMap<String, i64>,
    connector_start: BTreeMap<String, f64>,
    connector_end: BTreeMap<String, f64>,
    via_start: BTreeMap<String, f64>,
    via_end: BTreeMap<String, f64>,
    waived_face: BTreeSet<String>,
    via_ends: BTreeSet<(String, usize)>,
    via_bans: BTreeMap<(String, usize), Vec<i64>>,
    dog_windows: BTreeMap<String, Vec<[i64; 2]>>,
    in_windows: BTreeMap<(String, usize), Vec<BoolVar>>,
    separated_ends: Vec<[(String, usize); 2]>,
    tooth_vias: BTreeSet<String>,
    berth_vias: BTreeSet<String>,
    layer_spans: Vec<LayerSpan>,
    crossing_band: f64,
    change_band: f64,
}
struct WholeModel {
    model: Model,
    domain: RouteDomain,
    crossings: Vec<CrossingKey>,
    times: BTreeMap<CrossingKey, IntVar>,
    windows: BTreeMap<CrossingKey, [f64; 2]>,
    mover: BTreeMap<CrossingKey, BoolVar>,
    changes: BTreeMap<String, Changes>,
    totals: BTreeMap<String, Vec<BoolVar>>,
    firm_broken: BTreeMap<CutKey, BoolVar>,
    soft_broken: BTreeMap<CutKey, BoolVar>,
    objective_terms: Vec<(IntVar, i64)>,
    via_cuts: Vec<ViaCut>,
    filed_via_count: usize,
    gate_over: Vec<(serde_json::Value, IntVar)>,
}
fn other_lane<'a>(key: &'a CrossingKey, lane: &str) -> &'a String {
    if key.0 == lane {
        &key.1
    } else {
        &key.0
    }
}
fn prepare_domain(problem: &WholeProblem) -> Result<RouteDomain, String> {
    let frame = &problem.frame;
    let ctx = &problem.context;
    let rules = &problem.rules;
    let grid = rules.lane_pitch / 5.0;
    if !grid.is_finite() || grid <= 0.0 {
        return Err("AWX lane pitch must be positive".into());
    }
    let nl = problem.options.routing_layers.len();
    if nl < 2 {
        return Err("AWX requires at least two routing layers".into());
    }
    let layer = |name: &str| {
        problem
            .options
            .routing_layers
            .iter()
            .position(|l| l == name)
            .map(|i| i as i64)
            .ok_or_else(|| format!("AWX layer {name} is outside routing stack"))
    };
    let mut domain = RouteDomain {
        grid,
        entry: BTreeMap::new(),
        end: BTreeMap::new(),
        trunk_end: BTreeMap::new(),
        handoff: BTreeMap::new(),
        ring_start: BTreeMap::new(),
        branch: BTreeMap::new(),
        tooth_layer: BTreeMap::new(),
        berth_layer: BTreeMap::new(),
        connector_start: BTreeMap::new(),
        connector_end: BTreeMap::new(),
        via_start: BTreeMap::new(),
        via_end: BTreeMap::new(),
        waived_face: BTreeSet::new(),
        via_ends: BTreeSet::new(),
        via_bans: BTreeMap::new(),
        dog_windows: BTreeMap::new(),
        in_windows: BTreeMap::new(),
        separated_ends: vec![],
        tooth_vias: BTreeSet::new(),
        berth_vias: BTreeSet::new(),
        layer_spans: vec![],
        crossing_band: 0.0,
        change_band: 0.0,
    };
    let gg = 2.0 * grid;
    for (branch, ring) in &frame.rings {
        let members: Vec<_> = frame
            .m
            .iter()
            .filter(|n| frame.cls.get(*n) == Some(branch))
            .collect();
        let h = frame.hk[branch];
        let mut origin = f64::NEG_INFINITY;
        for n in members {
            origin = origin.max(ring.project_pt(frame.spine.xy(h, frame.o_h[n]))[0]);
        }
        let key = branch.clone();
        domain
            .ring_start
            .insert(key.clone(), (origin / gg - 1e-9).ceil() * gg);
    }
    for n in &frame.m {
        domain.entry.insert(n.clone(), frame.st[n][0]);
        let mut end = frame.spine.project_pt(frame.land[n])[0];
        let mut trunk_end = end;
        if let Some(branch) = frame.cls.get(n) {
            let name = branch.clone();
            let handoff = frame.hk[branch];
            let origin = domain.ring_start[&name];
            end = handoff + frame.rings[branch].project_pt(frame.land[n])[0] - origin;
            trunk_end = handoff;
            domain.handoff.insert(n.clone(), handoff);
            domain.branch.insert(n.clone(), name);
        }
        domain.end.insert(n.clone(), end);
        domain.trunk_end.insert(n.clone(), trunk_end);
        domain
            .tooth_layer
            .insert(n.clone(), layer(&ctx.tooth_layer[n])?);
        domain
            .berth_layer
            .insert(n.clone(), layer(&ctx.dest_layer[n])?);
        let paired = ctx.pairs.contains_key(n);
        let connectors = if paired {
            *rules
                .pair_end_connector
                .get(n)
                .ok_or("AWX pair connector metadata missing")?
        } else {
            [0.0; 2]
        };
        let dives = if paired {
            *rules
                .pair_dive_room
                .get(n)
                .ok_or("AWX pair dive metadata missing")?
        } else {
            [2.0 * rules.via_need; 2]
        };
        domain.connector_start.insert(n.clone(), connectors[0]);
        domain.connector_end.insert(n.clone(), connectors[1]);
        domain.via_start.insert(n.clone(), dives[0]);
        domain.via_end.insert(n.clone(), dives[1]);
    }
    let source_face = frame
        .spine
        .project_pt([frame.sb[2], (frame.sb[1] + frame.sb[3]) / 2.0])[0];
    domain.change_band = source_face + 2.0 * rules.via_need;
    domain.crossing_band = if nl == 2 { domain.change_band } else { 0.0 };
    prepare_copper_and_fronts(problem, &mut domain)?;
    Ok(domain)
}
fn build_crossing_model(problem: &WholeProblem, domain: RouteDomain) -> Result<WholeModel, String> {
    let frame = &problem.frame;
    let ctx = &problem.context;
    let rules = &problem.rules;
    let nl = problem.options.routing_layers.len();
    let mut model = Model::new();
    let mut crossings = vec![];
    let mut times = BTreeMap::new();
    let mut windows = BTreeMap::new();
    let launch: BTreeMap<_, _> = frame
        .launch
        .iter()
        .enumerate()
        .map(|(i, n)| (n, i))
        .collect();
    let final_order: BTreeMap<_, _> = frame
        .final_order
        .iter()
        .enumerate()
        .map(|(i, n)| (n, i))
        .collect();
    let same = |a: &String, b: &String| {
        domain.branch.get(a).is_some() && domain.branch.get(a) == domain.branch.get(b)
    };
    for ia in 0..frame.launch.len() {
        for ib in ia + 1..frame.launch.len() {
            let a = &frame.launch[ia];
            let b = &frame.launch[ib];
            if final_order[a] < final_order[b] {
                continue;
            }
            let lo = (domain.entry[a] + rules.clearance.max(domain.connector_start[a]))
                .max(domain.entry[b] + rules.clearance.max(domain.connector_start[b]))
                .max(domain.crossing_band);
            let hi = if same(a, b) {
                (domain.end[a] - rules.lane_pitch.max(domain.connector_end[a]))
                    .min(domain.end[b] - rules.lane_pitch.max(domain.connector_end[b]))
            } else {
                (domain.trunk_end[a]
                    - if domain.branch.contains_key(a) {
                        rules.clearance
                    } else {
                        rules.clearance.max(domain.connector_end[a])
                    })
                .min(
                    domain.trunk_end[b]
                        - if domain.branch.contains_key(b) {
                            rules.clearance
                        } else {
                            rules.clearance.max(domain.connector_end[b])
                        },
                )
            };
            let hi = hi.max(lo + domain.grid);
            let key = (a.clone(), b.clone());
            let var = model.int(
                quantize(lo, domain.grid)?,
                quantize(hi, domain.grid)?,
                &format!("t_{a}_{b}"),
            )?;
            crossings.push(key.clone());
            times.insert(key.clone(), var);
            windows.insert(key, [lo, hi]);
        }
    }
    // Python 464–476: the braid relation over every triple in launch order.
    for i in 0..frame.launch.len() {
        for j in i + 1..frame.launch.len() {
            for k in j + 1..frame.launch.len() {
                let a = &frame.launch[i];
                let b = &frame.launch[j];
                let c = &frame.launch[k];
                let ij = times.get(&(a.clone(), b.clone()));
                let ik = times.get(&(a.clone(), c.clone()));
                let jk = times.get(&(b.clone(), c.clone()));
                match (ij, ik, jk) {
                    (Some(&ij), Some(&ik), Some(&jk)) => {
                        let select = model.bool("");
                        model.le(&[(ij, 1), (ik, -1)], -1, &[select])?;
                        model.le(&[(ik, 1), (jk, -1)], -1, &[select])?;
                        model.le(&[(jk, 1), (ik, -1)], -1, &[!select])?;
                        model.le(&[(ik, 1), (ij, -1)], -1, &[!select])?;
                    }
                    (Some(&ij), Some(&ik), None) => model.le(&[(ij, 1), (ik, -1)], -1, &[])?,
                    (None, Some(&ik), Some(&jk)) => model.le(&[(jk, 1), (ik, -1)], -1, &[])?,
                    (Some(_), None, Some(_)) => {
                        return Err(format!("AWX inconsistent triple {a} {b} {c}"));
                    }
                    _ => {}
                }
            }
        }
    }
    // 477–499: every crossing chooses a mover; two-layer intervals are per lane.
    let w_move = quantize_up(rules.lane_pitch / (1.0_f64 + 16.0).sqrt(), domain.grid)?.max(1);
    let w_stay = quantize_up(rules.lane_pitch, domain.grid)?.max(1);
    let mut mover = BTreeMap::new();
    let mut lane_intervals: BTreeMap<String, Vec<i32>> = BTreeMap::new();
    for key in &crossings {
        let mv = model.bool("");
        mover.insert(key.clone(), mv);
        if nl > 2 {
            continue;
        }
        let time = times[key];
        lane_intervals
            .entry(key.0.clone())
            .or_default()
            .push(model.optional_interval(time, w_move, mv)?);
        lane_intervals
            .entry(key.0.clone())
            .or_default()
            .push(model.optional_interval(time, w_stay, !mv)?);
        lane_intervals
            .entry(key.1.clone())
            .or_default()
            .push(model.optional_interval(time, w_move, !mv)?);
        lane_intervals
            .entry(key.1.clone())
            .or_default()
            .push(model.optional_interval(time, w_stay, mv)?);
    }
    for n in &frame.m {
        if let Some(intervals) = lane_intervals.get(n) {
            model.no_overlap(intervals);
        }
    }
    // 500–513: opposite sweeps of a differential pair must have room to turn.
    let zigzag = quantize_up(
        2.0 * rules.pair_turn_straight_steps as f64 * rules.grid_step + rules.pair_pitch,
        domain.grid,
    )?;
    for n in &frame.m {
        if !ctx.pairs.contains_key(n) {
            continue;
        }
        let events: Vec<_> = crossings
            .iter()
            .filter(|k| &k.0 == n || &k.1 == n)
            .collect();
        for i in 0..events.len() {
            for j in i + 1..events.len() {
                if (launch[n] > launch[other_lane(events[i], n)])
                    == (launch[n] > launch[other_lane(events[j], n)])
                {
                    continue;
                }
                let first = model.bool("");
                model.le(
                    &[(times[events[i]], 1), (times[events[j]], -1)],
                    -zigzag,
                    &[first],
                )?;
                model.le(
                    &[(times[events[j]], 1), (times[events[i]], -1)],
                    -zigzag,
                    &[!first],
                )?;
            }
        }
    }
    // 514–580: ordered active layer changes, crossing clearances, and run layers.
    let mut changes = BTreeMap::new();
    let mut totals = BTreeMap::new();
    for n in &frame.m {
        let lo = (domain.entry[n] + domain.via_start[n]).max(if domain.waived_face.contains(n) {
            domain.entry[n] + domain.via_start[n]
        } else {
            domain.change_band
        });
        let mut lo = quantize(lo, domain.grid)?;
        if nl > 2 {
            lo = lo.max(quantize_up(domain.entry[n], domain.grid)?);
        }
        let hi = quantize(domain.end[n] - domain.via_end[n], domain.grid)?;
        let sentinel = checked_add(hi, 1)?;
        lo = lo.min(sentinel);
        let room = 2.0 * rules.via_need
            + if ctx.pairs.contains_key(n) {
                rules.pair_pitch
            } else {
                0.0
            };
        let sep = quantize(room, domain.grid)?;
        let mut positions = vec![];
        let mut active = vec![];
        for k in 0..KMAX {
            positions.push(model.int(lo, sentinel, &format!("c_{n}_{k}"))?);
            active.push(model.bool(""));
        }
        for k in 0..KMAX {
            model.le(&[(positions[k], 1)], hi, &[active[k]])?;
            model.eq(&[(positions[k], 1)], sentinel, &[!active[k]])?;
            if k > 0 {
                model.ge(
                    &[(positions[k], 1), (positions[k - 1], -1)],
                    sep,
                    &[active[k]],
                )?;
                model.implication(active[k], active[k - 1]);
            }
        }
        let mut before = BTreeMap::new();
        for key in &crossings {
            if &key.0 != n && &key.1 != n {
                continue;
            }
            let other = if &key.0 == n { &key.1 } else { &key.0 };
            let wide = room
                + if ctx.pairs.contains_key(other) {
                    rules.pair_pitch
                } else {
                    0.0
                };
            let h_st = quantize_up(1.1 * wide / 2.0, domain.grid)?;
            let h_mv = quantize_up(wide / 2.0, domain.grid)?;
            let stay = if &key.0 == n { !mover[key] } else { mover[key] };
            let mut bits = vec![];
            for k in 0..KMAX {
                let earlier = model.bool("");
                for (h, lit) in [(h_st, stay), (h_mv, !stay)] {
                    model.le(&[(positions[k], 1), (times[key], -1)], -h, &[earlier, lit])?;
                    model.ge(
                        &[(positions[k], 1), (times[key], -1)],
                        h,
                        &[!earlier, active[k], lit],
                    )?;
                }
                model.implication(earlier, active[k]);
                bits.push(earlier);
            }
            before.insert(key.clone(), bits);
        }
        let mut layers = vec![];
        if nl > 2 {
            for k in 0..=KMAX {
                layers.push(model.int(0, nl as i64 - 1, &format!("y_{n}_{k}"))?);
            }
            for (index, required, end) in [
                (0, domain.tooth_layer[n], 0),
                (KMAX, domain.berth_layer[n], 1),
            ] {
                if domain.via_ends.contains(&(n.clone(), end)) {
                    for banned in domain.via_bans.get(&(n.clone(), end)).into_iter().flatten() {
                        model.ne(&[(layers[index], 1)], *banned, &[])?;
                    }
                } else {
                    model.eq(&[(layers[index], 1)], required, &[])?;
                }
            }
            for k in 0..KMAX {
                model.ne(&[(layers[k + 1], 1), (layers[k], -1)], 0, &[active[k]])?;
                model.eq(&[(layers[k + 1], 1), (layers[k], -1)], 0, &[!active[k]])?;
            }
        } else {
            model.xor_with_one(&active, domain.tooth_layer[n] == domain.berth_layer[n])?;
        }
        totals.insert(n.clone(), active.clone());
        changes.insert(
            n.clone(),
            Changes {
                positions,
                active,
                before,
                layers,
            },
        );
    }
    Ok(WholeModel {
        model,
        domain,
        crossings,
        times,
        windows,
        mover,
        changes,
        totals,
        firm_broken: BTreeMap::new(),
        soft_broken: BTreeMap::new(),
        objective_terms: vec![],
        via_cuts: vec![],
        filed_via_count: 0,
        gate_over: vec![],
    })
}
fn opposite_hands(problem: &WholeProblem, lane: &str) -> bool {
    let ctx = &problem.context;
    let Some(tips) = ctx.pair_ends.get(lane) else {
        return false;
    };
    let a = super::pairs::hand_vector(
        ctx.tooth_dir.get(lane).copied(),
        tips[0][0],
        tips[0][1],
        false,
    );
    let b = super::pairs::hand_vector(
        ctx.stub_dir.get(lane).copied(),
        tips[1][0],
        tips[1][1],
        true,
    );
    a != 0 && b != 0 && a != b
}
fn run_at(
    model: &mut Model,
    before: &[BoolVar],
    layers: &[IntVar],
    nl: usize,
) -> Result<IntVar, String> {
    let index = model.int(0, KMAX as i64, "")?;
    let mut terms = vec![(index, 1)];
    terms.extend(before.iter().map(|b| (b.as_int_var(), -1)));
    model.eq(&terms, 0, &[])?;
    let layer = model.int(0, nl as i64 - 1, "")?;
    model.element(index, layers, layer)?;
    Ok(layer)
}
fn via_near(
    model: &mut Model,
    changes: &Changes,
    time: IntVar,
    room: i64,
) -> Result<BoolVar, String> {
    let mut nearby = vec![];
    for (&x, &active) in changes.positions.iter().zip(&changes.active) {
        let low = model.bool("");
        let high = model.bool("");
        let near = model.bool("");
        model.le(&[(x, 1), (time, -1)], room, &[low])?;
        model.ge(&[(x, 1), (time, -1)], checked_add(room, 1)?, &[!low])?;
        model.le(&[(time, 1), (x, -1)], room, &[high])?;
        model.ge(&[(time, 1), (x, -1)], checked_add(room, 1)?, &[!high])?;
        model.and(&[active, low, high], &[near]);
        model.or(&[!active, !low, !high, near], &[]);
        nearby.push(near.as_int_var());
    }
    let result = model.bool("");
    model.max(result.as_int_var(), &nearby)?;
    Ok(result)
}
fn add_spacing_constraints(problem: &WholeProblem, whole: &mut WholeModel) -> Result<(), String> {
    use super::cp_model::Affine;
    let frame = &problem.frame;
    let ctx = &problem.context;
    let rules = &problem.rules;
    let nl = problem.options.routing_layers.len();
    let g = whole.domain.grid;
    // Python 575–580: a crossed differential pair performs a real layer change.
    for n in &frame.m {
        if ctx.pairs.contains_key(n)
            && (nl > 2 || whole.domain.tooth_layer[n] == whole.domain.berth_layer[n])
            && opposite_hands(problem, n)
        {
            whole
                .model
                .eq(&[(whole.changes[n].active[0].as_int_var(), 1)], 1, &[])?;
        }
    }
    let via_distance = (rules.via_size + rules.clearance)
        .max(rules.via_drill + rules.hole_to_hole_clearance)
        + rules.grid_step;
    let stagger = rules.via_size.max(
        (via_distance * via_distance - rules.lane_pitch * rules.lane_pitch)
            .max(0.0)
            .sqrt(),
    );
    let stagger_width = quantize(stagger, g)?.max(1);
    let stacked_width = quantize_up(via_distance, g)?.max(1);
    let mut intervals: BTreeMap<String, BTreeMap<String, Vec<i32>>> = BTreeMap::new();
    let mut stacked: BTreeMap<String, BTreeMap<String, Vec<i32>>> = BTreeMap::new();
    for n in &frame.m {
        let changes = whole.changes[n].clone();
        for (index, (&position, &active)) in
            changes.positions.iter().zip(&changes.active).enumerate()
        {
            let mut present = active;
            if let Some(windows) = whole.domain.dog_windows.get(n) {
                let mut ins = vec![];
                for &[lo, hi] in windows {
                    let low = whole.model.bool("");
                    let high = whole.model.bool("");
                    let inside = whole.model.bool("");
                    whole.model.ge(&[(position, 1)], lo, &[low])?;
                    whole.model.le(
                        &[(position, 1)],
                        lo.checked_sub(1).ok_or("AWX dogbone bound overflow")?,
                        &[!low],
                    )?;
                    whole.model.le(&[(position, 1)], hi, &[high])?;
                    whole
                        .model
                        .ge(&[(position, 1)], checked_add(hi, 1)?, &[!high])?;
                    whole.model.and(&[low, high], &[inside]);
                    whole.model.or(&[!low, !high, inside], &[]);
                    ins.push(inside);
                }
                let staggered = whole.model.bool("");
                whole.model.implication(staggered, active);
                for &inside in &ins {
                    whole.model.implication(staggered, !inside);
                }
                let mut clause = vec![!active, staggered];
                clause.extend(&ins);
                whole.model.or(&clause, &[]);
                whole.domain.in_windows.insert((n.clone(), index), ins);
                present = staggered;
            }
            if let Some(branch) = whole.domain.branch.get(n) {
                let trunk = whole.model.bool("");
                let ring = whole.model.bool("");
                let h = quantize(whole.domain.handoff[n], g)?;
                whole.model.le(&[(position, 1)], h, &[trunk])?;
                whole
                    .model
                    .ge(&[(position, 1)], checked_add(h, 1)?, &[ring])?;
                whole.model.or(&[!trunk, present], &[]);
                whole.model.or(&[!ring, present], &[]);
                whole.model.eq(
                    &[(trunk.as_int_var(), 1), (ring.as_int_var(), 1)],
                    1,
                    &[present],
                )?;
                whole.model.eq(
                    &[(trunk.as_int_var(), 1), (ring.as_int_var(), 1)],
                    0,
                    &[!present],
                )?;
                for (f, p) in [("T".to_string(), trunk), (branch.clone(), ring)] {
                    intervals
                        .entry(f.clone())
                        .or_default()
                        .entry(n.clone())
                        .or_default()
                        .push(whole.model.optional_interval(position, stagger_width, p)?);
                    if nl > 2 {
                        stacked
                            .entry(f)
                            .or_default()
                            .entry(n.clone())
                            .or_default()
                            .push(whole.model.optional_interval(position, stacked_width, p)?);
                    }
                }
            } else {
                intervals
                    .entry("T".into())
                    .or_default()
                    .entry(n.clone())
                    .or_default()
                    .push(
                        whole
                            .model
                            .optional_interval(position, stagger_width, present)?,
                    );
                if nl > 2 {
                    stacked
                        .entry("T".into())
                        .or_default()
                        .entry(n.clone())
                        .or_default()
                        .push(
                            whole
                                .model
                                .optional_interval(position, stacked_width, present)?,
                        );
                }
            }
        }
    }
    let canonical = |a: &String, b: &String| {
        if a < b {
            (a.clone(), b.clone())
        } else {
            (b.clone(), a.clone())
        }
    };
    let crossing_pairs: BTreeSet<_> = whole
        .crossings
        .iter()
        .map(|(a, b)| canonical(a, b))
        .collect();
    let mut neighbors = crossing_pairs.clone();
    for order in [&frame.launch, &frame.final_order] {
        for p in order.windows(2) {
            neighbors.insert(canonical(&p[0], &p[1]));
        }
    }
    for (f, by_lane) in &intervals {
        for (a, b) in &neighbors {
            let collection = if nl > 2 && crossing_pairs.contains(&(a.clone(), b.clone())) {
                stacked.get(f).unwrap_or(by_lane)
            } else {
                by_lane
            };
            if let (Some(a), Some(b)) = (collection.get(a), collection.get(b)) {
                let mut both = a.clone();
                both.extend(b);
                whole.model.no_overlap(&both);
            }
        }
    }
    if nl > 2 {
        for [(a, ea), (b, eb)] in &whole.domain.separated_ends {
            let ai = if *ea == 0 { 0 } else { KMAX };
            let bi = if *eb == 0 { 0 } else { KMAX };
            whole.model.ne(
                &[
                    (whole.changes[a].layers[ai], 1),
                    (whole.changes[b].layers[bi], -1),
                ],
                0,
                &[],
            )?;
        }
        for between in &frame.tip_between {
            if let Some(pair) = whole.changes.get(&between.lane) {
                let pi = if between.end == 0 { 0 } else { KMAX };
                for n in &between.members {
                    if let Some(lane) = whole.changes.get(n) {
                        whole
                            .model
                            .ne(&[(pair.layers[pi], 1), (lane.layers[pi], -1)], 0, &[])?;
                    }
                }
            }
        }
    }
    // 664–725: crossing layer difference, per-crosser-layer spacing, stacked-via capacity.
    let mut layer_intervals: BTreeMap<(String, usize), Vec<i32>> = BTreeMap::new();
    let mut cumulative: BTreeMap<String, (Vec<i32>, Vec<Affine>)> = BTreeMap::new();
    let w_move = quantize_up(rules.lane_pitch / (17.0_f64).sqrt(), g)?.max(1);
    let w_stay = quantize_up(rules.lane_pitch, g)?.max(1);
    let room = quantize_up(2.0 * rules.via_need, g)?;
    for key in &whole.crossings {
        let a = &key.0;
        let b = &key.1;
        let ca = whole.changes[a].clone();
        let cb = whole.changes[b].clone();
        let time = whole.times[key];
        let mv = whole.mover[key];
        if nl > 2 {
            let la = run_at(&mut whole.model, &ca.before[key], &ca.layers, nl)?;
            let lb = run_at(&mut whole.model, &cb.before[key], &cb.layers, nl)?;
            whole.model.ne(&[(la, 1), (lb, -1)], 0, &[])?;
            for (lane, crossing_changes, mover) in [(a, &cb, mv), (b, &ca, !mv)] {
                let near = via_near(&mut whole.model, crossing_changes, time, room)?;
                for (width, presence) in [(w_move, mover), (w_stay, !mover)] {
                    let interval = whole.model.optional_interval(time, width, presence)?;
                    let demand = Affine {
                        terms: vec![(near.as_int_var(), nl as i64 - 2)],
                        offset: 1,
                    };
                    let row = cumulative.entry(lane.clone()).or_default();
                    row.0.push(interval);
                    row.1.push(demand);
                }
            }
            for layer in 0..nl {
                for (lane, crosser_layer, mover) in [(a, lb, mv), (b, la, !mv)] {
                    let on = whole.model.bool("");
                    whole.model.eq(&[(crosser_layer, 1)], layer as i64, &[on])?;
                    whole
                        .model
                        .ne(&[(crosser_layer, 1)], layer as i64, &[!on])?;
                    for (width, room_presence) in [(w_move, mover), (w_stay, !mover)] {
                        let presence = whole.model.bool("");
                        whole.model.and(&[room_presence, on], &[presence]);
                        whole.model.or(&[!room_presence, !on, presence], &[]);
                        layer_intervals
                            .entry((lane.clone(), layer))
                            .or_default()
                            .push(whole.model.optional_interval(time, width, presence)?);
                    }
                }
            }
        } else {
            let mut before = ca.before[key].clone();
            before.extend(&cb.before[key]);
            whole.model.xor_with_one(
                &before,
                whole.domain.tooth_layer[a] != whole.domain.tooth_layer[b],
            )?;
        }
    }
    for intervals in layer_intervals.values() {
        whole.model.no_overlap(intervals);
    }
    for (intervals, demands) in cumulative.values() {
        whole.model.cumulative(intervals, demands, nl as i64 - 1)?;
    }
    Ok(())
}
fn terminal_legs(
    problem: &WholeProblem,
    lane: &str,
    end: usize,
) -> Result<Vec<(String, [f64; 2], String)>, String> {
    let ctx = &problem.context;
    let layer = if end == 0 {
        &ctx.tooth_layer[lane]
    } else {
        &ctx.dest_layer[lane]
    };
    if let Some(legs) = ctx.pairs.get(lane) {
        let tips = ctx
            .pair_ends
            .get(lane)
            .ok_or("AWX pair terminal tips missing")?;
        let second = ctx
            .pair_layers
            .get(lane)
            .ok_or("AWX second pair leg layer missing")?;
        Ok(vec![
            (legs[0].clone(), tips[end][0], layer.clone()),
            (legs[1].clone(), tips[end][1], second[end].clone()),
        ])
    } else {
        let ends = &ctx.ends[lane];
        Ok(vec![(
            lane.into(),
            if end == 0 { ends.tooth } else { ends.berth },
            layer.clone(),
        )])
    }
}
fn run_to_via(problem: &WholeProblem, net: i32, end: [f64; 2], layer: &str) -> Option<Vec<usize>> {
    let ctx = &problem.context;
    let candidates: Vec<_> = ctx
        .base_segments
        .iter()
        .enumerate()
        .filter(|(_, s)| s.net_id == net && s.layer == layer)
        .collect();
    let mut current = end;
    let mut used = BTreeSet::new();
    let mut path = vec![];
    for _ in 0..=candidates.len() {
        if ctx.base_vias.iter().any(|v| {
            v.net_id == net && super::corridor::distance(v.point, current) < v.diameter / 2.0
        }) {
            return if path.is_empty() { None } else { Some(path) };
        }
        let next: Vec<_> = candidates
            .iter()
            .filter(|(i, s)| {
                !used.contains(i)
                    && (super::corridor::distance(s.a, current) < 1e-3
                        || super::corridor::distance(s.b, current) < 1e-3)
            })
            .collect();
        if next.len() != 1 {
            return None;
        }
        let (index, segment) = next[0];
        used.insert(*index);
        path.push(*index);
        current = if super::corridor::distance(segment.a, current) < 1e-3 {
            segment.b
        } else {
            segment.a
        };
    }
    None
}
fn has_via_end(problem: &WholeProblem, lane: &str, end: usize) -> Result<bool, String> {
    let ctx = &problem.context;
    let bounds = if end == 0 {
        problem.frame.sb
    } else {
        problem.frame.db
    };
    for (leg, point, layer) in terminal_legs(problem, lane, end)? {
        let nid = *ctx
            .byname
            .get(&leg)
            .ok_or("AWX physical leg net id missing")?;
        let vias: Vec<_> = ctx
            .base_vias
            .iter()
            .filter(|v| {
                v.net_id == nid
                    && bounds[0] - 1e-6 <= v.point[0]
                    && v.point[0] <= bounds[2] + 1e-6
                    && bounds[1] - 1e-6 <= v.point[1]
                    && v.point[1] <= bounds[3] + 1e-6
            })
            .collect();
        if vias.is_empty() {
            return Ok(false);
        }
        if !vias
            .iter()
            .any(|v| super::corridor::distance(v.point, point) < v.diameter / 2.0)
            && run_to_via(problem, nid, point, &layer).is_none()
        {
            return Ok(false);
        }
    }
    Ok(true)
}
fn point_pad_distance(point: [f64; 2], pad: &super::whole_context::ContextPad) -> f64 {
    let dx = point[0] - pad.point[0];
    let dy = point[1] - pad.point[1];
    let angle = pad.rect_rotation.to_radians();
    let local = [
        dx * angle.cos() + dy * angle.sin(),
        -dx * angle.sin() + dy * angle.cos(),
    ];
    (local[0].abs() - pad.size[0] / 2.0)
        .max(0.0)
        .hypot((local[1].abs() - pad.size[1] / 2.0).max(0.0))
}
fn site_clear(problem: &WholeProblem, lane: &str, end: usize) -> Result<bool, String> {
    let ctx = &problem.context;
    let ends = &ctx.ends[lane];
    let point = if end == 0 { ends.tooth } else { ends.berth };
    let nid = *ctx.byname.get(lane).ok_or("AWX single net id missing")?;
    let radius = problem.rules.via_size / 2.0 + problem.rules.clearance;
    if ctx.base_segments.iter().any(|s| {
        s.net_id != nid
            && super::corridor::point_segment_distance(point, s.a, s.b) < radius + s.width / 2.0
    }) {
        return Ok(false);
    }
    if ctx.base_vias.iter().any(|v| {
        v.net_id != nid && super::corridor::distance(v.point, point) < radius + v.diameter / 2.0
    }) {
        return Ok(false);
    }
    Ok(!ctx.pads.iter().any(|p| {
        p.net_id != nid
            && (p.point[0] - point[0]).abs() < 3.0
            && (p.point[1] - point[1]).abs() < 3.0
            && point_pad_distance(point, p) < radius
    }))
}
fn prepare_copper_and_fronts(
    problem: &WholeProblem,
    domain: &mut RouteDomain,
) -> Result<(), String> {
    let ctx = &problem.context;
    let frame = &problem.frame;
    let rules = &problem.rules;
    let nl = problem.options.routing_layers.len();
    // 97–155: physical run-to-via eligibility and the layer restrictions of moving stubs.
    let mut eligible = BTreeSet::new();
    for n in &frame.m {
        for end in 0..2 {
            if has_via_end(problem, n, end)? {
                eligible.insert((n.clone(), end));
            }
        }
    }
    if nl > 2 && problem.options.via_ends {
        domain.via_ends = eligible.clone();
    }
    let mut runs: BTreeMap<(String, usize), (BTreeSet<i32>, Vec<usize>)> = BTreeMap::new();
    for (n, end) in &domain.via_ends {
        let mut ids = BTreeSet::new();
        let mut segments = vec![];
        for (leg, pt, layer) in terminal_legs(problem, n, *end)? {
            let nid = ctx.byname[&leg];
            ids.insert(nid);
            segments.extend(run_to_via(problem, nid, pt, &layer).unwrap_or_default());
        }
        if !segments.is_empty() {
            runs.insert((n.clone(), *end), (ids, segments));
        }
    }
    let moving: BTreeSet<_> = runs.values().flat_map(|(_, s)| s.iter().copied()).collect();
    for (local, layer) in problem.options.routing_layers.iter().enumerate() {
        for (key, (nets, path)) in &runs {
            let mut hit = false;
            for &i in path {
                let a = &ctx.base_segments[i];
                for (j, b) in ctx.base_segments.iter().enumerate() {
                    if b.layer != *layer || moving.contains(&j) || nets.contains(&b.net_id) {
                        continue;
                    }
                    if super::joint_geometry::segment_segment(a.a, a.b, b.a, b.b)
                        < a.width / 2.0 + b.width / 2.0 + rules.fan_clearance - 0.01
                    {
                        hit = true;
                        break;
                    }
                }
                for pad in &ctx.pads {
                    if hit {
                        break;
                    }
                    if nets.contains(&pad.net_id)
                        || pad.pad_type == "np_thru_hole"
                        || (!pad.layers.contains(layer) && !pad.layers.iter().any(|l| l == "*.Cu"))
                    {
                        continue;
                    }
                    let h = [pad.size[0] / 2.0, pad.size[1] / 2.0];
                    let corners = [
                        [pad.point[0] - h[0], pad.point[1] - h[1]],
                        [pad.point[0] + h[0], pad.point[1] - h[1]],
                        [pad.point[0] + h[0], pad.point[1] + h[1]],
                        [pad.point[0] - h[0], pad.point[1] + h[1]],
                    ];
                    let inside = (a.a[0] - pad.point[0]).abs() <= h[0]
                        && (a.a[1] - pad.point[1]).abs() <= h[1];
                    if inside
                        || (0..4).any(|j| {
                            super::joint_geometry::segment_segment(
                                a.a,
                                a.b,
                                corners[j],
                                corners[(j + 1) % 4],
                            ) < a.width / 2.0 + rules.fan_clearance - 0.01
                        })
                    {
                        hit = true;
                        break;
                    }
                }
                if hit {
                    break;
                }
            }
            let original = if key.1 == 0 {
                domain.tooth_layer[&key.0]
            } else {
                domain.berth_layer[&key.0]
            };
            if hit && local as i64 != original {
                domain
                    .via_bans
                    .entry(key.clone())
                    .or_default()
                    .push(local as i64);
            }
        }
    }
    let keys: Vec<_> = runs.keys().cloned().collect();
    for i in 0..keys.len() {
        for j in i + 1..keys.len() {
            let a = &runs[&keys[i]];
            let b = &runs[&keys[j]];
            if !a.0.is_disjoint(&b.0) {
                continue;
            }
            let layer_of = |key: &(String, usize)| {
                if key.1 == 0 {
                    domain.tooth_layer[&key.0]
                } else {
                    domain.berth_layer[&key.0]
                }
            };
            if layer_of(&keys[i]) == layer_of(&keys[j]) {
                continue;
            }
            if a.1.iter().any(|&ia| {
                b.1.iter().any(|&ib| {
                    let sa = &ctx.base_segments[ia];
                    let sb = &ctx.base_segments[ib];
                    super::joint_geometry::segment_segment(sa.a, sa.b, sb.a, sb.b)
                        < sa.width / 2.0 + sb.width / 2.0 + rules.fan_clearance - 0.01
                })
            }) {
                domain
                    .separated_ends
                    .push([keys[i].clone(), keys[j].clone()]);
            }
        }
    }
    // 192–223: blocked-front spans and waived room for their measured dogbone sites.
    for (n, ends) in &problem.blocked_fronts {
        if !frame.m.contains(n) || ctx.pairs.contains_key(n) {
            continue;
        }
        for (&end, &[v0, v1, d0, d1]) in ends {
            if end > 1 || (end == 1 && domain.branch.contains_key(n)) {
                continue;
            }
            let vm = (rules.via_size - rules.track) / 2.0;
            let target = 1 - if end == 0 {
                domain.tooth_layer[n]
            } else {
                domain.berth_layer[n]
            };
            let span = if end == 0 {
                domain
                    .via_start
                    .insert(n.clone(), domain.via_start[n].min(v0));
                LayerSpan {
                    lane: n.clone(),
                    tag: "0".into(),
                    target,
                    blocked: vec![domain.tooth_layer[n]],
                    route: [domain.entry[n] + d0, domain.entry[n] + d1],
                    change: [
                        domain.entry[n] + v1 + domain.grid / 2.0,
                        domain.entry[n] + d1 + vm,
                    ],
                    dogbone: Some([domain.entry[n] + v0, domain.entry[n] + v1]),
                }
            } else {
                domain.via_end.insert(n.clone(), domain.via_end[n].min(v0));
                LayerSpan {
                    lane: n.clone(),
                    tag: "1".into(),
                    target,
                    blocked: vec![domain.berth_layer[n]],
                    route: [domain.end[n] - d1, domain.end[n] - d0],
                    change: [
                        domain.end[n] - d1 - vm,
                        domain.end[n] - v1 - domain.grid / 2.0,
                    ],
                    dogbone: Some([domain.end[n] - v1, domain.end[n] - v0]),
                }
            };
            if end == 0 {
                domain.waived_face.insert(n.clone());
            }
            domain.layer_spans.push(span);
        }
    }
    prepare_channel_cuts(problem, domain)?;
    // 287–332: opposite-hands crossover and required physical turns enlarge pair end rooms.
    for n in &frame.m {
        if !ctx.pairs.contains_key(n) {
            continue;
        }
        if opposite_hands(problem, n) {
            let rooms = rules
                .pair_crossover_room
                .get(n)
                .ok_or("AWX pair crossover metadata missing")?;
            domain.via_start.insert(n.clone(), rooms[0]);
            domain
                .via_end
                .insert(n.clone(), domain.via_end[n].max(rooms[1]));
        }
        for (end, u, direction, sign) in [
            (0, domain.entry[n], ctx.tooth_dir.get(n), 1.0),
            (1, domain.end[n], ctx.stub_dir.get(n), -1.0),
        ] {
            let Some(&direction) = direction else {
                continue;
            };
            let route = route_direction(problem, domain, n, u)?;
            let cosine = (direction[0] * sign * route[0] + direction[1] * sign * route[1])
                / (direction[0].hypot(direction[1]) * route[0].hypot(route[1]));
            let turn = cosine.clamp(-1.0, 1.0).acos().to_degrees() - 45.0;
            if turn >= 22.5 {
                let dive = if opposite_hands(problem, n) {
                    rules.pair_diagonal_crossover_run
                } else {
                    rules.pair_diagonal_via_straight
                };
                let extra = 2.0 * turn_room(problem, turn) + dive;
                if end == 0 {
                    *domain.via_start.get_mut(n).unwrap() += extra;
                } else {
                    *domain.via_end.get_mut(n).unwrap() += extra;
                }
            }
        }
    }
    // 333–363: movable surface tooth and measured-clear berth via windows on stacks.
    if nl > 2 {
        for n in &frame.m {
            if !eligible.contains(&(n.clone(), 0)) {
                domain.tooth_vias.insert(n.clone());
                domain.waived_face.insert(n.clone());
                if !ctx.pairs.contains_key(n) {
                    domain.via_start.insert(n.clone(), 0.0);
                }
            }
            if !ctx.pairs.contains_key(n)
                && !eligible.contains(&(n.clone(), 1))
                && site_clear(problem, n, 1)?
            {
                domain.berth_vias.insert(n.clone());
                domain.via_end.insert(n.clone(), 0.0);
            }
        }
    }
    for span in &domain.layer_spans {
        if let Some(w) = span.dogbone {
            domain
                .dog_windows
                .entry(span.lane.clone())
                .or_default()
                .push([quantize(w[0], domain.grid)?, quantize(w[1], domain.grid)?]);
        }
    }
    for n in &domain.tooth_vias {
        domain.dog_windows.entry(n.clone()).or_default().push([
            quantize(domain.entry[n] + domain.via_start[n], domain.grid)?,
            quantize(
                domain.entry[n] + domain.via_start[n] + rules.via_need,
                domain.grid,
            )?,
        ]);
    }
    for n in &domain.berth_vias {
        domain.dog_windows.entry(n.clone()).or_default().push([
            quantize(
                domain.end[n] - domain.via_end[n] - rules.via_need,
                domain.grid,
            )?,
            quantize(domain.end[n] - domain.via_end[n], domain.grid)?,
        ]);
    }
    Ok(())
}
fn prepare_channel_cuts(problem: &WholeProblem, domain: &mut RouteDomain) -> Result<(), String> {
    let nl = problem.options.routing_layers.len();
    let ctx = &problem.context;
    let frame = &problem.frame;
    let rules = &problem.rules;
    let mut seen = BTreeSet::new();
    let mut last_blocked: BTreeMap<(String, String), Vec<i64>> = BTreeMap::new();
    for group in problem.cuts.iter().chain(&problem.soft_cuts) {
        for cut in &group.lcuts {
            let key = (
                cut.lane.clone(),
                cut.island.clone(),
                cut.layer,
                cut.bounds
                    .map(|v| round_digits(v, 6).map(|x| x.to_bits()))
                    .into_iter()
                    .collect::<Result<Vec<_>, _>>()?,
                cut.span.map(|s| s.map(f64::to_bits)),
            );
            if !seen.insert(key) {
                continue;
            }
            let n = &cut.lane;
            if !frame.m.contains(n) || (ctx.pairs.contains_key(n) && nl == 2) {
                continue;
            }
            let [x0, y0, x1, y1] = cut.bounds;
            let corners = [[x0, y0], [x0, y1], [x1, y0], [x1, y1]];
            let ss: Vec<_> = corners
                .iter()
                .map(|&p| frame.spine.project_pt(p)[0])
                .collect();
            let min = ss.iter().copied().fold(f64::INFINITY, f64::min);
            let max = ss.iter().copied().fold(f64::NEG_INFINITY, f64::max);
            let mut spans = vec![];
            if let Some([lo, hi]) = cut.span {
                if hi >= domain.entry[n] && lo <= domain.end[n] {
                    spans.push([lo.max(domain.entry[n]), hi.min(domain.end[n])]);
                }
            } else if !(max < domain.entry[n] || min > domain.trunk_end[n]) {
                spans.push([min, max]);
            }
            if nl > 2 && cut.span.is_none() {
                if let Some(branch) = domain.branch.get(n) {
                    let us: Vec<_> = corners
                        .iter()
                        .map(|&p| {
                            domain.handoff[n] + frame.rings[branch].project_pt(p)[0]
                                - domain.ring_start[branch]
                        })
                        .collect();
                    let min = us.iter().copied().fold(f64::INFINITY, f64::min);
                    let max = us.iter().copied().fold(f64::NEG_INFINITY, f64::max);
                    if max > domain.handoff[n] && min < domain.end[n] {
                        spans.push([min.max(domain.handoff[n]), max]);
                    }
                }
            }
            let layer = i64::try_from(cut.layer).map_err(|_| "AWX layer index overflow")?;
            let blocked: Vec<i64> = cut.blocked.clone().unwrap_or_else(|| vec![1 - layer]);
            if nl > 2 && !spans.is_empty() {
                last_blocked.insert((n.clone(), format!("chan {}", cut.island)), blocked.clone());
            }
            for [s0, s1] in spans {
                let track = rules.track / 2.0 + rules.clearance;
                let via = rules.via_size / 2.0 + rules.clearance;
                let lo = s0 - via;
                let hi = s1 + via;
                let off0 = if nl == 2 {
                    domain.tooth_layer[n] != cut.layer as i64
                } else {
                    !ctx.pairs.contains_key(n)
                        && !domain.via_ends.contains(&(n.clone(), 0))
                        && blocked.contains(&domain.tooth_layer[n])
                };
                let off1 = if nl == 2 {
                    domain.berth_layer[n] != cut.layer as i64
                } else {
                    !ctx.pairs.contains_key(n)
                        && !domain.via_ends.contains(&(n.clone(), 1))
                        && blocked.contains(&domain.berth_layer[n])
                };
                let mut dog = None;
                if off0 && lo < domain.entry[n] + domain.via_start[n] + domain.grid {
                    dog = Some([domain.entry[n], lo]);
                    domain.via_start.insert(n.clone(), 0.0);
                    domain.waived_face.insert(n.clone());
                } else if !domain.branch.contains_key(n)
                    && off1
                    && hi > domain.end[n] - domain.via_end[n] - domain.grid
                {
                    dog = Some([hi, domain.end[n]]);
                    domain.via_end.insert(n.clone(), 0.0);
                }
                domain.layer_spans.push(LayerSpan {
                    lane: n.clone(),
                    tag: format!("chan {}", cut.island),
                    target: cut.layer as i64,
                    blocked: blocked.clone(),
                    route: [round_digits(s0 - track, 4)?, round_digits(s1 + track, 4)?],
                    change: [round_digits(lo, 4)?, round_digits(hi, 4)?],
                    dogbone: dog,
                });
            }
        }
    }
    for span in &mut domain.layer_spans {
        if let Some(blocked) = last_blocked.get(&(span.lane.clone(), span.tag.clone())) {
            span.blocked = blocked.clone();
        }
    }
    domain.layer_spans.sort_by(|a, b| {
        a.lane
            .cmp(&b.lane)
            .then(a.tag.cmp(&b.tag))
            .then(a.target.cmp(&b.target))
            .then(a.route[0].total_cmp(&b.route[0]))
            .then(a.route[1].total_cmp(&b.route[1]))
    });
    domain.layer_spans.dedup_by(|a, b| {
        a.lane == b.lane
            && a.tag == b.tag
            && a.target == b.target
            && a.route == b.route
            && a.change == b.change
            && a.dogbone == b.dogbone
    });
    Ok(())
}
fn turn_room(problem: &WholeProblem, degrees: f64) -> f64 {
    problem.rules.pair_turn_straight_steps as f64
        * problem.rules.grid_step
        * ((degrees.abs() / 45.0 - 1e-9).ceil() - 1.0).max(0.0)
        / 2.0
}
fn route_direction(
    problem: &WholeProblem,
    domain: &RouteDomain,
    lane: &str,
    u: f64,
) -> Result<[f64; 2], String> {
    if let Some(branch) = domain.branch.get(lane) {
        if u > domain.handoff[lane] {
            let ring = &problem.frame.rings[branch];
            return Ok(ring.d[ring.seg_of(u - domain.handoff[lane] + domain.ring_start[branch])]);
        }
    }
    Ok(problem.frame.spine.d[problem.frame.spine.seg_of(u)])
}
fn add_geometry_cuts(problem: &WholeProblem, whole: &mut WholeModel) -> Result<(), String> {
    let g = whole.domain.grid;
    let frame = &problem.frame;
    let rules = &problem.rules;
    // 397–419: every crossing of a lane avoids its hard island window, or pays a FIRM break.
    let mut islands = BTreeSet::new();
    for group in &problem.cuts {
        for cut in &group.cuts {
            islands.insert((
                cut.lane.clone(),
                rounded_scalar(cut.u_lo)?,
                rounded_scalar(cut.u_hi)?,
            ));
        }
    }
    for (n, lo, hi) in islands {
        let key = ("island".into(), n.clone(), String::new(), lo, hi);
        let mut enforced = vec![];
        if problem.options.firm_cuts {
            let broken = whole.model.bool("");
            whole.firm_broken.insert(key, broken);
            enforced.push(!broken);
        }
        for crossing in &whole.crossings {
            if crossing.0 != n && crossing.1 != n {
                continue;
            }
            let side = whole.model.bool("");
            let mut a = vec![side];
            a.extend(&enforced);
            let mut b = vec![!side];
            b.extend(&enforced);
            whole
                .model
                .le(&[(whole.times[crossing], 1)], quantize(lo.0, g)?, &a)?;
            whole.model.ge(
                &[(whole.times[crossing], 1)],
                checked_add(quantize(hi.0, g)?, 1)?,
                &b,
            )?;
        }
    }
    for group in &problem.cuts {
        whole.via_cuts.extend(group.vcuts.clone());
    }
    whole.filed_via_count = whole.via_cuts.len();
    // 420–437: physically required pair turns are built-in via exclusion windows.
    for n in &frame.m {
        if !problem.context.pairs.contains_key(n) {
            continue;
        }
        let mut turns = vec![];
        for (_, s, degrees) in frame.spine.corners(22.5) {
            if whole.domain.entry[n] < s && s < whole.domain.trunk_end[n] {
                turns.push((s, degrees));
            }
        }
        if let Some(branch) = whole.domain.branch.get(n) {
            let ring = &frame.rings[branch];
            for (_, s, degrees) in ring.corners(22.5) {
                let u = whole.domain.handoff[n] + s - whole.domain.ring_start[branch];
                if whole.domain.handoff[n] < u && u < whole.domain.end[n] {
                    turns.push((u, degrees));
                }
            }
            let before = route_direction(problem, &whole.domain, n, whole.domain.handoff[n] - g)?;
            let after = route_direction(problem, &whole.domain, n, whole.domain.handoff[n] + g)?;
            let angle = (super::corridor::dot(before, after)
                / (super::corridor::length(before) * super::corridor::length(after)))
            .clamp(-1.0, 1.0)
            .acos()
            .to_degrees();
            if angle >= 22.5
                && whole.domain.entry[n] < whole.domain.handoff[n]
                && whole.domain.handoff[n] < whole.domain.end[n]
            {
                turns.push((whole.domain.handoff[n], angle));
            }
        }
        let dive = if opposite_hands(problem, n) {
            rules.pair_diagonal_crossover_run
        } else {
            rules.pair_diagonal_via_straight
        };
        for (u, degrees) in turns {
            whole.via_cuts.push(ViaCut {
                lane: n.clone(),
                u,
                w: dive + turn_room(problem, degrees),
                built: false,
            });
        }
    }
    // 438–463: non-bus teeth and other copper pads reserve via room against reference paths.
    let mut items: Vec<_> = problem
        .context
        .pads
        .iter()
        .filter(|p| {
            p.component_ref != frame.src
                && p.component_ref != frame.dst
                && p.pad_type != "np_thru_hole"
                && !p.layers.is_empty()
        })
        .map(|p| (p.point, p.size[0].max(p.size[1]) / 2.0))
        .collect();
    let members: BTreeSet<_> = frame.m.iter().cloned().collect();
    items.extend(
        problem
            .context
            .foreign_teeth(&members, &[frame.sb, frame.db])
            .into_iter()
            .map(|tooth| (tooth.point, tooth.half_width)),
    );
    for (point, radius) in items {
        let [s, offset] = frame.spine.project_pt(point);
        for n in &frame.m {
            let pair = problem.context.pairs.contains_key(n);
            let dive = if opposite_hands(problem, n) {
                rules.pair_diagonal_crossover_run
            } else {
                rules.pair_diagonal_via_straight
            };
            let w = radius + rules.via_size / 2.0 + rules.clearance + if pair { dive } else { 0.0 };
            if !(whole.domain.entry[n] < s + w && s - w < whole.domain.trunk_end[n]) {
                continue;
            }
            let barrel_offset = if pair { rules.pair_dive_offset[n] } else { 0.0 };
            if (frame.ref_offset(n, s)? - offset).abs()
                < radius + rules.via_size / 2.0 + rules.clearance + barrel_offset + rules.lane_pitch
            {
                whole.via_cuts.push(ViaCut {
                    lane: n.clone(),
                    u: s,
                    w,
                    built: true,
                });
            }
        }
    }
    // 726–744: via exclusions; static built-in ones yield only in certified dogbone windows.
    let filed: BTreeSet<_> = whole.via_cuts[..whole.filed_via_count]
        .iter()
        .map(|c| Ok((c.lane.clone(), rounded_scalar(c.u)?, rounded_scalar(c.w)?)))
        .collect::<Result<_, String>>()?;
    let unique: BTreeSet<_> = whole
        .via_cuts
        .iter()
        .map(|c| {
            Ok((
                c.lane.clone(),
                rounded_scalar(c.u)?,
                rounded_scalar(c.w)?,
                c.built,
            ))
        })
        .collect::<Result<_, String>>()?;
    for (n, u, w, built) in unique {
        let Some(changes) = whole.changes.get(&n).cloned() else {
            continue;
        };
        let mut broken = vec![];
        if problem.options.firm_cuts && !built && filed.contains(&(n.clone(), u, w)) {
            let lit = whole.model.bool("");
            whole
                .firm_broken
                .insert(("via".into(), n.clone(), String::new(), u, w), lit);
            broken.push(lit);
        }
        for (index, (&position, &active)) in
            changes.positions.iter().zip(&changes.active).enumerate()
        {
            let low = whole.model.bool("");
            let high = whole.model.bool("");
            whole
                .model
                .le(&[(position, 1)], quantize(u.0 - w.0, g)?, &[low])?;
            whole
                .model
                .ge(&[(position, 1)], quantize(u.0 + w.0, g)?, &[high])?;
            let mut clause = vec![low, high, !active];
            if built {
                clause.extend(
                    whole
                        .domain
                        .in_windows
                        .get(&(n.clone(), index))
                        .into_iter()
                        .flatten()
                        .copied(),
                );
            }
            clause.extend(&broken);
            whole.model.or(&clause, &[]);
        }
    }
    // 745–780: SOFT island/via constraints preserve each source cut's broken flag and price.
    let mut soft = BTreeSet::new();
    for group in &problem.soft_cuts {
        for c in &group.cuts {
            soft.insert((
                "island".to_string(),
                c.lane.clone(),
                String::new(),
                rounded_scalar(c.u_lo)?,
                rounded_scalar(c.u_hi)?,
            ));
        }
        for c in &group.vcuts {
            soft.insert((
                "via".to_string(),
                c.lane.clone(),
                String::new(),
                rounded_scalar(c.u)?,
                rounded_scalar(c.w)?,
            ));
        }
    }
    for key in soft {
        let broken = whole.model.bool("");
        let n = &key.1;
        let p = key.3 .0;
        let q = key.4 .0;
        let mut hit = false;
        if key.0 == "island" {
            for crossing in &whole.crossings {
                if &crossing.0 != n && &crossing.1 != n {
                    continue;
                }
                let side = whole.model.bool("");
                whole.model.le(
                    &[(whole.times[crossing], 1)],
                    quantize(p, g)?,
                    &[side, !broken],
                )?;
                whole.model.ge(
                    &[(whole.times[crossing], 1)],
                    checked_add(quantize(q, g)?, 1)?,
                    &[!side, !broken],
                )?;
                hit = true;
            }
        } else if let Some(changes) = whole.changes.get(n) {
            hit = true;
            for (&position, &active) in changes.positions.iter().zip(&changes.active) {
                let low = whole.model.bool("");
                let high = whole.model.bool("");
                whole
                    .model
                    .le(&[(position, 1)], quantize(p - q, g)?, &[low])?;
                whole
                    .model
                    .ge(&[(position, 1)], quantize(p + q, g)?, &[high])?;
                whole.model.or(&[low, high, !active, broken], &[]);
            }
        }
        if hit {
            whole.soft_broken.insert(key, broken);
        } else {
            whole.model.eq(&[(broken.as_int_var(), 1)], 0, &[])?;
        }
    }
    // 781–816: blocked fronts and island-layer spans, soft penalties with hard exclusion semantics while held.
    for span in &whole.domain.layer_spans {
        let Some(changes) = whole.changes.get(&span.lane) else {
            continue;
        };
        let broken = whole.model.bool("");
        let mut before = vec![];
        for (&position, &active) in changes.positions.iter().zip(&changes.active) {
            let low = whole.model.bool("");
            let high = whole.model.bool("");
            let earlier = whole.model.bool("");
            whole
                .model
                .le(&[(position, 1)], quantize(span.change[0], g)?, &[low])?;
            whole
                .model
                .ge(&[(position, 1)], quantize_up(span.change[1], g)?, &[high])?;
            whole.model.or(&[low, high, !active, broken], &[]);
            whole
                .model
                .le(&[(position, 1)], quantize(span.route[0], g)?, &[earlier])?;
            whole.model.ge(
                &[(position, 1)],
                checked_add(quantize(span.route[0], g)?, 1)?,
                &[!earlier],
            )?;
            before.push(earlier);
        }
        if problem.options.routing_layers.len() > 2 {
            let layer = run_at(
                &mut whole.model,
                &before,
                &changes.layers,
                problem.options.routing_layers.len(),
            )?;
            for &blocked in &span.blocked {
                whole.model.ne(&[(layer, 1)], blocked, &[!broken])?;
            }
        } else {
            let odd = whole.model.bool("");
            let mut lits = before;
            lits.push(!odd);
            whole.model.xor(&lits);
            whole.model.eq(
                &[(odd.as_int_var(), 1)],
                whole.domain.tooth_layer[&span.lane] ^ span.target,
                &[!broken],
            )?;
        }
        whole.soft_broken.insert(
            (
                "layer".into(),
                span.lane.clone(),
                span.tag.clone(),
                rounded_scalar(span.route[0])?,
                OrderedScalar(0.0),
            ),
            broken,
        );
    }
    Ok(())
}
fn free_gate_room(
    problem: &WholeProblem,
    spine: &super::corridor::Spine,
    s: f64,
    layer: usize,
    ring: bool,
    islands: &BTreeMap<(String, BTreeSet<usize>), [f64; 4]>,
) -> Result<f64, String> {
    let r = &problem.rules;
    let bounds = problem.context.board_bounds;
    let edge = if r.board_edge_clearance != 0.0 {
        r.board_edge_clearance
    } else {
        problem.context.cfg.clearance
    } + r.track / 2.0;
    let span = (bounds[2] - bounds[0]).max(bounds[3] - bounds[1]);
    let g2 = r.grid_step / 2.0;
    let bar = r.track / 2.0 + r.clearance + g2;
    let pitch = r.lane_pitch.max(r.track + r.clearance + 2.0 * g2);
    let step = pitch / 8.0;
    let count = (2.0 * span / step).ceil();
    if !count.is_finite() || count < 0.0 || count > i64::MAX as f64 {
        return Err("AWX gate column sample overflow".into());
    }
    let count = count as usize;
    let effective_step = (-span + step) - (-span);
    let mut ok = Vec::with_capacity(count);
    let mut offsets = Vec::with_capacity(count);
    let center = [
        (problem.frame.db[0] + problem.frame.db[2]) / 2.0,
        (problem.frame.db[1] + problem.frame.db[3]) / 2.0,
    ];
    let offset_center = spine.project_pt(center)[1];
    for i in 0..count {
        let offset = -span + i as f64 * effective_step;
        let point = spine.xy(s, offset);
        let in_bounds = point[0] >= bounds[0] + edge
            && point[0] <= bounds[2] - edge
            && point[1] >= bounds[1] + edge
            && point[1] <= bounds[3] - edge;
        let blocked = std::iter::once(&problem.frame.sb)
            .chain(std::iter::once(&problem.frame.db))
            .chain(
                islands
                    .iter()
                    .filter(|((_, layers), _)| layers.contains(&layer))
                    .map(|(_, b)| b),
            )
            .any(|b| {
                point[0] > b[0] - bar
                    && point[0] < b[2] + bar
                    && point[1] > b[1] - bar
                    && point[1] < b[3] + bar
            });
        offsets.push(offset);
        ok.push(in_bounds && !blocked);
    }
    let mut room = 0.0;
    let mut i = 0;
    while i < count {
        if !ok[i] {
            i += 1;
            continue;
        }
        let mut j = i;
        while j + 1 < count && ok[j + 1] {
            j += 1;
        }
        let a = offsets[i];
        let b = offsets[j];
        if !ring || ((a + b) / 2.0 - offset_center) * (0.0 - offset_center) > 0.0 {
            room += b - a + pitch;
        }
        i = j + 1;
    }
    Ok(room)
}
fn add_part_gates(problem: &WholeProblem, whole: &mut WholeModel) -> Result<(), String> {
    let layers = &problem.options.routing_layers;
    let nl = layers.len();
    if nl <= 2 || !problem.options.solve_gates {
        return Ok(());
    }
    let ctx = &problem.context;
    let frame = &problem.frame;
    let r = &problem.rules;
    let all: BTreeSet<_> = (0..nl).collect();
    let skip: BTreeSet<_> = [frame.src.clone(), frame.dst.clone()].into_iter().collect();
    let labels = ctx.part_islands(&skip, &BTreeSet::new());
    let mut islands: BTreeMap<(String, BTreeSet<usize>), [f64; 4]> = BTreeMap::new();
    for pad in &ctx.pads {
        let Some(label) = labels.get(&pad.key) else {
            continue;
        };
        let npth = pad.pad_type == "np_thru_hole";
        let size = if npth {
            [pad.drill / 2.0; 2]
        } else {
            [pad.size[0] / 2.0, pad.size[1] / 2.0]
        };
        let on: BTreeSet<_> = if npth || pad.drill > 0.0 || pad.layers.iter().any(|l| l == "*.Cu") {
            all.clone()
        } else {
            pad.layers
                .iter()
                .filter_map(|l| layers.iter().position(|name| name == l))
                .collect()
        };
        if on.is_empty() {
            continue;
        }
        let b = islands.entry((label.clone(), on)).or_insert([
            f64::INFINITY,
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::NEG_INFINITY,
        ]);
        b[0] = b[0].min(pad.point[0] - size[0]);
        b[1] = b[1].min(pad.point[1] - size[1]);
        b[2] = b[2].max(pad.point[0] + size[0]);
        b[3] = b[3].max(pad.point[1] + size[1]);
    }
    let g2 = r.grid_step / 2.0;
    let pitch = r.lane_pitch.max(r.track + r.clearance + 2.0 * g2);
    let widths: BTreeMap<_, _> = frame
        .m
        .iter()
        .map(|n| {
            (
                n.clone(),
                pitch
                    + if ctx.pairs.contains_key(n) {
                        r.pair_pitch
                    } else {
                        0.0
                    },
            )
        })
        .collect();
    let mut frames = vec![("T".to_string(), &frame.spine, false)];
    frames.extend(
        frame
            .rings
            .iter()
            .map(|(name, spine)| (name.clone(), spine, true)),
    );
    let via_room = r.via_size / 2.0 + r.clearance + g2;
    let mut seen = BTreeSet::new();
    for ((label, on), b) in &islands {
        if *on == all {
            continue;
        }
        let corners = [[b[0], b[1]], [b[0], b[3]], [b[2], b[1]], [b[2], b[3]]];
        for (frame_name, spine, ring) in &frames {
            let ss: Vec<_> = corners.iter().map(|&p| spine.project_pt(p)[0]).collect();
            let lo = ss.iter().copied().fold(f64::INFINITY, f64::min).max(0.0);
            let hi = ss
                .iter()
                .copied()
                .fold(f64::NEG_INFINITY, f64::max)
                .min(spine.length);
            if hi < lo {
                continue;
            }
            let mut rooms = BTreeMap::new();
            for &layer in on {
                let mut room = f64::INFINITY;
                for s in [lo, (lo + hi) / 2.0, hi] {
                    room = room.min(free_gate_room(problem, spine, s, layer, *ring, &islands)?);
                }
                rooms.insert(layer, room);
            }
            let samples: BTreeSet<_> = [
                rounded_scalar((lo - via_room).max(0.0))?,
                rounded_scalar((lo + hi) / 2.0)?,
                rounded_scalar((hi + via_room).min(spine.length))?,
            ]
            .into_iter()
            .collect();
            for s in samples {
                let present: Vec<_> = frame
                    .m
                    .iter()
                    .filter_map(|n| {
                        if !ring {
                            if whole.domain.entry[n] < s.0 && s.0 < whole.domain.trunk_end[n] {
                                Some((n, s.0))
                            } else {
                                None
                            }
                        } else if whole.domain.branch.get(n) == Some(frame_name) {
                            let u =
                                whole.domain.handoff[n] + s.0 - whole.domain.ring_start[frame_name];
                            if whole.domain.handoff[n] < u && u < whole.domain.end[n] {
                                Some((n, u))
                            } else {
                                None
                            }
                        } else {
                            None
                        }
                    })
                    .collect();
                if present.is_empty() {
                    continue;
                }
                let needed: f64 = present.iter().map(|(n, _)| widths[*n]).sum();
                for &layer in on {
                    if !seen.insert((frame_name.clone(), label.clone(), layer, s)) {
                        continue;
                    }
                    let room = rooms[&layer];
                    if needed <= room {
                        continue;
                    }
                    let mut terms = vec![];
                    for &(n, u) in &present {
                        let changes = whole.changes[n].clone();
                        let mut before = vec![];
                        for &position in &changes.positions {
                            let earlier = whole.model.bool("");
                            whole.model.le(
                                &[(position, 1)],
                                quantize(u, whole.domain.grid)?,
                                &[earlier],
                            )?;
                            whole.model.ge(
                                &[(position, 1)],
                                checked_add(quantize(u, whole.domain.grid)?, 1)?,
                                &[!earlier],
                            )?;
                            before.push(earlier);
                        }
                        let run = run_at(&mut whole.model, &before, &changes.layers, nl)?;
                        let on_layer = whole.model.bool("");
                        whole.model.eq(&[(run, 1)], layer as i64, &[on_layer])?;
                        whole.model.ne(&[(run, 1)], layer as i64, &[!on_layer])?;
                        terms.push((
                            on_layer.as_int_var(),
                            super::numeric::checked_round_i64(widths[n] * 1000.0)?,
                        ));
                    }
                    let overflow = whole.model.int(0, (4 * present.len()) as i64, "")?;
                    terms.push((
                        overflow,
                        -super::numeric::checked_round_i64(pitch * 1000.0)?,
                    ));
                    whole.model.le(
                        &terms,
                        super::numeric::checked_round_i64(room * 1000.0)?,
                        &[],
                    )?;
                    whole.gate_over.push((
                        serde_json::json!([
                            frame_name,
                            label,
                            layers[layer],
                            round_digits(s.0, 2)?,
                            round_digits(room / pitch, 1)?,
                            present
                                .iter()
                                .map(|(n, _)| n.to_string())
                                .collect::<Vec<_>>()
                        ]),
                        overflow,
                    ));
                }
            }
        }
    }
    Ok(())
}
fn in_bin(
    model: &mut Model,
    variable: IntVar,
    lo: f64,
    hi: f64,
    grid: f64,
    active: Option<BoolVar>,
) -> Result<BoolVar, String> {
    let result = model.bool("");
    let lower = model.bool("");
    let upper = model.bool("");
    let qlo = quantize(lo, grid)?;
    let qhi = quantize(hi, grid)?;
    model.ge(&[(variable, 1)], qlo, &[lower])?;
    model.le(
        &[(variable, 1)],
        qlo.checked_sub(1).ok_or("AWX history bound overflow")?,
        &[!lower],
    )?;
    model.le(
        &[(variable, 1)],
        qhi.checked_sub(1).ok_or("AWX history bound overflow")?,
        &[upper],
    )?;
    model.ge(&[(variable, 1)], qhi, &[!upper])?;
    let mut clause = vec![!lower, !upper, result];
    if let Some(active) = active {
        clause.push(!active);
    }
    model.or(&clause, &[]);
    Ok(result)
}
fn bin_in_frame(
    domain: &RouteDomain,
    frame: &str,
    lane: &str,
    lo: f64,
    hi: f64,
) -> Option<[f64; 2]> {
    let interval = if frame == "T" {
        [
            lo,
            if domain.branch.contains_key(lane) {
                hi.min(domain.handoff[lane])
            } else {
                hi
            },
        ]
    } else if domain.branch.get(lane).is_some_and(|f| f == frame) {
        [lo.max(domain.handoff[lane]), hi]
    } else {
        return None;
    };
    if interval[0] < interval[1] {
        Some(interval)
    } else {
        None
    }
}
fn add_history_and_hints(problem: &WholeProblem, whole: &mut WholeModel) -> Result<(), String> {
    let pitch = problem.rules.lane_pitch;
    let bin_width = 2.0 * pitch;
    let hot_radius = 2.0 * problem.rules.via_need;
    let frame = &problem.frame;
    let nl = problem.options.routing_layers.len();
    let origin = |name: &str| {
        if name == "T" {
            frame.h0
        } else {
            frame.hk[name]
        }
    };
    let bin = |name: &str, u: f64| {
        super::numeric::checked_round_i64(((u - origin(name)) / bin_width + 1e-9).floor())
    };
    let mut hot: BTreeMap<(String, i64), i64> = BTreeMap::new();
    let mut named: BTreeMap<(String, i64), BTreeSet<String>> = BTreeMap::new();
    let mut dived: BTreeMap<(String, i64), BTreeSet<String>> = BTreeMap::new();
    for audit in &problem.history {
        let mut marked = BTreeSet::new();
        for finding in &audit.hot {
            let trunk = frame.spine.project_pt(finding.point);
            let mut candidates = vec![("T".to_string(), trunk[0], trunk[1])];
            for (name, spine) in &frame.rings {
                let [s, o] = spine.project_pt(finding.point);
                let u = frame.hk[name] + s - whole.domain.ring_start[name];
                if u >= frame.hk[name] {
                    candidates.push((name.clone(), u, o));
                }
            }
            let nearest = candidates
                .into_iter()
                .min_by(|a, b| a.2.abs().total_cmp(&b.2.abs()))
                .ok_or("AWX hot finding has no frame")?;
            let start = bin(&nearest.0, nearest.1 - hot_radius)?;
            let end = bin(&nearest.0, nearest.1 + hot_radius)?;
            for k in start..=end {
                let key = (nearest.0.clone(), k);
                marked.insert(key.clone());
                named
                    .entry(key.clone())
                    .or_default()
                    .extend(finding.lanes.iter().cloned());
                if finding.kind == "DIVE" {
                    dived
                        .entry(key)
                        .or_default()
                        .extend(finding.lanes.iter().cloned());
                }
            }
        }
        for key in marked {
            let count = hot.entry(key).or_default();
            *count = checked_add(*count, 1)?;
        }
    }
    let mut runs: Vec<((String, i64), i64, i64)> = vec![];
    for (key, &count) in &hot {
        if nl > 2 {
            if let Some(previous) = runs.last_mut() {
                if previous.0 .0 == key.0
                    && previous.1 == count
                    && checked_add(previous.0 .1, previous.2)? == key.1
                    && named.get(key) == named.get(&previous.0)
                    && dived.get(key) == dived.get(&previous.0)
                {
                    previous.2 = checked_add(previous.2, 1)?;
                    continue;
                }
            }
        }
        runs.push((key.clone(), count, 1));
    }
    let area_via = 2.0 * (2.0 * problem.rules.via_need).powi(2);
    let area_cross = pitch * pitch;
    for ((name, k), count, width) in runs {
        let lo = origin(&name) + k as f64 * bin_width;
        let hi = origin(&name) + checked_add(k, width)? as f64 * bin_width;
        let empty = BTreeSet::new();
        let named_here = if nl > 2 {
            named.get(&(name.clone(), k)).unwrap_or(&empty)
        } else {
            &empty
        };
        let dived_here = if nl > 2 {
            dived.get(&(name.clone(), k)).unwrap_or(&empty)
        } else {
            &empty
        };
        for crossing in &whole.crossings {
            let same = whole.domain.branch.get(&crossing.0).is_some()
                && whole.domain.branch.get(&crossing.0) == whole.domain.branch.get(&crossing.1);
            let interval = if same {
                bin_in_frame(&whole.domain, &name, &crossing.0, lo, hi)
            } else if name == "T" {
                Some([lo, hi])
            } else {
                None
            };
            let Some([a, b]) = interval else { continue };
            let window = whole.windows[crossing];
            if window[1] < a
                || window[0] >= b
                || (!named_here.is_empty()
                    && !named_here.contains(&crossing.0)
                    && !named_here.contains(&crossing.1))
            {
                continue;
            }
            let inside = in_bin(
                &mut whole.model,
                whole.times[crossing],
                a,
                b,
                whole.domain.grid,
                None,
            )?;
            whole.objective_terms.push((
                inside.as_int_var(),
                super::numeric::checked_round_i64(1000.0 * area_cross * count as f64)?,
            ));
        }
        for lane in &frame.m {
            let Some([a, b]) = bin_in_frame(&whole.domain, &name, lane, lo, hi) else {
                continue;
            };
            let lanes = if !dived_here.is_empty() {
                dived_here
            } else {
                named_here
            };
            if whole.domain.end[lane] < a
                || whole.domain.entry[lane] >= b
                || (!named_here.is_empty() && !lanes.contains(lane))
            {
                continue;
            }
            let pair_factor = if problem.context.pairs.contains_key(lane) {
                2.0
            } else {
                1.0
            };
            let coefficient =
                super::numeric::checked_round_i64(1000.0 * area_via * pair_factor * count as f64)?;
            let changes = whole.changes[lane].clone();
            for (&position, &active) in changes.positions.iter().zip(&changes.active) {
                let inside = in_bin(
                    &mut whole.model,
                    position,
                    a,
                    b,
                    whole.domain.grid,
                    Some(active),
                )?;
                whole
                    .objective_terms
                    .push((inside.as_int_var(), coefficient));
            }
        }
    }
    if let Some(hint) = &problem.hint {
        for (key, crossing) in &hint.cross {
            let Some((a, b)) = key.split_once('|') else {
                return Err("AWX malformed crossing hint key".into());
            };
            let normal = (a.to_owned(), b.to_owned());
            let reverse = (b.to_owned(), a.to_owned());
            if let Some(&variable) = whole
                .times
                .get(&normal)
                .or_else(|| whole.times.get(&reverse))
            {
                whole
                    .model
                    .hint(variable, quantize(crossing.u, whole.domain.grid)?);
            }
        }
        for (lane, positions) in &hint.changes {
            let changes = whole
                .changes
                .get(lane)
                .ok_or("AWX hint names an unknown lane")?;
            for k in 0..KMAX {
                if k < positions.len() {
                    whole.model.hint(
                        changes.positions[k],
                        quantize(positions[k], whole.domain.grid)?,
                    );
                    whole.model.hint(changes.active[k].as_int_var(), 1);
                } else {
                    whole.model.hint(changes.active[k].as_int_var(), 0);
                }
            }
        }
        for (lane, layers) in &hint.layers {
            let Some(changes) = whole.changes.get(lane) else {
                continue;
            };
            if changes.layers.is_empty()
                || !layers
                    .iter()
                    .all(|l| problem.options.routing_layers.contains(l))
            {
                continue;
            }
            if layers.is_empty() {
                return Err("AWX empty run-layer hint".into());
            }
            for k in 0..=KMAX {
                let layer = &layers[k.min(layers.len() - 1)];
                let local = problem
                    .options
                    .routing_layers
                    .iter()
                    .position(|l| l == layer)
                    .unwrap();
                whole.model.hint(changes.layers[k], local as i64);
            }
        }
    }
    Ok(())
}
const W_V: i64 = 1_000_000;
const W_OVER: i64 = 5 * W_V;
const W_SOFT: i64 = 3 * W_V;
const W_FIRM: i64 = 100 * W_V;
fn python_json(value: &serde_json::Value) -> Result<String, String> {
    use serde_json::Value;
    Ok(match value {
        Value::Null => "null".into(),
        Value::Bool(v) => {
            if *v {
                "true".into()
            } else {
                "false".into()
            }
        }
        Value::String(v) => serde_json::to_string(v).map_err(|e| e.to_string())?,
        Value::Number(v) => v.to_string(),
        Value::Array(values) => format!(
            "[{}]",
            values
                .iter()
                .map(python_json)
                .collect::<Result<Vec<_>, _>>()?
                .join(", ")
        ),
        Value::Object(values) => {
            let mut keys: Vec<_> = values.keys().collect();
            keys.sort();
            format!(
                "{{{}}}",
                keys.into_iter()
                    .map(|key| Ok(format!(
                        "{}: {}",
                        serde_json::to_string(key).map_err(|e| e.to_string())?,
                        python_json(&values[key])?
                    )))
                    .collect::<Result<Vec<_>, String>>()?
                    .join(", ")
            )
        }
    })
}
struct Objective {
    terms: Vec<(IntVar, i64)>,
    over: BTreeMap<String, IntVar>,
    stub_vias: BTreeMap<String, i64>,
    drops: BTreeMap<String, Vec<BoolVar>>,
    signature: String,
    root: Option<super::whole_types::RootProof>,
}
fn build_objective(problem: &WholeProblem, whole: &mut WholeModel) -> Result<Objective, String> {
    use sha1::{Digest, Sha1};
    let ctx = &problem.context;
    let frame = &problem.frame;
    let mut leg_vias = BTreeMap::new();
    for n in &frame.m {
        let legs = ctx
            .pairs
            .get(n)
            .map(|x| x.to_vec())
            .unwrap_or_else(|| vec![n.clone()]);
        for leg in legs {
            if leg_vias.contains_key(&leg) {
                continue;
            }
            let nid = *ctx.byname.get(&leg).ok_or("AWX stub leg net id missing")?;
            let vias: Vec<_> = ctx.base_vias.iter().filter(|v| v.net_id == nid).collect();
            let pads: Vec<_> = ctx.pads.iter().filter(|p| p.net_id == nid).collect();
            let ties: Vec<_> = pads
                .iter()
                .copied()
                .filter(|p| {
                    p.component_ref == frame.dst
                        && pads
                            .iter()
                            .any(|q| super::pairs::under_pad(p, q, problem.rules.via_size))
                })
                .collect();
            let tie = vias.iter().any(|v| {
                ties.iter().any(|p| {
                    (v.point[0] - p.point[0]).abs() <= p.size[0] / 2.0
                        && (v.point[1] - p.point[1]).abs() <= p.size[1] / 2.0
                })
            });
            let count = i64::try_from(vias.len()).map_err(|_| "AWX via count overflow")?;
            leg_vias.insert(leg, count - i64::from(tie));
        }
    }
    let mut stub_vias = BTreeMap::new();
    for n in &frame.m {
        let count = if let Some(legs) = ctx.pairs.get(n) {
            leg_vias[&legs[0]].max(leg_vias[&legs[1]])
        } else {
            leg_vias[n]
        };
        stub_vias.insert(n.clone(), count);
    }
    let mut drops: BTreeMap<String, Vec<BoolVar>> = BTreeMap::new();
    for (n, end) in &whole.domain.via_ends {
        let mut necks = BTreeSet::new();
        let legs = ctx
            .pairs
            .get(n)
            .map(|x| x.to_vec())
            .unwrap_or_else(|| vec![n.clone()]);
        for leg in legs {
            let component = if *end == 0 {
                ctx.src_ref.get(&leg).map(String::as_str)
            } else {
                Some(frame.dst.as_str())
            };
            let nid = ctx.byname[&leg];
            let copper: BTreeSet<_> = ctx
                .pads
                .iter()
                .filter(|p| p.net_id == nid && component == Some(p.component_ref.as_str()))
                .flat_map(|p| p.layers.iter())
                .filter(|l| problem.options.routing_layers.contains(l))
                .cloned()
                .collect();
            if copper.len() == 1 {
                necks.extend(copper.into_iter().map(Some));
            } else {
                necks.insert(None);
            }
        }
        if necks.len() == 1 && !necks.contains(&None) {
            let name = necks.iter().next().unwrap().as_ref().unwrap();
            let local = problem
                .options
                .routing_layers
                .iter()
                .position(|l| l == name)
                .unwrap() as i64;
            let layer = whole.changes[n].layers[if *end == 0 { 0 } else { KMAX }];
            let dropped = whole.model.bool("");
            whole.model.eq(&[(layer, 1)], local, &[dropped])?;
            whole.model.ne(&[(layer, 1)], local, &[!dropped])?;
            drops.entry(n.clone()).or_default().push(dropped);
        }
    }
    let mut over = BTreeMap::new();
    let mut terms = whole.objective_terms.clone();
    for n in &frame.m {
        let excess = whole.model.int(
            0,
            checked_add(KMAX as i64, stub_vias[n])?,
            &format!("over_{n}"),
        )?;
        let mut constraint = vec![(excess, 1)];
        for &active in &whole.totals[n] {
            constraint.push((active.as_int_var(), -1));
            terms.push((active.as_int_var(), W_V));
        }
        for &dropped in drops.get(n).into_iter().flatten() {
            constraint.push((dropped.as_int_var(), 1));
            terms.push((dropped.as_int_var(), -W_V));
        }
        whole.model.ge(&constraint, stub_vias[n] - 2, &[])?;
        over.insert(n.clone(), excess);
        terms.push((excess, W_OVER));
    }
    for &broken in whole.soft_broken.values() {
        terms.push((broken.as_int_var(), W_SOFT));
    }
    for &broken in whole.firm_broken.values() {
        terms.push((broken.as_int_var(), W_FIRM));
    }
    for (_, excess) in &whole.gate_over {
        terms.push((*excess, W_FIRM));
    }
    let mut members = frame.m.clone();
    members.sort();
    let states: Vec<_> = members
        .iter()
        .map(|n| {
            serde_json::json!([
                n,
                i64::from(whole.domain.tooth_layer[n] == 1),
                i64::from(whole.domain.berth_layer[n] == 1),
                stub_vias[n]
            ])
        })
        .collect();
    let built: Vec<_> = whole.via_cuts[whole.filed_via_count..]
        .iter()
        .map(|c| {
            Ok(serde_json::json!([
                c.lane,
                round_digits(c.u, 4)?,
                round_digits(c.w, 4)?
            ]))
        })
        .collect::<Result<_, String>>()?;
    let mut signature_data = vec![
        serde_json::json!(members),
        serde_json::json!(frame.launch),
        serde_json::json!(frame.final_order),
        serde_json::json!(states),
        serde_json::json!(KMAX),
        serde_json::json!(2),
        serde_json::json!(built),
    ];
    if problem.options.routing_layers.len() > 2 {
        signature_data.push(serde_json::json!(problem.options.routing_layers));
        signature_data.push(serde_json::json!(frame
            .m
            .iter()
            .collect::<BTreeSet<_>>()
            .iter()
            .map(|n| serde_json::json!([
                n,
                whole.domain.tooth_layer[*n],
                whole.domain.berth_layer[*n]
            ]))
            .collect::<Vec<_>>()));
        signature_data.push(serde_json::json!(whole.domain.via_ends));
        signature_data.push(serde_json::json!(whole
            .domain
            .via_bans
            .iter()
            .map(|(key, v)| {
                let mut names: Vec<_> = v
                    .iter()
                    .map(|&index| problem.options.routing_layers[index as usize].clone())
                    .collect();
                names.sort();
                serde_json::json!([key, names])
            })
            .collect::<Vec<_>>()));
        signature_data.push(serde_json::json!(whole.domain.separated_ends));
        signature_data.push(serde_json::json!(whole.domain.tooth_vias));
        signature_data.push(serde_json::json!(whole.domain.berth_vias));
    }
    let signature = format!(
        "{:x}",
        Sha1::digest(python_json(&serde_json::Value::Array(signature_data))?.as_bytes())
    );
    let root = problem
        .hint
        .as_ref()
        .and_then(|h| h.root.as_ref())
        .filter(|root| root.sig == signature)
        .cloned();
    if let Some(root) = &root {
        let mut floor = vec![];
        for &v in over.values() {
            floor.push((v, W_OVER));
        }
        for n in &frame.m {
            floor.extend(whole.totals[n].iter().map(|v| (v.as_int_var(), W_V)));
            floor.extend(
                drops
                    .get(n)
                    .into_iter()
                    .flatten()
                    .map(|v| (v.as_int_var(), -W_V)),
            );
        }
        let value = super::numeric::checked_add(
            super::numeric::checked_mul(W_OVER, root.over)?,
            super::numeric::checked_mul(W_V, root.vias)?,
        )?;
        whole.model.ge(&floor, value, &[])?;
    }
    whole.model.minimize(&terms);
    Ok(Objective {
        terms,
        over,
        stub_vias,
        drops,
        signature,
        root,
    })
}
struct Progress {
    stall: usize,
    plan: bool,
    limit: usize,
    fallback: bool,
}
impl Progress {
    fn line(&mut self, line: &str) -> bool {
        let solution = line
            .strip_prefix('#')
            .is_some_and(|s| s.chars().next().is_some_and(|c| c.is_ascii_digit()));
        if line.starts_with("#Bound") || solution {
            self.stall = 0;
            self.plan |= solution;
            let best = line
                .split("best:")
                .nth(1)
                .and_then(|s| s.split_whitespace().next())
                .and_then(|s| s.parse::<f64>().ok());
            let lower = line
                .split("next:[")
                .nth(1)
                .and_then(|s| s.split([',', ']']).next())
                .and_then(|s| s.trim().parse::<f64>().ok());
            if let (Some(best), Some(lower)) = (best, lower) {
                return best.is_finite()
                    && (best / W_V as f64).floor() <= (lower / W_V as f64).floor();
            }
        } else if line.starts_with("#Model") && self.plan && !self.fallback {
            self.stall = self.stall.saturating_add(1);
            return self.stall >= self.limit;
        }
        false
    }
}
fn feasible(response: &super::cp_model::CpSolverResponse) -> bool {
    matches!(
        response.status(),
        super::cp_model::CpSolverStatus::Optimal | super::cp_model::CpSolverStatus::Feasible
    )
}
fn proved(response: &super::cp_model::CpSolverResponse) -> bool {
    feasible(response)
        && (response.objective_value / W_V as f64).floor()
            <= (response.best_objective_bound / W_V as f64).floor()
}
fn run_model(
    problem: &WholeProblem,
    model: &Model,
    subsolvers: &[String],
    fallback: bool,
    deadline: Option<std::time::Instant>,
) -> Result<super::cp_model::CpSolverResponse, String> {
    let mut params = super::cp_model::SatParameters::default();
    params.num_workers = Some(problem.options.workers);
    params.subsolvers = subsolvers.to_vec();
    params.interleave_search = Some(true);
    params.max_num_deterministic_batches = Some(problem.options.deterministic_batches);
    params.share_glue_clauses = Some(false);
    params.share_binary_clauses = Some(false);
    params.log_search_progress = Some(true);
    params.log_to_stdout = Some(false);
    let mut progress = Progress {
        stall: 0,
        plan: false,
        limit: problem.options.stall_model_reductions,
        fallback,
    };
    if let Some(deadline) = deadline {
        params.max_time_in_seconds = Some(
            deadline
                .saturating_duration_since(std::time::Instant::now())
                .as_secs_f64(),
        );
    }
    let mut callback = |line: &str| {
        deadline.is_some_and(|d| std::time::Instant::now() >= d) || progress.line(line)
    };
    super::cp_sat_bridge::solve(&model.inner, &params, Some(&mut callback))
}
fn response_value(response: &super::cp_model::CpSolverResponse, index: i32) -> Result<i64, String> {
    let index = usize::try_from(index).map_err(|_| "AWX negative solution variable index")?;
    response
        .solution
        .get(index)
        .copied()
        .ok_or("AWX incomplete native solution".into())
}
fn evaluate_terms(
    response: &super::cp_model::CpSolverResponse,
    terms: &[(IntVar, i64)],
) -> Result<i64, String> {
    let mut total = 0;
    for (var, coefficient) in terms {
        total = super::numeric::checked_add(
            total,
            super::numeric::checked_mul(response_value(response, var.index())?, *coefficient)?,
        )?;
    }
    Ok(total)
}
fn total_changes(
    response: &super::cp_model::CpSolverResponse,
    whole: &WholeModel,
    lane: &str,
) -> Result<i64, String> {
    let mut count = 0;
    for &active in &whole.totals[lane] {
        count = checked_add(count, response_value(response, active.index())?)?;
    }
    Ok(count)
}
fn total_dropped(
    response: &super::cp_model::CpSolverResponse,
    objective: &Objective,
    lane: &str,
) -> Result<i64, String> {
    let mut count = 0;
    for &dropped in objective.drops.get(lane).into_iter().flatten() {
        count = checked_add(count, response_value(response, dropped.index())?)?;
    }
    Ok(count)
}
fn decode_plan(
    problem: &WholeProblem,
    whole: &WholeModel,
    objective: &Objective,
    response: &super::cp_model::CpSolverResponse,
    is_proved: bool,
) -> Result<super::whole_types::WholePlan, String> {
    use super::whole_types::{Crossing, RootProof, UnprovedPlan, WholePlan};
    let frame = &problem.frame;
    let nl = problem.options.routing_layers.len();
    let mut changes = BTreeMap::new();
    let mut layers = BTreeMap::new();
    let mut over_nets = BTreeMap::new();
    let mut sum_changes = 0;
    let mut sum_drops = 0;
    let mut sum_over = 0;
    for n in &frame.m {
        let lane = &whole.changes[n];
        let mut positions = vec![];
        for (&position, &active) in lane.positions.iter().zip(&lane.active) {
            if response_value(response, active.index())? != 0 {
                positions
                    .push(response_value(response, position.index())? as f64 * whole.domain.grid);
            }
        }
        if nl > 2 {
            let mut run_layers = vec![];
            for k in 0..=positions.len() {
                let local = response_value(response, lane.layers[k].index())?;
                let name = problem
                    .options
                    .routing_layers
                    .get(usize::try_from(local).map_err(|_| "AWX negative solved layer")?)
                    .ok_or("AWX solved layer outside stack")?
                    .clone();
                run_layers.push(name);
            }
            layers.insert(n.clone(), run_layers);
        }
        let count = total_changes(response, whole, n)?;
        let dropped = total_dropped(response, objective, n)?;
        sum_changes = checked_add(sum_changes, count)?;
        sum_drops = checked_add(sum_drops, dropped)?;
        sum_over = checked_add(
            sum_over,
            response_value(response, objective.over[n].index())?,
        )?;
        let excess = objective.stub_vias[n] - dropped + count - 2;
        if excess > 0 {
            over_nets.insert(n.clone(), excess);
        }
        changes.insert(n.clone(), positions);
    }
    let layer_at = |n: &String, u: f64| -> Result<i64, String> {
        let count = changes[n].iter().filter(|&&change| change < u).count();
        if nl > 2 {
            response_value(response, whole.changes[n].layers[count].index())
        } else {
            Ok(whole.domain.tooth_layer[n] ^ (count as i64 & 1))
        }
    };
    let mut cross = BTreeMap::new();
    let mut movers = BTreeMap::new();
    for key in &whole.crossings {
        let u = response_value(response, whole.times[key].index())? as f64 * whole.domain.grid;
        if layer_at(&key.0, u)? == layer_at(&key.1, u)? {
            return Err(format!(
                "AWX crossing {}|{} solved on one layer",
                key.0, key.1
            ));
        }
        let ring = whole.domain.branch.get(&key.0).is_some()
            && whole.domain.branch.get(&key.0) == whole.domain.branch.get(&key.1)
            && u > whole.domain.handoff[&key.0] + 1e-9;
        let name = format!("{}|{}", key.0, key.1);
        cross.insert(name.clone(), Crossing { u, ring });
        movers.insert(
            name,
            if response_value(response, whole.mover[key].index())? != 0 {
                key.0.clone()
            } else {
                key.1.clone()
            },
        );
    }
    for n in &frame.m {
        if !whole.domain.via_ends.contains(&(n.clone(), 1))
            && layer_at(n, 1e9)? != whole.domain.berth_layer[n]
        {
            return Err(format!("AWX solved lane {n} misses its berth layer"));
        }
    }
    let no_island_cuts = problem.cuts.iter().all(|g| g.cuts.is_empty());
    let no_soft = problem
        .soft_cuts
        .iter()
        .all(|g| g.cuts.is_empty() && g.vcuts.is_empty());
    let root = objective.root.clone().or_else(|| {
        if no_island_cuts && no_soft && whole.filed_via_count == 0 && is_proved {
            Some(RootProof {
                over: sum_over,
                vias: sum_changes - sum_drops,
                sig: objective.signature.clone(),
            })
        } else {
            None
        }
    });
    let mut gate_over = vec![];
    for (metadata, var) in &whole.gate_over {
        let over = response_value(response, var.index())?;
        if over > 0 {
            let data = metadata.as_array().ok_or("AWX gate metadata malformed")?;
            gate_over.push(serde_json::json!([
                data[0], data[1], data[2], data[3], over
            ]));
        }
    }
    let mut firm_broken = vec![];
    for (key, var) in &whole.firm_broken {
        if response_value(response, var.index())? != 0 {
            firm_broken.push(cut_json(key));
        }
    }
    let mut soft_broken = vec![];
    for (key, var) in &whole.soft_broken {
        if response_value(response, var.index())? != 0 {
            soft_broken.push(cut_json(key));
        }
    }
    let unproved = if !is_proved {
        Some(UnprovedPlan {
            best: response.objective_value,
            bound: response.best_objective_bound,
            over: sum_over,
            bound_over: super::numeric::checked_round_i64(
                (response.best_objective_bound / W_OVER as f64).floor(),
            )?,
        })
    } else {
        None
    };
    let crossover_lanes = frame
        .m
        .iter()
        .filter(|n| problem.context.pairs.contains_key(*n) && opposite_hands(problem, n))
        .cloned()
        .collect();
    Ok(WholePlan {
        arrival: frame.h0,
        handoff: frame.hk.clone(),
        rs: whole.domain.ring_start.clone(),
        cross,
        changes,
        entry: whole.domain.entry.clone(),
        end: whole.domain.end.clone(),
        tend: whole.domain.trunk_end.clone(),
        branch: whole.domain.branch.clone(),
        final_order: frame.final_order.clone(),
        launch: frame.launch.clone(),
        layers,
        root,
        proved: is_proved,
        gate_over,
        firm_broken,
        soft_broken,
        over_nets,
        unproved,
        crossover_lanes,
        movers,
    })
}
pub fn solve(problem: &WholeProblem) -> Result<super::whole_types::WholeOutcome, String> {
    solve_until(problem, None)
}
pub fn solve_until(
    problem: &WholeProblem,
    deadline: Option<std::time::Instant>,
) -> Result<super::whole_types::WholeOutcome, String> {
    use super::cp_model::{CpSolverStatus, SatParameters};
    use super::whole_types::WholeOutcome;
    problem.context.validate()?;
    if problem.options.workers <= 0 || problem.options.deterministic_batches < 0 {
        return Err("AWX invalid worker or work budget".into());
    }
    if problem
        .options
        .routing_layers
        .iter()
        .collect::<BTreeSet<_>>()
        .len()
        != problem.options.routing_layers.len()
    {
        return Err("AWX duplicate routing layer".into());
    }
    let domain = prepare_domain(problem)?;
    let mut whole = build_crossing_model(problem, domain)?;
    add_spacing_constraints(problem, &mut whole)?;
    add_geometry_cuts(problem, &mut whole)?;
    add_part_gates(problem, &mut whole)?;
    add_history_and_hints(problem, &mut whole)?;
    let objective = build_objective(problem, &mut whole)?;
    // 1165–1178: complete the fixed warm hints at an existing root floor with one worker.
    let mut warm = None;
    if problem.hint.is_some() {
        if let Some(root) = &objective.root {
            let mut params = SatParameters::default();
            params.num_workers = Some(1);
            params.fix_variables_to_their_hinted_value = Some(true);
            params.max_deterministic_time = Some(problem.options.warm_deterministic_time);
            if let Some(deadline) = deadline {
                params.max_time_in_seconds = Some(
                    deadline
                        .saturating_duration_since(std::time::Instant::now())
                        .as_secs_f64(),
                );
            }
            let response = super::cp_sat_bridge::solve(&whole.model.inner, &params, None)?;
            let floor = super::numeric::floor_div(
                super::numeric::checked_add(
                    super::numeric::checked_mul(W_OVER, root.over)?,
                    super::numeric::checked_mul(W_V, root.vias)?,
                )?,
                W_V,
            )?;
            if feasible(&response)
                && (response.objective_value / W_V as f64).floor() <= floor as f64
            {
                warm = Some(response);
            }
        }
    }
    let mut response = run_model(
        problem,
        &whole.model,
        &problem.options.primary_subsolvers,
        false,
        deadline,
    )?;
    let mut is_proved = proved(&response);
    if !is_proved {
        if let Some(warm) = warm {
            response = warm;
            is_proved = true;
        }
    }
    // 1179–1201: the fallback starts from every solved value and holds the attained lower bound.
    if !is_proved {
        if feasible(&response) {
            whole.model.clear_hints();
            let count = whole.model.inner.to_proto().variables.len();
            if response.solution.len() < count {
                return Err("AWX fallback solution cardinality mismatch".into());
            }
            whole.model.inner.raw_proto_mut().solution_hint =
                Some(cpsat_rs::proto::PartialVariableAssignment {
                    vars: (0..count).map(|i| i as i32).collect(),
                    values: response.solution[..count].to_vec(),
                });
        }
        let bound = (response.best_objective_bound - 1e-6).ceil();
        if !bound.is_finite() {
            return Err("AWX nonfinite objective bound".into());
        }
        whole.model.ge(
            &objective.terms,
            super::numeric::checked_round_i64(bound)?,
            &[],
        )?;
        response = run_model(
            problem,
            &whole.model,
            &problem.options.fallback_subsolvers,
            true,
            deadline,
        )?;
        is_proved = proved(&response);
    }
    let keep = !is_proved
        && response.status() == CpSolverStatus::Feasible
        && problem.options.allow_unproved;
    if !is_proved && !keep {
        return Ok(WholeOutcome::NoProvedPlan {
            status: response.status().as_str_name().into(),
            best: if feasible(&response) {
                Some(response.objective_value)
            } else {
                None
            },
            bound: response
                .best_objective_bound
                .is_finite()
                .then_some(response.best_objective_bound),
        });
    }
    // Decode exact integer cost independently of the floating native summary.
    let exact = evaluate_terms(&response, &objective.terms)?;
    if (response.objective_value - exact as f64).abs() > 0.5 {
        return Err("AWX exact objective mismatch".into());
    }
    Ok(WholeOutcome::Plan(decode_plan(
        problem, &whole, &objective, &response, is_proved,
    )?))
}
