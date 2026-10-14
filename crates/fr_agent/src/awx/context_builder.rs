//! AWX whole_ctx._plan / braid.setup context producer from actual native joint-escape copper.
use super::{
    board_snapshot::BoardSnapshot,
    corridor::{self, Point},
    joint_escape::{JointPlan, OptionMove},
    numeric, pairs,
    whole_context::{Ends, RoutingRules, WholeContext},
    whole_frame::{self, FlowFrame, WholeFrame},
};
use std::collections::{BTreeMap, BTreeSet};
pub struct ContextBuild {
    pub context: WholeContext,
    pub frame: WholeFrame,
    pub flow: FlowFrame,
    pub blocked_fronts: BTreeMap<String, BTreeMap<usize, [f64; 4]>>,
}
fn pad_box(snapshot: &BoardSnapshot, part: &str) -> Result<[f64; 4], String> {
    let points: Vec<_> = snapshot
        .pads
        .iter()
        .filter(|p| p.component_ref == part)
        .map(|p| p.point)
        .collect();
    if points.is_empty() {
        return Err(format!("AWX context: missing logical array {part}"));
    }
    Ok([
        points.iter().map(|p| p[0]).fold(f64::INFINITY, f64::min),
        points.iter().map(|p| p[1]).fold(f64::INFINITY, f64::min),
        points
            .iter()
            .map(|p| p[0])
            .fold(f64::NEG_INFINITY, f64::max),
        points
            .iter()
            .map(|p| p[1])
            .fold(f64::NEG_INFINITY, f64::max),
    ])
}
fn point_key(p: Point) -> Result<(i64, i64), String> {
    Ok((
        numeric::checked_round_i64(numeric::round_digits(p[0], 3)? * 1000.)?,
        numeric::checked_round_i64(numeric::round_digits(p[1], 3)? * 1000.)?,
    ))
}
struct NodeSnapper {
    seen: BTreeMap<(i64, i64), Vec<Point>>,
}
impl NodeSnapper {
    fn key(&mut self, p: Point) -> Result<(i64, i64), String> {
        let b = [(p[0] / 0.012).floor() as i64, (p[1] / 0.012).floor() as i64];
        for dx in -1..=1 {
            for dy in -1..=1 {
                if let Some(rows) = self.seen.get(&(b[0] + dx, b[1] + dy)) {
                    for q in rows {
                        if (q[0] - p[0]).abs() <= 0.006 && (q[1] - p[1]).abs() <= 0.006 {
                            return point_key(*q);
                        }
                    }
                }
            }
        }
        let q = [
            numeric::round_digits(p[0], 3)?,
            numeric::round_digits(p[1], 3)?,
        ];
        self.seen.entry((b[0], b[1])).or_default().push(q);
        point_key(q)
    }
}
/// braid._end_dir: walk the real stub to its pads and take its longest segment, pointing out.
fn end_direction(snapshot: &BoardSnapshot, net: i32, point: Point) -> Result<Point, String> {
    let mut snap = NodeSnapper {
        seen: BTreeMap::new(),
    };
    let mut graph: BTreeMap<(i64, i64), Vec<((i64, i64), usize)>> = BTreeMap::new();
    let segments: Vec<_> = snapshot
        .segments
        .iter()
        .filter(|s| s.net_id == net)
        .collect();
    for (i, s) in segments.iter().enumerate() {
        let a = snap.key(s.a)?;
        let b = snap.key(s.b)?;
        graph.entry(a).or_default().push((b, i));
        graph.entry(b).or_default().push((a, i));
    }
    let start = snap.key(point)?;
    let mut seen = BTreeSet::from([start]);
    let mut frontier = vec![start];
    let mut order = vec![];
    while !frontier.is_empty() {
        let mut next = vec![];
        for u in frontier {
            let pt = [u.0 as f64 / 1000., u.1 as f64 / 1000.];
            let inside = snapshot.pads.iter().filter(|p| p.net_id == net).any(|p| {
                (pt[0] - p.point[0]).abs() <= p.size[0] / 2. + 0.02
                    && (pt[1] - p.point[1]).abs() <= p.size[1] / 2. + 0.02
            });
            if inside && u != start {
                continue;
            }
            for &(v, i) in graph.get(&u).into_iter().flatten() {
                if seen.insert(v) {
                    order.push((i, u, v));
                    next.push(v)
                }
            }
        }
        frontier = next;
    }
    let mut best = None;
    for (i, near, far) in order {
        let len = corridor::distance(segments[i].a, segments[i].b);
        if best.as_ref().is_none_or(|(old, _, _)| len > *old) {
            best = Some((len, near, far));
        }
    }
    if let Some((_, near, far)) = best {
        let v = [
            (near.0 - far.0) as f64 / 1000.,
            (near.1 - far.1) as f64 / 1000.,
        ];
        if corridor::length(v) > 1e-6 {
            return corridor::unit(v);
        }
    }
    Ok([1., 0.])
}
fn selected_end(
    snapshot: &BoardSnapshot,
    plan: &JointPlan,
    menus: &super::joint_escape::JointMenus,
    net: &str,
    expected_pad: &super::whole_context::ContextPad,
    destination: &str,
) -> Result<(Point, String), String> {
    let short = super::escape_moves::short_name(net);
    let physical_keys = menus.physical_keys_for_net(short)?;
    let selected: Vec<_> = physical_keys
        .into_iter()
        .filter(|key| plan.chosen.contains_key(*key))
        .collect();
    if selected.len() > 1 {
        return Err(format!(
            "AWX bus {net} has multiple selected physical menu items"
        ));
    }
    let key = selected
        .first()
        .ok_or_else(|| format!("AWX bus {net} has no selected physical menu item"))?;
    let pad = menus
        .items
        .get(*key)
        .ok_or_else(|| format!("AWX bus {net} selected menu item {key} is absent"))?;
    if pad.component_ref != expected_pad.component_ref
        || pad.pad_number != expected_pad.pad_number
        || pad.net_id != expected_pad.net_id
        || corridor::distance(pad.point, expected_pad.point) > 0.005
    {
        return Err(format!(
            "AWX bus {net} selected physical item {key} does not match captured pad {}#{} net_id={} at {:?} (menu {}#{} net_id={} at {:?})",
            expected_pad.component_ref,
            expected_pad.pad_number,
            expected_pad.net_id,
            expected_pad.point,
            pad.component_ref,
            pad.pad_number,
            pad.net_id,
            pad.point
        ));
    }
    match plan.chosen.get(*key) {
        Some(OptionMove::Escape { movement }) => {
            let asked = super::destination_realize::AskedPad {
                net: pad.net.clone(),
                component_ref: pad.component_ref.clone(),
                pad_number: pad.pad_number.clone(),
                point: pad.point,
                size: pad.size,
            };
            let achieved = super::destination_realize::measure_tooth(
                &snapshot.native,
                &asked,
                Some(destination),
            )?
            .ok_or_else(|| format!("AWX bus {net} physical item {key} has no measured endpoint"))?;
            let layer_matches = if movement.layer == super::escape_moves::RUN {
                movement.runs_on.contains(&achieved.layer)
            } else {
                movement.layer == achieved.layer
            };
            if !layer_matches {
                return Err(format!(
                    "AWX bus {net} physical endpoint layer {} disagrees with selected move layer {}",
                    achieved.layer, movement.layer
                ));
            }
            Ok((achieved.tooth, achieved.layer))
        }
        Some(_) => Err(format!(
            "AWX bus {net} was strapped/dropped instead of having a selected escape"
        )),
        None => Err(format!("AWX bus {net} has no chosen joint escape")),
    }
}
fn layers_in_frame(
    ctx: &mut WholeContext,
    flow: FlowFrame,
    physical: &[String],
) -> Result<(), String> {
    for l in &mut ctx.copper_layers {
        *l = flow.to_frame_layer(l, physical)?;
    }
    for pad in &mut ctx.pads {
        for l in &mut pad.layers {
            *l = flow.to_frame_layer(l, physical)?
        }
        for shape in &mut pad.copper_shapes {
            shape.layer = flow.to_frame_layer(&shape.layer, physical)?
        }
    }
    for s in &mut ctx.base_segments {
        s.layer = flow.to_frame_layer(&s.layer, physical)?
    }
    for via in &mut ctx.base_vias {
        for l in &mut via.layers {
            *l = flow.to_frame_layer(l, physical)?
        }
    }
    for l in ctx
        .tooth_layer
        .values_mut()
        .chain(ctx.dest_layer.values_mut())
    {
        *l = flow.to_frame_layer(l, physical)?
    }
    for pair in ctx.pair_layers.values_mut() {
        for l in pair {
            *l = flow.to_frame_layer(l, physical)?
        }
    }
    Ok(())
}
pub fn build(
    snapshot: &BoardSnapshot,
    source: &str,
    destination: &str,
    nets: &BTreeSet<String>,
    explicit_pairs: &[pairs::Pair],
    source_plan: &JointPlan,
    destination_plan: &JointPlan,
    source_menus: &super::joint_escape::JointMenus,
    destination_menus: &super::joint_escape::JointMenus,
    rules: &RoutingRules,
    routing_layers: &[String],
) -> Result<ContextBuild, String> {
    rules.validate()?;
    if source == destination || nets.is_empty() {
        return Err("AWX context requires different arrays and nonempty admitted nets".into());
    }
    if routing_layers.len() < 2
        || routing_layers.iter().collect::<BTreeSet<_>>().len() != routing_layers.len()
        || routing_layers.iter().any(|l| !snapshot.layers.contains(l))
    {
        return Err("AWX routing stack is absent or duplicated".into());
    }
    // The local AWX contract is front copper, back copper, then explicitly
    // declared inner layers. Resolve front/back by their actual native stack
    // numbers: valid DSN stacks use names such as Top/Bottom, while KiCad
    // boards commonly spell the same two positions F.Cu/B.Cu. Preserve the
    // caller's remaining order verbatim; do not synthesize or reorder layers.
    let front = snapshot
        .layers
        .first()
        .ok_or("AWX native stack has no front copper layer")?;
    let back = snapshot
        .layers
        .last()
        .ok_or("AWX native stack has no back copper layer")?;
    if routing_layers[0].as_str() != front.as_str() || routing_layers[1].as_str() != back.as_str() {
        return Err(format!(
            "AWX local routing order must start with native front/back layers {front},{back} then declared inner layers"
        ));
    }
    let mut shortnames: BTreeMap<&str, &str> = BTreeMap::new();
    for name in snapshot.byname.keys() {
        let short = name.rsplit('/').next().unwrap();
        if let Some(old) = shortnames.insert(short, name) {
            if old != name {
                return Err(format!("AWX duplicate net basename {short}: {old}/{name}"));
            }
        }
    }
    let mut admitted = Vec::new();
    let mut endmap = BTreeMap::new();
    let mut tl = BTreeMap::new();
    let mut dl = BTreeMap::new();
    let mut td = BTreeMap::new();
    let mut sd = BTreeMap::new();
    let mut src_ref = BTreeMap::new();
    for net in nets {
        let id = *snapshot
            .byname
            .get(net)
            .ok_or_else(|| format!("AWX missing native net {net}"))?;
        let src: Vec<_> = snapshot
            .pads
            .iter()
            .filter(|p| p.net_id == id && p.component_ref == source)
            .collect();
        let dst: Vec<_> = snapshot
            .pads
            .iter()
            .filter(|p| p.net_id == id && p.component_ref == destination)
            .collect();
        if src.len() != 1 || dst.len() != 1 {
            return Err(format!(
                "AWX {net}: expected one actual pad per array, got {}/{}",
                src.len(),
                dst.len()
            ));
        }
        let (a, al) = selected_end(
            snapshot,
            source_plan,
            source_menus,
            net,
            src[0],
            destination,
        )?;
        let (b, bl) = selected_end(
            snapshot,
            destination_plan,
            destination_menus,
            net,
            dst[0],
            destination,
        )?;
        if !routing_layers.contains(&al) || !routing_layers.contains(&bl) {
            return Err(format!("AWX {net}: selected exit on unplanned layer"));
        }
        admitted.push(net.clone());
        endmap.insert(
            net.clone(),
            Ends {
                tooth: a,
                berth: b,
                destination: destination.into(),
            },
        );
        tl.insert(net.clone(), al);
        dl.insert(net.clone(), bl);
        td.insert(net.clone(), end_direction(snapshot, id, a)?);
        sd.insert(net.clone(), end_direction(snapshot, id, b)?);
        src_ref.insert(net.clone(), source.into());
    }
    let pairs = if explicit_pairs.is_empty() {
        pairs::pair_names(&admitted, true)
    } else {
        explicit_pairs.to_vec()
    };
    let mut pair_names = BTreeSet::new();
    let mut pair_members = BTreeSet::new();
    for p in &pairs {
        if p.p == p.n || !nets.contains(&p.p) || !nets.contains(&p.n) {
            return Err(format!("AWX half/invalid pair {}", p.base));
        }
        if nets.contains(&p.base)
            || snapshot.byname.contains_key(&p.base)
            || !pair_names.insert(p.base.clone())
            || !pair_members.insert(p.p.clone())
            || !pair_members.insert(p.n.clone())
        {
            return Err(format!("AWX conflicting physical pair alias {}", p.base));
        }
    }
    let (lanes, prs) = pairs::members(&admitted, &pairs);
    let mut byname = snapshot.byname.clone();
    let mut pair_ends = BTreeMap::new();
    let mut pair_layers = BTreeMap::new();
    for (base, legs) in &prs {
        let p = &endmap[&legs[0]];
        let n = &endmap[&legs[1]];
        pair_ends.insert(base.clone(), [[p.tooth, n.tooth], [p.berth, n.berth]]);
        endmap.insert(
            base.clone(),
            Ends {
                tooth: pairs::mid(p.tooth, n.tooth),
                berth: pairs::mid(p.berth, n.berth),
                destination: destination.into(),
            },
        );
        byname.insert(base.clone(), byname[&legs[0]]);
        tl.insert(base.clone(), tl[&legs[0]].clone());
        dl.insert(base.clone(), dl[&legs[0]].clone());
        td.insert(base.clone(), td[&legs[0]]);
        sd.insert(base.clone(), sd[&legs[0]]);
        src_ref.insert(base.clone(), source.into());
        pair_layers.insert(base.clone(), [tl[&legs[1]].clone(), dl[&legs[1]].clone()]);
    }
    let sb = pad_box(snapshot, source)?;
    let db = pad_box(snapshot, destination)?;
    let sc = [(sb[0] + sb[2]) / 2., (sb[1] + sb[3]) / 2.];
    let dc = [(db[0] + db[2]) / 2., (db[1] + db[3]) / 2.];
    let axis = corridor::sub(dc, sc);
    let mut moment = 0.;
    let mut sp = vec![];
    let mut dp = vec![];
    for lane in &lanes {
        let id = byname[lane];
        let a = snapshot
            .pads
            .iter()
            .find(|p| p.component_ref == source && p.net_id == id)
            .ok_or("AWX missing source chirality ball")?
            .point;
        let b = snapshot
            .pads
            .iter()
            .find(|p| p.component_ref == destination && p.net_id == id)
            .ok_or("AWX missing destination chirality ball")?
            .point;
        moment += corridor::cross(axis, corridor::sub(a, sc))
            + corridor::cross(axis, corridor::sub(b, sc));
        sp.push(a);
        dp.push(b);
    }
    let mirror = if moment < 0. {
        Some(
            numeric::checked_round_i64(
                ((snapshot.board_bounds[1] + snapshot.board_bounds[3]) / 2.) / 0.05,
            )? as f64
                * 0.05,
        )
    } else {
        None
    };
    let mp = |mut p: Point| {
        if let Some(y) = mirror {
            p[1] = 2. * y - p[1]
        }
        p
    };
    let mut flow = FlowFrame::choose(
        &sp.iter().copied().map(mp).collect::<Vec<_>>(),
        &dp.iter().copied().map(mp).collect::<Vec<_>>(),
        mp(dc),
    )?;
    flow.mirror_axis_y = mirror;
    let native_to_logical_components = snapshot
        .pads
        .iter()
        .map(|p| (p.key.clone(), p.component_ref.clone()))
        .collect();
    let plane_nets = snapshot
        .zones
        .iter()
        .filter_map(|z| {
            snapshot
                .byname
                .iter()
                .find(|(_, n)| **n == z.net_id)
                .map(|(name, _)| name.clone())
        })
        .collect();
    let mut ctx = WholeContext {
        cfg: rules.clone(),
        layers: routing_layers.to_vec(),
        copper_layers: snapshot.layers.clone(),
        board_bounds: snapshot.board_bounds,
        board_outline: snapshot.board_outline.clone(),
        pads: snapshot.pads.clone(),
        source: source.into(),
        destination: destination.into(),
        lanes,
        byname,
        pairs: prs,
        ends: endmap,
        src_ref,
        tooth_layer: tl,
        dest_layer: dl,
        tooth_dir: td,
        stub_dir: sd,
        pair_ends,
        pair_layers,
        paths: BTreeMap::new(),
        base_segments: snapshot.segments.clone(),
        base_vias: snapshot.vias.clone(),
        dest_cut: None,
        plane_nets,
        native_to_logical_components,
        chirality_mirrored: mirror.is_some(),
    };
    ctx = ctx.transformed(flow);
    layers_in_frame(&mut ctx, flow, &snapshot.layers)?;
    // Default production path: the mature batched taut_fast relaxation with taut_clean assessment/reseeding.
    let back = ctx
        .lanes
        .iter()
        .filter(|n| ctx.tooth_layer[*n] == "B.Cu")
        .count();
    let bundle = if 2 * back > ctx.lanes.len() {
        "B.Cu"
    } else {
        "F.Cu"
    };
    ctx.paths = super::taut_paths::paths(&ctx, bundle)?;
    ctx.validate()?;
    let frame = whole_frame::build(&ctx, destination)
        .map_err(|e| format!("AWX whole-frame refused actual endpoints: {e:?}"))?;
    // whole_ends 737–744 emits front spans only for exits already named by feedback;
    // the first fanout has none. Subsequent callers supply the selected EndsModel.front.
    let blocked_fronts = BTreeMap::new();
    Ok(ContextBuild {
        context: ctx,
        frame,
        flow,
        blocked_fronts,
    })
}

pub fn build_with_fronts(
    snapshot: &BoardSnapshot,
    source: &str,
    destination: &str,
    nets: &BTreeSet<String>,
    explicit_pairs: &[pairs::Pair],
    source_plan: &JointPlan,
    destination_plan: &JointPlan,
    source_menus: &super::joint_escape::JointMenus,
    destination_menus: &super::joint_escape::JointMenus,
    rules: &RoutingRules,
    routing_layers: &[String],
    fronts: &BTreeMap<String, BTreeMap<usize, [f64; 4]>>,
) -> Result<ContextBuild, String> {
    let mut out = build(
        snapshot,
        source,
        destination,
        nets,
        explicit_pairs,
        source_plan,
        destination_plan,
        source_menus,
        destination_menus,
        rules,
        routing_layers,
    )?;
    for (n, ends) in fronts {
        if !out.context.lanes.contains(n) || out.context.pairs.contains_key(n) {
            return Err(format!("AWX front feedback names invalid single lane {n}"));
        }
        for (&end, span) in ends {
            if end > 1
                || span.iter().any(|x| !x.is_finite())
                || span[0] > span[1]
                || span[2] > span[3]
            {
                return Err(format!("AWX front feedback has invalid span for {n}"));
            }
        }
    }
    out.blocked_fronts = fronts.clone();
    Ok(out)
}
