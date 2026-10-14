//! Native materialization and per-net rollback for the AWX driver; original boards are never modified in place.
use super::{
    corridor::Point,
    whole_context::{InstalledPlan, RoutingRules},
    whole_frame::FlowFrame,
    whole_geometry::GeometryResult,
};
use crate::board_adapter::BoardAdapter;
use fr_engine::{
    drc::NetIncompletes,
    ids::FixedState,
    rules::{ViaInfo, ViaInfoId, default_item_clearance_classes::ItemClass},
};
use fr_geom::{Circle, ConvexShape, IntPoint, Polyline};
use serde::Serialize;
use std::collections::{BTreeMap, BTreeSet};
#[derive(Clone, Debug, Serialize)]
pub struct NativeGrade {
    pub connected: Vec<String>,
    pub open: Vec<String>,
    pub new_violations: usize,
    pub native_violations: usize,
    pub vias: usize,
    pub copper_mm: f64,
}
pub fn strip_nets(board: &mut BoardAdapter, names: &BTreeSet<String>) -> Result<(), String> {
    let ids: BTreeSet<_> = names
        .iter()
        .map(|n| board.net_number(n))
        .collect::<Result<_, _>>()?;
    let selected: Vec<_> = board
        .board
        .get_items()
        .into_iter()
        .filter(|&k| {
            let item = board.board.item(k);
            (item.is_trace() || item.is_via()) && item.net_numbers().iter().any(|n| ids.contains(n))
        })
        .collect();
    for &k in &selected {
        let item = board.board.item(k);
        if item.net_numbers().iter().any(|n| !ids.contains(n)) {
            return Err("Unsupported: selected bus copper also belongs to another net".into());
        }
    }
    let remove: Vec<_> = selected
        .into_iter()
        .filter(|&key| !board.is_source_protected_copper(key))
        .collect();
    let mut candidate = board.clone();
    for &key in &remove {
        let item = candidate.board.item(key);
        if item.component_no() > 0
            || item.fixed_state() == FixedState::SystemFixed
        {
            return Err("AWX selected copper includes source-protected board geometry".into());
        }
        // The selected net set is the rollback transaction boundary. AWX materialized
        // copper is UserFixed so the router cannot shove it; temporarily lower that state
        // only on this private candidate while removing the explicitly selected copper.
        if item.is_user_fixed() {
            candidate.board.item_mut(key).set_fixed_state(FixedState::Unfixed);
        }
        candidate.board.remove_item(key);
        if candidate.board.item(key).is_on_board() {
            return Err("AWX could not remove all selected copper".into());
        }
    }
    *board = candidate;
    Ok(())
}
pub fn via(
    board: &mut BoardAdapter,
    net: &str,
    p: Point,
    diameter: f64,
    drill: f64,
) -> Result<(), String> {
    let n = board.net_number(net)?;
    let via_info = ensure_via_candidate(board, net, diameter, drill)?;
    let pad = board.board.rules.via_infos[via_info].get_padstack();
    let class = board
        .board
        .rules
        .net_class_of(n)
        .get_trace_clearance_class();
    let point = board.point(p)?;
    board
        .board
        .insert_via(pad, point, &[n], class, FixedState::UserFixed, true);
    Ok(())
}

/// Ensure an exact native padstack and ViaInfo exist for a requested fabrication size.
/// Dimensions are rounded once to the board's native unit, and the explicit drill radius
/// preserves the resulting physical value. This deliberately leaves shared net-class via
/// rules unchanged; callers can build a temporary search rule from the returned ViaInfoId.
pub fn ensure_via_candidate(
    board: &mut BoardAdapter,
    net: &str,
    diameter_mm: f64,
    drill_mm: f64,
) -> Result<ViaInfoId, String> {
    if !diameter_mm.is_finite()
        || !drill_mm.is_finite()
        || diameter_mm <= drill_mm
        || drill_mm <= 0.0
    {
        return Err("invalid native AWX fabrication via".into());
    }
    let radius = (diameter_mm * board.units_per_mm / 2.0).round();
    let drill_diameter = (drill_mm * board.units_per_mm).round();
    if !radius.is_finite()
        || !drill_diameter.is_finite()
        || radius <= 0.0
        || radius > (i32::MAX / 2) as f64
        || drill_diameter <= 0.0
        || drill_diameter > i32::MAX as f64
        || 2.0 * radius <= drill_diameter
    {
        return Err("native AWX via dimensions exceed board resolution or engine range".into());
    }
    let radius = radius as i32;
    let drill_diameter = drill_diameter as i32;
    let drill_radius = drill_diameter as f64 / 2.0;
    let padstack_name = format!("awx_via_r{radius}_d{drill_diameter}");
    let net_id = board.net_number(net)?;
    let net_class = board
        .board
        .rules
        .nets
        .get(net_id)
        .ok_or_else(|| format!("native via net {net} is absent from board rules"))?
        .get_net_class();

    let existing_padstack = board
        .board
        .library
        .padstacks
        .get_by_name(&padstack_name)
        .map(|stack| stack.id);
    let padstack = if let Some(id) = existing_padstack {
        let stack = board
            .board
            .library
            .padstacks
            .get(id)
            .ok_or("native AWX via padstack lookup failed")?;
        let shape_ok = (stack.from_layer()..=stack.to_layer())
            .filter_map(|layer| stack.get_shape(layer))
            .all(|shape| (shape.max_width() - (2 * radius) as f64).abs() <= 1.0);
        if stack.from_layer() != 0 || stack.to_layer() != board.board.layer_count() - 1 || !shape_ok
        {
            return Err(format!(
                "native AWX via padstack name collision for {padstack_name}"
            ));
        }
        board
            .board
            .library_mut()
            .padstacks
            .get_mut(id)
            .ok_or("native AWX via padstack mutation failed")?
            .set_explicit_drill_radius(Some(drill_radius))?;
        id
    } else {
        let layers = board.board.layer_count() as usize;
        let shapes = vec![
            Some(ConvexShape::Circle(Circle::new(
                IntPoint::new(0, 0),
                radius
            )));
            layers
        ];
        let library = board.board.library_mut();
        let id = library
            .padstacks
            .add(padstack_name.clone(), shapes, true, false);
        library
            .padstacks
            .get_mut(id)
            .ok_or("new native AWX via padstack lookup failed")?
            .set_explicit_drill_radius(Some(drill_radius))?;
        id
    };
    board.board.library_mut().add_via_padstack(padstack);

    let via_clearance = board.board.rules.net_classes[net_class]
        .default_item_clearance_classes
        .get(ItemClass::Via);
    let via_info_name = format!("{padstack_name}-nc{}", net_class.0);
    let via_info = if let Some(id) = board.board.rules.via_infos.get_by_name(&via_info_name) {
        let info = &board.board.rules.via_infos[id];
        if info.get_padstack() != padstack || info.get_clearance_class_index() != via_clearance {
            return Err(format!("native ViaInfo name collision for {via_info_name}"));
        }
        id
    } else {
        board
            .board
            .rules_mut()
            .via_infos
            .add(ViaInfo::new(via_info_name, padstack, via_clearance, true))
            .ok_or("native AWX ViaInfo registration failed")?
    };

    Ok(via_info)
}
pub fn trace(
    board: &mut BoardAdapter,
    net: &str,
    layer: &str,
    a: Point,
    b: Point,
    width: f64,
) -> Result<(), String> {
    let n = board.net_number(net)?;
    let l = board.board.layer_structure.get_no(layer);
    if l < 0 {
        return Err(format!("unknown native AWX layer {layer}"));
    }
    let half = (width / 2. * board.units_per_mm).round() as i32;
    if half <= 0 {
        return Err("AWX trace width below native resolution".into());
    }
    let class = board
        .board
        .rules
        .net_class_of(n)
        .get_trace_clearance_class();
    let pa = board.point(a)?;
    let pb = board.point(b)?;
    if pa == pb {
        return Ok(());
    }
    let polyline = Polyline::from_two_points(&pa, &pb);
    board
        .board
        .insert_trace_without_cleaning(polyline, l, half, &[n], class, FixedState::UserFixed)
        .ok_or("native AWX trace insertion failed")?;
    Ok(())
}
pub fn materialize(
    before: &BoardAdapter,
    plan: &InstalledPlan,
    frame: FlowFrame,
    rules: &RoutingRules,
    protected: &BTreeSet<String>,
) -> Result<BoardAdapter, String> {
    let mut result = before.clone();
    let physical_stack: Vec<_> = before
        .board
        .layer_structure
        .layers
        .iter()
        .map(|l| l.name.clone())
        .collect();
    for v in &plan.virtual_vias {
        if protected.contains(&v.net) {
            return Err("AWX plan writes protected net".into());
        }
        via(
            &mut result,
            &v.net,
            frame.to_board(v.point),
            v.diameter,
            v.drill,
        )?;
    }
    for p in &plan.virtual_tracks {
        if protected.contains(&p.net) {
            return Err("AWX plan writes protected net".into());
        }
        trace(
            &mut result,
            &p.net,
            &frame.to_board_layer(&p.layer, &physical_stack)?,
            frame.to_board(p.a),
            frame.to_board(p.b),
            rules.track_width,
        )?;
    }
    result.assert_invariants(before, protected, false)?;
    Ok(result)
}
pub fn grade(
    before: &BoardAdapter,
    after: &BoardAdapter,
    names: &BTreeSet<String>,
) -> Result<NativeGrade, String> {
    let mut connected = Vec::new();
    let mut open = Vec::new();
    for name in names {
        let n = after.net_number(name)?;
        let items = after.board.get_connectable_items(n);
        if NetIncompletes::new(&after.board, n, &items).count() == 0 {
            connected.push(name.clone());
        } else {
            open.push(name.clone());
        }
    }
    let before_violations = before.violations();
    let violations = after.violations();
    let new_violations = violations.difference(&before_violations).count();
    let ids: BTreeSet<_> = names
        .iter()
        .map(|n| after.net_number(n))
        .collect::<Result<_, _>>()?;
    let vias = after
        .board
        .get_vias()
        .iter()
        .filter(|&&k| {
            after
                .board
                .item(k)
                .net_numbers()
                .iter()
                .any(|n| ids.contains(n))
        })
        .count();
    let copper_mm = after
        .board
        .get_traces()
        .iter()
        .filter(|&&k| {
            after
                .board
                .item(k)
                .net_numbers()
                .iter()
                .any(|n| ids.contains(n))
        })
        .map(|&k| after.board.item(k).trace().length() / after.units_per_mm)
        .sum();
    Ok(NativeGrade {
        connected,
        open,
        new_violations,
        native_violations: violations.len(),
        vias,
        copper_mm,
    })
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum SourceCopperSignature {
    Trace {
        nets: Vec<String>,
        layer: String,
        half_width: i32,
        points: Vec<String>,
        clearance: String,
        fixed: FixedState,
    },
    Via {
        nets: Vec<String>,
        center: String,
        first_layer: String,
        last_layer: String,
        padstack: String,
        attach_allowed: bool,
        placed_absolute: bool,
        copper_shapes: Vec<(String, Option<String>)>,
        explicit_drill_radius_bits: Option<u64>,
        clearance: String,
        fixed: FixedState,
    },
}

impl SourceCopperSignature {
    fn nets(&self) -> &[String] {
        match self {
            Self::Trace { nets, .. } | Self::Via { nets, .. } => nets,
        }
    }
}

fn source_copper_signature(
    board: &BoardAdapter,
    key: fr_engine::board::ItemKey,
) -> Result<SourceCopperSignature, String> {
    let item = board.board.item(key);
    let mut nets = item
        .net_numbers()
        .iter()
        .map(|&net| {
            board
                .board
                .rules
                .nets
                .get(net)
                .map(|entry| entry.name.clone())
                .ok_or_else(|| "source copper net identity is missing".to_string())
        })
        .collect::<Result<Vec<_>, _>>()?;
    nets.sort();
    let clearance = board
        .board
        .rules
        .clearance_matrix
        .get_name(item.clearance_class())
        .ok_or("source copper clearance class is missing")?
        .to_string();
    if item.is_trace() {
        let trace = item.trace();
        let layer = board
            .board
            .layer_structure
            .layers
            .get(trace.layer() as usize)
            .ok_or("source trace layer is missing")?
            .name
            .clone();
        let polyline = trace.polyline();
        let mut points: Vec<_> = (0..polyline.corner_count())
            .map(|i| crate::board_adapter::canonical_point_key(&polyline.corner(i)))
            .collect();
        let reverse: Vec<_> = points.iter().cloned().rev().collect();
        if reverse < points {
            points = reverse;
        }
        Ok(SourceCopperSignature::Trace {
            nets,
            layer,
            half_width: trace.half_width(),
            points,
            clearance,
            fixed: item.fixed_state(),
        })
    } else if item.is_via() {
        let center = item.center(&board.board);
        let padstack = item
            .padstack(&board.board)
            .ok_or("source via padstack is missing")?;
        let copper_shapes = board
            .board
            .layer_structure
            .layers
            .iter()
            .enumerate()
            .map(|(layer, info)| {
                (
                    info.name.clone(),
                    padstack
                        .get_shape(layer as i32)
                        .map(|shape| format!("{shape:?}")),
                )
            })
            .collect();
        let explicit_drill_radius_bits = padstack
            .get_explicit_drill_radius()
            .map(f64::to_bits);
        let padstack_name = padstack.name.clone();
        let first_layer = board
            .board
            .layer_structure
            .layers
            .get(item.first_layer(&board.board) as usize)
            .ok_or("source via first layer is missing")?
            .name
            .clone();
        let last_layer = board
            .board
            .layer_structure
            .layers
            .get(item.last_layer(&board.board) as usize)
            .ok_or("source via last layer is missing")?
            .name
            .clone();
        Ok(SourceCopperSignature::Via {
            nets,
            center: crate::board_adapter::canonical_point_key(&center),
            first_layer,
            last_layer,
            padstack: padstack_name,
            attach_allowed: padstack.attach_allowed,
            placed_absolute: padstack.placed_absolute,
            copper_shapes,
            explicit_drill_radius_bits,
            clearance,
            fixed: item.fixed_state(),
        })
    } else {
        Err("source signature requested for non-copper item".into())
    }
}

fn protected_source_copper_counts(
    board: &BoardAdapter,
) -> Result<BTreeMap<SourceCopperSignature, usize>, String> {
    let mut counts = BTreeMap::new();
    for key in board.board.get_items() {
        if board.is_source_protected_copper(key) {
            *counts.entry(source_copper_signature(board, key)?).or_default() += 1;
        }
    }
    Ok(counts)
}

fn consume_protected_source(
    counts: &mut BTreeMap<SourceCopperSignature, usize>,
    signature: &SourceCopperSignature,
) -> bool {
    let Some(count) = counts.get_mut(signature) else {
        return false;
    };
    if *count == 0 {
        return false;
    }
    *count -= 1;
    true
}

pub fn restore_nets(
    original: &BoardAdapter,
    trial: &mut BoardAdapter,
    names: &BTreeSet<String>,
) -> Result<(), String> {
    let mut candidate = trial.clone();
    restore_nets_into(original, &mut candidate, names)?;
    *trial = candidate;
    Ok(())
}

fn restore_nets_into(
    original: &BoardAdapter,
    trial: &mut BoardAdapter,
    names: &BTreeSet<String>,
) -> Result<(), String> {
    strip_nets(trial, names)?;
    let mut trial_source = protected_source_copper_counts(trial)?;
    for key in original.board.get_traces() {
        let item = original.board.item(key);
        let signature = if original.is_source_protected_copper(key) {
            Some(source_copper_signature(original, key)?)
        } else {
            None
        };
        let mut target_nets = Vec::new();
        for &n in item.net_numbers() {
            let Some(net) = original.board.rules.nets.get(n) else {
                continue;
            };
            if names.contains(&net.name) {
                target_nets.push(net.name.clone());
            }
        }
        if target_nets.is_empty() {
            continue;
        }
        if let Some(signature) = &signature {
            if consume_protected_source(&mut trial_source, signature) {
                continue;
            }
            return Err("AWX rollback is missing source-protected trace copper".into());
        }
        for net_name in target_nets {
            let layer = &original.board.layer_structure.layers
                [item.first_layer(&original.board) as usize]
                .name;
            let points = item.trace().polyline().corner_approx_arr();
            let width = 2. * item.trace().half_width() as f64 / original.units_per_mm;
            for pair in points.windows(2) {
                trace(
                    trial,
                    &net_name,
                    layer,
                    [
                        pair[0].x / original.units_per_mm,
                        pair[0].y / original.units_per_mm,
                    ],
                    [
                        pair[1].x / original.units_per_mm,
                        pair[1].y / original.units_per_mm,
                    ],
                    width,
                )?;
            }
        }
    }
    for key in original.board.get_vias() {
        let item = original.board.item(key);
        let signature = if original.is_source_protected_copper(key) {
            Some(source_copper_signature(original, key)?)
        } else {
            None
        };
        let mut target_nets = Vec::new();
        for &n in item.net_numbers() {
            let Some(net) = original.board.rules.nets.get(n) else {
                continue;
            };
            if names.contains(&net.name) {
                target_nets.push(net.name.clone());
            }
        }
        if target_nets.is_empty() {
            continue;
        }
        if let Some(signature) = &signature {
            if consume_protected_source(&mut trial_source, signature) {
                continue;
            }
            return Err("AWX rollback is missing source-protected via copper".into());
        }
        let stack = item
            .padstack(&original.board)
            .ok_or("source via padstack missing")?;
        let p = item.center(&original.board).to_float();
        for net_name in target_nets {
            let target = trial
                .board
                .library
                .padstacks
                .get_by_name(&stack.name)
                .ok_or("source via library identity lost")?
                .id;
            let point = trial.point([p.x / original.units_per_mm, p.y / original.units_per_mm])?;
            let net_no = trial.net_number(&net_name)?;
            trial.board.insert_via(
                target,
                point,
                &[net_no],
                item.clearance_class(),
                item.fixed_state(),
                true,
            );
        }
    }
    if trial_source.iter().any(|(signature, count)| {
        *count > 0 && signature.nets().iter().any(|net| names.contains(net))
    }) {
        return Err("AWX rollback found extra source-protected copper on a selected net".into());
    }
    Ok(())
}
pub fn require_native_geometry(geometry: &GeometryResult) -> Result<(), String> {
    if !geometry.metadata_complete || geometry.failed || !geometry.folded.is_empty() {
        return Err("AWX native submission requires a complete, unfailed geometry plan".into());
    }
    for (n, lane) in &geometry.lanes {
        if !lane.snapped {
            return Err(format!(
                "AWX lane {n} is not snapped to the source router lattice"
            ));
        }
        if !lane.paired_legs.is_empty() && lane.ends.len() != 2 {
            return Err(format!(
                "AWX paired lane {n} lacks both real end connectors"
            ));
        }
        if !lane.vias.is_empty() && !lane.paired_legs.is_empty() && lane.paired_vias.is_empty() {
            return Err("AWX paired via centers are not physical barrel locations".into());
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::{ensure_via_candidate, restore_nets, strip_nets, trace};
    use crate::board_adapter::BoardAdapter;
    use fr_engine::ids::FixedState;
    use std::collections::BTreeSet;

    fn protected_source_board(
        with_prior_other_copper: bool,
        bus_copies: usize,
        with_via: bool,
        mismatch_via_shape: bool,
    ) -> BoardAdapter {
        let fixture = std::str::from_utf8(include_bytes!("../../tests/fixtures/two_chip.dsn"))
            .expect("fixture is UTF-8");
        let mut dsn = fixture
            .trim_end()
            .strip_suffix(')')
            .expect("fixture root closes")
            .to_string();
        if mismatch_via_shape {
            dsn = dsn.replace(
                "(shape (circle Top 0.5))",
                "(shape (circle Top 0.4))",
            );
        }
        let prior = if with_prior_other_copper {
            "(wire (path Top 0.2 10 7 11 7) (net OTHER) (type protect)) "
        } else {
            ""
        };
        let bus_wires = "(wire (path Top 0.2 3 3 4 3) (net BUS) (type protect)) "
            .repeat(bus_copies);
        let via = if with_via {
            "(via VIA 5 5 (net BUS) (type protect))"
        } else {
            ""
        };
        dsn.push_str(&format!("\n (wiring {prior}{bus_wires}{via})\n)"));
        BoardAdapter::load(dsn.as_bytes()).expect("protected source board loads")
    }

    #[test]
    fn registering_native_via_does_not_change_shared_net_class_rule() {
        let mut board = BoardAdapter::load(include_bytes!("../../tests/fixtures/two_chip.dsn"))
            .expect("fixture board loads");
        let first = board.net_number("BUS").unwrap();
        let second = board.net_number("OTHER").unwrap();
        let first_class = board.board.rules.nets.get(first).unwrap().get_net_class();
        let second_class = board.board.rules.nets.get(second).unwrap().get_net_class();
        assert_eq!(first_class, second_class);
        let shared_rule = board.board.rules.net_classes[first_class]
            .get_via_rule()
            .expect("fixture has a via rule");
        let rule_before = board.board.rules.via_rules[shared_rule].vias().to_vec();

        let candidate = ensure_via_candidate(&mut board, "BUS", 0.8, 0.3).unwrap();

        assert_eq!(
            board.board.rules.net_classes[first_class].get_via_rule(),
            Some(shared_rule)
        );
        assert_eq!(
            board.board.rules.net_classes[second_class].get_via_rule(),
            Some(shared_rule)
        );
        assert_eq!(
            board.board.rules.via_rules[shared_rule].vias(),
            rule_before.as_slice()
        );
        assert!(!board.board.rules.via_rules[shared_rule].contains(candidate));
        let padstack = board.board.rules.via_infos[candidate].get_padstack();
        let stack = board.board.library.padstacks.get(padstack).unwrap();
        assert!((2.0 * stack.get_drill_radius() / board.units_per_mm - 0.3).abs() < 0.001);
    }

    #[test]
    fn strip_nets_removes_user_fixed_transaction_copper() {
        let mut board = BoardAdapter::load(include_bytes!("../../tests/fixtures/two_chip.dsn"))
            .expect("fixture board loads");
        trace(&mut board, "BUS", "Top", [0.0, 0.0], [1.0, 0.0], 0.2)
            .expect("transaction trace inserts");
        let names = BTreeSet::from(["BUS".to_string()]);

        strip_nets(&mut board, &names).expect("selected net copper is removed");

        let bus = board.net_number("BUS").unwrap();
        assert!(board
            .board
            .get_traces()
            .iter()
            .all(|&key| !board.board.item(key).contains_net(bus)));
        assert!(board
            .board
            .get_vias()
            .iter()
            .all(|&key| !board.board.item(key).contains_net(bus)));
    }

    #[test]
    fn strip_nets_preserves_source_protected_copper_identity() {
        let mut board = protected_source_board(false, 1, false, false);
        let bus = board.net_number("BUS").unwrap();
        trace(&mut board, "BUS", "Top", [5.0, 5.0], [6.0, 5.0], 0.2)
            .expect("generated trace inserts");
        let source = board
            .board
            .get_traces()
            .into_iter()
            .find(|&key| board.board.item(key).contains_net(bus) && board.is_source_protected_copper(key))
            .expect("fixture BUS source trace");
        let generated = board
            .board
            .get_traces()
            .into_iter()
            .find(|&key| key != source)
            .expect("generated BUS trace");
        assert_eq!(board.board.item(source).fixed_state(), FixedState::UserFixed);
        assert!(board.is_source_protected_copper(source));
        let result = strip_nets(&mut board, &BTreeSet::from(["BUS".to_string()]));

        assert!(result.is_ok());
        assert!(board.board.item(source).is_on_board());
        assert!(board.is_source_protected_copper(source));
        assert!(!board.board.item(generated).is_on_board());
    }

    #[test]
    fn restore_nets_removes_generated_fixed_copper_and_restores_source_once() {
        let original = protected_source_board(false, 2, false, false);
        // Read all source instances separately to exercise signature multiplicity.
        let mut original_source_keys: Vec<_> = original
            .board
            .get_traces()
            .into_iter()
            .filter(|&key| {
                original.board.item(key).contains_net(original.net_number("BUS").unwrap())
                    && original.is_source_protected_copper(key)
            })
            .collect();
        original_source_keys.sort();
        assert_eq!(original_source_keys.len(), 2);
        // This independent import adds earlier copper, so the BUS source ItemKey differs
        // even though its physical source signature is identical.
        let mut trial = protected_source_board(true, 2, false, false);
        let mut trial_source_keys: Vec<_> = trial
            .board
            .get_traces()
            .into_iter()
            .filter(|&key| {
                trial.board.item(key).contains_net(trial.net_number("BUS").unwrap())
                    && trial.is_source_protected_copper(key)
            })
            .collect();
        trial_source_keys.sort();
        assert_eq!(trial_source_keys.len(), original_source_keys.len());
        assert_ne!(original_source_keys[0], trial_source_keys[0]);
        trace(&mut trial, "BUS", "Top", [2.0, 0.0], [3.0, 0.0], 0.2)
            .expect("generated trial trace inserts");
        let generated_key = trial
            .board
            .get_traces()
            .into_iter()
            .find(|&key| !trial.is_source_protected_copper(key))
            .expect("generated trace exists");
        assert!(!trial.is_source_protected_copper(generated_key));

        restore_nets(&original, &mut trial, &BTreeSet::from(["BUS".to_string()]))
            .expect("failed transaction copper is rolled back");

        let bus = trial.net_number("BUS").unwrap();
        let mut bus_traces: Vec<_> = trial
            .board
            .get_traces()
            .into_iter()
            .filter(|&key| trial.board.item(key).contains_net(bus))
            .collect();
        bus_traces.sort();
        assert_eq!(bus_traces, trial_source_keys);
        assert!(bus_traces.iter().all(|&key| trial.is_source_protected_copper(key)));
        assert!(!trial.board.item(generated_key).is_on_board());
    }

    #[test]
    fn restore_nets_rejects_extra_selected_source_multiplicity_atomically() {
        let original = protected_source_board(false, 1, false, false);
        let mut trial = protected_source_board(true, 2, false, false);
        let before_traces = trial.board.get_traces();
        let before_vias = trial.board.get_vias();
        let names = BTreeSet::from(["BUS".to_string()]);

        let result = restore_nets(&original, &mut trial, &names);

        assert!(result.is_err());
        assert_eq!(trial.board.get_traces(), before_traces);
        assert_eq!(trial.board.get_vias(), before_vias);
        assert!(before_traces.iter().all(|&key| trial.board.item(key).is_on_board()));
    }

    #[test]
    fn restore_nets_rejects_mismatched_via_physics_atomically() {
        let original = protected_source_board(false, 1, true, false);
        let mut trial = protected_source_board(true, 1, true, true);
        trace(&mut trial, "BUS", "Top", [2.0, 0.0], [3.0, 0.0], 0.2)
            .expect("generated trial trace inserts");
        let before_traces = trial.board.get_traces();
        let before_vias = trial.board.get_vias();
        let names = BTreeSet::from(["BUS".to_string()]);

        let result = restore_nets(&original, &mut trial, &names);

        assert!(result.is_err());
        assert_eq!(trial.board.get_traces(), before_traces);
        assert_eq!(trial.board.get_vias(), before_vias);
        assert!(before_traces.iter().all(|&key| trial.board.item(key).is_on_board()));
        assert!(before_vias.iter().all(|&key| trial.board.item(key).is_on_board()));
    }

    #[test]
    fn source_point_signatures_normalize_equivalent_rationals() {
        use fr_geom::Line;

        let first = Line::new_ints(0, 0, 1, 1)
            .intersection(&Line::new_ints(0, 1, 1, 0));
        let equivalent = Line::new_ints(0, 0, 2, 2)
            .intersection(&Line::new_ints(-1, 2, 2, -1));

        assert_eq!(
            crate::board_adapter::canonical_point_key(&first),
            crate::board_adapter::canonical_point_key(&equivalent)
        );
    }
}
