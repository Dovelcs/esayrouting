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
    let remove: Vec<_> = board
        .board
        .get_items()
        .into_iter()
        .filter(|&k| {
            let item = board.board.item(k);
            (item.is_trace() || item.is_via()) && item.net_numbers().iter().any(|n| ids.contains(n))
        })
        .collect();
    for &k in &remove {
        let item = board.board.item(k);
        if item.net_numbers().iter().any(|n| !ids.contains(n)) {
            return Err("Unsupported: selected bus copper also belongs to another net".into());
        }
    }
    board.board.remove_items(remove);
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
pub fn restore_nets(
    original: &BoardAdapter,
    trial: &mut BoardAdapter,
    names: &BTreeSet<String>,
) -> Result<(), String> {
    strip_nets(trial, names)?;
    for key in original.board.get_traces() {
        let item = original.board.item(key);
        for &n in item.net_numbers() {
            let Some(net) = original.board.rules.nets.get(n) else {
                continue;
            };
            if !names.contains(&net.name) {
                continue;
            }
            let layer = &original.board.layer_structure.layers
                [item.first_layer(&original.board) as usize]
                .name;
            let points = item.trace().polyline().corner_approx_arr();
            let width = 2. * item.trace().half_width() as f64 / original.units_per_mm;
            for pair in points.windows(2) {
                trace(
                    trial,
                    &net.name,
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
        let stack = item
            .padstack(&original.board)
            .ok_or("source via padstack missing")?;
        let p = item.center(&original.board).to_float();
        for &n in item.net_numbers() {
            let Some(net) = original.board.rules.nets.get(n) else {
                continue;
            };
            if !names.contains(&net.name) {
                continue;
            }
            let target = trial
                .board
                .library
                .padstacks
                .get_by_name(&stack.name)
                .ok_or("source via library identity lost")?
                .id;
            let point = trial.point([p.x / original.units_per_mm, p.y / original.units_per_mm])?;
            let net_no = trial.net_number(&net.name)?;
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
    use super::ensure_via_candidate;
    use crate::board_adapter::BoardAdapter;

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
}
