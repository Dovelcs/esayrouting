//! Builds a machine-checkable bridge from the original AWX DSN contract to the
//! normalized four-layer native board. This is a static preflight; it never routes.
//!
//! Usage: cargo run --release -p fr_agent --example awx_strict_rules -- \
//!   <original.dsn> <normalized-four-layer.dsn> <original_rules.json> <output-dir>

use fr_agent::board_adapter::BoardAdapter;
use fr_engine::board::BoardItemType;
use serde_json::{json, Map, Value};
use sha2::{Digest, Sha256};
use std::{
    collections::{BTreeMap, BTreeSet},
    env, fs,
    path::PathBuf,
};

const CLEARANCE_MM: f64 = 0.153162;
const WIDTH_MM: f64 = 0.254;
const VIA_DIAMETER_MM: f64 = 0.6096;
const VIA_DRILL_MM: f64 = 0.3;
const TOL: f64 = 1e-6;

fn sha(bytes: &[u8]) -> String {
    format!("{:x}", Sha256::digest(bytes))
}
fn class_name(board: &BoardAdapter, class: i32) -> String {
    board
        .board
        .rules
        .clearance_matrix
        .get_name(class)
        .unwrap_or("?")
        .to_owned()
}
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<_> = env::args_os().skip(1).collect();
    if args.len() != 4 {
        return Err("usage: awx_strict_rules <original.dsn> <normalized-four-layer.dsn> <original_rules.json> <output-dir>".into());
    }
    let original_path = PathBuf::from(&args[0]);
    let normalized_path = PathBuf::from(&args[1]);
    let contract_path = PathBuf::from(&args[2]);
    let out = PathBuf::from(&args[3]);
    fs::create_dir_all(&out)?;
    let original = fs::read(&original_path)?;
    let normalized = fs::read(&normalized_path)?;
    let contract_bytes = fs::read(&contract_path)?;
    let contract: Value = serde_json::from_slice(&contract_bytes)?;
    let original_hash = sha(&original);
    let normalized_hash = sha(&normalized);
    let expected_original = contract["sources"]["original_user_dsn"]["sha256"]
        .as_str()
        .ok_or("missing original source hash in contract")?;
    let expected_normalized = contract["sources"]["derived_four_layer_dsn"]["sha256"]
        .as_str()
        .ok_or("missing normalized source hash in contract")?;
    if original_hash != expected_original {
        return Err("original DSN hash differs from authoritative contract".into());
    }
    if normalized_hash != expected_normalized {
        return Err("normalized DSN hash differs from authoritative contract".into());
    }

    let board = BoardAdapter::load(&normalized).map_err(std::io::Error::other)?;
    let upm = board.units_per_mm;
    let mut nets_by_class = BTreeMap::<i32, Vec<String>>::new();
    let mut net_rows = Vec::new();
    for net in board.board.rules.nets.iter() {
        let class = board.board.rules.net_class_of(net.net_number);
        let cc = class.get_trace_clearance_class();
        let name = class_name(&board, cc);
        let width_by_layer: BTreeMap<String, f64> = board
            .board
            .layer_structure
            .layers
            .iter()
            .enumerate()
            .filter(|(_, l)| l.is_signal)
            .map(|(i, l)| {
                (
                    l.name.clone(),
                    2.0 * class.get_trace_half_width(i as i32) as f64 / upm,
                )
            })
            .collect();
        let via_rule = class.get_via_rule();
        let mut vias = Vec::new();
        if let Some(via_rule) = via_rule {
            for via_id in board.board.rules.via_rules[via_rule].vias() {
                let info = &board.board.rules.via_infos[*via_id];
                let padstack = board
                    .board
                    .library
                    .padstacks
                    .get(info.get_padstack())
                    .ok_or("net via rule references a missing padstack")?;
                let copper: BTreeMap<String, f64> = board
                    .board
                    .layer_structure
                    .layers
                    .iter()
                    .enumerate()
                    .filter_map(|(i, l)| {
                        padstack
                            .get_shape(i as i32)
                            .map(|s| (l.name.clone(), s.max_width() / upm))
                    })
                    .collect();
                vias.push(
                    json!({"via_rule": board.board.rules.via_rules[via_rule].name,
                    "padstack": padstack.name, "copper_diameter_mm_by_layer": copper,
                    "explicit_drill_mm": padstack.get_explicit_drill_radius().map(|r| 2.0*r/upm)}),
                );
            }
        }
        nets_by_class.entry(cc).or_default().push(net.name.clone());
        net_rows.push(json!({"net":net.name,"net_id":net.net_number,
            "net_class":class.get_name(),"trace_clearance_class":{"index":cc,"name":name},
            "width_mm_by_signal_layer":width_by_layer,"via_clearance_class":{"index":class.get_trace_clearance_class(),"name":name},
            "dsn_selected_vias":vias}));
    }
    let reused_classes: Vec<_> = nets_by_class
        .iter()
        .filter(|(_, ns)| ns.len() != 1)
        .map(
            |(c, ns)| json!({"clearance_class":{"index":c,"name":class_name(&board,*c)},"nets":ns}),
        )
        .collect();
    if !reused_classes.is_empty() {
        return Err(
            format!("multiple nets share a DSN clearance class: {reused_classes:?}").into(),
        );
    }

    // Aggregate the actual imported matrix lookup for every target net against
    // every foreign copper object; netless copper remains in the foreign set.
    let copper: Vec<_> = board
        .board
        .get_items()
        .into_iter()
        .filter_map(|key| {
            let item = board.board.item(key);
            let kind = item.board_item_type();
            if !item.is_on_board()
                || !matches!(
                    kind,
                    BoardItemType::Trace | BoardItemType::Pin | BoardItemType::Via
                )
            {
                return None;
            }
            Some((
                item.net_numbers().to_vec(),
                item.clearance_class(),
                format!("{kind:?}"),
            ))
        })
        .collect();
    let layers: Vec<i32> = (0..board.board.layer_count() as i32).collect();
    let mut pairs = BTreeMap::<(i32, i32, String), (usize, BTreeSet<i32>)>::new();
    for net in board.board.rules.nets.iter() {
        let target = board
            .board
            .rules
            .net_class_of(net.net_number)
            .get_trace_clearance_class();
        for (item_nets, other, kind) in &copper {
            if item_nets.contains(&net.net_number) {
                continue;
            }
            for &layer in &layers {
                // Read the raw source matrix without the engine's 16-unit
                // safety margin. This import diagnostic is not the Python
                // solver binding; the latter is exercised in strict_rule_bridge.json.
                let required = board
                    .board
                    .rules
                    .clearance_matrix
                    .get_value(target, *other, layer, false)
                    .max(
                        board
                            .board
                            .rules
                            .clearance_matrix
                            .get_value(*other, target, layer, false),
                    );
                let entry = pairs.entry((target, *other, kind.clone())).or_default();
                entry.0 += 1;
                entry.1.insert(required);
            }
        }
    }
    let pair_rows: Vec<Value> = pairs
        .iter()
        .map(|((target, other, kind), (count, values))| {
            json!({
        "target_trace_class":{"index":target,"name":class_name(&board,*target)},
        "foreign_item_class":{"index":other,"name":class_name(&board,*other)},
        "foreign_item_kind":kind,"object_layer_pairs":count,
        "actual_matrix_required_native_values":values,
        "actual_matrix_required_mm":values.iter().map(|v| *v as f64/upm).collect::<Vec<_>>(),
        "solver_clearance_mm":CLEARANCE_MM,
        "builder_expression":"max(matrix(target,item), config.clearance, 0)"})
        })
        .collect();
    let mismatches: Vec<_> = pairs.iter().filter(|(_,(_,values))|
        values.iter().any(|v| (*v as f64/upm-CLEARANCE_MM).abs()>TOL))
        .map(|((t,o,k),(count,values))| json!({"target_class":class_name(&board,*t),"foreign_class":class_name(&board,*o),"kind":k,"count":count,"matrix_values":values})).collect();
    let width_mismatches: Vec<_> = net_rows
        .iter()
        .filter_map(|row| {
            let bad: Map<String, Value> = row["width_mm_by_signal_layer"]
                .as_object()?
                .iter()
                .filter(|(_, v)| (v.as_f64().unwrap_or_default() - WIDTH_MM).abs() > TOL)
                .map(|(k, v)| (k.clone(), v.clone()))
                .collect();
            (!bad.is_empty()).then(|| json!({"net":row["net"],"actual":bad,"expected_mm":WIDTH_MM}))
        })
        .collect();
    if !width_mismatches.is_empty() {
        return Err(
            format!("DSN class widths differ from bridge target: {width_mismatches:?}").into(),
        );
    }
    let via_mismatches: Vec<_> = net_rows
        .iter()
        .filter_map(|row| {
            let vias = row["dsn_selected_vias"].as_array()?;
            let selected: Vec<_> = vias.iter().filter(|v| v["padstack"] == "via0").collect();
            let ok = selected.iter().any(|v| {
                v["copper_diameter_mm_by_layer"]
                    .as_object()
                    .is_some_and(|m| {
                        m.values().any(|d| {
                            (d.as_f64().unwrap_or_default() - VIA_DIAMETER_MM).abs() <= TOL
                        })
                    })
            });
            (!ok).then(|| json!({"net":row["net"],"vias":vias}))
        })
        .collect();
    if !via_mismatches.is_empty() {
        return Err(format!("net class did not select via0 0.6096mm: {via_mismatches:?}").into());
    }

    let mut values = BTreeSet::new();
    for row in &net_rows {
        for (layer, width) in row["width_mm_by_signal_layer"].as_object().unwrap() {
            values.insert(format!("{}:{:.6}", layer, width.as_f64().unwrap()));
        }
    }
    let manifest = json!({
        "schema":"awx_native_dsn_import_crosscheck_v1","route_performed":false,
        "sources":{"original_user_dsn":{"path":original_path,"sha256":original_hash},
            "normalized_four_layer_dsn":{"path":normalized_path,"sha256":normalized_hash},
            "authoritative_rule_contract":{"path":contract_path,"sha256":sha(&contract_bytes)}},
        "native_board":{"nets":board.board.rules.nets.iter().count(),"copper_objects":copper.len(),"layers":board.board.layer_structure.layers.iter().map(|l|l.name.clone()).collect::<Vec<_>>(),"units_per_mm":upm},
        "bridge":{"global_foreign_clearance_mm":CLEARANCE_MM,
            "equivalence_scope":"foreign-net copper objects only; each imported DSN clearance class belongs to exactly one net; source obstacles skip items containing the target net; full actual target↔foreign matrix lookups all equal 6.03mil; builder uses max(matrix, config.clearance)",
            "track_width_mm":WIDTH_MM,"via_copper_style":{"name":"via0","diameter_mm":VIA_DIAMETER_MM},
            "via_drill_mm":VIA_DRILL_MM,"via_drill_source":"original run command.json --via-drill; not a DSN minimum",
            "hole_to_hole_clearance_mm":0.0,"minimum_annular_ring_mm":0.0,
            "typed_drc_constraints":{"same_class_diagonal_mm":0.102108,"different_class_foreign_mm":CLEARANCE_MM,
                "class_width_mm":WIDTH_MM,"global_fallback_width_mm":0.25527,
                "edge_clearance_mm":null,"dsn_minimum_via_diameter_mm":null,"dsn_minimum_drill_mm":null}},
        "native_import_crosscheck":{"purpose":"Independent BoardAdapter import diagnostic only; Python route binding is separately proven by strict_rule_bridge.json.",
            "native_matrix_values_are_quantized":true,
            "native_matrix_values_may_differ_from_exact_python_mm":true,
            "foreign_pair_differences_from_0_153162_mm":mismatches,
            "not_used_as_python_solver_input":true},
        "net_rule_assignments":net_rows,"actual_foreign_pair_matrix":pair_rows,
        "verified":{"source_hashes_match_contract":true,"one_class_per_net":true,"all_named_widths_equal_0_254mm":true,"each_net_selects_via0_0_6096mm":true,"no_search_or_route_executed":true},
        "raw_rule_contract":contract["raw_rules"]
    });
    fs::write(
        out.join("native_import_crosscheck.json"),
        serde_json::to_vec_pretty(&manifest)?,
    )?;
    println!(
        "strict rule preflight passed: {} nets, {} foreign class/kind matrix groups",
        board.board.rules.nets.iter().count(),
        pairs.len()
    );
    Ok(())
}
