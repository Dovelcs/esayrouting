use fr_agent::board_adapter::BoardAdapter;
use fr_engine::drc::NetIncompletes;
use serde::{Deserialize, Serialize};
use std::{
    collections::{BTreeMap, BTreeSet},
    env, fs,
    path::PathBuf,
};

#[derive(Deserialize)]
struct Config {
    dsn: PathBuf,
    sessions: Vec<SessionConfig>,
    via_diameter_mm: f64,
    via_drill_mm: f64,
    json_out: PathBuf,
    csv_out: PathBuf,
}
#[derive(Deserialize)]
struct SessionConfig {
    label: String,
    ses: PathBuf,
}
#[derive(Clone, Debug, Serialize)]
struct Endpoint {
    xy_mm: [f64; 2],
    layers: Vec<String>,
    connected_group_pins: Vec<String>,
    connected_group_identity_comparable: bool,
    item_id_run_local: i32,
}
#[derive(Clone, Debug, Serialize)]
struct Airline {
    net: String,
    from: Endpoint,
    to: Endpoint,
    partition_key: Option<String>,
}
#[derive(Serialize)]
struct SessionInventory {
    label: String,
    ses: PathBuf,
    incomplete_count: usize,
    net_incomplete_counts: BTreeMap<String, usize>,
    connected_groups: BTreeMap<String, Vec<Vec<String>>>,
    airlines: Vec<Airline>,
}
#[derive(Serialize)]
struct Report {
    source_dsn: PathBuf,
    via_candidates: Vec<ViaCandidateOut>,
    sessions: Vec<SessionInventory>,
    comparisons: Vec<Comparison>,
    interpretation: &'static str,
}
#[derive(Serialize)]
struct ViaCandidateOut {
    net: String,
    diameter_mm: f64,
    drill_mm: f64,
}
#[derive(Serialize)]
struct Comparison {
    session_a: String,
    session_b: String,
    same_net_partition_airlines: Vec<String>,
    airlines_only_in_a: Vec<String>,
    airlines_only_in_b: Vec<String>,
}

fn pin_refs(
    board: &BoardAdapter,
    keys: impl Iterator<Item = fr_engine::board::ItemKey>,
) -> Vec<String> {
    let mut pins = Vec::new();
    for key in keys {
        let item = board.board.item(key);
        if !item.is_pin() {
            continue;
        }
        let component = board.board.components.get(item.component_no());
        let pin = match &item.kind {
            fr_engine::board::ItemKind::Pin(p) => board
                .board
                .library
                .packages
                .get(component.get_package())
                .get_pin(p.pin_index)
                .map(|p| p.name.clone())
                .unwrap_or_else(|| format!("index:{}", p.pin_index)),
            _ => continue,
        };
        pins.push(format!("{}-{}", component.name, pin));
    }
    pins.sort();
    pins.dedup();
    pins
}

fn item_has_copper_layer(board: &BoardAdapter, key: fr_engine::board::ItemKey, layer: i32) -> bool {
    let item = board.board.item(key);
    if item.is_pin() || item.is_via() {
        return item
            .padstack(&board.board)
            .and_then(|padstack| padstack.get_shape(layer))
            .is_some();
    }
    item.first_layer(&board.board) == layer
}

fn endpoint(
    board: &BoardAdapter,
    net: fr_engine::ids::NetNo,
    key: fr_engine::board::ItemKey,
    corner: fr_geom::FloatPoint,
) -> Endpoint {
    let item = board.board.item(key);
    let mut layers = Vec::new();
    for (index, layer) in board.board.layer_structure.layers.iter().enumerate() {
        if layer.is_signal && item_has_copper_layer(board, key, index as i32) {
            layers.push(layer.name.clone());
        }
    }
    let group = board
        .board
        .get_connected_sets(net)
        .into_iter()
        .find(|set| set.contains(item.id()));
    let group_pins = group
        .as_ref()
        .map(|set| pin_refs(board, set.iter()))
        .unwrap_or_default();
    let comparable = group.is_some() && !group_pins.is_empty();
    Endpoint {
        xy_mm: [corner.x / board.units_per_mm, corner.y / board.units_per_mm],
        layers,
        connected_group_pins: group_pins,
        connected_group_identity_comparable: comparable,
        item_id_run_local: item.id().0,
    }
}

fn edge_key(edge: &Airline) -> Option<String> {
    if !edge.from.connected_group_identity_comparable
        || !edge.to.connected_group_identity_comparable
    {
        return None;
    }
    let mut groups = [
        edge.from.connected_group_pins.clone(),
        edge.to.connected_group_pins.clone(),
    ];
    groups.sort();
    Some(format!(
        "{}|{}|{}",
        edge.net,
        groups[0].join(","),
        groups[1].join(",")
    ))
}

fn resolved_destination(path: &std::path::Path) -> Result<PathBuf, Box<dyn std::error::Error>> {
    if path.exists() {
        return Ok(fs::canonicalize(path)?);
    }
    let parent = path
        .parent()
        .filter(|p| !p.as_os_str().is_empty())
        .unwrap_or(std::path::Path::new("."));
    Ok(fs::canonicalize(parent)?.join(path.file_name().ok_or("destination has no filename")?))
}

fn same_file(a: &std::path::Path, b: &std::path::Path) -> bool {
    if a == b {
        return true;
    }
    #[cfg(unix)]
    {
        use std::os::unix::fs::MetadataExt;
        if let (Ok(a), Ok(b)) = (fs::metadata(a), fs::metadata(b)) {
            return a.dev() == b.dev() && a.ino() == b.ino();
        }
    }
    false
}

fn inventory(
    config: &Config,
    input: &[u8],
    session: &SessionConfig,
) -> Result<SessionInventory, String> {
    let ses = fs::read(&session.ses).map_err(|e| format!("read {}: {e}", session.ses.display()))?;
    let source = BoardAdapter::load(input)?;
    let candidates: Vec<_> = source
        .board
        .rules
        .nets
        .iter()
        .map(|net| {
            (
                net.name.clone(),
                config.via_diameter_mm,
                config.via_drill_mm,
            )
        })
        .collect();
    let board = BoardAdapter::load_session_with_via_candidates(input, &ses, &candidates)?;
    let mut net_incomplete_counts = BTreeMap::new();
    let mut connected_groups = BTreeMap::new();
    let mut airlines = Vec::new();
    for net_rule in board.board.rules.nets.iter() {
        let net = net_rule.net_number;
        let sets = board.board.get_connected_sets(net);
        let mut groups: Vec<_> = sets
            .iter()
            .map(|set| pin_refs(&board, set.iter()))
            .collect();
        groups.sort();
        connected_groups.insert(net_rule.name.clone(), groups);
        let incompletes =
            NetIncompletes::new(&board.board, net, &board.board.get_connectable_items(net));
        net_incomplete_counts.insert(net_rule.name.clone(), incompletes.count() as usize);
        for line in incompletes.incompletes {
            let from = endpoint(&board, net, line.from_item, line.from_corner);
            let to = endpoint(&board, net, line.to_item, line.to_corner);
            let mut partition = [
                from.connected_group_pins.clone(),
                to.connected_group_pins.clone(),
            ];
            partition.sort();
            let partition_key = if from.connected_group_identity_comparable
                && to.connected_group_identity_comparable
            {
                Some(format!(
                    "{}|{}|{}",
                    net_rule.name,
                    partition[0].join(","),
                    partition[1].join(",")
                ))
            } else {
                None
            };
            airlines.push(Airline {
                net: net_rule.name.clone(),
                from,
                to,
                partition_key,
            });
        }
    }
    airlines.sort_by_key(|edge| edge_key(edge));
    let incomplete_count = net_incomplete_counts.values().sum();
    Ok(SessionInventory {
        label: session.label.clone(),
        ses: session.ses.clone(),
        incomplete_count,
        net_incomplete_counts,
        connected_groups,
        airlines,
    })
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let config_path = env::args_os()
        .nth(1)
        .ok_or("usage: cargo run -p fr_agent --example residual_inventory -- <config.json>")?;
    let config_path = PathBuf::from(config_path);
    let config_path = fs::canonicalize(config_path)?;
    let config: Config = serde_json::from_slice(&fs::read(&config_path)?)?;
    let base = config_path
        .parent()
        .unwrap_or_else(|| std::path::Path::new("."));
    let resolve = |path: &PathBuf| {
        if path.is_absolute() {
            path.clone()
        } else {
            base.join(path)
        }
    };
    let dsn_path = resolve(&config.dsn);
    let dsn_path = fs::canonicalize(dsn_path)?;
    let dsn = fs::read(&dsn_path)?;
    let resolved_sessions: Vec<_> = config
        .sessions
        .iter()
        .map(|s| SessionConfig {
            label: s.label.clone(),
            ses: resolve(&s.ses),
        })
        .collect();
    let mut sessions = Vec::new();
    for s in &resolved_sessions {
        sessions.push(inventory(&config, &dsn, s).map_err(std::io::Error::other)?);
    }
    let mut comparisons = Vec::new();
    for i in 0..sessions.len() {
        for j in i + 1..sessions.len() {
            let a: BTreeSet<_> = sessions[i].airlines.iter().filter_map(edge_key).collect();
            let b: BTreeSet<_> = sessions[j].airlines.iter().filter_map(edge_key).collect();
            comparisons.push(Comparison {
                session_a: sessions[i].label.clone(),
                session_b: sessions[j].label.clone(),
                same_net_partition_airlines: a.intersection(&b).cloned().collect(),
                airlines_only_in_a: a.difference(&b).cloned().collect(),
                airlines_only_in_b: b.difference(&a).cloned().collect(),
            });
        }
    }
    let report = Report {
        source_dsn: dsn_path.clone(),
        via_candidates: {
            let source = BoardAdapter::load(&dsn)?;
            source.board.rules.nets.iter().map(|net| ViaCandidateOut {
                net: net.name.clone(), diameter_mm: config.via_diameter_mm, drill_mm: config.via_drill_mm,
            }).collect()
        },
        sessions, comparisons,
        interpretation: "Airlines are recomputed by native NetIncompletes on each fully replayed session. Identity comparison uses net plus stable pin-reference sets from the exact get_connected_sets partition containing each airline endpoint. Edges whose partition lacks pin references are marked uncomparable and excluded from cross-session key matches. Endpoint XY and physical Cu layers are measurements that can move when routed copper changes topology. Item IDs are run-local only. A matching partition identity is evidence of the same residual connection, not byte-for-byte equality with another router's airline selection.",
    };
    let json_path = resolve(&config.json_out);
    let csv_path = resolve(&config.csv_out);
    let mut inputs = vec![config_path.clone(), dsn_path.clone()];
    inputs.extend(
        resolved_sessions
            .iter()
            .map(|s| fs::canonicalize(&s.ses))
            .collect::<Result<Vec<_>, _>>()?,
    );
    let json_destination = resolved_destination(&json_path)?;
    let csv_destination = resolved_destination(&csv_path)?;
    let mut all_paths = inputs
        .iter()
        .map(|p| (p.as_path(), "input"))
        .collect::<Vec<_>>();
    all_paths.push((json_destination.as_path(), "JSON output"));
    all_paths.push((csv_destination.as_path(), "CSV output"));
    for i in 0..all_paths.len() {
        for j in i + 1..all_paths.len() {
            if same_file(all_paths[i].0, all_paths[j].0) {
                return Err(format!(
                    "{} aliases {}: {} and {}",
                    all_paths[i].1,
                    all_paths[j].1,
                    all_paths[i].0.display(),
                    all_paths[j].0.display()
                )
                .into());
            }
        }
    }
    if let Some(parent) = json_destination.parent() {
        fs::create_dir_all(parent)?;
    }
    if let Some(parent) = csv_destination.parent() {
        fs::create_dir_all(parent)?;
    }
    fs::write(&json_destination, serde_json::to_vec_pretty(&report)?)?;
    let mut csv = String::from("session,net,from_x_mm,from_y_mm,from_layers,from_group_pins,to_x_mm,to_y_mm,to_layers,to_group_pins,partition_key,from_item_id_run_local,to_item_id_run_local\n");
    for s in &report.sessions {
        for a in &s.airlines {
            let quote = |v: &str| format!("\"{}\"", v.replace('"', "\"\""));
            csv.push_str(&format!(
                "{},{},{:.6},{:.6},{},{},{:.6},{:.6},{},{},{},{},{}\n",
                quote(&s.label),
                quote(&a.net),
                a.from.xy_mm[0],
                a.from.xy_mm[1],
                quote(&a.from.layers.join("|")),
                quote(&a.from.connected_group_pins.join("|")),
                a.to.xy_mm[0],
                a.to.xy_mm[1],
                quote(&a.to.layers.join("|")),
                quote(&a.to.connected_group_pins.join("|")),
                quote(a.partition_key.as_deref().unwrap_or("uncomparable")),
                a.from.item_id_run_local,
                a.to.item_id_run_local
            ));
        }
    }
    fs::write(csv_destination, csv)?;
    Ok(())
}
