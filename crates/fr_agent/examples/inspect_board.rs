use fr_agent::{
    board_adapter::BoardAdapter,
    spec::{Execution, RouteGroup, RouteSpec},
};
use std::{collections::BTreeMap, path::Path};
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().collect();
    let bytes = std::fs::read(&args[1])?;
    let adapter = if args.iter().any(|a| a == "--raw") {
        BoardAdapter::load_raw(&bytes)?
    } else {
        BoardAdapter::load(&bytes)?
    };
    let b = &adapter.board;
    let pads = adapter.pads();
    let mut nets: BTreeMap<String, Vec<_>> = BTreeMap::new();
    for p in &pads {
        if !p.net.is_empty() {
            nets.entry(p.net.clone()).or_default().push(p);
        }
    }
    let mut net_rows = Vec::new();
    let mut rules = Vec::new();
    let mut groups = Vec::new();
    for (name, pins) in &nets {
        let mut rule = adapter.derived_rule(pins[0])?;
        let net_no = adapter.net_number(name)?;
        let incomplete_net =
            fr_engine::drc::NetIncompletes::new(b, net_no, &b.get_connectable_items(net_no));
        let existing_vias = b
            .get_vias()
            .into_iter()
            .filter(|&k| b.item(k).net_numbers().contains(&net_no))
            .count();
        rule.via_budget = None;
        rule.allowed_layers = b
            .layer_structure
            .layers
            .iter()
            .enumerate()
            .filter(|(i, l)| {
                l.is_signal
                    && b.rules
                        .net_class_of(net_no)
                        .is_active_routing_layer(*i as i32)
            })
            .map(|(_, l)| l.name.clone())
            .collect();
        net_rows.push(serde_json::json!({"name":name,"pins":pins.len(),"width_mm":rule.width_mm,"clearance_mm":rule.clearance_mm,"allowed_layers":rule.allowed_layers,"incomplete":incomplete_net.count(),"connected_components":incomplete_net.connected_group_count(),"existing_vias":existing_vias}));
        if pins.len() == 2 && incomplete_net.count() > 0 {
            groups.push(RouteGroup {
                id: format!("net_{}", groups.len()),
                nets: vec![name.clone()],
                endpoint_groups: vec![],
                channels: vec![],
                priority: 0,
                depends_on: vec![],
                joint_escape: false,
                awx: None,
            });
            rules.push(rule);
        }
    }
    let protected_names: Vec<String> = (1..=b.rules.nets.max_net_number())
        .filter_map(|number| b.rules.nets.get(number))
        .filter(|net| !nets.contains_key(&net.name))
        .map(|net| net.name.clone())
        .collect();
    let traces = b.get_traces().len();
    let vias = b.get_vias().len();
    let components: std::collections::BTreeSet<_> = pads.iter().map(|p| p.chip.clone()).collect();
    let incomplete = fr_engine::pipeline::stats::incomplete_count(b, None);
    let statistics = fr_engine::scoring::BoardStatistics::from_board(b);
    let layers: Vec<_> = b
        .layer_structure
        .layers
        .iter()
        .map(|l| serde_json::json!({"name":l.name,"signal":l.is_signal}))
        .collect();
    println!(
        "{}",
        serde_json::to_string_pretty(
            &serde_json::json!({"layers":layers,"nets":nets.len(),"pads":pads.len(),"components":b.components.count(),"component_names":components,"total_nets":b.rules.nets.max_net_number(),"dsn_declared_nets":fr_dsn::Dsn::parse(&bytes)?.network.nets.len(),"total_trace_length_mm_exact":b.get_traces().iter().map(|&k|b.item(k).trace().length()/adapter.units_per_mm).sum::<f64>(),"trace_length_mm":statistics.traces.total_length_mm,"clearance_violations":statistics.clearance_violations.total_count,"violation_set":adapter.violations(),"maximum_connections":statistics.connections.maximum_count,"existing_traces":traces,"existing_vias":vias,"incomplete_connections":incomplete,"two_terminal_nets":groups.len(),"units_per_mm":adapter.units_per_mm,"net_rules":net_rows})
        )?
    );
    if args.len() > 2 && args[2] != "--raw" {
        let job_path = Path::new(&args[2]);
        let spec = RouteSpec {
            version: 1,
            route_remaining: true,
            allow_partial_output: true,
            protected_nets: protected_names,
            input: std::fs::canonicalize(&args[1])?,
            output: std::fs::canonicalize(job_path.parent().unwrap())?.join("agent_routed.ses"),
            nets: if args.iter().any(|a| a == "--targeted") {
                rules
            } else {
                Vec::new()
            },
            endpoint_groups: vec![],
            channels: vec![],
            groups: if args.iter().any(|a| a == "--targeted") {
                groups
            } else {
                Vec::new()
            },
            execution: Execution {
                threads: 1,
                budget_ms: 30000,
                seed: 0,
            },
            advanced: None,
            awx: None,
        };
        use std::io::Write;
        let mut job = std::fs::OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(job_path)?;
        job.write_all(toml::to_string_pretty(&spec)?.as_bytes())?;
        job.sync_all()?;
    }
    Ok(())
}
