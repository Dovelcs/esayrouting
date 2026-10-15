use serde_json::{json, Value};
use sha2::{Digest, Sha256};
use std::{
    env,
    error::Error,
    fs,
    path::{Path, PathBuf},
    process::Command,
    time::{SystemTime, UNIX_EPOCH},
};

const EXPECTED_AWX_HEAD: &str = "caad45cd252f9a50f1fb4b3b15461bdf1b9457a1";
const EXPECTED_BOARD_SHA256: &str =
    "5620693ced59e17facd09e136f55e57094329d67cff11d3a7315b809e9e146dd";
const EXPECTED_SCENE_SHA256: &str =
    "7f3c9caeac9712fe56f96867dfb6adbe055f0ff2e1437d0068eb41f5cf480e3c";
const EXPECTED_DSN_SHA256: &str =
    "ba88c2f93e33195512a00622a7eb815c36dd9c037496fb03f19cfa5d8a677972";
const EXPECTED_USER_DSN_SHA256: &str =
    "5a2c595ab8651eb939cc28e9f228807dd60b8d21740fdc44f391c3b016eabbba";
const EXPECTED_ACTIVE_BOARD_SHA256: &str =
    "2d2cab17c2b201a08240365a5f416a9f577bffd337b61ede290ae61fdfa13e2a";
const EXPECTED_RULE_BRIDGE_SHA256: &str =
    "a1e6ebfe5ac97a5b48637dc5e3056e4fa4e7fa0b81d9a415115b958f21dfa7a4";
const EXPECTED_POLICY_ADAPTER_SHA256: &str =
    "fdd76498655030578020a9f163d74ad57ca87aad9ae503b45c42297070c5b1bb";

fn sha256(path: &Path) -> Result<String, Box<dyn Error>> {
    Ok(format!("{:x}", Sha256::digest(fs::read(path)?)))
}

fn workspace_root() -> Result<PathBuf, Box<dyn Error>> {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .ancestors()
        .nth(4)
        .map(Path::to_path_buf)
        .ok_or_else(|| "cannot determine workspace root".into())
}

fn check_job(job: &Value) -> Result<(), Box<dyn Error>> {
    let checks = [
        (job["grid_step_mm"].as_f64() == Some(0.25), "grid step"),
        (job["track_width_mm"].as_f64() == Some(0.254), "track width"),
        (job["clearance_mm"].as_f64() == Some(0.153162), "clearance"),
        (
            job["via_diameter_mm"].as_f64() == Some(0.6096),
            "via diameter",
        ),
        (job["via_drill_mm"].as_f64() == Some(0.3), "via drill"),
        (
            job["hole_to_hole_clearance_mm"].as_f64() == Some(0.0),
            "hole-to-hole clearance",
        ),
        (
            job["board_edge_clearance_mm"].as_f64() == Some(0.0),
            "board-edge clearance",
        ),
        (job["fab_tier"].as_str() == Some("standard"), "fab tier"),
        (
            job["fab_overrides"]["hole_to_hole"].as_f64() == Some(0.0),
            "hole-to-hole fab override",
        ),
        (
            job["fab_overrides"]["board_edge"].as_f64() == Some(0.0),
            "board-edge fab override",
        ),
        (
            job["fab_overrides"]["annular"].as_f64() == Some(0.0),
            "annular fab override",
        ),
        (
            job["fab_overrides"]["pad_hole_to_hole"].as_f64() == Some(0.0),
            "pad-hole fab override",
        ),
        (
            job["escalation"].as_str() == Some("off"),
            "escalation policy",
        ),
        (
            job["strict_sizes"].as_bool() == Some(true),
            "strict-size gate",
        ),
        (
            job["power_tap_neckdown"].as_bool() == Some(false),
            "power tap neckdown",
        ),
        (
            job["allow_clearance_descent"].as_bool() == Some(false),
            "clearance descent",
        ),
        (
            job["route_worker_count"].as_u64() == Some(1),
            "route worker count",
        ),
    ];
    for (ok, name) in checks {
        if !ok {
            return Err(format!("strict job configuration mismatch: {name}").into());
        }
    }
    let layers = job["layers"].as_array().ok_or("job has no layers")?;
    let expected = ["F.Cu", "In1.Cu", "In2.Cu", "B.Cu"];
    if layers.len() != expected.len()
        || layers
            .iter()
            .zip(expected)
            .any(|(got, want)| got.as_str() != Some(want))
    {
        return Err("strict job copper-layer order mismatch".into());
    }
    Ok(())
}

fn check_policy_preflight(preflight: &Value, job: &Value) -> Result<(), Box<dyn Error>> {
    let policy = &preflight["policy"];
    let config = &preflight["grid_route_config"];
    let checks = [
        (
            config["track_width_mm"].as_f64() == Some(0.254),
            "track width",
        ),
        (
            config["clearance_mm"].as_f64() == Some(0.153162),
            "clearance",
        ),
        (
            config["via_diameter_mm"].as_f64() == Some(0.6096),
            "via diameter",
        ),
        (config["via_drill_mm"].as_f64() == Some(0.3), "via drill"),
        (config["grid_step_mm"].as_f64() == Some(0.25), "grid step"),
        (
            config["hole_to_hole_clearance_mm"].as_f64() == Some(0.0),
            "hole-to-hole clearance",
        ),
        (
            config["board_edge_clearance_mm"].as_f64() == Some(0.0),
            "board-edge clearance",
        ),
        (
            config["power_tap_neckdown"].as_bool() == Some(false),
            "power tap neckdown",
        ),
        (policy["fab_tier"].as_str() == Some("standard"), "fab tier"),
        (
            policy["escalation"].as_str() == Some("off"),
            "escalation policy",
        ),
        (
            policy["fab_overrides"] == job["fab_overrides"],
            "fab overrides",
        ),
        (
            policy["active_standard_floor"]["annular"].as_f64() == Some(0.0),
            "annular floor",
        ),
        (
            policy["active_standard_floor"]["pad_hole_to_hole"].as_f64() == Some(0.0),
            "pad-hole floor",
        ),
        (
            policy["source_board_floors"]["clearance"].as_f64() == Some(0.153162),
            "source matrix clearance",
        ),
        (
            policy["active_ladder_length"].as_u64() == Some(1),
            "fab ladder length",
        ),
        (
            policy["may_narrow"].as_bool() == Some(false),
            "clearance descent",
        ),
        (
            preflight["route_entrypoint"].as_str() == Some("route.batch_route"),
            "route entrypoint",
        ),
        (
            preflight["route_batch_policy_reset"].as_bool() == Some(false),
            "batch policy reset",
        ),
        (
            preflight["route_batch_floor_enforcement"].as_bool() == Some(false),
            "batch floor enforcement",
        ),
        (
            preflight["repair_planes_policy_reset"].as_bool() == Some(false),
            "plane policy reset",
        ),
        (
            preflight["repair_planes_policy_calls_found"]
                .as_array()
                .is_some_and(Vec::is_empty),
            "plane policy calls",
        ),
        (
            preflight["batch_route_subprocess_calls"]
                .as_array()
                .is_some_and(Vec::is_empty),
            "route subprocess calls",
        ),
    ];
    for (ok, name) in checks {
        if !ok {
            return Err(format!("public API strict preflight mismatch: {name}").into());
        }
    }
    for field in ["descent_rungs", "cli_floor_pins"] {
        if !policy[field].as_array().is_some_and(Vec::is_empty) {
            return Err(format!("public API strict preflight is not empty: policy.{field}").into());
        }
    }
    let ledger = &preflight["policy_after_preflight"];
    if ledger["count"].as_u64() != Some(0)
        || ledger["fab_tier_escalations"].as_u64() != Some(0)
        || ledger["narrowed"]
            .as_array()
            .map_or(true, |items| !items.is_empty())
    {
        return Err("public API strict preflight ledger is not clean".into());
    }
    Ok(())
}

fn check_rule_bridge(
    strict: &Path,
    board: &Path,
    rules_project: &Path,
    rules_dru: &Path,
) -> Result<Value, Box<dyn Error>> {
    let bridge = strict.join("bridge");
    let manifest_path = bridge.join("strict_rule_bridge.json");
    let matrix_path = bridge.join("foreign_pair_matrix.json");
    let widths_path = bridge.join("net_widths_and_vias_by_id.json");
    let clearances_path = bridge.join("net_clearances_by_id.json");
    if sha256(&manifest_path)? != EXPECTED_RULE_BRIDGE_SHA256 {
        return Err("strict rule bridge manifest SHA differs from the reviewed gate".into());
    }
    let manifest: Value = serde_json::from_slice(&fs::read(&manifest_path)?)?;
    let matrix: Value = serde_json::from_slice(&fs::read(&matrix_path)?)?;
    let widths: Value = serde_json::from_slice(&fs::read(&widths_path)?)?;
    let clearances: Value = serde_json::from_slice(&fs::read(&clearances_path)?)?;
    let board_sha = sha256(board)?;
    let pro_sha = sha256(rules_project)?;
    let dru_sha = sha256(rules_dru)?;
    let source = &manifest["sources"];
    let solver = &manifest["solver_configuration"];
    let assign = &manifest["assignment_proof"];
    let contract = &manifest["source_rule_values"];
    let clearance_layers = solver["config.layer_clearances_mm"]
        .as_object()
        .ok_or("strict bridge has no four-layer clearance map")?;
    let expected_layers = ["F.Cu", "In1.Cu", "In2.Cu", "B.Cu"];
    let layer_map_pass = clearance_layers.len() == expected_layers.len()
        && expected_layers
            .iter()
            .all(|layer| clearance_layers.get(*layer).and_then(Value::as_f64) == Some(0.153162));
    let foreign_rows = matrix["actual_foreign_pad_pair_rows"]
        .as_array()
        .ok_or("foreign-pair manifest has no row list")?;
    let foreign_pairs_pass = foreign_rows.len() == 6865
        && foreign_rows.iter().all(|row| {
            row["required_clearance_mil"].as_f64() == Some(6.03)
                && row["required_clearance_mm"].as_f64() == Some(0.153162)
        });
    let widths_map = widths
        .as_object()
        .ok_or("net width/via map is not an object")?;
    let dimensions_pass = widths_map.len() == 89
        && widths_map.values().all(|entry| {
            entry["track_width_mm"].as_f64() == Some(0.254)
                && entry["via_size_mm"].as_f64() == Some(0.6096)
                && entry["via_drill_mm"].as_f64() == Some(0.3)
        });
    let original_user_dsn_path = PathBuf::from(
        source["original_user_dsn"]["path"]
            .as_str()
            .ok_or("strict bridge has no raw user DSN path")?,
    );
    let original_user_dsn_sha = sha256(&original_user_dsn_path)?;
    let normalized_dsn_sha = source["normalized_four_layer_dsn"]["sha256"]
        .as_str()
        .ok_or("strict bridge has no normalized four-layer DSN hash")?;
    let source_hashes_pass = source["strict_bridge_board"]["sha256"].as_str()
        == Some(EXPECTED_ACTIVE_BOARD_SHA256)
        && source["strict_bridge_board"]["input_sha256_before_proxy_removal"].as_str()
            == Some(EXPECTED_BOARD_SHA256)
        && source["strict_project"]["sha256"].as_str() == Some(pro_sha.as_str())
        && source["strict_layer_clearance_dru"]["sha256"].as_str() == Some(dru_sha.as_str())
        && source["original_user_dsn"]["sha256"].as_str() == Some(EXPECTED_USER_DSN_SHA256)
        && original_user_dsn_sha == EXPECTED_USER_DSN_SHA256
        && normalized_dsn_sha == EXPECTED_DSN_SHA256
        && board_sha == EXPECTED_ACTIVE_BOARD_SHA256;
    let assigned_rules_pass = assign["input_scene_nets"].as_u64() == Some(89)
        && assign["explicit_net_memberships"].as_u64() == Some(89)
        && assign["each_net_resolves_to_exactly_one_class"].as_bool() == Some(true)
        && assign["unique_clearance_class_per_net"].as_bool() == Some(true)
        && assign["all_named_net_foreign_pairs_use_cross_class_6_03mil"].as_bool() == Some(true)
        && assign["input_pads"].as_u64() == Some(685)
        && assign["netless_pads"].as_u64() == Some(185);
    let skips = solver["config.layer_clearance_parse_notes"]
        .as_array()
        .ok_or("layer-clearance parse notes missing")?;
    let matrix_floor_pass = solver["config.clearance_mm"].as_f64() == Some(0.153162)
        && solver["config.net_clearances_count"].as_u64() == Some(89)
        && solver["config.net_clearance_floor_mm"].as_f64() == Some(0.153162)
        && solver["config.obstacle_clearance_netless_id0_mm"].as_f64() == Some(0.153162)
        && solver["config.obstacle_clearance_named_net_min_mm"].as_f64() == Some(0.153162)
        && solver["config.obstacle_clearance_named_net_max_mm"].as_f64() == Some(0.153162);
    let expected_clearances = clearances
        .as_object()
        .ok_or("net clearance map is not an object")?;
    let complete_clearance_map = expected_clearances.len() == 89
        && expected_clearances
            .values()
            .all(|value| value.as_f64().is_some());
    let values_pass = solver["config.track_width_mm"].as_f64() == Some(0.254)
        && solver["config.via_size_mm"].as_f64() == Some(0.6096)
        && solver["config.via_drill_mm"].as_f64() == Some(0.3)
        && solver["config.hole_to_hole_clearance_mm"].as_f64() == Some(0.0)
        && solver["config.board_edge_clearance_mm"].as_f64() == Some(0.0)
        && solver["config.layers"].as_array().is_some_and(|layers| {
            layers.len() == expected_layers.len()
                && layers
                    .iter()
                    .zip(expected_layers)
                    .all(|(got, want)| got.as_str() == Some(want))
        })
        && contract["per_named_class_width_mil"].as_f64() == Some(10.0)
        && contract["via0_copper_diameter_mil"].as_f64() == Some(24.0)
        && contract["job_via_drill_mm"].as_f64() == Some(0.3);
    if manifest["route_performed"].as_bool() != Some(false)
        || !source_hashes_pass
        || !layer_map_pass
        || !foreign_pairs_pass
        || !dimensions_pass
        || !assigned_rules_pass
        || !matrix_floor_pass
        || !complete_clearance_map
        || !values_pass
        || !skips.is_empty()
    {
        return Err(format!(
            "strict rule bridge no-search gate failed: source_hashes={source_hashes_pass}, layers={layer_map_pass}, foreign_pairs={foreign_pairs_pass}, dimensions={dimensions_pass}, assignments={assigned_rules_pass}, matrix_floor={matrix_floor_pass}, net_clearances={complete_clearance_map}, values={values_pass}, skips={}",
            skips.len()
        )
        .into());
    }
    Ok(json!({
        "manifest_sha256": sha256(&manifest_path)?,
        "foreign_pair_matrix_sha256": sha256(&matrix_path)?,
        "width_via_map_sha256": sha256(&widths_path)?,
        "clearance_map_sha256": sha256(&clearances_path)?,
        "active_input_sha256": board_sha,
        "frozen_source_board_sha256": EXPECTED_BOARD_SHA256,
        "project_sha256": pro_sha,
        "dru_sha256": dru_sha,
        "original_user_dsn_sha256": original_user_dsn_sha,
        "normalized_four_layer_dsn_sha256": normalized_dsn_sha,
        "layer_clearances_mm": solver["config.layer_clearances_mm"],
        "layer_clearance_parse_skips": skips,
        "named_net_count": expected_clearances.len(),
        "width_via_count": widths_map.len(),
        "foreign_pair_rows": foreign_rows.len(),
        "foreign_pair_required_clearance_mm": 0.153162,
        "netless_pad_count": 185,
        "routing_floor_mm": 0.153162,
        "via_diameter_mm": 0.6096,
        "via_drill_mm": 0.3,
        "track_width_mm": 0.254,
        "hole_to_hole_clearance_mm": 0.0,
        "board_edge_clearance_mm": 0.0,
    }))
}

fn run_geometry_preflight(
    strict: &Path,
    python: &str,
    board: &Path,
) -> Result<Value, Box<dyn Error>> {
    let scratch = env::temp_dir().join(format!("awx_pad_preflight_{}", std::process::id()));
    if scratch.exists() {
        fs::remove_dir_all(&scratch)?;
    }
    fs::create_dir_all(scratch.join("run"))?;
    for name in [
        "validate_awx_input.py",
        "input_scene.json",
        "native_base.json",
    ] {
        fs::copy(strict.join(name), scratch.join(name))?;
    }
    let root = workspace_root()?;
    let awx = root.join("work/kicad_routing_tools");
    let python_path = env::join_paths([
        strict.join("pydeps"),
        root.join("outputs/fix_validation/awx_parity_runtime/pydeps"),
        root.join("outputs/fix_validation/awx_parity_runtime/native"),
        awx.join("py_router"),
        awx.join("rust_router"),
    ])?;
    let output = Command::new(python)
        .arg(scratch.join("validate_awx_input.py"))
        .arg(board)
        .env("PYTHONPATH", python_path)
        .env("PYTHONDONTWRITEBYTECODE", "1")
        .current_dir(&root)
        .output();
    let output = output?;
    if !output.status.success() {
        let _ = fs::remove_dir_all(&scratch);
        return Err(format!(
            "full-pad input geometry readback failed: {}",
            String::from_utf8_lossy(&output.stderr)
        )
        .into());
    }
    let report_path = scratch.join("run/output_readback.json");
    let report_bytes = fs::read(&report_path)?;
    let report_sha = format!("{:x}", Sha256::digest(&report_bytes));
    let _ = fs::remove_dir_all(&scratch);
    let data: Value = serde_json::from_slice(&report_bytes)?;
    let expected_pads = data["scene_counts"]["pins"].as_u64();
    let parsed_pads = data["parsed_pad_geometry"].as_array().map(Vec::len);
    let errors = data["errors"]
        .as_object()
        .ok_or("pad readback has no errors object")?;
    let error_counts: serde_json::Map<String, Value> = errors
        .iter()
        .map(|(key, value)| {
            (
                key.clone(),
                json!(value.as_array().map(Vec::len).unwrap_or(usize::MAX)),
            )
        })
        .collect();
    let any_errors = errors
        .values()
        .any(|value| value.as_array().map_or(true, |items| !items.is_empty()));
    if data["status"].as_str() != Some("pass")
        || expected_pads != Some(685)
        || parsed_pads != Some(685)
        || any_errors
    {
        return Err(format!("pad geometry readback did not prove all 685 pads: status={}, parsed={parsed_pads:?}, errors={error_counts:?}", data["status"]).into());
    }
    Ok(json!({
        "status": data["status"],
        "parsed_pad_count": parsed_pads,
        "source_pad_count": expected_pads,
        "parser_counts": data["parser_counts"],
        "error_counts": error_counts,
        "full_geometry_records": true,
        "readback_report_sha256": report_sha,
    }))
}

fn run_policy_preflight(
    strict: &Path,
    python: &str,
    board: &Path,
    rules_project: &Path,
    rules_dru: &Path,
    adapter: &Path,
    job: &Value,
    rule_bridge: &Value,
) -> Result<Value, Box<dyn Error>> {
    let scratch = env::temp_dir().join(format!("awx_strict_preflight_{}", std::process::id()));
    if scratch.exists() {
        fs::remove_dir_all(&scratch)?;
    }
    fs::create_dir_all(&scratch)?;
    let scratch_board = scratch.join("board.kicad_pcb");
    fs::copy(board, &scratch_board)?;
    fs::copy(rules_project, scratch.join("board.kicad_pro"))?;
    fs::copy(rules_dru, scratch.join("board.kicad_dru"))?;

    let root = workspace_root()?;
    let awx = root.join("work/kicad_routing_tools");
    let python_paths = [
        strict.to_path_buf(),
        strict.join("pydeps"),
        root.join("outputs/fix_validation/awx_parity_runtime/pydeps"),
        root.join("outputs/fix_validation/awx_parity_runtime/native"),
        awx.join("py_router"),
        awx.join("rust_router"),
    ];
    let python_path = env::join_paths(python_paths)?;
    let code = "import json,sys; from policy.adapter import preflight; print(json.dumps(preflight(sys.argv[1]), sort_keys=True))";
    let output = Command::new(python)
        .arg("-c")
        .arg(code)
        .arg(&scratch_board)
        .env("PYTHONPATH", python_path)
        .env("PYTHONDONTWRITEBYTECODE", "1")
        .current_dir(&root)
        .output();
    let _ = fs::remove_dir_all(&scratch);
    let output = output?;
    if !output.status.success() {
        return Err(format!(
            "policy adapter no-search preflight failed: {}",
            String::from_utf8_lossy(&output.stderr)
        )
        .into());
    }
    let result: Value = serde_json::from_slice(&output.stdout)?;
    check_policy_preflight(&result, job)?;
    let adapter_sha = sha256(adapter)?;
    if adapter_sha != EXPECTED_POLICY_ADAPTER_SHA256 {
        return Err("policy adapter SHA differs from the reviewed gate".into());
    }
    let geometry_readback = run_geometry_preflight(strict, python, board)?;
    let record = json!({
        "adapter_path": adapter,
        "adapter_sha256": adapter_sha,
        "board_sha256": sha256(board)?,
        "rules_project_sha256": sha256(rules_project)?,
        "rules_dru_sha256": sha256(rules_dru)?,
        "rule_bridge": rule_bridge,
        "input_geometry_readback": geometry_readback,
        "result": result,
    });
    fs::write(
        strict.join("policy/runtime_preflight.json"),
        serde_json::to_vec_pretty(&record)?,
    )?;
    Ok(record)
}

fn run() -> Result<i32, Box<dyn Error>> {
    let args: Vec<_> = env::args().skip(1).collect();
    if args.len() > 1 || args.first().is_some_and(|arg| arg != "--execute-once") {
        return Err("usage: awx_strict_run [--execute-once]".into());
    }
    let execute = args.first().is_some();
    let root = workspace_root()?;
    let strict = root.join("outputs/fix_validation/awx_python_strict");
    let job_path = strict.join("job.json");
    let job: Value = serde_json::from_slice(&fs::read(&job_path)?)?;
    check_job(&job)?;

    let board = strict.join(
        job["input_board"]
            .as_str()
            .ok_or("job input_board missing")?,
    );
    let scene = strict.join(
        job["scene_reference"]
            .as_str()
            .ok_or("job scene_reference missing")?,
    );
    let dsn = strict.join(job["dsn_source"].as_str().ok_or("job dsn_source missing")?);
    let rules_project = strict.join(
        job["rules_project"]
            .as_str()
            .ok_or("job rules_project missing")?,
    );
    let rules_dru = strict.join(job["rules_dru"].as_str().ok_or("job rules_dru missing")?);
    if !rules_project.is_file() || !rules_dru.is_file() {
        return Err("rule-owner bridge .kicad_pro/.kicad_dru files are not ready".into());
    }
    let awx = root.join("work/kicad_routing_tools");
    let head = Command::new("git")
        .args([
            "-C",
            awx.to_str().ok_or("non-UTF8 AWX path")?,
            "rev-parse",
            "HEAD",
        ])
        .output()?;
    if !head.status.success() || String::from_utf8(head.stdout)?.trim() != EXPECTED_AWX_HEAD {
        return Err("original AWX checkout HEAD differs from the frozen source".into());
    }
    let status = Command::new("git")
        .args([
            "-C",
            awx.to_str().ok_or("non-UTF8 AWX path")?,
            "status",
            "--porcelain",
        ])
        .output()?;
    if !status.status.success() || !status.stdout.is_empty() {
        return Err("original AWX checkout is not clean".into());
    }
    let input_hashes = json!({
        "frozen_source_board": sha256(&strict.join("board.kicad_pcb"))?,
        "active_board": sha256(&board)?,
        "scene": sha256(&scene)?,
        "dsn": sha256(&dsn)?,
    });
    if input_hashes["frozen_source_board"] != EXPECTED_BOARD_SHA256
        || input_hashes["scene"] != EXPECTED_SCENE_SHA256
        || input_hashes["dsn"] != EXPECTED_DSN_SHA256
        || job["input_board_sha256"].as_str() != input_hashes["active_board"].as_str()
    {
        return Err(format!("frozen input hash mismatch: {input_hashes}").into());
    }
    let run_dir = strict.join(
        job["run_directory"]
            .as_str()
            .ok_or("job run_directory missing")?,
    );
    let launch_record = strict.join("one_shot_launch.json");
    if run_dir.exists() || launch_record.exists() {
        return Err(
            "strict run or one-shot launch record already exists; refusing a second invocation"
                .into(),
        );
    }
    let python = job["python"].as_str().ok_or("job python path missing")?;
    if !Path::new(python).is_file() {
        return Err(format!("configured Python interpreter does not exist: {python}").into());
    }
    let adapter = strict.join("policy/adapter.py");
    let rule_preflight = check_rule_bridge(&strict, &board, &rules_project, &rules_dru)?;
    let policy_preflight = run_policy_preflight(
        &strict,
        python,
        &board,
        &rules_project,
        &rules_dru,
        &adapter,
        &job,
        &rule_preflight,
    )?;
    println!("strict preflight ready; AWX HEAD={EXPECTED_AWX_HEAD}; input hashes={input_hashes}");
    println!(
        "strict policy preflight passed; adapter SHA256={}",
        policy_preflight["adapter_sha256"]
    );
    if !execute {
        println!("No routing started. Use --execute-once only after the root rule and policy gates pass.");
        return Ok(0);
    }
    let now = SystemTime::now().duration_since(UNIX_EPOCH)?.as_secs();
    fs::write(
        &launch_record,
        serde_json::to_vec_pretty(&json!({
            "status": "launching",
            "unix_seconds": now,
            "awx_head": EXPECTED_AWX_HEAD,
            "input_hashes": input_hashes,
            "job_sha256": sha256(&job_path)?,
        "rules_project_sha256": sha256(&rules_project)?,
        "rules_dru_sha256": sha256(&rules_dru)?,
            "route_invocations_allowed": 1
        }))?,
    )?;

    let mut child = Command::new(python)
        .arg(strict.join("run_awx_cli.py"))
        .current_dir(&root)
        .env("PYTHON", python)
        .env("PYTHONDONTWRITEBYTECODE", "1")
        .spawn()?;
    let python_pid = child.id();
    fs::write(
        &launch_record,
        serde_json::to_vec_pretty(&json!({
            "status": "running",
            "unix_seconds": now,
            "runner_pid": std::process::id(),
            "python_pid": python_pid,
            "awx_head": EXPECTED_AWX_HEAD,
            "input_hashes": input_hashes,
            "job_sha256": sha256(&job_path)?,
            "rules_project_sha256": sha256(&rules_project)?,
            "rules_dru_sha256": sha256(&rules_dru)?,
            "adapter_sha256": policy_preflight["adapter_sha256"],
            "rule_bridge_sha256": policy_preflight["rule_bridge"]["manifest_sha256"],
            "route_log": strict.join("run/route.log"),
            "measurement": strict.join("run/measurement.json"),
            "route_invocations_allowed": 1
        }))?,
    )?;
    let output = child.wait()?;
    let code = output.code().unwrap_or(1);
    fs::write(
        &launch_record,
        serde_json::to_vec_pretty(&json!({
            "status": if code == 0 { "completed" } else { "failed" },
            "unix_seconds": now,
            "return_code": code,
            "runner_pid": std::process::id(),
            "python_pid": python_pid,
            "awx_head": EXPECTED_AWX_HEAD,
            "input_hashes": input_hashes,
            "job_sha256": sha256(&job_path)?,
        "rules_project_sha256": sha256(&rules_project)?,
        "rules_dru_sha256": sha256(&rules_dru)?,
            "route_invocations_allowed": 1
        }))?,
    )?;
    Ok(code)
}

fn main() {
    match run() {
        Ok(code) => std::process::exit(code),
        Err(error) => {
            eprintln!("awx_strict_run: {error}");
            std::process::exit(2);
        }
    }
}
