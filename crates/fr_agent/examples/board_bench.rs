//! Configuration-driven native executable benchmark with shared Rust board measurements.
use fr_agent::board_adapter::BoardAdapter;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    fs,
    path::{Path, PathBuf},
    process::Command,
    time::Instant,
};
#[derive(Deserialize)]
struct Config {
    input: PathBuf,
    cwd: PathBuf,
    log_dir: PathBuf,
    output: PathBuf,
    native_archive_sha256: String,
    source_original: PathBuf,
    normalization: serde_json::Value,
    source_identity_verified: bool,
    normalization_verified: bool,
    native_eda_validated: bool,
    allow_import_limited_comparison: bool,
    import_caveats: serde_json::Value,
    runs: Vec<Run>,
}
#[derive(Deserialize, Serialize)]
struct Run {
    name: String,
    binary: PathBuf,
    args: Vec<String>,
    session: PathBuf,
    seed: u64,
    workers: usize,
    budget_ms: u64,
    commit: String,
    native_version: String,
    #[serde(default)]
    archive: Option<PathBuf>,
}
fn hash(bytes: &[u8]) -> String {
    format!("{:x}", Sha256::digest(bytes))
}
fn resolve(base: &Path, path: &Path) -> PathBuf {
    if path.is_absolute() {
        path.to_owned()
    } else {
        base.join(path)
    }
}
fn item_geometry(adapter: &BoardAdapter, key: fr_engine::board::ItemKey) -> serde_json::Value {
    let b = &adapter.board;
    let item = b.item(key);
    let bbox = item.bounding_box(b);
    let u = adapter.units_per_mm;
    let kind = match &item.kind {
        fr_engine::board::ItemKind::Pin(_) => "pin",
        fr_engine::board::ItemKind::Via(_) => "via",
        fr_engine::board::ItemKind::Trace(_) => "trace",
        fr_engine::board::ItemKind::ObstacleArea(_) => "keepout",
        fr_engine::board::ItemKind::ConductionArea(_) => "copper_area",
        fr_engine::board::ItemKind::ComponentOutline(_) => "component_outline",
        fr_engine::board::ItemKind::BoardOutline(_) => "board_outline",
    };
    let nets: Vec<_> = item
        .net_numbers()
        .iter()
        .filter_map(|&n| b.rules.nets.get(n).map(|net| net.name.clone()))
        .collect();
    let trace_points = if item.is_trace() {
        Some(
            item.trace()
                .polyline()
                .corner_approx_arr()
                .iter()
                .map(|p| [p.x / u, p.y / u])
                .collect::<Vec<_>>(),
        )
    } else {
        None
    };
    serde_json::json!({"type":kind,"nets":nets,"component_number":item.component_no(),"layers":[item.first_layer(b),item.last_layer(b)],"bbox_mm":[[bbox.ll.x as f64/u,bbox.ll.y as f64/u],[bbox.ur.x as f64/u,bbox.ur.y as f64/u]],"trace_points_mm":trace_points})
}
fn violation_evidence(adapter: &BoardAdapter) -> (Vec<String>, Vec<serde_json::Value>) {
    let b = &adapter.board;
    let mut entries = std::collections::BTreeMap::new();
    for key in b.item_repository().iter() {
        for violation in fr_engine::drc::clearance_violation::clearance_violations(b, key) {
            let mut items = vec![
                item_geometry(adapter, violation.first_item),
                item_geometry(adapter, violation.second_item),
            ];
            items.sort_by_key(|v| v.to_string());
            let row = serde_json::json!({"items":items,"layer":violation.layer,"required_mm":violation.expected_clearance/adapter.units_per_mm,"actual_mm":violation.actual_clearance/adapter.units_per_mm,"category":format!("{:?}",violation.category(b))});
            let signature = hash(row.to_string().as_bytes());
            entries.insert(signature, row);
        }
    }
    let signatures = entries.keys().cloned().collect();
    let mut samples: Vec<_> = entries.into_values().collect();
    samples.sort_by(|a, b| {
        let margin = |v: &serde_json::Value| {
            v["required_mm"].as_f64().unwrap() - v["actual_mm"].as_f64().unwrap()
        };
        margin(b).total_cmp(&margin(a))
    });
    samples.truncate(5);
    (signatures, samples)
}
fn source_revision(cwd: &Path) -> serde_json::Value {
    let run = |args: &[&str]| {
        Command::new("git")
            .args(args)
            .current_dir(cwd)
            .output()
            .ok()
            .filter(|o| o.status.success())
            .map(|o| o.stdout)
    };
    let commit = run(&["rev-parse", "HEAD"]).map(|b| String::from_utf8_lossy(&b).trim().to_owned());
    let diff = run(&["diff", "--binary", "HEAD"]);
    let status = run(&["status", "--porcelain"]).map(|b| String::from_utf8_lossy(&b).to_string());
    serde_json::json!({"commit":commit,"tracked_diff_sha256":diff.as_ref().map(|b|hash(b)),"status":status,"note":"binary_sha256 captures exact executable; diff digest covers tracked edits only, status lists untracked source"})
}
fn quality(adapter: &BoardAdapter) -> serde_json::Value {
    let b = &adapter.board;
    let (violation_signatures, violation_samples) = violation_evidence(adapter);
    let mut airlines = Vec::new();
    for n in 1..=b.rules.nets.max_net_number() {
        let incomplete = fr_engine::drc::NetIncompletes::new(b, n, &b.get_connectable_items(n));
        for a in incomplete.incompletes {
            airlines.push(serde_json::json!({"net_number":n,"from_mm":[a.from_corner.x/adapter.units_per_mm,a.from_corner.y/adapter.units_per_mm],"to_mm":[a.to_corner.x/adapter.units_per_mm,a.to_corner.y/adapter.units_per_mm]}));
        }
    }
    let length: f64 = b
        .get_traces()
        .iter()
        .map(|&k| b.item(k).trace().polyline().length_approx() / adapter.units_per_mm)
        .sum();
    serde_json::json!({"unrouted":airlines.len(),"clearance_violations":adapter.violations().len(),"native_drc_record_count":adapter.violations().len(),"unique_geometry_violation_count":violation_signatures.len(),"vias":b.get_vias().len(),"traces":b.get_traces().len(),"trace_length_mm":length,"airlines_mm":airlines,"units_per_mm":adapter.units_per_mm,"violation_signatures":violation_signatures,"violation_samples":violation_samples})
}
fn violation_delta(before: &serde_json::Value, after: &serde_json::Value) -> serde_json::Value {
    let signatures = |v: &serde_json::Value| {
        v["violation_signatures"]
            .as_array()
            .unwrap()
            .iter()
            .map(|x| x.as_str().unwrap().to_owned())
            .collect::<std::collections::BTreeSet<String>>()
    };
    let a = signatures(before);
    let b = signatures(after);
    let added: Vec<_> = b.difference(&a).cloned().collect();
    let removed: Vec<_> = a.difference(&b).cloned().collect();
    serde_json::json!({"preexisting_retained":a.intersection(&b).count(),"new_count":added.len(),"removed_count":removed.len(),"new_geometry_signatures":added,"removed_geometry_signatures":removed})
}
fn resource(stderr: &str) -> serde_json::Value {
    let mut wall = None;
    let mut user = None;
    let mut sys = None;
    let mut rss = None;
    for line in stderr.lines() {
        let fields: Vec<_> = line.split_whitespace().collect();
        if fields.len() >= 6 && fields[1] == "real" && fields[3] == "user" && fields[5] == "sys" {
            wall = fields[0].parse::<f64>().ok();
            user = fields[2].parse::<f64>().ok();
            sys = fields[4].parse::<f64>().ok();
        }
        if line.contains("maximum resident set size") {
            rss = fields.first().and_then(|s| s.parse::<u64>().ok());
        }
    }
    serde_json::json!({"wall_seconds":wall,"user_seconds":user,"system_seconds":sys,"peak_rss_bytes":rss})
}
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let path = PathBuf::from(
        std::env::args()
            .nth(1)
            .ok_or("usage: board_bench benchmark.json")?,
    );
    let bytes = fs::read(&path)?;
    let config: Config = serde_json::from_slice(&bytes)?;
    let base = path.parent().unwrap_or(Path::new("."));
    let input = resolve(base, &config.input);
    let cwd = resolve(base, &config.cwd);
    let logs = resolve(base, &config.log_dir);
    let output = resolve(base, &config.output);
    if output.exists() {
        return Err("benchmark output already exists".into());
    }
    fs::create_dir_all(&logs)?;
    let original = resolve(base, &config.source_original);
    let original_bytes = fs::read(&original)?;
    let original_hash = hash(&original_bytes);
    let revision_start = source_revision(&cwd);
    let source = fs::read(&input)?;
    let source_hash = hash(&source);
    let raw = quality(&BoardAdapter::load_raw(&source)?);
    let normalized = quality(&BoardAdapter::load(&source)?);
    let mut runs = Vec::new();
    for run in &config.runs {
        if !config.source_identity_verified
            || !config.normalization_verified
            || (!config.native_eda_validated && !config.allow_import_limited_comparison)
        {
            return Err(
                "comparative runs require source/normalization checks and explicit EDA or import-limited scope".into(),
            );
        }
        if run.name.is_empty()
            || !run
                .name
                .bytes()
                .all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == b'_')
        {
            return Err("run name must be lowercase snake_case".into());
        }
        let binary = resolve(base, &run.binary);
        let session = resolve(base, &run.session);
        let binary_hash = hash(&fs::read(&binary)?);
        let (
            exit_code,
            successful,
            wall,
            resources,
            stage_report,
            stage_log_lines,
            archived_session_hash,
        ) = if let Some(path) = &run.archive {
            let archived: serde_json::Value =
                serde_json::from_slice(&fs::read(resolve(base, path))?)?;
            if archived["source_sha256"] != source_hash
                || archived["source_original_sha256"] != original_hash
            {
                return Err("archive source identity mismatch".into());
            }
            let row = archived["runs"]
                .as_array()
                .ok_or("archive runs missing")?
                .iter()
                .find(|row| row["configuration"]["name"] == run.name)
                .ok_or("archive run missing")?;
            if row["binary_sha256"] != binary_hash
                || row["configuration"]["args"] != serde_json::to_value(&run.args)?
                || row["configuration"]["workers"] != run.workers
                || row["configuration"]["budget_ms"] != run.budget_ms
            {
                return Err("archive executable/arguments mismatch".into());
            }
            (
                row["exit_code"].clone(),
                row["successful"].clone(),
                row["runner_wall_seconds"].clone(),
                row["resources"].clone(),
                row["stage_report"].clone(),
                row.get("native_stage_log_lines")
                    .cloned()
                    .unwrap_or_else(|| {
                        let archive_path = resolve(base, path);
                        let logfile = archive_path
                            .parent()
                            .unwrap_or(Path::new("."))
                            .join(format!("{}_stderr.log", run.name));
                        let text = fs::read_to_string(logfile).unwrap_or_default();
                        serde_json::json!(text
                            .lines()
                            .filter(|line| line.contains("stage completed")
                                || line.contains("Fanout pass")
                                || line.contains("pass #")
                                || line.contains("Pipeline thread limits")
                                || line.contains("optimizer mode"))
                            .collect::<Vec<_>>())
                    }),
                row["session_sha256"].as_str().map(str::to_owned),
            )
        } else {
            if session.exists() {
                return Err(format!("session already exists: {}", session.display()).into());
            }
            let begin = Instant::now();
            let result = Command::new("/usr/bin/time")
                .arg("-l")
                .arg(&binary)
                .args(&run.args)
                .current_dir(&cwd)
                .output()?;
            let wall = begin.elapsed().as_secs_f64();
            let stderr = String::from_utf8_lossy(&result.stderr);
            fs::write(
                logs.join(format!("{}_stdout.log", run.name)),
                &result.stdout,
            )?;
            fs::write(
                logs.join(format!("{}_stderr.log", run.name)),
                &result.stderr,
            )?;
            let phases: Vec<_> = stderr
                .lines()
                .filter(|line| {
                    line.contains("stage completed")
                        || line.contains("Fanout pass")
                        || line.contains("pass #")
                        || line.contains("Pipeline thread limits")
                        || line.contains("optimizer mode")
                })
                .map(str::to_owned)
                .collect();
            (
                serde_json::json!(result.status.code()),
                serde_json::json!(result.status.success()),
                serde_json::json!(wall),
                resource(&stderr),
                serde_json::from_slice::<serde_json::Value>(&result.stdout)
                    .unwrap_or(serde_json::Value::Null),
                serde_json::json!(phases),
                None,
            )
        };
        let (session_hash, measured) = if session.exists() {
            let ses = fs::read(&session)?;
            (
                Some(hash(&ses)),
                Some(quality(&BoardAdapter::load_session(&source, &ses)?)),
            )
        } else {
            (None, None)
        };
        if archived_session_hash.is_some() && archived_session_hash != session_hash {
            return Err("archive session hash mismatch".into());
        }
        runs.push(serde_json::json!({"configuration":run,"binary_sha256":binary_hash,"session_sha256":session_hash,"exit_code":exit_code,"stage_report":stage_report,"native_stage_log_lines":stage_log_lines,"successful":successful,"runner_wall_seconds":wall,"resources":resources,"execution_reused":run.archive.is_some(),"quality_remeasured_by_current_adapter":true,"drc_delta_from_normalized":measured.as_ref().map(|q|violation_delta(&normalized,q)),"quality":measured}));
    }
    if hash(&fs::read(&original)?) != original_hash {
        return Err("original DSN changed during benchmark".into());
    }
    let after_hash = hash(&fs::read(&input)?);
    if source_hash != after_hash {
        return Err("source DSN changed during benchmark".into());
    }
    let report = serde_json::json!({"schema_version":1,"original_raw":{"source_format_requires_name_normalization":true,"native_import_quality_measured":false,"pads":config.normalization["pads"],"source_wires":config.normalization["source_wires"],"source_vias":config.normalization["source_vias"],"reason":"original unquoted multi-token names cannot be faithfully loaded by native parser; no electrical metrics inferred from the malformed import"},"source_original":original,"source_original_sha256":original_hash,"normalization":config.normalization,"source_identity_verified":config.source_identity_verified,"normalization_verified":config.normalization_verified,"native_eda_validated":config.native_eda_validated,"allow_import_limited_comparison":config.allow_import_limited_comparison,"import_caveats":config.import_caveats,"source":input,"source_sha256":source_hash,"source_sha256_after":after_hash,"configuration_sha256":hash(&bytes),"source_revision":revision_start,"native_archive_sha256":config.native_archive_sha256,"measurement_semantics":"shared BoardAdapter raw / CLI-normalized / normalized-session; geometry lengths and airline coordinates scaled by units_per_mm; timings use macOS time -l","drc_metric_definitions":{"native_drc_record_count":"Adapter::violations native item-ID pair/layer/expected and actual distance bits deduplicated records","unique_geometry_violation_count":"ID-independent sorted item type/net/layer/mm geometry plus category and required/actual distance hashed as exact JSON numbers","source_session_replay_difference":"This input measured 1086 native records / 1049 geometry signatures before SES export and 1080 / 1043 after unchanged SES replay; six differences remain unassigned to a cause and must not be counted as routing improvement. Both measured outputs use the same final replay checker."},"raw_to_normalized_drc_delta":violation_delta(&raw,&normalized),"raw":raw,"normalized":normalized,"runs":runs});
    use std::io::Write;
    let mut f = fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(&output)?;
    f.write_all(&serde_json::to_vec_pretty(&report)?)?;
    let text = |value: &serde_json::Value| {
        if value.is_null() {
            String::new()
        } else {
            value.to_string()
        }
    };
    let mut csv=String::from("stage,unrouted,native_drc_record_count,unique_geometry_violation_count,vias,traces,trace_length_mm,wall_seconds,user_seconds,system_seconds,peak_rss_bytes,exit_code,new_drc_signatures,removed_drc_signatures\n");
    csv.push_str(&format!(
        "original_raw_unparsed,,,,{},{},,,,,,,,\n",
        text(&report["original_raw"]["source_vias"]),
        text(&report["original_raw"]["source_wires"])
    ));
    for (stage, quality) in [
        ("raw_name_normalized_before_cli", &report["raw"]),
        ("common_normalized", &report["normalized"]),
    ] {
        csv.push_str(&format!(
            "{stage},{},{},{},{},{},{},,,,,,,\n",
            text(&quality["unrouted"]),
            text(&quality["native_drc_record_count"]),
            text(&quality["unique_geometry_violation_count"]),
            text(&quality["vias"]),
            text(&quality["traces"]),
            text(&quality["trace_length_mm"])
        ));
    }
    for row in report["runs"].as_array().unwrap() {
        let q = &row["quality"];
        let r = &row["resources"];
        let d = &row["drc_delta_from_normalized"];
        csv.push_str(&format!(
            "{},{},{},{},{},{},{},{},{},{},{},{},{},{}\n",
            row["configuration"]["name"].as_str().unwrap(),
            text(&q["unrouted"]),
            text(&q["native_drc_record_count"]),
            text(&q["unique_geometry_violation_count"]),
            text(&q["vias"]),
            text(&q["traces"]),
            text(&q["trace_length_mm"]),
            text(&r["wall_seconds"]),
            text(&r["user_seconds"]),
            text(&r["system_seconds"]),
            text(&r["peak_rss_bytes"]),
            text(&row["exit_code"]),
            text(&d["new_count"]),
            text(&d["removed_count"])
        ));
    }
    let mut csv_file = fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(output.with_extension("csv"))?;
    csv_file.write_all(csv.as_bytes())?;
    println!("{}", output.display());
    Ok(())
}
