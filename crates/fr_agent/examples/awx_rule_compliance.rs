use fr_dsn::model::Rule;
use serde_json::{json, Value};
use sha2::{Digest, Sha256};
use std::collections::HashMap;
use std::error::Error;
use std::fs;
use std::path::{Path, PathBuf};

const ORIGINAL_DSN: &str = "/Users/apple/Downloads/Autorouter_PCB1_2_2026-10-13.dsn";
const COMPLIANCE_CLASS: &str = "design/configuration_compliance";
const MM_PER_MIL: f64 = 0.0254;
const ABS_TOLERANCE_MM: f64 = 0.000001;

#[derive(Debug)]
struct RawDsnClass {
    class_index: usize,
    name: String,
    members: Vec<String>,
    width_mil: Option<f64>,
    via_names: Vec<String>,
    source_line: usize,
    width_line: Option<usize>,
    raw_header: String,
}

#[derive(Debug, serde::Deserialize)]
struct ComplianceConfig {
    original_dsn_path: PathBuf,
    rules_contract_path: PathBuf,
    route_geometry_path: PathBuf,
    run_manifest_path: PathBuf,
    json_output_path: PathBuf,
    csv_output_path: PathBuf,
}

fn workspace_root() -> Result<PathBuf, Box<dyn Error>> {
    let mut current = std::env::current_dir()?;
    loop {
        if current
            .join("outputs/fix_validation/awx_python_clean/run/command.json")
            .is_file()
        {
            return Ok(current);
        }
        if !current.pop() {
            break;
        }
    }
    Err("workspace root not found (expected outputs/fix_validation/awx_python_clean/run/command.json)".into())
}

fn read_json(path: &Path) -> Result<Value, Box<dyn Error>> {
    Ok(serde_json::from_slice(&fs::read(path)?)?)
}

fn sha256(bytes: &[u8]) -> String {
    format!("{:x}", Sha256::digest(bytes))
}

fn coord_pair(value: &Value, field: &str) -> Result<[f64; 2], Box<dyn Error>> {
    let pair = value
        .get(field)
        .and_then(Value::as_array)
        .ok_or_else(|| format!("missing coordinate pair '{field}'"))?;
    if pair.len() != 2 {
        return Err(format!("coordinate pair '{field}' must have two entries").into());
    }
    Ok([
        pair[0].as_f64().ok_or("non-numeric coordinate")?,
        pair[1].as_f64().ok_or("non-numeric coordinate")?,
    ])
}

fn text(value: Option<&Value>) -> String {
    value
        .map(|v| {
            v.as_str()
                .map(str::to_owned)
                .unwrap_or_else(|| v.to_string())
        })
        .unwrap_or_default()
}

fn close(actual: f64, declared: f64, _actual_value: &Value) -> bool {
    (actual - declared).abs() <= ABS_TOLERANCE_MM
}

fn csv_escape(value: &str) -> String {
    format!("\"{}\"", value.replace('"', "\"\""))
}

fn optional_number(value: Option<&Value>) -> String {
    value.map(Value::to_string).unwrap_or_default()
}

fn quoted_members(header: &str) -> Vec<String> {
    let mut members = Vec::new();
    let mut rest = header;
    while let Some(open) = rest.find('\'') {
        rest = &rest[open + 1..];
        let Some(close) = rest.find('\'') else { break };
        members.push(rest[..close].to_owned());
        rest = &rest[close + 1..];
    }
    members
}

fn number_after<'a>(text: &'a str, marker: &str) -> Option<&'a str> {
    let rest = text.split_once(marker)?.1.trim_start();
    Some(
        rest.split(|c: char| c.is_ascii_whitespace() || c == ')')
            .next()?,
    )
}

fn parse_original_classes(text: &str) -> Result<Vec<RawDsnClass>, String> {
    let lines: Vec<&str> = text.lines().collect();
    let starts: Vec<usize> = lines
        .iter()
        .enumerate()
        .filter_map(|(index, line)| line.trim_start().starts_with("(class ").then_some(index))
        .collect();
    let mut classes = Vec::new();
    for (position, &start) in starts.iter().enumerate() {
        let end = starts.get(position + 1).copied().unwrap_or(lines.len());
        let header = lines[start].trim();
        let payload = header
            .strip_prefix("(class")
            .ok_or_else(|| format!("malformed class declaration on line {}", start + 1))?
            .trim();
        let quote = payload.find('\'').ok_or_else(|| {
            format!(
                "class declaration has no quoted net member on line {}",
                start + 1
            )
        })?;
        let name = payload[..quote].trim().to_owned();
        let members = quoted_members(payload);
        let block = lines[start..end].join("\n");
        let width_mil = number_after(&block, "(width ").and_then(|number| number.parse().ok());
        let width_line = lines[start..end]
            .iter()
            .position(|line| line.contains("(width "))
            .map(|offset| start + offset + 1);
        let via_names = block
            .split_once("(use_via ")
            .and_then(|(_, tail)| tail.split_once(')').map(|(names, _)| names))
            .map(|names| names.split_whitespace().map(str::to_owned).collect())
            .unwrap_or_default();
        classes.push(RawDsnClass {
            class_index: position,
            name,
            members,
            width_mil,
            via_names,
            source_line: start + 1,
            width_line,
            raw_header: header.to_owned(),
        });
    }
    Ok(classes)
}

fn class_for_net<'a>(
    classes: &'a [RawDsnClass],
    net_name: &str,
) -> Result<Option<&'a RawDsnClass>, String> {
    let matches: Vec<_> = classes
        .iter()
        .filter(|class| {
            class
                .members
                .iter()
                .any(|member| member.eq_ignore_ascii_case(net_name))
        })
        .collect();
    if matches.len() > 1 {
        return Err(format!(
            "net '{net_name}' appears in multiple DSN class member lists"
        ));
    }
    Ok(matches.first().copied())
}

fn class_width(
    classes: &[RawDsnClass],
    dsn: &fr_dsn::Dsn,
    net_name: &str,
    original_dsn_path: &Path,
) -> Result<(f64, String), String> {
    if let Some(class) = class_for_net(classes, net_name)? {
        if let Some(width) = class.width_mil {
            return Ok((
                width * MM_PER_MIL,
                format!(
                    "{}:{} (class at line {}: {}) rule (width {width})",
                    original_dsn_path.display(),
                    class.width_line.unwrap_or(class.source_line),
                    class.source_line,
                    class.raw_header
                ),
            ));
        }
    }
    let width = dsn
        .structure
        .rules
        .iter()
        .find_map(|rule| match rule {
            Rule::Width(value) => Some(*value),
            Rule::Clearance { .. } => None,
        })
        .ok_or_else(|| {
            "neither matching class width nor global structure width exists".to_string()
        })?;
    Ok((
        width * MM_PER_MIL,
        format!(
            "{}:structure rule (width {width}) [unmatched net; default width]",
            original_dsn_path.display()
        ),
    ))
}

fn class_via_names(classes: &[RawDsnClass], net_name: &str) -> Result<Vec<String>, String> {
    class_for_net(classes, net_name)?
        .map(|class| class.via_names.clone())
        .ok_or_else(|| {
            format!("net '{net_name}' has no explicit DSN class assignment for via style")
        })
}

fn push_finding(
    findings: &mut Vec<Value>,
    csv_rows: &mut Vec<String>,
    kind: &str,
    severity: &str,
    actual: f64,
    declared: f64,
    net_id: Option<i64>,
    net_name: &str,
    layer: &str,
    layers: &str,
    coords: Value,
    uuid: &str,
    rule_source: &str,
) {
    let row = json!({
        "type": kind,
        "severity": severity,
        "actual": actual,
        "declared": declared,
        "net_id": net_id,
        "net_name": net_name,
        "layer": layer,
        "layers": layers,
        "coords": coords,
        "item_uuid": uuid,
        "raw_rule_source": rule_source,
        "classification": COMPLIANCE_CLASS
    });
    let coord_text = serde_json::to_string(&row["coords"]).unwrap_or_default();
    let fields = [
        text(row.get("type")).into(),
        text(row.get("severity")).into(),
        optional_number(row.get("actual")),
        optional_number(row.get("declared")),
        optional_number(row.get("net_id")),
        text(row.get("net_name")),
        text(row.get("layer")),
        text(row.get("layers")),
        coord_text,
        text(row.get("item_uuid")),
        text(row.get("raw_rule_source")),
        text(row.get("classification")),
    ];
    csv_rows.push(
        fields
            .iter()
            .map(|field| csv_escape(field))
            .collect::<Vec<_>>()
            .join(","),
    );
    findings.push(row);
}

fn main() -> Result<(), Box<dyn Error>> {
    let root = workspace_root()?;
    let args: Vec<_> = std::env::args_os().skip(1).collect();
    let (original_dsn_path, rules_path, geometry_path, job_path, json_out, csv_out) =
        if args.len() == 1 {
            let config: ComplianceConfig = serde_json::from_slice(&fs::read(&args[0])?)?;
            (
                config.original_dsn_path,
                config.rules_contract_path,
                config.route_geometry_path,
                config.run_manifest_path,
                config.json_output_path,
                config.csv_output_path,
            )
        } else if args.is_empty() {
            (
                PathBuf::from(ORIGINAL_DSN),
                root.join("outputs/fix_validation/awx_original_drc/original_rules.json"),
                root.join("outputs/fix_validation/awx_original_drc/route_geometry.json"),
                root.join("outputs/fix_validation/awx_python_clean/run/command.json"),
                root.join("outputs/fix_validation/awx_original_drc/compliance.json"),
                root.join("outputs/fix_validation/awx_original_drc/compliance.csv"),
            )
        } else {
            return Err("usage: awx_rule_compliance [config.json]".into());
        };

    let rules_contract = read_json(&rules_path)?;
    let original_bytes = fs::read(&original_dsn_path)?;
    let original_hash = sha256(&original_bytes);
    let expected_hash = rules_contract["sources"]["original_user_dsn"]["sha256"]
        .as_str()
        .ok_or("original_rules.json has no original DSN sha256")?;
    if original_hash != expected_hash {
        return Err(format!(
            "original DSN hash mismatch: expected {expected_hash}, got {original_hash}"
        )
        .into());
    }
    let dsn = fr_dsn::Dsn::parse(&original_bytes)?;
    let original_text = std::str::from_utf8(&original_bytes)?;
    let raw_classes = parse_original_classes(original_text)?;
    let geometry_bytes = fs::read(&geometry_path)?;
    let geometry_hash = sha256(&geometry_bytes);
    let geometry: Value = serde_json::from_slice(&geometry_bytes)?;
    let job = read_json(&job_path)?;
    let job_hash = sha256(&fs::read(&job_path)?);

    let argv = job["argv"]
        .as_array()
        .ok_or("run command has no argv array")?;
    let drill_arg = argv
        .iter()
        .position(|arg| arg.as_str() == Some("--via-drill"))
        .and_then(|index| argv.get(index + 1))
        .and_then(Value::as_str);
    let requested_drill_mm: f64 = if let Some(drill_arg) = drill_arg {
        drill_arg.parse()?
    } else {
        job["route_settings"]["via_drill_mm"]
            .as_f64()
            .ok_or("run manifest has neither --via-drill nor route_settings.via_drill_mm")?
    };
    let contract_drill_mm = rules_contract["sources"]["original_run_job"]["fields"]["--via-drill"]
        .as_f64()
        .ok_or("original_rules.json has no job drill contract")?;
    if (requested_drill_mm - contract_drill_mm).abs() > ABS_TOLERANCE_MM {
        return Err("command.json via-drill does not match original_rules.json".into());
    }

    let via0_mm = rules_contract["raw_rules"]["via_definitions"]["padstacks"]
        .as_array()
        .and_then(|items| items.iter().find(|item| item["name"] == "via0"))
        .and_then(|item| item["copper_diameter_mm"].as_f64())
        .ok_or("original_rules.json has no via0 copper diameter")?;
    let nets: HashMap<i64, String> = geometry["nets"]
        .as_array()
        .ok_or("route_geometry.json has no nets array")?
        .iter()
        .filter_map(|net| Some((net["id"].as_i64()?, net["name"].as_str()?.to_owned())))
        .collect();
    let mut assignment_samples = Vec::new();
    if let Some(geometry_nets) = geometry["nets"].as_array() {
        let mut saw_spaced_class = false;
        for net in geometry_nets {
            let Some(net_name) = net["name"].as_str() else {
                continue;
            };
            if net_name.is_empty() {
                continue;
            }
            let Ok(Some(class)) = class_for_net(&raw_classes, net_name) else {
                continue;
            };
            if assignment_samples.len() == 1 && (!class.name.contains(' ') || saw_spaced_class) {
                continue;
            }
            saw_spaced_class |= class.name.contains(' ');
            let parser_class = dsn.network.classes.get(class.class_index);
            let sample = json!({
                "geometry_net_id": net["id"],
                "geometry_net_name": net_name,
                "dsn_class_name": class.name.clone(),
                "dsn_class_members": class.members.clone(),
                "raw_class_header": class.raw_header.clone(),
                "dsn_class_width_mil": class.width_mil,
                "dsn_class_use_via": class.via_names.clone(),
                "fr_dsn_parsed_class_name": parser_class.map(|parsed| parsed.name.as_str()),
                "fr_dsn_parsed_net_tokens": parser_class.map(|parsed| parsed.nets.clone())
            });
            eprintln!("DSN class assignment evidence: {sample}");
            assignment_samples.push(sample);
            if assignment_samples.len() == 2 && saw_spaced_class {
                break;
            }
        }
    }

    let mut findings = Vec::new();
    let mut csv_rows = vec![
        "type,severity,actual,declared,net_id,net_name,layer,layers,coords,item_uuid,raw_rule_source,classification".to_string(),
    ];
    let mut unknowns = Vec::new();
    let tracks = geometry["tracks"]
        .as_array()
        .ok_or("route_geometry.json has no tracks array")?;
    let mut segments_checked = 0usize;
    for segment in tracks {
        let actual_value = segment.get("width_mm").ok_or("track has no width_mm")?;
        let actual = actual_value
            .as_f64()
            .ok_or("segment width is not numeric")?;
        let net_id = segment.get("net_id").and_then(Value::as_i64);
        let net_from_geometry = text(segment.get("net"));
        let net_name = if net_from_geometry.is_empty() {
            net_id
                .and_then(|id| nets.get(&id))
                .map(String::as_str)
                .unwrap_or("")
        } else {
            net_from_geometry.as_str()
        };
        let item_uuid = text(segment.get("uuid"));
        let layer = text(segment.get("layer"));
        let start = coord_pair(segment, "start")?;
        let end = coord_pair(segment, "end")?;
        let coords = json!({"start": start, "end": end});
        if net_name.is_empty() {
            unknowns.push(json!({"kind":"segment_width", "item_uuid":item_uuid, "reason":"net id missing from geometry net table; class-specific width cannot be resolved"}));
            continue;
        }
        match class_width(&raw_classes, &dsn, net_name, &original_dsn_path) {
            Ok((declared, source)) => {
                segments_checked += 1;
                if !close(actual, declared, actual_value) {
                    let severity = if actual < declared { "narrower_than_declared" } else { "wider_than_declared" };
                    push_finding(&mut findings, &mut csv_rows, "declared_trace_width_deviation", severity, actual, declared, net_id, net_name, &layer, "", coords, &item_uuid, &source);
                }
            }
            Err(reason) => unknowns.push(json!({"kind":"segment_width", "item_uuid":item_uuid, "net":net_name, "reason":reason})),
        }
    }

    let vias = geometry["vias"]
        .as_array()
        .ok_or("route_geometry.json has no vias array")?;
    let mut vias_style_checked = 0usize;
    let mut vias_drill_checked = 0usize;
    for via in vias {
        let size_value = via.get("diameter_mm").ok_or("via has no diameter_mm")?;
        let size = size_value.as_f64().ok_or("via size is not numeric")?;
        let drill_value = via.get("drill_mm").ok_or("via has no drill_mm")?;
        let drill = drill_value.as_f64().ok_or("via drill is not numeric")?;
        let net_id = via.get("net_id").and_then(Value::as_i64);
        let net_from_geometry = text(via.get("net"));
        let net_name = if net_from_geometry.is_empty() {
            net_id
                .and_then(|id| nets.get(&id))
                .map(String::as_str)
                .unwrap_or("")
        } else {
            net_from_geometry.as_str()
        };
        let item_uuid = text(via.get("uuid"));
        let at = coord_pair(via, "at")?;
        let layers = via
            .get("layers")
            .and_then(Value::as_array)
            .map(|items| {
                items
                    .iter()
                    .map(|item| text(Some(item)))
                    .collect::<Vec<_>>()
                    .join("|")
            })
            .unwrap_or_default();
        let coords = json!({"at": at});

        if net_name.is_empty() {
            unknowns.push(json!({"kind":"via_style", "item_uuid":item_uuid, "reason":"via net id missing from geometry net table; class-specific via style cannot be resolved"}));
        } else {
            match class_via_names(&raw_classes, net_name) {
                Ok(allowed) if allowed.iter().any(|name| name == "via0") => {
                    vias_style_checked += 1;
                    let source = format!("{}:network.class member net(s) {:?} (use_via via0); via0 padstack TopLayer/BottomLayer circle 24 mil", original_dsn_path.display(), allowed);
                    if !close(size, via0_mm, size_value) {
                        let severity = if size < via0_mm { "smaller_than_selected_style" } else { "larger_than_selected_style" };
                        push_finding(&mut findings, &mut csv_rows, "via_style_deviation", severity, size, via0_mm, net_id, net_name, "", &layers, coords.clone(), &item_uuid, &source);
                    }
                }
                Ok(allowed) => unknowns.push(json!({"kind":"via_style", "item_uuid":item_uuid, "net":net_name, "reason":format!("assigned class has no use_via via0; allowed styles: {:?}", allowed)})),
                Err(reason) => unknowns.push(json!({"kind":"via_style", "item_uuid":item_uuid, "net":net_name, "reason":reason})),
            }
        }

        vias_drill_checked += 1;
        if !close(drill, requested_drill_mm, drill_value) {
            let severity = if drill < requested_drill_mm {
                "smaller_than_requested"
            } else {
                "larger_than_requested"
            };
            let source = if let Some(drill_arg) = drill_arg {
                format!("{}:argv --via-drill {} mm", job_path.display(), drill_arg)
            } else {
                format!(
                    "{}:route_settings.via_drill_mm {} mm",
                    job_path.display(),
                    requested_drill_mm
                )
            };
            push_finding(
                &mut findings,
                &mut csv_rows,
                "requested_drill_deviation",
                severity,
                drill,
                requested_drill_mm,
                net_id,
                net_name,
                "",
                &layers,
                coords,
                &item_uuid,
                &source,
            );
        }
    }

    let finding_count = findings.len();
    let output = json!({
        "classification": COMPLIANCE_CLASS,
        "sources": {
            "original_dsn_path": original_dsn_path.display().to_string(),
            "original_dsn_sha256": original_hash,
            "original_run_job_path": job_path.display().to_string(),
            "original_run_job_sha256": job_hash,
            "route_geometry_path": geometry_path.display().to_string(),
            "route_geometry_sha256": geometry_hash,
            "route_geometry_schema": geometry.get("schema"),
            "source_output_sha256": geometry.get("source_output_sha256"),
            "rules_contract_path": rules_path.display().to_string()
        },
        "contracts": {
            "trace_width_resolution": "Resolve each geometry net through the explicit class member lists in original DSN (case-insensitive); use that class's width rule, otherwise the structure global width.",
            "via_style": "Check use_via by resolved class membership; via0 diameter is a selected-style conformance value, not a manufacturing minimum.",
            "via_drill": "Compare with the exact original command --via-drill request; this is invocation conformance, not a DSN minimum-drill rule.",
            "classification": COMPLIANCE_CLASS,
            "numeric_tolerance_mm": ABS_TOLERANCE_MM
        },
        "coverage": {
            "tracks_total": tracks.len(),
            "segments_checked": segments_checked,
            "vias_total": vias.len(),
            "via_style_checked": vias_style_checked,
            "via_drill_checked": vias_drill_checked,
            "findings": findings.len(),
            "unknowns": unknowns.len(),
            "width_tolerance": "fixed 1e-6 mm; no tolerance inferred from JSON float formatting",
            "size_request_tolerance": "fixed 1e-6 mm",
            "dsn_class_assignment_samples": assignment_samples
        },
        "findings": findings,
        "unknowns": unknowns
    });
    if let Some(parent) = json_out.parent() {
        fs::create_dir_all(parent)?;
    }
    if let Some(parent) = csv_out.parent() {
        fs::create_dir_all(parent)?;
    }
    fs::write(&json_out, serde_json::to_vec_pretty(&output)?)?;
    fs::write(&csv_out, csv_rows.join("\n") + "\n")?;
    println!(
        "wrote {} finding(s) to {} and {}",
        finding_count,
        json_out.display(),
        csv_out.display()
    );
    Ok(())
}
