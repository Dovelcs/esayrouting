use fr_agent::spec::LoadedSpec;
use std::{fs::OpenOptions, io::Write, path::Path};
const TEMPLATE: &str = include_str!("../../../examples/agent_route/basic.toml");
fn run() -> Result<(), String> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.len() == 1 && args[0] == "--version" {
        println!("fastroute_agent {}", env!("CARGO_PKG_VERSION"));
        return Ok(());
    }
    if args.len() == 1 && args[0] == "--help" {
        println!("usage: fastroute_agent <check|plan|route|awx-points|awx-residual|prepare-stack|init> job.toml\nawx-points: source AWX GridRouter for native pin-set airlines; awx-residual: only the target net's frozen initial incompletes from an input SES; check: structure only; plan: validation and trial routing; route: verified SES export; init: new configuration\nDistances are mm. Differential coupling, impedance, and cross-net paths are unsupported hard constraints.");
        return Ok(());
    }
    if args.len() != 2 {
        return Err("usage: fastroute_agent <check|plan|route|awx-points|awx-residual|prepare-stack|init> job.toml".into());
    }
    let path = Path::new(&args[1]);
    match args[0].as_str() {
        "init" => {
            let mut f = OpenOptions::new()
                .write(true)
                .create_new(true)
                .open(path)
                .map_err(|e| format!("{}: {e}", path.display()))?;
            f.write_all(TEMPLATE.as_bytes())
                .map_err(|e| e.to_string())?;
            println!("created {}", path.display());
        }
        "prepare-stack" => return fr_agent::stack_prepare::run(path),
        "awx-points" => return fr_agent::awx_endpoint::run(path),
        "awx-residual" => return fr_agent::awx_endpoint::run_residual(path),
        "check" => {
            let loaded = LoadedSpec::load(path).map_err(|e| e.to_string())?;
            println!(
                "{}",
                serde_json::json!({"stage":"structure_validated","board_validated":false,"constraints_satisfied":false,"input":loaded.spec.input,"output":loaded.spec.output,"rules":loaded.spec.structure_reports(),"detail":"Run plan to validate board references and trial constraints; structure check does not execute routing."})
            );
        }
        "plan" | "route" => {
            let loaded = LoadedSpec::load(path).map_err(|e| e.to_string())?;
            let report = match fr_agent::execution::run(&loaded, args[0] == "plan") {
                Ok(report) => report,
                Err(error) => {
                    // Preserve requested rule inventory without claiming it executed.
                    println!(
                        "{}",
                        serde_json::json!({"success":false,"error":error,"rules":loaded.spec.structure_reports()})
                    );
                    return Err("routing or board validation failed; see structured report".into());
                }
            };
            println!(
                "{}",
                serde_json::to_string_pretty(&report).map_err(|e| e.to_string())?
            );
            if !report.success {
                return Err("routing incomplete; see structured report".into());
            }
        }
        _ => {
            return Err("unknown command; expected check, plan, route, awx-points, or init".into())
        }
    }
    Ok(())
}
fn main() {
    if let Err(e) = run() {
        eprintln!("fastroute_agent: {e}");
        std::process::exit(1);
    }
}
