use fr_agent::board_adapter::BoardAdapter;
use std::{path::PathBuf, process::Command};
struct Scratch(PathBuf);
impl Drop for Scratch {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}
fn scratch() -> Scratch {
    let base = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../../../work");
    let path = base.join(format!(
        "cli_test_{}_{}",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos()
    ));
    std::fs::create_dir_all(&path).unwrap();
    Scratch(path)
}
fn command(command: &str, path: &std::path::Path) -> std::process::Output {
    Command::new(env!("CARGO_BIN_EXE_fastroute_agent"))
        .arg(command)
        .arg(path)
        .output()
        .unwrap()
}
const JOB: &str = r#"version=1
route_remaining=false
input="board.dsn"
output="routed.ses"
[[nets]]
name="BUS"
width_mm=0.15
clearance_mm=0.1
allowed_layers=["Top"]
via_budget=0
max_length_mm=25.0
[[groups]]
id="bus"
nets=["BUS"]
joint_escape=true
[execution]
threads=8
budget_ms=5000
seed=42
"#;
#[test]
fn complete_cli_flow_reloads_checked_ses() {
    let dir = scratch();
    let source = include_bytes!("fixtures/two_chip.dsn");
    std::fs::write(dir.0.join("board.dsn"), source).unwrap();
    let job = dir.0.join("job.toml");
    std::fs::write(&job, JOB).unwrap();
    assert!(command("init", &dir.0.join("template.toml"))
        .status
        .success());
    assert!(command("check", &job).status.success());
    let plan = command("plan", &job);
    assert!(
        plan.status.success(),
        "{}",
        String::from_utf8_lossy(&plan.stderr)
    );
    assert!(!dir.0.join("routed.ses").exists());
    let route = command("route", &job);
    assert!(
        route.status.success(),
        "{}",
        String::from_utf8_lossy(&route.stderr)
    );
    let report: serde_json::Value = serde_json::from_slice(&route.stdout).unwrap();
    assert_eq!(report["success"], true);
    let original = BoardAdapter::load(source).unwrap();
    let routed =
        BoardAdapter::load_session(source, &std::fs::read(dir.0.join("routed.ses")).unwrap())
            .unwrap();
    let n = routed.net_number("BUS").unwrap();
    assert_eq!(
        fr_engine::drc::NetIncompletes::new(
            &routed.board,
            n,
            &routed.board.get_connectable_items(n)
        )
        .count(),
        0
    );
    assert!(routed.violations().is_subset(&original.violations()));
    assert_eq!(std::fs::read(dir.0.join("board.dsn")).unwrap(), source);
}
#[test]
fn failed_cli_never_exports_or_changes_source() {
    let dir = scratch();
    let source = include_bytes!("fixtures/two_chip.dsn");
    std::fs::write(dir.0.join("board.dsn"), source).unwrap();
    let job = dir.0.join("job.toml");
    std::fs::write(&job, JOB.replace("max_length_mm=25.0", "max_length_mm=1.0")).unwrap();
    let route = command("route", &job);
    assert!(!route.status.success());
    assert!(!dir.0.join("routed.ses").exists());
    assert_eq!(std::fs::read(dir.0.join("board.dsn")).unwrap(), source);
}
