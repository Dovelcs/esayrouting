use fr_agent::stack_prepare;
use std::{fs, path::Path};
use tempfile::tempdir;

fn write_job(dir: &Path, input: &Path, output_dir: &Path) -> std::path::PathBuf {
    let job = dir.join("job.toml");
    fs::write(
        &job,
        format!(
            "input = {:?}\noutput_dir = {:?}\nbudget_ms = 1\ngrid_step_mm = 0.1\nmax_iterations = 1\nvia_drill_mm = 0.3\nmin_annular_ring_mm = 0.1\nhole_to_hole_clearance_mm = 0.2\n",
            input, output_dir
        ),
    )
    .unwrap();
    job
}

fn assert_rejected_without_mutating_input(job: &Path, input: &Path, original: &[u8]) {
    let error = stack_prepare::run(job).expect_err("aliased output must be rejected");
    assert!(error.contains("aliases original input"), "{error}");
    assert_eq!(fs::read(input).unwrap(), original);
}

#[test]
fn rejects_derived_dsn_overwriting_input_in_same_directory() {
    let dir = tempdir().unwrap();
    let input = dir.path().join("board_2.dsn");
    let original = b"original source DSN";
    fs::write(&input, original).unwrap();
    let job = write_job(dir.path(), &input, dir.path());

    assert_rejected_without_mutating_input(&job, &input, original);
}

#[cfg(unix)]
#[test]
fn rejects_hard_link_output_alias_before_any_write() {
    let dir = tempdir().unwrap();
    let input = dir.path().join("source.dsn");
    let original = b"original source DSN";
    fs::write(&input, original).unwrap();
    fs::hard_link(&input, dir.path().join("board_2.dsn")).unwrap();
    let job = write_job(dir.path(), &input, dir.path());

    assert_rejected_without_mutating_input(&job, &input, original);
}

#[cfg(unix)]
#[test]
fn rejects_symlink_output_alias_before_any_write() {
    let dir = tempdir().unwrap();
    let input = dir.path().join("source.dsn");
    let original = b"original source DSN";
    fs::write(&input, original).unwrap();
    std::os::unix::fs::symlink(&input, dir.path().join("cross_layer_6.dsn")).unwrap();
    let job = write_job(dir.path(), &input, dir.path());

    assert_rejected_without_mutating_input(&job, &input, original);
}
