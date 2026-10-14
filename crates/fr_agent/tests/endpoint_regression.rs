use std::{
    fs,
    path::PathBuf,
    time::{SystemTime, UNIX_EPOCH},
};

fn temp_path(label: &str) -> PathBuf {
    let nonce = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    std::env::temp_dir().join(format!(
        "fr_agent_endpoint_{label}_{}_{}",
        std::process::id(),
        nonce
    ))
}

#[test]
fn endpoint_rejects_output_aliasing_input_without_touching_source() {
    let input = temp_path("input.dsn");
    let log = temp_path("events.jsonl");
    let config = temp_path("job.toml");
    let source = include_bytes!("fixtures/two_chip.dsn");
    fs::write(&input, source).unwrap();
    let toml = format!(
        "input = {:?}\noutput = {:?}\nlog = {:?}\nbudget_ms = 10\ngrid_step_mm = 0.1\nmax_iterations = 10\nthreads = 1\nvia_drill_mm = 0.2\nmin_annular_ring_mm = 0.1\nhole_to_hole_clearance_mm = 0.2\n",
        input.to_string_lossy(), input.to_string_lossy(), log.to_string_lossy()
    );
    fs::write(&config, toml).unwrap();

    let error = fr_agent::awx_endpoint::run(&config).unwrap_err();
    assert!(error.contains("must be different files"));
    assert_eq!(fs::read(&input).unwrap(), source);

    let _ = fs::remove_file(input);
    let _ = fs::remove_file(log);
    let _ = fs::remove_file(config);
}

#[test]
fn endpoint_fails_closed_for_unknown_pth_hidden_by_power_layer() {
    let input = temp_path("power_pth.dsn");
    let output = temp_path("power_pth.ses");
    let log = temp_path("power_pth.jsonl");
    let config = temp_path("power_pth.toml");
    let source = std::str::from_utf8(include_bytes!("fixtures/two_chip.dsn")).unwrap();
    let source = source
        .replace(
            "(layer Bottom (type signal))",
            "(layer Plane (type power)) (layer Bottom (type signal))",
        )
        .replace(
            "(padstack PAD (shape (rect Top -0.2 -0.2 0.2 0.2)))",
            "(padstack PAD (shape (rect Top -0.2 -0.2 0.2 0.2)) (shape (rect Plane -0.2 -0.2 0.2 0.2)))",
        );
    fs::write(&input, &source).unwrap();
    fs::write(
        &config,
        format!(
            "input = {:?}\noutput = {:?}\nlog = {:?}\nbudget_ms = 10\ngrid_step_mm = 0.1\nmax_iterations = 10\nthreads = 1\nvia_drill_mm = 0.2\nmin_annular_ring_mm = 0.1\nhole_to_hole_clearance_mm = 0.2\n",
            input.to_string_lossy(),
            output.to_string_lossy(),
            log.to_string_lossy()
        ),
    )
    .unwrap();

    let error = fr_agent::awx_endpoint::run(&config).unwrap_err();
    assert!(error.contains("explicit PTH drill metadata"));
    assert!(!output.exists());

    let _ = fs::remove_file(input);
    let _ = fs::remove_file(output);
    let _ = fs::remove_file(log);
    let _ = fs::remove_file(config);
}

#[test]
fn endpoint_replays_inherited_dsn_traces_and_writes_session() {
    let input = temp_path("inherited_trace.dsn");
    let output = temp_path("inherited_trace.ses");
    let log = temp_path("inherited_trace.jsonl");
    let config = temp_path("inherited_trace.toml");
    let source = std::str::from_utf8(include_bytes!("fixtures/two_chip.dsn")).unwrap();
    let source = source.replace(
        "\n)\n",
        "\n (wiring (wire (path Top 0.15 2 4 18 4) (net BUS) (type protect)) (wire (path Top 0.15 2 5 18 5) (net OTHER) (type protect)))\n)\n",
    );
    fs::write(&input, &source).unwrap();
    fs::write(
        &config,
        format!(
            "input = {:?}\noutput = {:?}\nlog = {:?}\nbudget_ms = 10\ngrid_step_mm = 0.1\nmax_iterations = 10\nthreads = 1\nvia_drill_mm = 0.2\nmin_annular_ring_mm = 0.1\nhole_to_hole_clearance_mm = 0.2\n",
            input.to_string_lossy(),
            output.to_string_lossy(),
            log.to_string_lossy()
        ),
    )
    .unwrap();

    fr_agent::awx_endpoint::run(&config).unwrap();
    assert!(fs::metadata(&output).unwrap().len() > 0);

    let _ = fs::remove_file(input);
    let _ = fs::remove_file(output);
    let _ = fs::remove_file(log);
    let _ = fs::remove_file(config);
}
