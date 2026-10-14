fn main() -> Result<(), Box<dyn std::error::Error>> {
    let a: Vec<String> = std::env::args().collect();
    if a.len() != 4 {
        return Err("usage: normalize_board input.dsn output.dsn mapping.json".into());
    }
    let n = fr_agent::dsn_normalize::normalize(&std::fs::read(&a[1])?)?;
    fr_agent::dsn_normalize::write_normalized(
        &n,
        std::path::Path::new(&a[2]),
        std::path::Path::new(&a[3]),
    )?;
    println!("{}", serde_json::to_string(&n.evidence)?);
    Ok(())
}
