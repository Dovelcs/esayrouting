use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    collections::BTreeMap,
    fs,
    io::{Read, Write},
    path::{Path, PathBuf},
    process::{Command, Stdio},
};

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct NativeConfig {
    version: String,
    archive_sha256: String,
    archive_url: String,
    prefix: PathBuf,
    binary: PathBuf,
    destination: PathBuf,
    local_archive: Option<PathBuf>,
}
#[derive(Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct Installation {
    version: String,
    archive_sha256: String,
    files: BTreeMap<String, String>,
}
fn run(program: &str, args: &[&std::ffi::OsStr]) -> Result<(), String> {
    let status = Command::new(program)
        .args(args)
        .stdin(Stdio::null())
        .status()
        .map_err(|e| format!("{program}: {e}"))?;
    if status.success() {
        Ok(())
    } else {
        Err(format!("{program} failed: {status}"))
    }
}
fn digest(path: &Path) -> Result<String, String> {
    let mut f = fs::File::open(path).map_err(|e| e.to_string())?;
    let mut h = Sha256::new();
    let mut b = [0u8; 65536];
    loop {
        let n = f.read(&mut b).map_err(|e| e.to_string())?;
        if n == 0 {
            break;
        }
        h.update(&b[..n]);
    }
    Ok(format!("{:x}", h.finalize()))
}
fn config(path: &Path) -> Result<NativeConfig, String> {
    let mut c: NativeConfig = toml::from_str(&fs::read_to_string(path).map_err(|e| e.to_string())?)
        .map_err(|e| e.to_string())?;
    if c.version != "9.15.6755"
        || c.archive_sha256 != "de0400a45939a66ee13cd8360c230e830fc5e03a6ed5a8a8b60f58a39e4a67bc"
    {
        return Err("only the pinned official OR-Tools 9.15.6755 archive is supported".into());
    }
    let base =
        fs::canonicalize(path.parent().unwrap_or(Path::new("."))).map_err(|e| e.to_string())?;
    for p in [&mut c.prefix, &mut c.binary, &mut c.destination] {
        if !p.is_absolute() {
            *p = base.join(&*p);
        }
    }
    if let Some(p) = &mut c.local_archive {
        if !p.is_absolute() {
            *p = base.join(&*p);
        }
    }
    Ok(c)
}
fn version(prefix: &Path, expected: &str) -> Result<(), String> {
    let file = fs::read_to_string(prefix.join("lib/cmake/ortools/ortoolsConfigVersion.cmake"))
        .map_err(|e| e.to_string())?;
    if !file
        .lines()
        .any(|l| l.trim() == format!("set(PACKAGE_VERSION \"{expected}\")"))
    {
        return Err("native installed version does not match configuration".into());
    }
    Ok(())
}
fn runtime_files(prefix: &Path) -> Result<BTreeMap<String, String>, String> {
    let mut paths = vec![
        PathBuf::from("bin/protoc"),
        PathBuf::from("lib/cmake/ortools/ortoolsConfigVersion.cmake"),
    ];
    for entry in fs::read_dir(prefix.join("lib")).map_err(|e| e.to_string())? {
        let entry = entry.map_err(|e| e.to_string())?;
        if entry.path().extension().is_some_and(|e| e == "dylib") {
            paths.push(PathBuf::from("lib").join(entry.file_name()));
        }
    }
    let mut result = BTreeMap::new();
    for p in paths {
        result.insert(p.to_string_lossy().into_owned(), digest(&prefix.join(p))?);
    }
    Ok(result)
}
fn verify(c: &NativeConfig) -> Result<(), String> {
    version(&c.prefix, &c.version)?;
    let i: Installation = serde_json::from_slice(
        &fs::read(c.prefix.join("native_manifest.json")).map_err(|e| {
            format!("native setup receipt unavailable; run cargo agent-task setup: {e}")
        })?,
    )
    .map_err(|e| e.to_string())?;
    if i.version != c.version
        || i.archive_sha256 != c.archive_sha256
        || i.files != runtime_files(&c.prefix)?
    {
        return Err("native archive fingerprint or installed files differ; refusing reuse".into());
    }
    Ok(())
}
fn write_new(path: &Path, bytes: &[u8]) -> Result<(), String> {
    let mut f = fs::OpenOptions::new()
        .create_new(true)
        .write(true)
        .open(path)
        .map_err(|e| e.to_string())?;
    f.write_all(bytes)
        .and_then(|_| f.sync_all())
        .map_err(|e| e.to_string())
}
fn setup(c: &NativeConfig) -> Result<(), String> {
    if !cfg!(all(target_os = "macos", target_arch = "aarch64")) {
        return Err("pinned native setup supports macOS arm64 only".into());
    }
    if c.prefix.join("native_manifest.json").exists() {
        verify(c)?;
        println!("verified OR-Tools {}", c.version);
        return Ok(());
    }
    let stage = tempfile::tempdir().map_err(|e| e.to_string())?;
    let archive = if let Some(p) = &c.local_archive {
        p.clone()
    } else {
        let p = stage.path().join("ortools.tar.gz");
        run(
            "curl",
            &[
                "-fL".as_ref(),
                "--retry".as_ref(),
                "2".as_ref(),
                c.archive_url.as_ref(),
                "-o".as_ref(),
                p.as_os_str(),
            ],
        )?;
        p
    };
    if digest(&archive)? != c.archive_sha256 {
        return Err("official native archive SHA256 mismatch".into());
    }
    let extracted = stage.path().join("install");
    fs::create_dir(&extracted).map_err(|e| e.to_string())?;
    run(
        "tar",
        &[
            "-xzf".as_ref(),
            archive.as_os_str(),
            "-C".as_ref(),
            extracted.as_os_str(),
            "--strip-components=1".as_ref(),
        ],
    )?;
    version(&extracted, &c.version)?;
    let files = runtime_files(&extracted)?;
    if fs::symlink_metadata(&c.prefix).is_ok() {
        // Adopt a preexisting installation only after comparing every native runtime file.
        version(&c.prefix, &c.version)?;
        if runtime_files(&c.prefix)? != files {
            return Err(
                "existing native installation differs from the pinned archive; no files changed"
                    .into(),
            );
        }
    } else {
        fs::create_dir_all(c.prefix.parent().ok_or("native prefix has no parent")?)
            .map_err(|e| e.to_string())?;
        fs::create_dir(&c.prefix).map_err(|e| e.to_string())?;
        copy_tree(&extracted, &c.prefix)?;
    }
    let receipt = Installation {
        version: c.version.clone(),
        archive_sha256: c.archive_sha256.clone(),
        files,
    };
    write_new(
        &c.prefix.join("native_manifest.json"),
        &serde_json::to_vec_pretty(&receipt).map_err(|e| e.to_string())?,
    )?;
    println!("verified and installed OR-Tools {}", c.version);
    Ok(())
}
fn copy_tree(from: &Path, to: &Path) -> Result<(), String> {
    for entry in fs::read_dir(from).map_err(|e| e.to_string())? {
        let entry = entry.map_err(|e| e.to_string())?;
        let target = to.join(entry.file_name());
        let kind = entry.file_type().map_err(|e| e.to_string())?;
        if kind.is_symlink() {
            #[cfg(unix)]
            std::os::unix::fs::symlink(
                fs::read_link(entry.path()).map_err(|e| e.to_string())?,
                &target,
            )
            .map_err(|e| e.to_string())?;
            #[cfg(not(unix))]
            return Err("native package symbolic links require Unix".into());
        } else if kind.is_dir() {
            fs::create_dir(&target).map_err(|e| e.to_string())?;
            copy_tree(&entry.path(), &target)?;
        } else {
            let mut source = fs::File::open(entry.path()).map_err(|e| e.to_string())?;
            let mut out = fs::OpenOptions::new()
                .create_new(true)
                .write(true)
                .open(&target)
                .map_err(|e| e.to_string())?;
            std::io::copy(&mut source, &mut out).map_err(|e| e.to_string())?;
            fs::set_permissions(
                &target,
                entry.metadata().map_err(|e| e.to_string())?.permissions(),
            )
            .map_err(|e| e.to_string())?;
        }
    }
    Ok(())
}
fn package(c: &NativeConfig) -> Result<(), String> {
    verify(c)?;
    if fs::symlink_metadata(&c.destination).is_ok() {
        return Err(format!(
            "refusing to replace existing package {}",
            c.destination.display()
        ));
    }
    fs::create_dir_all(c.destination.parent().ok_or("package has no parent")?)
        .map_err(|e| e.to_string())?;
    let stage = tempfile::tempdir_in(c.destination.parent().unwrap()).map_err(|e| e.to_string())?;
    for dir in ["bin", "lib", "licenses"] {
        fs::create_dir(stage.path().join(dir)).map_err(|e| e.to_string())?;
    }
    let binary = stage.path().join("bin/fastroute_agent");
    fs::copy(&c.binary, &binary).map_err(|e| e.to_string())?;
    for e in fs::read_dir(c.prefix.join("lib")).map_err(|e| e.to_string())? {
        let e = e.map_err(|e| e.to_string())?;
        if e.path().extension().is_some_and(|v| v == "dylib") {
            let target = stage.path().join("lib").join(e.file_name());
            if e.file_type().map_err(|e| e.to_string())?.is_symlink() {
                #[cfg(unix)]
                std::os::unix::fs::symlink(
                    fs::read_link(e.path()).map_err(|e| e.to_string())?,
                    target,
                )
                .map_err(|e| e.to_string())?;
            } else {
                fs::copy(e.path(), target).map_err(|e| e.to_string())?;
            }
        }
    }
    let root = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .parent()
        .unwrap();
    fs::copy(
        root.join("LICENSE"),
        stage.path().join("licenses/fastroute_gpl_3.txt"),
    )
    .map_err(|e| e.to_string())?;
    fs::copy(
        c.prefix.join("share/doc/ortools/LICENSE"),
        stage.path().join("licenses/ortools_apache_2.txt"),
    )
    .map_err(|e| e.to_string())?;
    fs::create_dir(stage.path().join("licenses/native_dependencies")).map_err(|e| e.to_string())?;
    copy_tree(
        &c.prefix.join("share/licenses"),
        &stage.path().join("licenses/native_dependencies"),
    )?;
    copy_tree(
        &root.join("licenses/native"),
        &stage.path().join("licenses"),
    )?;
    for rpath in [
        "@executable_path/../../vendor/ortools/lib",
        "@executable_path/../../../vendor/ortools/lib",
    ] {
        run(
            "install_name_tool",
            &["-delete_rpath".as_ref(), rpath.as_ref(), binary.as_os_str()],
        )?;
    }
    run(
        "codesign",
        &[
            "--force".as_ref(),
            "--sign".as_ref(),
            "-".as_ref(),
            binary.as_os_str(),
        ],
    )?;
    let status = Command::new(&binary)
        .arg("--version")
        .env_remove("ORTOOLS_PREFIX")
        .env_remove("PROTOC")
        .env_remove("DYLD_LIBRARY_PATH")
        .status()
        .map_err(|e| e.to_string())?;
    if !status.success() {
        return Err("relocated package native loading failed".into());
    }
    // Claim the final directory exclusively, then publish only into our newly created directory.
    fs::create_dir(&c.destination).map_err(|e| e.to_string())?;
    for e in fs::read_dir(stage.path()).map_err(|e| e.to_string())? {
        let e = e.map_err(|e| e.to_string())?;
        fs::rename(e.path(), c.destination.join(e.file_name())).map_err(|e| e.to_string())?;
    }
    println!(
        "package: {}",
        c.destination.join("bin/fastroute_agent").display()
    );
    Ok(())
}
fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let outcome = (|| -> Result<(), String> {
        if args.is_empty() || args.len() > 2 {
            return Err("usage: cargo agent-task <setup|package> [native.toml]".into());
        }
        let path = Path::new(
            args.get(1)
                .map(String::as_str)
                .unwrap_or("examples/agent_tasks/native.toml"),
        );
        let c = config(path)?;
        match args[0].as_str() {
            "setup" => setup(&c),
            "package" => package(&c),
            _ => Err("unknown native task".into()),
        }
    })();
    if let Err(e) = outcome {
        eprintln!("agent-task: {e}");
        std::process::exit(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn installed() -> (tempfile::TempDir, NativeConfig) {
        let dir = tempfile::tempdir().unwrap();
        let prefix = dir.path().join("native");
        fs::create_dir_all(prefix.join("lib/cmake/ortools")).unwrap();
        fs::create_dir(prefix.join("bin")).unwrap();
        fs::write(prefix.join("bin/protoc"), b"compiler").unwrap();
        fs::write(prefix.join("lib/libortools.9.15.dylib"), b"native library").unwrap();
        fs::write(
            prefix.join("lib/cmake/ortools/ortoolsConfigVersion.cmake"),
            b"set(PACKAGE_VERSION \"9.15.6755\")\n",
        )
        .unwrap();
        let c = NativeConfig {
            version: "9.15.6755".into(),
            archive_sha256: "de0400a45939a66ee13cd8360c230e830fc5e03a6ed5a8a8b60f58a39e4a67bc"
                .into(),
            archive_url: "https://example.invalid".into(),
            prefix,
            binary: dir.path().join("binary"),
            destination: dir.path().join("package"),
            local_archive: None,
        };
        let i = Installation {
            version: c.version.clone(),
            archive_sha256: c.archive_sha256.clone(),
            files: runtime_files(&c.prefix).unwrap(),
        };
        write_new(
            &c.prefix.join("native_manifest.json"),
            &serde_json::to_vec(&i).unwrap(),
        )
        .unwrap();
        (dir, c)
    }
    #[test]
    fn verified_receipt_requires_current_runtime_bytes() {
        let (_dir, c) = installed();
        verify(&c).unwrap();
        fs::write(
            c.prefix.join("lib/libortools.9.15.dylib"),
            b"changed library",
        )
        .unwrap();
        assert!(verify(&c).unwrap_err().contains("fingerprint"));
    }
    #[test]
    fn existing_destination_is_not_modified() {
        let (_dir, c) = installed();
        fs::create_dir(&c.destination).unwrap();
        fs::write(c.destination.join("user_file"), b"user data").unwrap();
        assert!(package(&c).unwrap_err().contains("refusing to replace"));
        assert_eq!(
            fs::read(c.destination.join("user_file")).unwrap(),
            b"user data"
        );
    }
    #[test]
    fn copy_and_receipt_never_overwrite_existing_files() {
        let dir = tempfile::tempdir().unwrap();
        let from = dir.path().join("from");
        let to = dir.path().join("to");
        fs::create_dir(&from).unwrap();
        fs::create_dir(&to).unwrap();
        fs::write(from.join("file"), b"replacement").unwrap();
        fs::write(to.join("file"), b"user data").unwrap();
        assert!(copy_tree(&from, &to).is_err());
        assert!(write_new(&to.join("file"), b"replacement").is_err());
        assert_eq!(fs::read(to.join("file")).unwrap(), b"user data");
    }
}
