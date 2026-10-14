use std::env;
use std::path::PathBuf;

// Proto files vendored from OR-Tools v9.15.
// If you have a different version of OR-Tools installed, the protos may not
// match. OR-Tools maintains backward compatibility within the same major
// version, but new fields in newer versions will be ignored.
const ORTOOLS_PROTO_VERSION: &str = "9.15";

fn main() {
    // Emit the proto version as a cfg for downstream use
    println!("cargo:rustc-env=CPSAT_RS_ORTOOLS_PROTO_VERSION={ORTOOLS_PROTO_VERSION}");

    // 1. Compile protos with prost
    //
    // Requires `protoc` on PATH (install via `brew install protobuf` on macOS,
    // `apt install protobuf-compiler` on Linux, or set PROTOC env var).
    prost_build::compile_protos(
        &["proto/cp_model.proto", "proto/sat_parameters.proto"],
        &["proto/"],
    )
    .expect(
        "proto compilation failed. Ensure `protoc` is installed: \
         `brew install protobuf` (macOS) or `apt install protobuf-compiler` (Linux), \
         or set the PROTOC env var to the protoc binary path.",
    );

    // 2. Find OR-Tools
    let ortools_prefix = env::var("ORTOOLS_PREFIX")
        .or_else(|_| env::var("ORTOOL_PREFIX"))
        .unwrap_or_else(|_| {
            let candidates = [
                // macOS Homebrew (Apple Silicon + Intel)
                "/opt/homebrew/opt/or-tools",
                "/usr/local/opt/or-tools",
                // Linux system install
                "/usr",
                "/usr/local",
                // Custom install
                "/opt/ortools",
            ];
            for path in &candidates {
                if PathBuf::from(path).join("include/ortools").exists() {
                    return path.to_string();
                }
            }
            panic!(
                "OR-Tools not found. Install with:\n  \
                 macOS:  brew install or-tools\n  \
                 Linux:  see https://developers.google.com/optimization/install\n  \
                 Or set ORTOOLS_PREFIX env var to your OR-Tools installation prefix."
            );
        });

    let ortools_lib = format!("{ortools_prefix}/lib");

    // 3. Link OR-Tools (uses the C API directly, no C++ shim needed)
    println!("cargo:rustc-link-search=native={ortools_lib}");
    println!("cargo:rustc-link-lib=dylib=ortools");

    // Link C++ stdlib (platform-specific)
    let target_os = env::var("CARGO_CFG_TARGET_OS").unwrap_or_default();
    match target_os.as_str() {
        "macos" => println!("cargo:rustc-link-lib=dylib=c++"),
        "windows" => {} // MSVC links C++ runtime automatically
        _ => println!("cargo:rustc-link-lib=dylib=stdc++"), // Linux and others
    }

    // Link protobuf if available separately (needed on some platforms)
    let protobuf_lib = format!("{ortools_prefix}/lib");
    if PathBuf::from(&protobuf_lib).join("libprotobuf.dylib").exists()
        || PathBuf::from(&protobuf_lib).join("libprotobuf.so").exists()
    {
        println!("cargo:rustc-link-lib=dylib=protobuf");
    }
    // Also check Homebrew protobuf (separate package on macOS)
    if PathBuf::from("/opt/homebrew/opt/protobuf/lib").exists() {
        println!("cargo:rustc-link-search=native=/opt/homebrew/opt/protobuf/lib");
        println!("cargo:rustc-link-lib=dylib=protobuf");
    }

    // Rerun triggers
    println!("cargo:rerun-if-changed=proto/cp_model.proto");
    println!("cargo:rerun-if-changed=proto/sat_parameters.proto");
    println!("cargo:rerun-if-env-changed=ORTOOLS_PREFIX");
    println!("cargo:rerun-if-env-changed=ORTOOL_PREFIX");
    println!("cargo:rerun-if-env-changed=PROTOC");
}
