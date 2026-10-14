// Official OR-Tools 9.15.6755 macOS arm64 archive SHA256:
// de0400a45939a66ee13cd8360c230e830fc5e03a6ed5a8a8b60f58a39e4a67bc
fn main() {
    println!("cargo:rustc-link-lib=dylib=highs");
    println!("cargo:rerun-if-changed=src/awx/cp_sat_bridge.cc");
    println!("cargo:rerun-if-changed=src/awx/highs_bridge.cc");
    let prefix =
        std::env::var("ORTOOLS_PREFIX").expect("ORTOOLS_PREFIX must point to official OR-Tools");
    let ortools_lib = std::path::Path::new(&prefix).join("lib");
    println!("cargo:rustc-link-search=native={}", ortools_lib.display());
    // cp_sat_bridge.cc includes OR-Tools' CP-SAT headers directly, so its
    // instantiated Abseil container/logging calls need the same direct
    // shared-library edges exported by the official OR-Tools CMake target.
    // libortools depends on these dylibs but does not re-export their symbols.
    println!("cargo:rustc-link-lib=dylib=absl_raw_hash_set.2508.0.0");
    println!("cargo:rustc-link-lib=dylib=absl_raw_logging_internal.2508.0.0");
    println!(
        "cargo:rerun-if-changed={}",
        ortools_lib
            .join("libabsl_raw_hash_set.2508.0.0.dylib")
            .display()
    );
    println!(
        "cargo:rerun-if-changed={}",
        ortools_lib
            .join("libabsl_raw_logging_internal.2508.0.0.dylib")
            .display()
    );
    cc::Build::new()
        .cpp(true)
        .std("c++17")
        .define("OR_PROTO_DLL", "")
        .include(std::path::Path::new(&prefix).join("include"))
        .include(std::path::Path::new(&prefix).join("include/highs"))
        .file("src/awx/cp_sat_bridge.cc")
        .file("src/awx/highs_bridge.cc")
        .compile("awx_cp_sat_bridge");
    println!("cargo:rerun-if-env-changed=ORTOOLS_PREFIX");
    if cfg!(target_os = "macos") {
        // Installed layout: bin/fastroute_agent and lib/*.dylib.
        println!("cargo:rustc-link-arg=-Wl,-rpath,@executable_path/../lib");
        // Cargo layout: target/{debug,release}/fastroute_agent.
        println!("cargo:rustc-link-arg=-Wl,-rpath,@executable_path/../../vendor/ortools/lib");
        // Cargo integration tests and examples live one directory deeper.
        println!("cargo:rustc-link-arg=-Wl,-rpath,@executable_path/../../../vendor/ortools/lib");
    }
}
