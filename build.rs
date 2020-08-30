use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    // Put the linker script somewhere the linker can find it
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();

    cc::Build::new()
        .warnings(true)
        .flag("-Wall")
        .flag("-Wextra")
        .file("CMSIS_5/CMSIS/DSP/Source/FastMathFunctions/arm_sin_f32.c")
        .file("CMSIS_5/CMSIS/DSP/Source/FastMathFunctions/arm_cos_f32.c")
        .file("CMSIS_5/CMSIS/DSP/Source/CommonTables/arm_common_tables.c")
        .include("CMSIS_5/CMSIS/DSP/Include/")
        .include("CMSIS_5/CMSIS/Core/Include/")
        .compile("libcmsis.a");

    let bindings = bindgen::Builder::default()
        .clang_arg("-target")
        .clang_arg("thumbv7em-none-eabihf")
        .use_core()
        .ctypes_prefix("cty")
        .header("wrapper.h")
        .generate()
        .expect("Failed to generate bindings");

    bindings
        .write_to_file("src/bindings.rs")
        .expect("Failed to write bindings");

    println!("cargo:rustc-link-search={}", out.display());

    // Only re-run the build script when memory.x is changed,
    // instead of when any part of the source code changes.
    println!("cargo:rerun-if-changed=memory.x");
}
