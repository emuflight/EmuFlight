extern crate cbindgen;

use std::env;
use std::fs;
use std::path::{Path, PathBuf};

fn project_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).to_path_buf()
}

fn main() {
    let out_dir = project_root()
        .join("include");

    fs::create_dir_all(&out_dir).unwrap();

    let mut cbindgen_config = cbindgen::Config::default();
    cbindgen_config.language = cbindgen::Language::C;

    cbindgen::Builder::new()
        .with_crate(project_root())
        .with_config(cbindgen_config)
        .generate()
        .expect("Unable to generate bindings")
        .write_to_file(out_dir.join("rustemu.h"));
}
