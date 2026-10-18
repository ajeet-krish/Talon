//! Build script for the Python extension module.
//!
//! PyO3 expects the consuming crate to add the platform link arguments for an
//! extension module; on macOS that is `-undefined dynamic_lookup`, so the
//! `cdylib` resolves the CPython symbols from the interpreter that loads it
//! instead of linking `libpython` at build time.

fn main() {
    pyo3_build_config::add_extension_module_link_args();
}
