fn main() {
    if let Ok(v) = std::env::var("FW_VERSION") {
        println!("cargo:rustc-env=FW_VERSION={}", v);
    } else {
        println!(
            "cargo:rustc-env=FW_VERSION=v{}",
            env!("CARGO_PKG_VERSION")
        );
    }

    if let Ok(h) = std::env::var("FW_HASH") {
        println!("cargo:rustc-env=FW_HASH={}", h);
    } else {
        println!("cargo:rustc-env=FW_HASH=local");
    }
}
