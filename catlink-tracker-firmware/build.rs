fn main() {
    // Specify linker arguments.

    // `--nmagic` is required if memory section addresses are not aligned to 0x10000,
    // for example the FLASH and RAM sections in your `memory.x`.
    // See https://github.com/rust-embedded/cortex-m-quickstart/pull/95
    println!("cargo:rustc-link-arg=--nmagic");

    // Use this project's custom linker script
    // This custom linker script has 2 differences compared to the default one provided
    // by cortex-m-rt:
    // - .vector_table is in SRAM1
    // - .rodata is in SRAM1
    println!("cargo:rustc-link-arg=-Tcustom_link.x");
}
