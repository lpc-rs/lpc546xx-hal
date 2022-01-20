use std::env;
use std::fs::{self, File};
use std::io::Write;
use std::path::PathBuf;

fn main() {
    // Put the linker script somewhere the linker can find it
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());

    let mut feature_count = 0;

    if cfg!(feature = "flash-256") {
        feature_count += 1;
    }

    if cfg!(feature = "flash-512") {
        feature_count += 1;
    }

    if cfg!(feature = "ram-136") {
        feature_count += 1;
    }

    if cfg!(feature = "ram-200") {
        feature_count += 1;
    }

    if !cfg!(feature = "disable-linker-script") {
        if feature_count != 2 {
            panic!("\n\nMust select exactly two packages (one flash, one ram) for linker script generation!\nChoices: 'ram-136'/'ram-200' and 'flash-256'/'flash-512'\nAlternatively, pick the mcu-feature that matches your MCU, for example 'mcu-LPC54608J512ET180 '\n\n");
        }

        let flash_features: Vec<u32> = [
            (256, cfg!(feature = "flash-256")),
            (512, cfg!(feature = "flash-512")),
        ]
        .iter()
        .filter(|(_, f)| *f)
        .map(|(f, _)| *f)
        .collect();

        if flash_features.len() != 1 {
            panic!("\n\nMust select exactly one flash size for linker script generation!\n\
            Choices: 'flash-256' or 'flash-512'\n \
            Alternatively, pick the mcu-feature that matches your MCU, for example 'mcu-LPC54608J512ET180'\n\n");
        }

        let flash_size = flash_features[0];

        let ram_features: Vec<u32> = [
            (96, cfg!(feature = "ram-136")),
            (160, cfg!(feature = "ram-200")),
        ]
        .iter()
        .filter(|(_, f)| *f)
        .map(|(f, _)| *f)
        .collect();

        if ram_features.len() != 1 {
            panic!("\n\nMust select exactly one ram size for linker script generation!\n\
            Choices: 'ram-136' or 'ram-200'\n \
            Alternatively, pick the mcu-feature that matches your MCU, for example 'mcu-LPC54608J512ET180'\n\n");
        }

        let ram_size = ram_features[0];

        let linker = format!(
            r#"MEMORY
{{
    FLASH : ORIGIN = 0x0, LENGTH = {}K          /* PROGRAM_FLASH */
    RAM : ORIGIN = 0x20000000, LENGTH = {}K     /* alias SRAM_UPPER alias RAM */
    SRAMX : ORIGIN = 0x4000000, LENGTH = 32K    /* alias RAM2 */
    USB_RAM : ORIGIN = 0x40100000, LENGTH = 8K  /* alias RAM3 */


}}"#,
            flash_size, ram_size
        );

        File::create(out.join("memory.x"))
            .unwrap()
            .write_all(linker.as_bytes())
            .unwrap();
        println!("cargo:rustc-link-search={}", out.display());
    }

    println!("cargo:rerun-if-changed=build.rs");

    // Copy Binary blob required for SDRAM enabling when cortex-m-rt calls __pre_init
    // somewhere the linker can find it, and tell cargo to link it
    if !cfg!(feature = "disable-pre-init-blob") {
        let blob_name = "startup";
        let blob_file = format!("lib{}.a", blob_name);
        let blob_path = format!("startup-code/{}", blob_file);
        fs::copy(&blob_path, out.join(blob_file))
            .expect("Failed to copy binary blob for startup (ram enable)");
        println!("cargo:rustc-link-lib=static={}", blob_name);
        println!("cargo:rustc-link-search={}", out.display());
    }

    println!("cargo:rerun-if-changed=build.rs");
}
