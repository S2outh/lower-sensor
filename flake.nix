
{
  description = "embassy flake";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    fenix.url = "github:nix-community/fenix/monthly";
    naersk = {
      url = "github:nix-community/naersk";
      inputs.fenix.follows = "fenix";
    };
  };

  outputs = { self, nixpkgs, flake-utils, fenix, naersk }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        probe-rs-overlay = (final: prev: {
          probe-rs-tools = prev.probe-rs-tools.overrideAttrs {
            cargoBuildFeatures = [ "remote" ];
          };
        }); 
        pkgs = import nixpkgs {
          inherit system;
          overlays = [
            probe-rs-overlay 
          ];
        };
        fpkgs = fenix.packages.${system};
        profile = fpkgs.complete;
        std-lib = fpkgs.targets.thumbv6m-none-eabi.latest;
        rust-analyzer-nightly = fpkgs.rust-analyzer;
        rust-toolchain = fpkgs.combine [
          profile.rustc
          profile.rust-src
          profile.cargo
          profile.rustfmt
          profile.clippy
          profile.llvm-tools
          std-lib.rust-std
        ];
      in
      {
        devShells.default =
        pkgs.mkShell {
          buildInputs = with pkgs; [
            rust-toolchain
            rust-analyzer-nightly

            # extra cargo tools
            cargo-edit
            cargo-expand
            cargo-show-asm
            cargo-binutils

            # for flashing
            probe-rs-tools
          ];

          # set the rust src for rust_analyzer
          RUST_SRC_PATH = "${rust-toolchain}/lib/rustlib/src/rust/library";
          # set default defmt log level
          DEFMT_LOG = "info";
        };

        packages.default = 
        (naersk.lib.${system}.override {
          cargo = rust-toolchain;
          rustc = rust-toolchain;
        }).buildPackage {
          src = ./.;
          FW_VERSION = builtins.getEnv "FW_VERSION";
          FW_HASH    = builtins.getEnv "FW_HASH";

          DEFMT_LOG = "info";
        };
      }
    );
}

