{
  description = "embassy flake";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    systems.url = "github:nix-systems/default";
    flake-parts.url = "github:hercules-ci/flake-parts";
    fenix.url = "github:nix-community/fenix/monthly";
    naersk = {
      url = "github:nix-community/naersk";
      inputs.fenix.follows = "fenix";
    };
  };

  outputs = inputs@{ flake-parts, ... }:
    flake-parts.lib.mkFlake { inherit inputs; } ({ ... }: {
      systems = import inputs.systems;
      perSystem = { pkgs, system, inputs', ... }: 
      let
        probe-rs-tools = pkgs.probe-rs-tools.overrideAttrs {
          cargoBuildFeatures = [ "remote" ];
        };
        fpkgs = inputs'.fenix.packages;
        profile = fpkgs.complete;
        std-lib = fpkgs.targets.thumbv6m-none-eabi.latest;
        rust-analyzer-nightly = fpkgs.rust-analyzer;
        rust-toolchain = fpkgs.combine [
          profile.rustc-unwrapped
          profile.rust-src
          profile.cargo
          profile.rustfmt
          profile.clippy
          profile.llvm-tools
          std-lib.rust-std
        ];
      in {
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
        (inputs.naersk.lib.${system}.override {
          cargo = rust-toolchain;
          rustc = rust-toolchain;
        }).buildPackage {
          src = ./.;
          FW_VERSION = builtins.getEnv "FW_VERSION";
          FW_HASH    = builtins.getEnv "FW_HASH";

          DEFMT_LOG = "info";
        };
      };
    });
}

