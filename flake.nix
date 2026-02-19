{
  description = "embassy flake";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    fenix = {
      url = "github:nix-community/fenix/monthly";
      inputs.nixpkgs.follows = "nixpkgs";
    };
    naersk = {
      url = "github:nix-community/naersk";
      inputs.nixpkgs.follows = "nixpkgs";
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
            fenix.overlays.default 
            probe-rs-overlay 
          ];
        };
        profile = pkgs.fenix.complete;
        rust-analyzer = pkgs.fenix.rust-analyzer;
        std-lib = pkgs.fenix.targets.thumbv6m-none-eabi.latest;
        rust-toolchain = pkgs.fenix.combine [
          profile.rustc-unwrapped
          profile.rust-src
          profile.cargo
          profile.rustfmt
          profile.clippy
          std-lib.rust-std
        ];
      in
      {
        devShells.default =
        pkgs.mkShell {
          buildInputs = with pkgs; [
            rust-toolchain
            rust-analyzer

            # extra cargo tools
            cargo-edit
            cargo-expand

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
