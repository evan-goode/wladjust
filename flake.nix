{
  inputs = {
    naersk.url = "github:nix-community/naersk/master";
    nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";
    utils.url = "github:numtide/flake-utils";
  };

  outputs =
    {
      self,
      nixpkgs,
      utils,
      naersk,
    }:
    utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = import nixpkgs { inherit system; };
        naersk-lib = pkgs.callPackage naersk { };
        extraLibs = with pkgs; [
          pkg-config
          wayland
          libxkbcommon
          libnotify
        ];
      in
      {
        defaultPackage = naersk-lib.buildPackage {
          src = ./.;
          buildInputs = extraLibs;
        };
        devShell =
          with pkgs;
          mkShell {
            buildInputs = [
              cargo
              rustc
              rustfmt
              pre-commit
              rustPackages.clippy
              rust-analyzer
            ]
            ++ extraLibs;
            RUST_SRC_PATH = rustPlatform.rustLibSrc;
          };
      }
    );
}
