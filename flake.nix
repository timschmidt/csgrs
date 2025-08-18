{
  description = "My configuration";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs";
    rust-overlay = {
      url = "github:oxalica/rust-overlay";
      inputs.nixpkgs.follows = "nixpkgs";
    };

    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs =
    {
      self,
      nixpkgs,
      rust-overlay,
      flake-utils,
      ...
    }:
    {
      overlays.default = import ./nix/overlay.nix;
    }
    // (flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [
            (import rust-overlay)
            self.overlays.default
          ];
        };
      in
      {
        formatter = pkgs.nixfmt-rfc-style;

        packages.csgrs = pkgs.csgrs;
        packages.default = self.packages.${system}.csgrs;

        devShells.default = pkgs.callPackage ./nix/shell.nix { };
      }
    ));
}
