{ nixpkgsSrc ? builtins.fetchTarball {
  url =
    "https://github.com/NixOS/nixpkgs/archive/e675946ecde5606c505540de2024e2732bae4185.tar.gz"; # nixos-unstable
  sha256 = "1xnqhz0wxkgkwpwkal93k5rj72j39pvck542i9jyxh9bm25rc4j5";
}, pkgs ? import nixpkgsSrc { }, compiler ? null }:

let
  haskellPackages = if compiler == null then
    pkgs.haskellPackages
  else
    pkgs.haskell.packages.${compiler};

in haskellPackages.developPackage {
  name = "";
  root = pkgs.nix-gitignore.gitignoreSource [ ] ./.;
  overrides = _self: _super: { };
}
