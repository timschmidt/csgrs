{
  lib,
  rust-bin,
  makeRustPlatform,
}:
let
  cargoToml = builtins.fromTOML (builtins.readFile ../Cargo.toml);

  toolchain = rust-bin.stable.${cargoToml.package.rust-version}.default;

  rustPlatform = makeRustPlatform {
    cargo = toolchain;
    rustc = toolchain;
  };
in
rustPlatform.buildRustPackage {
  pname = cargoToml.package.name;
  version = cargoToml.package.version;
  doCheck = false;

  src = lib.cleanSource ./..;

  cargoLock = {
    lockFile = ../Cargo.lock;
  };
}
