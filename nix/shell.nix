{
  mkShell,
  csgrs,
  rust-analyzer,
}:
mkShell {
  inputsFrom = [ csgrs ];

  packages = [ rust-analyzer ];
}
