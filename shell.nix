{ pkgs }:
let
  inherit (pkgs) stdenv lib;
in
pkgs.mkShell {
  nativeBuildInputs = with pkgs; [
    cmake
  ];

  buildInputs = with pkgs; [
    eigen
    suitesparse
    blas
    lapack

    python310
    python310Packages.pip
  ] ++ (lib.optionals stdenv.isDarwin (with pkgs.darwin.apple_sdk.frameworks; [
    OpenGL
  ]));
}
