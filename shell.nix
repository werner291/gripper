let
  moz_overlay = import (builtins.fetchTarball https://github.com/mozilla/nixpkgs-mozilla/archive/master.tar.gz);
  nixpkgs = import <nixpkgs> { overlays = [ moz_overlay ]; };
  rust = (nixpkgs.rustChannelOf { date = "2020-08-18"; channel = "nightly"; }).rust.override {
    extensions = [ "rust-src" "rust-analysis" "rls-preview" ];
    };
in
  with nixpkgs;
  stdenv.mkDerivation {
    name = "rust";
    
    nativeBuildInputs = [
        pkgconfig
        python3
    ];

    buildInputs = [
        rust glib gtk3 libGL cmake
    ];

    LD_LIBRARY_PATH = with pkgs.xlibs; "${pkgs.libGL}/lib";

  }
