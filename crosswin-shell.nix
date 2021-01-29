#{ sources ? import ./nix/sources.nix }:

let 

  pkgs = import <nixpkgs> {
    overlays = [
      (import (builtins.fetchTarball "https://github.com/oxalica/rust-overlay/archive/master.tar.gz"))
    ];
    crossSystem = {config = "x86_64-w64-mingw32"; };
  };

  rchan = (pkgs.rustChannelOf {
    date = "2021-01-15";
    channel = "nightly";
  }).rust.override {
    targets = [ "x86_64-pc-windows-gnu" ];
  };

in 
    with pkgs; mkShell { 
        nativeBuildInputs = [ ];
        buildInputs = [ rchan windows.pthreads windows.mingw_w64_pthreads ]; 
    }
