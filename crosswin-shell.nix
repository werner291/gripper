#{ sources ? import ./nix/sources.nix }:

let 

  pkgs = import <nixpkgs> {
    overlays = [
      (import (builtins.fetchTarball "https://github.com/oxalica/rust-overlay/archive/master.tar.gz"))
    ];
  };

  rchan = (pkgs.rustChannelOf {
    date = "2021-01-15";
    channel = "nightly";
  }).rust.override {
    targets = [ "x86_64-pc-windows-gnu" ];
    extensions = [
      "rust-src"
      "rustc-dev"
    ];
  };
  
  pkgs-mingw = import <nixpkgs> { crossSystem = {config = "x86_64-w64-mingw32"; }; };

in 
    pkgs-mingw.mkShell { 
        nativeBuildInputs = [ rchan ];
        buildInputs = [ rchan pkgs-mingw.windows.pthreads pkgs-mingw.windows.mingw_w64_pthreads ]; 
        
    }
