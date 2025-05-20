{
  description = "run + build + develop flake for sidestepper";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    naersk.url = "github:nix-community/naersk";
  };

  outputs = { self, nixpkgs, flake-utils, naersk }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        inherit (nixpkgs) lib;
        pkgs = (import nixpkgs) {
          inherit system;
        };
        naersk' = pkgs.callPackage naersk {};
      in
      {
        defaultPackage = naersk'.buildPackage {
          src = ./.;
        };

        devShells.default = pkgs.mkShell {
          nativeBuildInputs = with pkgs; [ rustup rustc cargo cargo-zigbuild zig ];
        };
      }
    );
}
