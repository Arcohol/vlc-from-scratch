{
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";

    # Used by zephyr-nix to make a specific python environment for zephyr.
    zephyr.url = "github:zephyrproject-rtos/zephyr/v4.4.0";
    zephyr.flake = false;

    zephyr-nix.url = "github:nix-community/zephyr-nix";
    zephyr-nix.inputs.nixpkgs.follows = "nixpkgs";
    zephyr-nix.inputs.zephyr.follows = "zephyr";
  };

  outputs =
    {
      nixpkgs,
      flake-utils,
      zephyr-nix,
      ...
    }:
    flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = import nixpkgs {
          inherit system;
          config = {
            allowUnfree = true;
            segger-jlink.acceptLicense = true;
            permittedInsecurePackages = [
              "segger-jlink-qt4-874"
            ];
          };
        };
        zephyr = zephyr-nix.packages.${system};
        zephyr-sdk = zephyr.sdk-1_0.override { targets = [ "arm-zephyr-eabi" ]; };
        zephyr-pythonEnv = zephyr.pythonEnv;
      in
      {
        devShells.default = pkgs.mkShell {
          buildInputs = with pkgs; [
            dtc
            cmake
            ninja
            clang-tools
            segger-jlink
            (nrfutil.withExtensions [ "nrfutil-device" ])
            zephyr-sdk
            zephyr-pythonEnv
          ];

          shellHook = ''
            export ZEPHYR_TOOLCHAIN_VARIANT=zephyr
            export ZEPHYR_SDK_INSTALL_DIR=${zephyr-sdk}
          '';
        };
      }
    );
}
