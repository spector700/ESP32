{
  # Flake inputs
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";
  };

  # Flake outputs
  outputs =
    { self, nixpkgs }:
    let
      # Systems supported
      allSystems = [
        "x86_64-linux" # 64-bit Intel/AMD Linux
        "aarch64-linux" # 64-bit ARM Linux
        "x86_64-darwin" # 64-bit Intel macOS
        "aarch64-darwin" # 64-bit ARM macOS
      ];

      # Helper to provide system-specific attributes
      forAllSystems =
        f: nixpkgs.lib.genAttrs allSystems (system: f { pkgs = import nixpkgs { inherit system; }; });
    in
    {
      # Development environment output
      devShells = forAllSystems (
        { pkgs }:
        {
          default = pkgs.mkShell {
            # The Nix packages provided in the environment
            packages = with pkgs; [
              # for pio cli and vscode extension
              platformio-core

              # for running openocd manually
              openocd

              # formatting and stuff
              clang-tools
            ];

            # clear LD_LIBRARY_PATH (NixOS/nixpkgs#263201, NixOS/nixpkgs#262775, NixOS/nixpkgs#262080)
            # then add path for platformio debugging in vscode
            # (ldd ~/.platformio/packages/toolchain-rp2040-earlephilhower/bin/arm-none-eabi-gdb)
            runScript = "env LD_LIBRARY_PATH=${pkgs.lib.makeLibraryPath [ pkgs.ncurses5 ]} zsh";

            shellHook = ''
              echo "welcome to ESP32" | ${pkgs.lolcat}/bin/lolcat
              export PLATFORMIO_CORE_DIR=$PWD/.platformio
            '';
          };
        }
      );
    };
}
