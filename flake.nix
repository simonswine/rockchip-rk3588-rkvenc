{
  description = "Native ARM64 kernel module builds for the RK3588 encoder";

  inputs.nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";

  outputs = { self, nixpkgs }:
    let
      system = "aarch64-linux";
      pkgs = import nixpkgs { inherit system; };
      # Only include module sources, never local build outputs or test videos.
      src = pkgs.lib.fileset.toSource {
        root = ./.;
        fileset = pkgs.lib.fileset.unions [
          ./Makefile
          ./Kbuild
          (pkgs.lib.fileset.fileFilter
            (file: (file.hasExt "c" && !(pkgs.lib.hasSuffix ".mod.c" file.name)) || file.hasExt "h") ./.)
        ];
      };
      buildModule = kernel: pkgs.stdenv.mkDerivation {
        pname = "rkvenc";
        version = "0.1-${kernel.modDirVersion}";
        inherit src;
        nativeBuildInputs = kernel.moduleBuildDependencies ++ [ pkgs.python3 ];
        hardeningDisable = [ "pic" ];
        makeFlags = [
          "KDIR=${kernel.dev}/lib/modules/${kernel.modDirVersion}/build"
          "W=1"
        ];
        enableParallelBuilding = true;
        installPhase = ''
          runHook preInstall
          install -Dm644 rkvenc.ko \
            "$out/lib/modules/${kernel.modDirVersion}/extra/rkvenc.ko"
          runHook postInstall
        '';
        meta.platforms = [ system ];
      };
      modules = {
        linux-6_12 = buildModule pkgs.linuxPackages_6_12.kernel;
        linux-6_18 = buildModule pkgs.linuxPackages_6_18.kernel;
        linux-7_3 = buildModule pkgs.linuxPackages_7_3.kernel;
      };
    in {
      packages.${system} = modules;
      checks.${system} = modules;
    };
}
