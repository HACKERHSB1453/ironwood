# Ironwood

Open-source (GPL-3.0-or-later), Linux-native parametric CAD, aiming at functional parity with the
mainstream history-based CAD/CAM/CAE workflow, with an async, incrementally-recomputing architecture.
See `docs/ARCHITECTURE.md` and `docs/FEATURE_INVENTORY.md`.

**Status: P0 in progress** (skeleton + interface contracts; most functionality is stubbed).

## Build

Requirements: GCC or Clang with C++23, CMake >= 3.28, Ninja, OpenCASCADE 7.9, Qt 6 (Widgets), oneTBB, GoogleTest.
ccache is used automatically when installed. Tracy is optional (`-DIRONWOOD_TRACY=ON`, needs network).

```sh
cmake --preset debug
cmake --build --preset debug
ctest --preset debug
./build/debug/ironwood-cli --version
```

Presets: `debug`, `release`, `asan`, `debug-clang`. Headless-only: `-DIRONWOOD_BUILD_GUI=OFF`.

## License

GPL-3.0-or-later; see `LICENSE`. Third-party components: `THIRD_PARTY.md`.

## Clean-room statement

Ironwood's behaviour is derived from public documentation and observation of publicly available behaviour.
No proprietary vendor code, binaries, assets or decompiled logic is used, and none may be contributed.
