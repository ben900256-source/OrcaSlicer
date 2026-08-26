# Fast Windows development builds

`scripts/dev-build.ps1` is an opt-in workflow for short, bounded edit/build/test
cycles on Windows. It does not replace `build_release_vs.bat`, change CI, build
dependencies, or alter production flags. It reuses dependencies already staged
under `deps/build` (`deps/build-arm64` for ARM64).

The default command builds only the core slicing library in RelWithDebInfo with
eight compile workers:

```powershell
.\scripts\dev-build.ps1
```

The first Ninja build is a full bootstrap of the selected target. Ninja and
compiler caching are intended to improve later builds; they cannot avoid that
initial cost.

## Tasks

```powershell
# Core slicing library only.
.\scripts\dev-build.ps1 -Task Core

# Build and run one complete Catch2 suite.
.\scripts\dev-build.ps1 -Task FffTests
.\scripts\dev-build.ps1 -Task ConfigTests -Filter '[Preset]'

# Build both relevant suites and run the Pin-support filters.
.\scripts\dev-build.ps1 -Task PinTests

# Build the GUI and its runtime DLL staging target.
.\scripts\dev-build.ps1 -Task App
.\scripts\dev-build.ps1 -Task App -Run

# Build the complete development tree. Installation is explicit.
.\scripts\dev-build.ps1 -Task Full
.\scripts\dev-build.ps1 -Task Full -Install
```

`-Run` launches `src\orca-slicer.exe` from a Ninja tree, or
`src\<Configuration>\orca-slicer.exe` from a Visual Studio tree. The App task
also builds `COPY_DLLS`, so the build-tree executable has its runtime files.
The app is launched normally and is not tied to the build script's lifetime.

Use `-CheckOnly` to validate CMake, Visual Studio, Ninja/cache selection, the
dependency prefix, target mapping, directories, and commands without configuring
or changing a build tree:

```powershell
.\scripts\dev-build.ps1 -Task PinTests -CheckOnly
.\scripts\dev-build.ps1 -Task App -Backend VisualStudio -Cache Off -CheckOnly
```

## Configuration and bounded concurrency

The configurable parameters are:

```text
-Task          Core | FffTests | ConfigTests | PinTests | App | Full
-Configuration Debug | RelWithDebInfo | Release
-Backend       Auto | Ninja | VisualStudio
-Cache         Auto | On | Off
-Jobs          positive integer (default 8)
-Architecture  x64 | ARM64
-Filter        Catch2 filter for a test task
-Run           App only
-Install       Full only
-CheckOnly
```

`Auto` prefers Ninja. The script locates Visual Studio through `vswhere.exe` and
imports `VsDevCmd.bat`, so it can be run from an ordinary PowerShell session.
Ninja gets a compile pool matching `-Jobs` and a single-worker link pool. Visual
Studio uses one MSBuild project worker plus `/MP<Jobs>` and
`/p:CL_MPCount=<Jobs>`; this avoids multiplying MSBuild project concurrency by
compiler concurrency.

Separate trees under `build-dev` prevent generator and compiler-launcher state
from colliding. Ninja trees are separated by architecture, configuration, and
cache selection; Visual Studio uses one multi-configuration tree per
architecture. Each tree records its configure signature, so an identical
invocation goes straight to the incremental build. CMake still regenerates the
tree automatically when its build-system inputs change.

## Compiler caching

With `-Cache Auto`, the driver prefers `sccache`, then accepts ccache 4.10 or
newer. Older ccache releases do not have the MSVC PCH support required by this
workflow and are ignored. Auto mode continues without a cache when neither tool
is compatible. `-Cache On` instead stops with installation guidance. The script
does not install software or change antivirus settings.

CMake compiler launchers work with Ninja, not Visual Studio generators, so a
Visual Studio build uses no compiler cache. Use Ninja when cache reuse matters.
Cache statistics remain available through the selected tool, for example:

```powershell
sccache --show-stats
ccache --show-stats
```

For a cold/warm comparison, clear only the cache tool's statistics, build once,
remove the selected build outputs (not the cache), rebuild, and record cacheable
requests, hits, misses, elapsed time, and cache disk use. Cache absence does not
block this workflow.

## Fast-mode CMake controls

The driver configures these default-off development controls:

- `ORCA_FAST_DEV=ON` selects embedded MSVC debug information (`/Z7`) and omits
  the repository's global development-unfriendly `/LTCG` link flag. The large
  `libslic3r_gui` static archive alone retains a target-specific `/Zi /FS`
  compiler PDB because `/Z7` makes that archive exceed MSVC's 4 GiB limit.
- `SLIC3R_MSVC_COMPILE_JOBS=<Jobs>` emits a bounded `/MP<Jobs>` for Visual
  Studio. Leaving it empty outside this driver preserves the existing `/MP`.

Fast mode requires CMake 3.25 or newer because it uses CMP0141. All controls are
off in normal build trees, so the normal `/Zi /FS`, `/MP`, and `/LTCG` behavior
is unchanged.

## When to use the release flow

Use `build_release_vs.bat` and the repository's normal install/packaging steps
for final Release checks, distributable packages, signing, localization
generation, dependency rebuilds, and CI-equivalent verification. Select
`-Configuration Release` here only for local slicing-performance or targeted
compatibility checks; it is not a packaging substitute.
