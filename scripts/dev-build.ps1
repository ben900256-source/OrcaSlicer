#requires -Version 5.1

<#
.SYNOPSIS
Builds focused OrcaSlicer development targets on Windows.

.DESCRIPTION
Configures an opt-in fast-development tree that reuses deps/build, prefers
Ninja, bounds compilation to eight workers by default, and uses sccache or a
PCH-capable ccache when available. Production build scripts are not changed.

.EXAMPLE
.\scripts\dev-build.ps1 -Task Core

.EXAMPLE
.\scripts\dev-build.ps1 -Task FffTests -Filter '[SupportMaterial]'

.EXAMPLE
.\scripts\dev-build.ps1 -Task App -Run
#>
[CmdletBinding()]
param(
    [ValidateSet('Core', 'FffTests', 'ConfigTests', 'PinTests', 'App', 'Full')]
    [string] $Task = 'Core',

    [ValidateSet('Debug', 'RelWithDebInfo', 'Release')]
    [string] $Configuration = 'RelWithDebInfo',

    [ValidateSet('Auto', 'Ninja', 'VisualStudio')]
    [string] $Backend = 'Auto',

    [ValidateSet('Auto', 'On', 'Off')]
    [string] $Cache = 'Auto',

    [ValidateScript({ $_ -gt 0 })]
    [int] $Jobs = 8,

    [ValidateSet('x64', 'ARM64')]
    [string] $Architecture = 'x64',

    [string] $Filter,

    [switch] $Run,
    [switch] $Install,
    [switch] $CheckOnly
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

function Get-ApplicationPath {
    param([Parameter(Mandatory = $true)][string] $Name)

    $command = Get-Command -Name $Name -CommandType Application -ErrorAction SilentlyContinue |
        Select-Object -First 1
    if ($null -eq $command) {
        return $null
    }
    return [System.IO.Path]::GetFullPath($command.Source)
}

function Get-VsInstallation {
    $vswhere = Get-ApplicationPath -Name 'vswhere.exe'
    if ($null -eq $vswhere) {
        $programFilesX86 = [Environment]::GetFolderPath('ProgramFilesX86')
        if ($programFilesX86) {
            $candidate = Join-Path $programFilesX86 'Microsoft Visual Studio\Installer\vswhere.exe'
            if (Test-Path -LiteralPath $candidate -PathType Leaf) {
                $vswhere = $candidate
            }
        }
    }
    if ($null -eq $vswhere) {
        throw 'Visual Studio Installer (vswhere.exe) was not found. Install Visual Studio 2022 Build Tools with Desktop development with C++.'
    }

    $query = @(
        '-latest',
        '-products', '*',
        '-requires', 'Microsoft.VisualStudio.Component.VC.Tools.x86.x64'
    )
    $installationJson = (& $vswhere @query -format json -utf8) -join "`n"
    if ($LASTEXITCODE -ne 0 -or -not $installationJson) {
        throw 'No Visual Studio installation with the MSVC C++ tools was found.'
    }
    try {
        $installation = @($installationJson | ConvertFrom-Json) | Select-Object -First 1
    } catch {
        throw "vswhere.exe returned invalid JSON: $($_.Exception.Message)"
    }
    if ($null -eq $installation -or -not $installation.installationPath -or -not $installation.installationVersion) {
        throw 'vswhere.exe did not report a usable Visual Studio installation.'
    }
    $installationPath = [string]$installation.installationPath
    $installationVersion = [string]$installation.installationVersion

    $majorVersion = ([version]$installationVersion.Trim()).Major
    $generator = switch ($majorVersion) {
        16 { 'Visual Studio 16 2019' }
        17 { 'Visual Studio 17 2022' }
        18 { 'Visual Studio 18 2026' }
        default { throw "Visual Studio $majorVersion is not supported by this script." }
    }

    return [pscustomobject]@{
        Path      = [System.IO.Path]::GetFullPath($installationPath.Trim())
        Version   = $installationVersion.Trim()
        Generator = $generator
        VsWhere   = $vswhere
    }
}

function Import-VsDevEnvironment {
    param(
        [Parameter(Mandatory = $true)][string] $InstallationPath,
        [Parameter(Mandatory = $true)][string] $TargetArchitecture
    )

    $targetValue = $TargetArchitecture.ToLowerInvariant()
    $hostValue = $env:PROCESSOR_ARCHITECTURE
    if ($env:PROCESSOR_ARCHITEW6432) {
        $hostValue = $env:PROCESSOR_ARCHITEW6432
    }
    if ($hostValue -eq 'ARM64') {
        $hostArchitecture = 'arm64'
    } else {
        $hostArchitecture = 'x64'
    }
    $hostFolder = if ($hostArchitecture -eq 'arm64') { 'Hostarm64' } else { 'Hostx64' }
    if ($env:VSCMD_VER -and $env:VSCMD_ARG_TGT_ARCH -eq $targetValue -and $env:VCToolsInstallDir) {
        $existingCl = Join-Path $env:VCToolsInstallDir "bin\$hostFolder\$targetValue\cl.exe"
        if (Test-Path -LiteralPath $existingCl -PathType Leaf) {
            return [System.IO.Path]::GetFullPath($existingCl)
        }
    }

    $vsDevCmd = Join-Path $InstallationPath 'Common7\Tools\VsDevCmd.bat'
    if (-not (Test-Path -LiteralPath $vsDevCmd -PathType Leaf)) {
        throw "VsDevCmd.bat was not found at '$vsDevCmd'."
    }

    $command = "`"$vsDevCmd`" -no_logo -arch=$targetValue -host_arch=$hostArchitecture && set"
    $environmentLines = & $env:ComSpec /s /c $command
    if ($LASTEXITCODE -ne 0) {
        throw "VsDevCmd.bat failed for target architecture $TargetArchitecture. Ensure the matching MSVC tools are installed."
    }

    $existingPathName = [Environment]::GetEnvironmentVariables('Process').Keys |
        Where-Object { $_ -ieq 'PATH' } |
        Select-Object -First 1
    foreach ($line in $environmentLines) {
        $separator = $line.IndexOf('=')
        if ($separator -le 0) {
            continue
        }
        $name = $line.Substring(0, $separator)
        $value = $line.Substring($separator + 1)
        # Preserve the host's existing casing so Start-Process does not see two
        # PATH keys in its case-insensitive environment dictionary.
        if ($name -ieq 'PATH' -and $existingPathName) {
            $name = $existingPathName
        }
        [Environment]::SetEnvironmentVariable($name, $value, 'Process')
    }

    $clPath = $null
    if ($env:VCToolsInstallDir) {
        $clCandidate = Join-Path $env:VCToolsInstallDir "bin\$hostFolder\$targetValue\cl.exe"
        if (Test-Path -LiteralPath $clCandidate -PathType Leaf) {
            $clPath = $clCandidate
        }
    }
    if ($null -eq $clPath) {
        $clPath = Get-ApplicationPath -Name 'cl.exe'
    }
    if ($null -eq $clPath) {
        throw "VsDevCmd.bat completed, but cl.exe is unavailable for $TargetArchitecture."
    }
    return [System.IO.Path]::GetFullPath($clPath)
}

function Get-CMakeVersion {
    param([Parameter(Mandatory = $true)][string] $CMakePath)

    $versionOutput = @(& $CMakePath --version)
    if ($LASTEXITCODE -ne 0 -or $versionOutput.Count -eq 0) {
        throw "Unable to determine the CMake version from '$CMakePath'."
    }
    $versionMatch = [regex]::Match($versionOutput[0], '(\d+\.\d+(?:\.\d+)?)')
    if (-not $versionMatch.Success) {
        throw "Unable to determine the CMake version from '$CMakePath'."
    }
    return [version]$versionMatch.Groups[1].Value
}

function Select-CompilerCache {
    param(
        [Parameter(Mandatory = $true)][string] $RequestedCache,
        [Parameter(Mandatory = $true)][string] $SelectedBackend
    )

    if ($RequestedCache -eq 'Off') {
        return [pscustomobject]@{ Name = 'none'; Path = $null; Note = 'disabled by -Cache Off' }
    }
    if ($SelectedBackend -eq 'VisualStudio') {
        if ($RequestedCache -eq 'On') {
            throw 'Compiler caching requires the Ninja backend because CMake compiler launchers are not supported by Visual Studio generators.'
        }
        return [pscustomobject]@{ Name = 'none'; Path = $null; Note = 'Visual Studio generators do not support CMake compiler launchers' }
    }

    $sccache = Get-ApplicationPath -Name 'sccache.exe'
    if ($null -ne $sccache) {
        return [pscustomobject]@{ Name = 'sccache'; Path = $sccache; Note = $null }
    }

    $ccache = Get-ApplicationPath -Name 'ccache.exe'
    $ccacheNote = $null
    if ($null -ne $ccache) {
        $versionOutput = (& $ccache --version) -join "`n"
        if ($LASTEXITCODE -eq 0 -and $versionOutput -match 'ccache version (\d+\.\d+(?:\.\d+)?)') {
            $ccacheVersion = [version]$Matches[1]
            if ($ccacheVersion -ge [version]'4.10') {
                return [pscustomobject]@{ Name = 'ccache'; Path = $ccache; Note = $null }
            }
            $ccacheNote = "ccache $ccacheVersion at '$ccache' is too old; MSVC PCH support requires ccache 4.10 or newer"
        } else {
            $ccacheNote = "the ccache version at '$ccache' could not be determined"
        }
    }

    if ($RequestedCache -eq 'On') {
        $detail = if ($ccacheNote) { " $ccacheNote." } else { '' }
        throw "No compatible compiler cache was found.$detail Install sccache (preferred) or ccache 4.10+, then ensure it is on PATH."
    }
    if ($ccacheNote) {
        return [pscustomobject]@{ Name = 'none'; Path = $null; Note = "$ccacheNote; continuing uncached" }
    }
    return [pscustomobject]@{ Name = 'none'; Path = $null; Note = 'no sccache or compatible ccache found; continuing uncached' }
}

function ConvertTo-WindowsCommandLineArgument {
    param([AllowEmptyString()][string] $Argument)

    if ($Argument.Length -gt 0 -and $Argument -notmatch '[\s"]') {
        return $Argument
    }

    $result = '"'
    $backslashes = 0
    foreach ($character in $Argument.ToCharArray()) {
        if ($character -eq '\') {
            $backslashes++
        } elseif ($character -eq '"') {
            $result += ('\' * ($backslashes * 2 + 1)) + '"'
            $backslashes = 0
        } else {
            $result += ('\' * $backslashes) + $character
            $backslashes = 0
        }
    }
    $result += ('\' * ($backslashes * 2)) + '"'
    return $result
}

function Format-DevCommand {
    param(
        [Parameter(Mandatory = $true)][string] $FilePath,
        [string[]] $Arguments = @()
    )

    $parts = @((ConvertTo-WindowsCommandLineArgument -Argument $FilePath))
    $parts += @($Arguments | ForEach-Object { ConvertTo-WindowsCommandLineArgument -Argument $_ })
    return $parts -join ' '
}

function Stop-DevProcessTree {
    param([Parameter(Mandatory = $true)][int] $ProcessId)

    $taskkill = Join-Path $env:SystemRoot 'System32\taskkill.exe'
    if (Test-Path -LiteralPath $taskkill -PathType Leaf) {
        & $taskkill /PID $ProcessId /T /F 2>$null | Out-Null
    } else {
        Stop-Process -Id $ProcessId -Force -ErrorAction SilentlyContinue
    }
}

function Invoke-DevProcess {
    param(
        [Parameter(Mandatory = $true)][string] $FilePath,
        [string[]] $Arguments = @(),
        [Parameter(Mandatory = $true)][string] $WorkingDirectory
    )

    Write-Output ("> " + (Format-DevCommand -FilePath $FilePath -Arguments $Arguments))
    $argumentLine = (@($Arguments | ForEach-Object { ConvertTo-WindowsCommandLineArgument -Argument $_ }) -join ' ')
    $process = $null
    $processStarted = $false
    $exitCode = $null
    try {
        $startInfo = New-Object System.Diagnostics.ProcessStartInfo
        $startInfo.FileName = $FilePath
        $startInfo.Arguments = $argumentLine
        $startInfo.WorkingDirectory = $WorkingDirectory
        $startInfo.UseShellExecute = $false
        # ProcessStartInfo inherits the current process environment. VsDevCmd
        # has already populated that environment, so copying selected entries
        # is unnecessary and fails on runtimes where EnvironmentVariables is
        # not exposed as an indexable dictionary.
        $process = New-Object System.Diagnostics.Process
        $process.StartInfo = $startInfo
        if (-not $process.Start()) {
            throw "Failed to start '$FilePath'."
        }
        $processStarted = $true
        while (-not $process.WaitForExit(250)) {
            # The bounded wait lets PowerShell deliver Ctrl+C so finally can
            # terminate this exact process tree rather than unrelated compilers.
        }
        $exitCode = $process.ExitCode
    } finally {
        if ($processStarted -and -not $process.HasExited) {
            Stop-DevProcessTree -ProcessId $process.Id
        }
        if ($null -ne $process) {
            $process.Dispose()
        }
    }
    if ($exitCode -ne 0) {
        throw "'$FilePath' exited with code $exitCode."
    }
}

if ($env:OS -ne 'Windows_NT') {
    throw 'scripts/dev-build.ps1 is a Windows-only development driver.'
}
if ($Run -and $Task -ne 'App') {
    throw '-Run is only valid with -Task App. Test tasks run automatically.'
}
if ($Install -and $Task -ne 'Full') {
    throw '-Install is only valid with -Task Full.'
}
if ($Filter -and $Task -notin @('FffTests', 'ConfigTests', 'PinTests')) {
    throw '-Filter is only valid with a test task.'
}

$scriptDirectory = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = [System.IO.Path]::GetFullPath((Join-Path $scriptDirectory '..'))
$cmake = Get-ApplicationPath -Name 'cmake.exe'
if ($null -eq $cmake) {
    throw 'cmake.exe was not found on PATH. Install CMake 3.25 or newer.'
}
$cmakeVersion = Get-CMakeVersion -CMakePath $cmake
if ($cmakeVersion -lt [version]'3.25') {
    throw "Fast development mode requires CMake 3.25 or newer; found $cmakeVersion at '$cmake'."
}

$vs = Get-VsInstallation
$ninja = $null
$msvcCompiler = $null
$resourceCompiler = $null
$manifestTool = $null
if ($Backend -in @('Auto', 'Ninja')) {
    $msvcCompiler = Import-VsDevEnvironment -InstallationPath $vs.Path -TargetArchitecture $Architecture
    $ninja = Get-ApplicationPath -Name 'ninja.exe'
    if ($env:PROCESSOR_ARCHITECTURE -eq 'ARM64' -or $env:PROCESSOR_ARCHITEW6432 -eq 'ARM64') {
        $sdkHostArchitecture = 'arm64'
    } else {
        $sdkHostArchitecture = 'x64'
    }
    if ($env:WindowsSdkVerBinPath) {
        $rcCandidate = Join-Path $env:WindowsSdkVerBinPath "$sdkHostArchitecture\rc.exe"
        $mtCandidate = Join-Path $env:WindowsSdkVerBinPath "$sdkHostArchitecture\mt.exe"
        if (Test-Path -LiteralPath $rcCandidate -PathType Leaf) {
            $resourceCompiler = [System.IO.Path]::GetFullPath($rcCandidate)
        }
        if (Test-Path -LiteralPath $mtCandidate -PathType Leaf) {
            $manifestTool = [System.IO.Path]::GetFullPath($mtCandidate)
        }
    }
    if ($null -eq $resourceCompiler) {
        $resourceCompiler = Get-ApplicationPath -Name 'rc.exe'
    }
    if ($null -eq $manifestTool) {
        $manifestTool = Get-ApplicationPath -Name 'mt.exe'
    }
    if ($null -eq $resourceCompiler -or $null -eq $manifestTool) {
        throw 'The Windows SDK resource compiler (rc.exe) or manifest tool (mt.exe) is missing from the Visual Studio developer environment.'
    }
    $sdkToolDirectory = Split-Path -Parent $resourceCompiler
    if (-not (($env:Path -split ';') -contains $sdkToolDirectory)) {
        $env:Path = "$sdkToolDirectory;$env:Path"
    }
}

if ($Backend -eq 'Ninja') {
    if ($null -eq $ninja) {
        throw 'The Ninja backend was requested, but ninja.exe was not found on PATH or in the Visual Studio developer environment.'
    }
    $selectedBackend = 'Ninja'
} elseif ($Backend -eq 'VisualStudio') {
    $selectedBackend = 'VisualStudio'
} elseif ($null -ne $ninja) {
    $selectedBackend = 'Ninja'
} else {
    $selectedBackend = 'VisualStudio'
}

$cacheSelection = Select-CompilerCache -RequestedCache $Cache -SelectedBackend $selectedBackend
$architectureSuffix = $Architecture.ToLowerInvariant()
if ($Architecture -eq 'ARM64') {
    $depsDirectory = Join-Path $repoRoot 'deps\build-arm64'
} else {
    $depsDirectory = Join-Path $repoRoot 'deps\build'
}
$dependencyPrefix = Join-Path $depsDirectory 'OrcaSlicer_dep\usr\local'
if (-not (Test-Path -LiteralPath $dependencyPrefix -PathType Container)) {
    throw "Prebuilt dependencies were not found at '$dependencyPrefix'. Build them with build_release_vs.bat deps $architectureSuffix first."
}

$buildRoot = Join-Path $repoRoot 'build-dev'
if ($selectedBackend -eq 'Ninja') {
    $cacheTreeName = if ($cacheSelection.Name -eq 'none') { 'nocache' } else { $cacheSelection.Name }
    $treeName = ('ninja-{0}-{1}-{2}' -f $architectureSuffix, $Configuration.ToLowerInvariant(), $cacheTreeName)
    $buildDirectory = Join-Path $buildRoot $treeName
    $generator = 'Ninja'
} else {
    $buildDirectory = Join-Path $buildRoot ('vs-{0}' -f $architectureSuffix)
    $generator = $vs.Generator
}
$installDirectory = Join-Path $buildDirectory 'OrcaSlicer'

$buildTests = $Task -in @('FffTests', 'ConfigTests', 'PinTests', 'Full')
$targets = switch ($Task) {
    'Core'        { @('libslic3r') }
    'FffTests'    { @('fff_print_tests') }
    'ConfigTests' { @('libslic3r_tests') }
    'PinTests'    { @('fff_print_tests', 'libslic3r_tests') }
    'App'         { @('OrcaSlicer_app_gui', 'COPY_DLLS') }
    'Full' {
        if ($selectedBackend -eq 'Ninja') { @('all') } else { @('ALL_BUILD') }
    }
}

$configureArguments = @(
    '-S', $repoRoot,
    '-B', $buildDirectory,
    '-G', $generator
)
if ($selectedBackend -eq 'Ninja') {
    $compilerPath = $msvcCompiler.Replace('\', '/')
    $resourceCompilerPath = $resourceCompiler.Replace('\', '/')
    $manifestToolPath = $manifestTool.Replace('\', '/')
    $ninjaPath = $ninja.Replace('\', '/')
    $configureArguments += @(
        "-DCMAKE_BUILD_TYPE=$Configuration",
        "-DCMAKE_C_COMPILER=$compilerPath",
        "-DCMAKE_CXX_COMPILER=$compilerPath",
        "-DCMAKE_RC_COMPILER=$resourceCompilerPath",
        "-DCMAKE_MT=$manifestToolPath",
        "-DCMAKE_MAKE_PROGRAM=$ninjaPath"
    )
} else {
    $configureArguments += @('-A', $Architecture, "-DCMAKE_GENERATOR_INSTANCE=$($vs.Path)")
}
$configureArguments += @(
    '-DORCA_FAST_DEV=ON',
    "-DSLIC3R_MSVC_COMPILE_JOBS=$Jobs",
    "-DDEP_BUILD_DIR=$depsDirectory",
    "-DCMAKE_PREFIX_PATH=$dependencyPrefix",
    "-DCMAKE_INSTALL_PREFIX=$installDirectory",
    ('-DBUILD_TESTS=' + $(if ($buildTests) { 'ON' } else { 'OFF' })),
    '-DORCA_TOOLS=ON'
)
if ($cacheSelection.Path) {
    $launcherPath = $cacheSelection.Path.Replace('\', '/')
    $configureArguments += @(
        "-DCMAKE_C_COMPILER_LAUNCHER=$launcherPath",
        "-DCMAKE_CXX_COMPILER_LAUNCHER=$launcherPath"
    )
}

$buildArguments = @(
    '--build', $buildDirectory,
    '--config', $Configuration,
    '--target'
) + $targets
if ($selectedBackend -eq 'Ninja') {
    $buildArguments += @('--parallel', $Jobs)
} else {
    $buildArguments += @('--', '/m:1', "/p:CL_MPCount=$Jobs")
}

function Get-TestExecutable {
    param([Parameter(Mandatory = $true)][string] $Suite)

    if ($selectedBackend -eq 'VisualStudio') {
        return Join-Path $buildDirectory "tests\$Suite\$Configuration\${Suite}_tests.exe"
    }
    return Join-Path $buildDirectory "tests\$Suite\${Suite}_tests.exe"
}

$testCommands = @()
if ($Task -eq 'FffTests') {
    $arguments = [string[]]@()
    if ($Filter) { $arguments = @($Filter) }
    $testCommands += [pscustomobject]@{ Suite = 'fff_print'; File = (Get-TestExecutable 'fff_print'); Arguments = $arguments }
} elseif ($Task -eq 'ConfigTests') {
    $arguments = [string[]]@()
    if ($Filter) { $arguments = @($Filter) }
    $testCommands += [pscustomobject]@{ Suite = 'libslic3r'; File = (Get-TestExecutable 'libslic3r'); Arguments = $arguments }
} elseif ($Task -eq 'PinTests') {
    if ($Filter) {
        $testCommands += [pscustomobject]@{ Suite = 'fff_print'; File = (Get-TestExecutable 'fff_print'); Arguments = @($Filter) }
        $testCommands += [pscustomobject]@{ Suite = 'libslic3r'; File = (Get-TestExecutable 'libslic3r'); Arguments = @($Filter) }
    } else {
        $testCommands += [pscustomobject]@{ Suite = 'fff_print'; File = (Get-TestExecutable 'fff_print'); Arguments = @('[OrganicContacts]') }
        $testCommands += [pscustomobject]@{ Suite = 'libslic3r'; File = (Get-TestExecutable 'libslic3r'); Arguments = @('[MiniaturePin]') }
    }
}

if ($selectedBackend -eq 'VisualStudio') {
    $appExecutable = Join-Path $buildDirectory "src\$Configuration\orca-slicer.exe"
} else {
    $appExecutable = Join-Path $buildDirectory 'src\orca-slicer.exe'
}
$installArguments = @('--install', $buildDirectory, '--config', $Configuration)

Write-Output "Task:          $Task"
Write-Output "Targets:       $($targets -join ', ')"
Write-Output "Configuration: $Configuration"
Write-Output "Architecture:  $Architecture"
Write-Output "Generator:     $generator"
Write-Output "Compiler cache: $($cacheSelection.Name)"
if ($cacheSelection.Note) {
    Write-Output "Cache note:    $($cacheSelection.Note)"
}
Write-Output "Compile jobs:  $Jobs"
Write-Output "Link jobs:     1"
Write-Output "Dependencies:  $depsDirectory"
Write-Output "Build tree:    $buildDirectory"
Write-Output "Install tree:  $installDirectory"
Write-Output "Configure:     $(Format-DevCommand -FilePath $cmake -Arguments $configureArguments)"
Write-Output "Build:         $(Format-DevCommand -FilePath $cmake -Arguments $buildArguments)"
foreach ($testCommand in $testCommands) {
    Write-Output "Run $($testCommand.Suite): $(Format-DevCommand -FilePath $testCommand.File -Arguments $testCommand.Arguments)"
}
if ($Install) {
    Write-Output "Install:       $(Format-DevCommand -FilePath $cmake -Arguments $installArguments)"
}
if ($Run) {
    Write-Output "Launch:        $(Format-DevCommand -FilePath $appExecutable)"
}

if ($CheckOnly) {
    Write-Output 'Check-only validation passed; no build directory was created or changed.'
    return
}

$configureStamp = Join-Path $buildDirectory '.orca-dev-config'
$configureSignature = $cmake + "`n" + ($configureArguments -join "`n")
$configureIsCurrent = $false
if ((Test-Path -LiteralPath (Join-Path $buildDirectory 'CMakeCache.txt') -PathType Leaf) -and
    (Test-Path -LiteralPath $configureStamp -PathType Leaf)) {
    $configureIsCurrent = ([IO.File]::ReadAllText($configureStamp) -ceq $configureSignature)
}

if ($configureIsCurrent) {
    Write-Output '> Configuration unchanged; reusing the existing CMake tree.'
} else {
    Invoke-DevProcess -FilePath $cmake -Arguments $configureArguments -WorkingDirectory $repoRoot
    [IO.File]::WriteAllText($configureStamp, $configureSignature)
}
Invoke-DevProcess -FilePath $cmake -Arguments $buildArguments -WorkingDirectory $repoRoot

foreach ($testCommand in $testCommands) {
    if (-not (Test-Path -LiteralPath $testCommand.File -PathType Leaf)) {
        throw "The $($testCommand.Suite) test executable was not produced at '$($testCommand.File)'."
    }
    Invoke-DevProcess -FilePath $testCommand.File -Arguments $testCommand.Arguments -WorkingDirectory (Split-Path -Parent $testCommand.File)
}

if ($Install) {
    Invoke-DevProcess -FilePath $cmake -Arguments $installArguments -WorkingDirectory $repoRoot
}

if ($Run) {
    if (-not (Test-Path -LiteralPath $appExecutable -PathType Leaf)) {
        throw "The build-tree executable was not produced at '$appExecutable'."
    }
    $appStartInfo = New-Object System.Diagnostics.ProcessStartInfo
    $appStartInfo.FileName = $appExecutable
    $appStartInfo.WorkingDirectory = Split-Path -Parent $appExecutable
    $appStartInfo.UseShellExecute = $true
    $appProcess = [System.Diagnostics.Process]::Start($appStartInfo)
    Write-Output "Started OrcaSlicer from the build tree (PID $($appProcess.Id))."
}
