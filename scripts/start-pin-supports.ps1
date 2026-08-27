<#
.SYNOPSIS
Starts the packaged Windows Pin Supports build with the built-in Prusa XL miniature profiles.

.DESCRIPTION
All default paths are resolved relative to this checkout. The launcher verifies the
packaged Prusa vendor bundle, initializes an isolated data directory with the mixed-
nozzle XL, tool-2 miniature PLA, and requested process selected, then opens the
support-contact coupon by default. It never reads or writes the normal OrcaSlicer
profile directory.

.EXAMPLE
  .\scripts\start-pin-supports.ps1

.EXAMPLE
  .\scripts\start-pin-supports.ps1 -Preset UltraDetail

.EXAMPLE
  .\scripts\start-pin-supports.ps1 -Empty

.EXAMPLE
  .\scripts\start-pin-supports.ps1 -InputFile .\pin-3mm.3mf -Wait

.EXAMPLE
  .\scripts\start-pin-supports.ps1 -CheckOnly
#>
[CmdletBinding()]
param(
    [string] $InstallDir = '',
    [string] $DataDir = '',
    [string] $InputFile = '',
    [ValidateSet('Balanced', 'UltraDetail')]
    [string] $Preset = 'Balanced',
    [switch] $Empty,
    [switch] $Wait,
    [switch] $CheckOnly
)

$ErrorActionPreference = 'Stop'

$requiredBundleVersion = '02.04.00.07'
$machineName = 'Prusa XL 5T T2 0.25 nozzle (others 0.4)'
$filamentName = 'Prusa Generic Miniature PLA @XL 5T'
$processNames = @{
    Balanced    = '0.06mm Miniature Balanced + Pin @Prusa XL 5T T2 0.25'
    UltraDetail = '0.05mm Miniature Ultra Detail + Pin @Prusa XL 5T T2 0.25'
}
$processName = $processNames[$Preset]

$repoRoot = Split-Path $PSScriptRoot -Parent
if ([string]::IsNullOrWhiteSpace($InstallDir)) {
    $InstallDir = Join-Path $repoRoot 'build\OrcaSlicer'
}
if ([string]::IsNullOrWhiteSpace($DataDir)) {
    $DataDir = Join-Path $repoRoot 'build\pin-supports-user-data'
}
if (-not $Empty -and [string]::IsNullOrWhiteSpace($InputFile)) {
    $InputFile = Join-Path $repoRoot 'tests\data\support_contact_coupon.obj'
}

$InstallDir = [System.IO.Path]::GetFullPath($InstallDir)
$DataDir = [System.IO.Path]::GetFullPath($DataDir)
$executable = Join-Path $InstallDir 'orca-slicer.exe'
$profilesDir = Join-Path $InstallDir 'resources\profiles'
$vendorBundlePath = Join-Path $profilesDir 'Prusa.json'
$vendorProfileDir = Join-Path $profilesDir 'Prusa'

if (-not (Test-Path -LiteralPath $executable -PathType Leaf)) {
    throw "Native OrcaSlicer executable not found at '$executable'. Build the packaged Release target first."
}
if (-not (Test-Path -LiteralPath $vendorBundlePath -PathType Leaf)) {
    throw "Packaged Prusa vendor bundle not found at '$vendorBundlePath'."
}

$vendorBundle = Get-Content -Raw -LiteralPath $vendorBundlePath | ConvertFrom-Json
if ($vendorBundle.version -ne $requiredBundleVersion) {
    throw "Packaged Prusa bundle is version '$($vendorBundle.version)'; expected '$requiredBundleVersion'. Rebuild the packaged resources."
}

$requiredProfiles = @(
    @{ List = 'machine_list'; Name = $machineName; RelativePath = 'machine\Prusa XL 5T T2 0.25 nozzle (others 0.4).json' },
    @{ List = 'filament_list'; Name = $filamentName; RelativePath = 'filament\Prusa Generic Miniature PLA @XL 5T.json' },
    @{ List = 'process_list'; Name = 'process_common_fdm_miniature_pin'; RelativePath = 'process\process_common_fdm_miniature_pin.json' },
    @{ List = 'process_list'; Name = $processNames.Balanced; RelativePath = 'process\0.06mm Miniature Balanced + Pin @Prusa XL 5T T2 0.25.json' },
    @{ List = 'process_list'; Name = $processNames.UltraDetail; RelativePath = 'process\0.05mm Miniature Ultra Detail + Pin @Prusa XL 5T T2 0.25.json' }
)

foreach ($requiredProfile in $requiredProfiles) {
    $registered = @($vendorBundle.($requiredProfile.List) | Where-Object { $_.name -eq $requiredProfile.Name })
    if ($registered.Count -ne 1) {
        throw "Profile '$($requiredProfile.Name)' is not registered exactly once in packaged $($requiredProfile.List)."
    }

    $profilePath = Join-Path $vendorProfileDir $requiredProfile.RelativePath
    if (-not (Test-Path -LiteralPath $profilePath -PathType Leaf)) {
        throw "Registered packaged profile is missing at '$profilePath'."
    }
    $profile = Get-Content -Raw -LiteralPath $profilePath | ConvertFrom-Json
    if ($profile.name -ne $requiredProfile.Name) {
        throw "Packaged profile '$profilePath' has name '$($profile.name)', expected '$($requiredProfile.Name)'."
    }
}

if (-not $Empty) {
    $InputFile = [System.IO.Path]::GetFullPath($InputFile)
    if (-not (Test-Path -LiteralPath $InputFile -PathType Leaf)) {
        throw "Input model or project not found at '$InputFile'."
    }
}

Write-Output "Executable: $executable"
Write-Output "Profile data: $DataDir"
Write-Output "Machine:    $machineName"
Write-Output "Process:    $processName"
Write-Output "Filament:   $filamentName"
if (-not $Empty) {
    Write-Output "Input:      $InputFile"
}

if ($CheckOnly) {
    Write-Output 'Launcher check passed; the isolated profile directory was not changed and OrcaSlicer was not started.'
    return
}

New-Item -ItemType Directory -Force -Path $DataDir | Out-Null

$filamentPresets = @($filamentName, $filamentName, $filamentName, $filamentName, $filamentName)
$filamentColors = @('#26A69A', '#26A69A', '#26A69A', '#26A69A', '#26A69A')
$machineSettings = [ordered]@{
    machine = $machineName
    process = $processName
    filament = $filamentName
    filament_01 = $filamentName
    filament_02 = $filamentName
    filament_03 = $filamentName
    filament_04 = $filamentName
    filament_colors = ($filamentColors -join ',')
}
$isolatedConfig = [ordered]@{
    header = 'OrcaSlicer isolated Pin Supports configuration'
    app = [ordered]@{
        preset_folder = 'default'
        filament_presets = $filamentPresets
        filament_colors = $filamentColors
    }
    filaments = @($filamentName)
    models = @(
        [ordered]@{
            vendor = 'Prusa'
            model = 'Prusa XL 5T'
            nozzle_diameter = '0.4+0.25+0.4+0.4+0.4'
        }
    )
    presets = [ordered]@{
        machine = $machineName
    }
    orca_presets = @($machineSettings)
}

$configPath = Join-Path $DataDir 'OrcaSlicer.conf'
$configJson = ($isolatedConfig | ConvertTo-Json -Depth 8) -replace "`r`n", "`n"
$utf8WithoutBom = New-Object System.Text.UTF8Encoding($false)
$md5 = [System.Security.Cryptography.MD5]::Create()
try {
    $digest = $md5.ComputeHash($utf8WithoutBom.GetBytes($configJson))
}
finally {
    $md5.Dispose()
}

# Match OrcaSlicer's Windows config trailer so AppConfig can safely split and
# verify the generated file during startup.
$checksum = ($digest | ForEach-Object { $_.ToString('X2') }) -join ''
$configContents = "$configJson`n# MD5 checksum $checksum`n"
[System.IO.File]::WriteAllText($configPath, $configContents, $utf8WithoutBom)
Write-Output "Initialized isolated selections in '$configPath'."

# Start-Process joins ArgumentList into a Windows command line. Quote path values
# explicitly so checkouts under directories containing spaces remain supported.
$quotedArguments = @(
    '--datadir',
    ('"{0}"' -f $DataDir)
)
if (-not $Empty) {
    $quotedArguments += ('"{0}"' -f $InputFile)
}

$startParameters = @{
    FilePath = $executable
    WorkingDirectory = $InstallDir
    ArgumentList = $quotedArguments
    PassThru = $true
}
$process = Start-Process @startParameters

Write-Output "Started OrcaSlicer (PID $($process.Id))."

if ($Wait) {
    $process.WaitForExit()
    if ($process.ExitCode -ne 0) {
        throw "OrcaSlicer exited with code $($process.ExitCode)."
    }
}
