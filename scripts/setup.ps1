# One-time (or after changing the module schema): publish, regenerate bindings,
# build the wasm physics core, install the web client's dependencies.
#
#   powershell -File scripts/setup.ps1 [-Server local] [-Db physics-sidecar] [-Fresh]
#
# -Fresh wipes the database, which you want whenever the schema changes shape.
#
# Deliberately no `$ErrorActionPreference = 'Stop'`: in Windows PowerShell 5.1
# that turns any native command's stderr -- cargo's progress output, for one --
# into a terminating error. Exit codes are checked explicitly instead.

param(
  [string]$Server = 'local',
  [string]$Db = 'physics-sidecar',
  [switch]$Fresh
)

$root = Split-Path -Parent $PSScriptRoot
Push-Location $root

function Step($msg) { Write-Host "`n== $msg" -ForegroundColor Cyan }

function Invoke-Checked {
  param([string]$Exe, [string[]]$CmdArgs)
  & $Exe @CmdArgs
  if ($LASTEXITCODE -ne 0) {
    Pop-Location
    throw "$Exe $($CmdArgs -join ' ') failed with exit code $LASTEXITCODE"
  }
}

try {
  Step 'checking toolchain'
  foreach ($cmd in @('cargo', 'rustup', 'spacetime', 'npm', 'node')) {
    if (-not (Get-Command $cmd -ErrorAction SilentlyContinue)) {
      throw "$cmd not found on PATH"
    }
  }
  if ((rustup target list --installed) -notcontains 'wasm32-unknown-unknown') {
    Step 'installing the wasm32-unknown-unknown target'
    Invoke-Checked rustup @('target', 'add', 'wasm32-unknown-unknown')
  }

  # The bindings `spacetime generate` writes have to compile against the SDK
  # version this project pins. If the CLI has moved on, say so now rather than
  # letting cargo fail later with a wall of type errors.
  $pinned = $null
  $m = Select-String -Path module/Cargo.toml -Pattern 'spacetimedb\s*=\s*"(\d+\.\d+)' |
    Select-Object -First 1
  if ($m) { $pinned = $m.Matches[0].Groups[1].Value }
  $cliRaw = (& spacetime --version 2>&1 | Out-String)
  $cli = if ($cliRaw -match 'version (\d+\.\d+)') { $Matches[1] } else { $null }
  if ($pinned -and $cli -and ($pinned -ne $cli)) {
    Write-Host "warning: spacetime CLI is $cli, this project pins SDK $pinned." -ForegroundColor Yellow
    Write-Host "         If the generated bindings do not compile, bump the version in" -ForegroundColor Yellow
    Write-Host "         module/Cargo.toml, sidecar/Cargo.toml and web/package.json." -ForegroundColor Yellow
  }

  Step 'publishing the SpacetimeDB module'
  $publish = @('publish', '--server', $Server, '--module-path', 'module', '--yes')
  if ($Fresh) { $publish += '--delete-data=always' }
  $publish += $Db
  Invoke-Checked spacetime $publish

  Step 'generating client bindings'
  Invoke-Checked spacetime @('generate', '--lang', 'rust',
    '--out-dir', 'sidecar/src/module_bindings', '--module-path', 'module')
  Invoke-Checked spacetime @('generate', '--lang', 'typescript',
    '--out-dir', 'web/src/module_bindings', '--module-path', 'module')

  Step 'building the shared physics core (native + wasm)'
  Invoke-Checked cargo @('build', '--release')
  # Shared with scripts/setup.sh so the build-and-copy has one implementation.
  Invoke-Checked node @('scripts/build-wasm.mjs')

  Step 'installing web dependencies'
  Push-Location web
  Invoke-Checked npm @('install', '--no-fund', '--no-audit')
  Pop-Location

  Step 'verifying native and wasm agree bit for bit'
  node scripts/verify-determinism.mjs | Select-Object -Last 2

  Write-Host "`nReady. Start it with: powershell -File scripts/dev.ps1" -ForegroundColor Green
}
finally {
  Pop-Location
}
