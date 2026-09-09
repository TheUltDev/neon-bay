# Run the demo: the authoritative sidecar in one window, the web client in
# another. Assumes `spacetime start` is already running and scripts/setup.ps1
# has been run at least once.
#
#   powershell -File scripts/dev.ps1 [-Bots 6] [-Db physics-sidecar]

param(
  [int]$Bots = 6,
  [string]$Db = 'physics-sidecar',
  [string]$Uri = 'http://127.0.0.1:3000'
)

$root = Split-Path -Parent $PSScriptRoot

try {
  $ping = Invoke-WebRequest -Uri "$Uri/v1/ping" -TimeoutSec 3 -UseBasicParsing
  if ($ping.StatusCode -ne 200) { throw }
}
catch {
  Write-Host "No SpacetimeDB at $Uri. Start one with:  spacetime start" -ForegroundColor Yellow
  exit 1
}

$sidecar = Join-Path $root 'target/release/sidecar.exe'
if (-not (Test-Path $sidecar)) {
  Write-Host 'Building the sidecar...' -ForegroundColor Cyan
  Push-Location $root
  cargo build -p sidecar --release
  Pop-Location
}

# `input` is a private table, which SpacetimeDB shows to the database's owner
# and to nobody else. The sidecar is that owner -- the identity that published
# the module -- so it needs that identity's token. Handed over in the
# environment rather than on the command line, where it would be visible to
# anything that can list processes.
$token = (& spacetime login show --token 2>$null | Select-String 'auth token' |
  ForEach-Object { ($_ -split ' ')[-1] })
if (-not $token) {
  Write-Host 'Could not read an auth token from `spacetime login show --token`.' -ForegroundColor Yellow
  Write-Host 'The sidecar has to connect as the identity that published the module.' -ForegroundColor Yellow
  exit 1
}
$env:STDB_TOKEN = $token

Write-Host 'Starting the authoritative sidecar...' -ForegroundColor Cyan
Start-Process -FilePath $sidecar -ArgumentList @('--bots', $Bots, '--db', $Db, '--uri', $Uri) -WorkingDirectory $root

Write-Host 'Starting the web client on http://localhost:5173 ...' -ForegroundColor Cyan
Push-Location (Join-Path $root 'web')
npm run dev
Pop-Location
