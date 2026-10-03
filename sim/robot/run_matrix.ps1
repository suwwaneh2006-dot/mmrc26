# =============================================================================
#  sim/robot/run_matrix.ps1 - closed-loop match simulation over many mazes.
#  For every generated 10x10 island maze: normal / mirrored, true motors at
#  1.00 / 0.85 / 1.15 x the firmware's assumed speed. Prints one row per run.
#  Usage (PowerShell):  .\run_matrix.ps1 [-Extra "--calibrate"]
# =============================================================================
param([string]$Extra = "", [int]$SeedOffset = 0)
Set-Location $PSScriptRoot
$rows = @()
$mazes = Get-ChildItem ..\mazes\generated\*.txt | Sort-Object Name
$i = 0
foreach ($m in $mazes) {
  $i++
  foreach ($cfg in @(@{mir=$false; err='1.0'}, @{mir=$true; err='0.85'}, @{mir=$false; err='1.15'})) {
    $a = @($m.FullName, '--quiet', '--seed', "$($i*7 + $SeedOffset)", '--model-error', $cfg.err)
    if ($cfg.mir) { $a += '--mirror' }
    if ($Extra) { $a += $Extra.Split(' ') }
    $o = & .\robot_sim.exe @a 2>&1 | Out-String
    $g = { param($p) if ($o -match $p) { $Matches[1] } else { '?' } }
    $rows += [pscustomobject]@{
      maze = $m.BaseName.Replace('mmrc26-island-10x10-', ''); mirror = $cfg.mir; motor = $cfg.err
      runs = & $g 'runs to goal\s+(\d+)'; rets = & $g 'returns\s+(\d+)'; aborts = & $g 'aborts\s+(\d+)'
      crash = & $g 'CRASHES\s+(\d+)'; clear = & $g 'min clearance\s+([\d\.]+)'
      alongMax = & $g 'max ([\d\.]+) mm, lateral'; wrong = & $g '(\d+) WRONG'; best = & $g 'best time\s+([\d\.]+)'
      score = & $g 'score = .* = (\d+)'; verdict = & $g 'VERDICT\s+(\w+)'
    }
  }
}
$rows | Format-Table -AutoSize | Out-String -Width 200
"PASS: " + ($rows | Where-Object verdict -eq 'PASS').Count + " / " + $rows.Count
"crashes total: " + (($rows | ForEach-Object { [int]$_.crash }) | Measure-Object -Sum).Sum