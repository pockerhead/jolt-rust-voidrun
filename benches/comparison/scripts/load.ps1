# Prints the CPU use of other processes over a 5 s window, as a percentage of the whole machine,
# and the heaviest of them. The comparison binaries (any path containing "comparison"), the Idle
# process and this PowerShell are excluded. A process is
# matched across the two snapshots by id and start time, so a reused id is not compared.
$own = 'comparison'
$cores = (Get-CimInstance Win32_ComputerSystem).NumberOfLogicalProcessors
function Snap {
    Get-Process | Where-Object { $_.Id -ne $PID -and $_.Id -ne 0 -and -not ($_.Path -and $_.Path -like "*$own*") } |
        ForEach-Object {
            $start = try { $_.StartTime.Ticks } catch { 0 }
            [pscustomobject]@{ key = "$($_.Id)/$start"; name = $_.ProcessName; cpu = $_.TotalProcessorTime.TotalSeconds }
        }
}
$a = Snap; Start-Sleep -Seconds 5; $b = Snap
$byKey = @{}; foreach ($p in $a) { $byKey[$p.key] = $p.cpu }
$rows = foreach ($p in $b) {
    if ($byKey.ContainsKey($p.key)) {
        $d = [Math]::Max(0, $p.cpu - $byKey[$p.key])
        [pscustomobject]@{ name = $p.name; pct = $d / 5 / $cores * 100 }
    }
}
$total = ($rows | Measure-Object pct -Sum).Sum
$top = ($rows | Sort-Object pct -Descending | Select-Object -First 8 | ForEach-Object { "{0}={1:N1}%" -f $_.name, $_.pct }) -join ' '
"{0:N1} {1}" -f $total, $top
