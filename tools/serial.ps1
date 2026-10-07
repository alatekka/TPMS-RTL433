param([string]$Port = 'COM5', [int]$Seconds = 20, [switch]$Reset, [string[]]$Send = @(), [int]$SendDelayMs = 1500, [int]$FirstDelayMs = -1)
$p = New-Object System.IO.Ports.SerialPort $Port, 115200, 'None', 8, 'One'
$p.ReadTimeout = 500
$p.DtrEnable = $false
$p.RtsEnable = $false
$p.ReadBufferSize = 1048576
$p.Open()
if ($Reset) {
  # EN low via RTS, then release (normal boot)
  $p.RtsEnable = $true
  Start-Sleep -Milliseconds 120
  $p.RtsEnable = $false
}
$sb = New-Object System.Text.StringBuilder
$start = Get-Date
$end = $start.AddSeconds($Seconds)
$sendIdx = 0
if ($FirstDelayMs -lt 0) { $FirstDelayMs = $SendDelayMs }
$nextSend = $start.AddMilliseconds($FirstDelayMs)
while ((Get-Date) -lt $end) {
  if ($sendIdx -lt $Send.Count -and (Get-Date) -ge $nextSend) {
    foreach ($c in ($Send[$sendIdx] -split ";")) { $p.Write($c.Trim() + "`n"); Start-Sleep -Milliseconds 150 }
    $sendIdx++
    $nextSend = (Get-Date).AddMilliseconds($SendDelayMs)
  }
  try { $chunk = $p.ReadExisting(); if ($chunk) { [void]$sb.Append($chunk) } } catch {}
  Start-Sleep -Milliseconds 50
}
$p.Close()
$sb.ToString()
