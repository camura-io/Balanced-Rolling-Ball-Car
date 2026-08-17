$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

# This is the only approved flash sequence for this H750 board.
$programmer = 'C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe'
$hexPath = Join-Path $PSScriptRoot 'MDK-ARM\H750_single_stepper_board\H750_single_stepper_board.hex'

if (-not (Test-Path -LiteralPath $programmer -PathType Leaf)) {
    throw "STM32CubeProgrammer CLI not found: $programmer"
}

if (-not (Test-Path -LiteralPath $hexPath -PathType Leaf)) {
    throw "Firmware hex not found: $hexPath"
}

$hexFile = Get-Item -LiteralPath $hexPath
if ($hexFile.Length -eq 0) {
    throw "Firmware hex is empty: $hexPath"
}

Write-Host "Flashing: $($hexFile.FullName)"
Write-Host "Method: ST-LINK, SWD 4 MHz, hardware reset, verify"

# Do not change this connection sequence without explicit user approval.
& $programmer -c port=SWD freq=4000 mode=NORMAL reset=HWrst -w $hexPath -v
if ($LASTEXITCODE -ne 0) {
    throw "Flash or verification failed with exit code $LASTEXITCODE. Stop here; do not retry or change settings."
}

# Start the verified firmware with a physical hardware reset.
& $programmer -c port=SWD freq=4000 mode=NORMAL -hardRst
if ($LASTEXITCODE -ne 0) {
    throw "Firmware was written, but hardware reset failed with exit code $LASTEXITCODE."
}

Write-Host 'Flash verified and hardware reset completed.'
