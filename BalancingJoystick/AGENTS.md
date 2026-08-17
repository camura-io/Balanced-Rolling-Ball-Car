# H750 Single Stepper Board Agent Rules

## Mandatory Flash Rule

Before every STM32H750 flash operation, read `FLASH_WORKFLOW.md` and run only
`flash_h750_4mhz_hardware_reset.ps1` from this directory.

Do not substitute another ST-LINK connection mode, frequency, programmer, or
firmware artifact unless the user explicitly asks for that change.

Never use full-chip erase, Option Bytes changes, RDP/debug-lock operations,
J-Link, `mode=HWRSTPULSE`, or a manually specified `mode=UR` for this project.
If the prescribed flash script fails, stop after that attempt, keep the source
and target configuration unchanged, report the complete error, and wait for
the user to check the board connection.

