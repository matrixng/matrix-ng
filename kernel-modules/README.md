# MATRIX Creator & Voice DKMS

This folder contains the source code for the backbone of the MATRIX devices' communication, the kernel modules, in DKMS format.

## Status

| | |
| - | - |
| UART comms | :white_check_mark: |
| Sensors | :white_check_mark: |
| Everloop | :white_check_mark: |
| Microphones | :x: |

Currently, the whole of the audio subsystem needs to be reworked due to significant kernel API changes since the last official update.

All other modules have been updated to implement these changes, however the audio interfaces the `mic`, `codec` and `playback` drivers rely on have changed so much that most likely a completely new approach is needed.