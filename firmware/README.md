# Firmware

This folder contains the source code for all the firmware bits of the MATRIX Creator (and in the future, the Voice).

Currently, the subfolders contain the latest, unmodified versions from the official MATRIX repos, incl. license and all the tidbits.

## Primary goal

The primary goal is to rewrite these sources to bare minimum while also upgrading to the latest base.


### MATRIX Creator FPGA

The FPGA on the Creator is a Xilinx Spartan 6 model. Due to the licensing of the Xilinx SDK, the possibility of complete integration is questionable.

### MATRIX Creator MCU

The MCU on the Creator is an Atmel ATSAM3S2C, with the current firmware being built on top of ChibiOS.

Further investigation is needed to verify the possibility of rewriting the firmware on a newer base.