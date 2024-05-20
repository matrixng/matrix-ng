# MATRIX-ng

This project aims to provide after-market support for the MATRIX Labs Creator (and possibly the MATRIX Labs Voice).

WIP

## Status

| | |
| - | - |  
| [Kernel modules](./kernel-modules/README.md) | :interrobang: | 
| [HAL](./hal/README.md) | :x: |
| [Init](./init/README.md) | :interrobang: |
| [Firmware](./firmware/README.md) | :interrobang: |
| [Tools](./tools/README.md) | :interrobang: |
| Debian packaging | :interrobang: |

<details>
    <summary>Legend</summary>

| | |
| - | - |  
| :x: | Not working |
| :white_check_mark: | Working |
| :question: | Unknown status |
| :interrobang: | Work in progress |
</details>

# Further information

Since the last official update from MATRIX Labs in 2021, Raspbian, as well as its surrounding support packages for low level communications (GPIO, PWM, etc.) have undergone significant changes.

These are, in no particular order:

1. `gpio-sysfs` has been replaced with `libgpiod`. This means that a large chunk of the original scripts are not compatible anymore, however rewriting most of those calls is straightforward.
2. `openocd` has also received a number of updates that makes it hard to compare the changes made by MATRIX Labs. The primary goal is to create appropriate targets that can be used with Vanilla OpenOCD and not rely on the aforementioned changes at all.
3. `xc3sprog` needs to be reviewed to determine the level of compatibility
4. `wiringpi` has been deprecated, however a large chunk of the HAL relies on it. This will need to be rewritten using `libgpiod`
5. The Debian packaging is in the process of being completely rewritten, to alleviate the circular dependencies established in the original packages.
6. A new APT repository will be set up, hosted on GitHub, to avoid any future domain-related issues - as long as GitHub and this repo is accessible, so will be the packages