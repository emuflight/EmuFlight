![EmuFlight](docs/assets/images/EMUFlightLogo-2022_Trans.png)

## EmuFlight is flight controller software (firmware) used to fly multi-rotor aircraft.
* This fork differs from Baseflight, Cleanflight and Betaflight in that it focuses on flight performance, innovative filtering, and leading-edge features.

> [!WARNING]
> Always remove propellers and use a smoke-stopper when bench-testing. Never arm with propellers attached while connected to a computer.

## EmuFlight Releases

* https://github.com/emuflight/EmuConfigurator/releases
* https://github.com/emuflight/EmuFlight/releases

## Support and Developers Channel

* ![Discord](https://img.shields.io/discord/547211754845765635?label=EmuFlight%20Discord&style=for-the-badge&logo=discord&logoColor=white&color=00A3E0)
* To get support as well as the latest updates, join our discord at <https://discord.gg/BWqgBg3>.

## Features

* Support for targets that use the STM32 H7, F7, and F4 processors
* Support for HelioRC Spring FC's
* IMUF (Kalman-based) Filtering for all FC's
* Additional DShot levels: 1200, 2400, 4800
* OSD based configuration without needing third-party OSD software/firmware/comm devices
* Per-Axis Lowpass filters
* Feathered PIDs
* Error-based Boosts for P, I, & D. (EmuBoost/DBoost)
* i-Decay
* Throttle Point Attenuation (TPA) for P, I, & D
* Stick Point Attenuation (SPA)
* Advanced Dynamic Gyro & D-Term Filters
* Smith-Predictor on Gyro
* Optional ABG Filters for Gyro and DTerm
* PT(n) for LPF and RC-Smoothing
* Rate-Dynamics (Stick-feel modifier)
* Angle-mode 2.0
* Dual Axis Steering
* NFE racer mode
* Fixed yaw PIDsum limit
* LPF filter for voltage and current
* Motor output limit
* Motor output smoothing
* Thrust linearization
* Axis Lock
* and MUCH, MUCH more

## Installation & Documentation

* See: https://github.com/emuflight/EmuFlight/wiki
* CLI parameter reference: [docs/CLI/parameters-reference.md](docs/CLI/parameters-reference.md)

## Configuration Tool

* To configure EmuFlight you should use the EmuFlight-Configurator GUI tool (Windows/OSX/Linux) which can be found here: 
* https://github.com/emuflight/EmuConfigurator

## HD Video System Compatibility

> [!CAUTION]
> Do not use HD video systems (HDZero, Walksnail, or DJI) to configure EmuFlight. These systems send Betaflight-formatted MSP; EmuFlight's PID, rate, and filter structs differ significantly from Betaflight's, and MSP configuration writes are not source-gated. Sending configuration from any of these devices may silently corrupt your tune. Use **EmuConfigurator**, the **CLI**, or the **EmuFlight OSD** for all configuration.

> [!TIP]
> Stock DJI goggles do not display OSD. [WTFOS](https://github.com/fpv-wtf/wtfos) enables EmuFlight OSD passthrough on DJI hardware and is the recommended solution for DJI users.

## Contributing

* Contributions are welcome and encouraged. You can contribute in many ways:

  - Documentation updates and corrections.
  - How-To guides - received help? Help others!
  - Bug reporting & fixes.
  - New feature ideas & suggestions.
  - Coding and Pull Requests.

* The best place to start is the EmuFlight Discord (https://discord.gg/BWqgBg3). Next place is the github issue tracker:
  - https://github.com/emuflight/EmuFlight/issues
  - https://github.com/emuflight/EmuConfigurator/issues

* Before creating new issues please check to see if there is an existing one.

## Build Environment

| Requirement | Version |
|-------------|---------|
| ARM GCC toolchain | 9.x – 13.x (xPack ARM GCC recommended) |
| Python | 3.12+ |
| GNU Make | 4.x+ |

Automated builds and unit tests run via **GitHub Actions** on every push and pull request — see [`.github/workflows/`](.github/workflows/) for CI configuration.

To install the ARM toolchain:
```bash
make arm_sdk_install   # downloads xPack ARM GCC into tools/
make test              # run unit tests
```

## Development Guidelines

EmuFlight development guidance lives in [`.github/instructions/`](.github/instructions/):

- Unit test development best practices

Read these before opening a pull request.

## Developers

Contribution of bugfixes and new features is encouraged. Please be aware that we have a thorough review process for pull requests. Be prepared to explain what you want to achieve with your pull request.
Before starting to write code, review the [Development Guidelines](#development-guidelines) section above.

## Open Source / Contributors

EmuFlight is software that is **open source** and is available free of charge without warranty to all users.

EmuFlight is forked from Betaflight, so thanks goes to all those whom have contributed to Cleanflight, Betaflight, Butterflight and their origins.

Origins for this fork (Thanks!):
* **Alexinparis** (for MultiWii),
* **timecop** (for Baseflight),
* **Dominic Clifton** (for Cleanflight),
* **borisbstyle** (for Betaflight),
* **Sambas** (for the original STM32F4 port), and
* **Marinus** (for forking IMUF thus beginning EmuFlight).

The EmuFlight Configurator is forked from Betaflight Configurator and its origins.

Origins for EmuFlight Configurator:
* **Dominic Clifton** (for Cleanflight configurator), and
* **ctn** (for the original Configurator).

Big thanks to current and past contributors:
* Budden, Martin (martinbudden)
* Bardwell, Joshua (joshuabardwell)
* Blackman, Jason (blckmn)
* ctzsnooze
* Höglund, Anders (andershoglund)
* Ledvina, Petr (ledvinap) - **IO code awesomeness!**
* kc10kevin
* Keeble, Gary (MadmanK)
* Keller, Michael (mikeller) - **Configurator brilliance**
* Kravcov, Albert (skaman82) - **Configurator brilliance**
* MJ666
* Nathan (nathantsoi)
* ravnav
* sambas - **bringing us the F4**
* savaga
* Stålheim, Anton (KiteAnton)
* Tim Sweet, Kaylin Doerr, Shawn Loftin - **Filtering improvements, IMUF**

And many many others either mentioned in release notes, GitHub history, or possibly haven't been mentioned.... Thank you!
