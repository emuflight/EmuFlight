![EmuFlight](docs/assets/images/EMUFlightLogo-2022_Trans.png)

## EmuFlight is flight controller software (firmware) used to fly multi-rotor aircraft.
* This fork differs from Baseflight, Cleanflight and Betaflight in that it focuses on flight performance, innovative filtering, and leading-edge features.

## WARNING

* **DJI components bypass Configurator safety-checks. Do not attempt arming while connected to Configurator with LiPo plugged.  Always remove propellers and use a smoke-stopper for extra safety.**

## EmuFlight Releases

* [![Codacy Badge](https://api.codacy.com/project/badge/Grade/5422b54319254b6f9b6d01464ae9380c)](https://www.codacy.com/gh/emuflight/EmuFlight?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=emuflight/EmuFlight&amp;utm_campaign=Badge_Grade)
* https://github.com/emuflight/EmuConfigurator/releases
* https://github.com/emuflight/EmuFlight/releases

## Support and Developers Channel

* ![Discord](https://img.shields.io/discord/547211754845765635?label=EmuFlight%20Discord&style=for-the-badge&logo=discord&logoColor=white&color=00A3E0)
* To get support as well as the latest updates, join our discord at <https://discord.gg/BWqgBg3>.

## Features

* Support for targets that use the STM32 F7 and F4 processors
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

## Configuration Tool

* To configure EmuFlight you should use the EmuFlight-Configurator GUI tool (Windows/OSX/Linux) which can be found here: 
* https://github.com/emuflight/EmuConfigurator

## DJI OSD [In]Compatibility

* Setting PIDs and Rates (except for Feed Forward) is supported with the DJI OSD. The filtering menus (MISC PP, FILT PP, FILT GLB) are not currently supported and may result in unintended problems.

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

* If you want to contribute financially on an ongoing basis, please consider becoming a Patreon member (https://www.patreon.com/EmuFlight).

## Developers

Contribution of bugfixes and new features is encouraged. Please be aware that we have a thorough review process for pull requests. Be prepared to explain what you want to achieve with your pull request.
Before starting to write code, please read our [development guidelines](docs/development/Development.md ) and [coding style definition](docs/development/CodingStyle.md).

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
