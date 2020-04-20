![EmuFlight](docs/assets/images/EmuFlight.png)

EmuFlight is flight controller software (firmware) used to fly multi-rotor craft.

This fork differs from Baseflight, Cleanflight and Betaflight in that it focuses on flight performance, innovative filtering, leading-edge feature additions, and wide target support.

## News

Currently working on the first official release of EmuFlight. The Pegasus configurator will also be our official configurator in the coming months.


To get the latest updates as well as support from us, you can join our discord at <https://discord.gg/gdP9CwE>.

## Features

EmuFlight has some of the following features:

* Multi-color RGB LED strip support (each LED can be a different color using variable length WS2811 Addressable RGB strips - use for Orientation Indicators, Low Battery Warning, Flight Mode Status, Initialization Troubleshooting, etc)
* DShot (150, 300, 600 and 1200), Multishot, and Oneshot (125 and 42) motor protocol support
* Blackbox flight recorder logging (to onboard flash or external microSD card where equipped)
* Support for targets that use the STM32 F7, F4 and F3 processors
* PWM, PPM, and Serial (SBus, SumH, SumD, Spektrum 1024/2048, XBus, etc) RX connection with failsafe detection
* Multiple telemetry protocols (CSRF, FrSky, HoTT smart-port, MSP, etc)
* RSSI via ADC - Uses ADC to read PWM RSSI signals, tested with FrSky D4R-II, X8R, X4R-SB, & XSR
* OSD support & configuration without needing third-party OSD software/firmware/comm devices
* OLED Displays - Display information on: Battery voltage/current/mAh, profile, rate profile, mode, version, sensors, etc
* In-flight manual PID tuning and rate adjustment
* Rate profiles and in-flight selection of them
* Configurable serial ports for Serial RX, Telemetry, ESC telemetry, MSP, GPS, OSD, Sonar, etc - Use most devices on any port, softserial included
* VTX support for Unify Pro and IRC Tramp
* Latest IMUF filtering technology
* and MUCH, MUCH more.

## Installation & Documentation

See: https://github.com/emuflight/EmuFlight/wiki

## Support and Developers Channel

There's a dedicated Discord chat channel here:

https://discord.gg/gdP9CwE

Etiquette: Don't ask to ask and please wait around long enough for a reply - sometimes people are out flying, asleep or at work and can't answer immediately.

## Configuration Tool

To configure EmuFlight you should use the EmuFlight-configurator GUI tool (Windows/OSX/Linux) which can be found here:

https://github.com/emuflight/EmuConfigurator

## Contributing

Contributions are welcome and encouraged. You can contribute in many ways:

* Documentation updates and corrections.
* How-To guides - received help? Help others!
* Bug reporting & fixes.
* New feature ideas & suggestions.

The best place to start is the EmuFlight Discord (registration [here](https://discord.gg/fXPP49e). Next place is the github issue tracker:

https://github.com/emuflight/EmuFlight/issues

https://github.com/emuflight/EmuConfigurator/issues

Before creating new issues please check to see if there is an existing one, search first otherwise you waste peoples time when they could be coding instead!

If you want to contribute to our efforts financially, please consider making a donation to us through PayPal (Not yet setup).

If you want to contribute financially on an ongoing basis, you should consider becoming a patron for us on Patreon (Not yet setup).

## Developers

Contribution of bugfixes and new features is encouraged. Please be aware that we have a thorough review process for pull requests, and be prepared to explain what you want to achieve with your pull request.
Before starting to write code, please read our [development guidelines](docs/development/Development.md ) and [coding style definition](docs/development/CodingStyle.md).

## EmuFlight Releases

[![Build Status](https://travis-ci.org/emuflight/EmuFlight.svg?branch=master)](https://travis-ci.org/emuflight/EmuFlight) [![Codacy Badge](https://api.codacy.com/project/badge/Grade/5422b54319254b6f9b6d01464ae9380c)](https://www.codacy.com/gh/emuflight/EmuFlight?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=emuflight/EmuFlight&amp;utm_campaign=Badge_Grade)

https://github.com/emuflight/EmuFlight/releases

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

And many many others who haven't been mentioned....
