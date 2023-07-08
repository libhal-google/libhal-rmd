# libhal-rmd

[![✅ Checks](https://github.com/libhal/libhal-rmd/actions/workflows/ci.yml/badge.svg)](https://github.com/libhal/libhal-rmd/actions/workflows/ci.yml)
[![Coverage](https://libhal.github.io/libhal-rmd/coverage/coverage.svg)](https://libhal.github.io/libhal-rmd/coverage/)
[![Codacy Badge](https://app.codacy.com/project/badge/Grade/b084e6d5962d49a9afcb275d62cd6586)](https://www.codacy.com/gh/libhal/libhal-rmd/dashboard?utm_source=github.com&utm_medium=referral&utm_content=libhal/libhal-rmd&utm_campaign=Badge_Grade)
[![GitHub stars](https://img.shields.io/github/stars/libhal/libhal-rmd.svg)](https://github.com/libhal/libhal-rmd/stargazers)
[![GitHub forks](https://img.shields.io/github/forks/libhal/libhal-rmd.svg)](https://github.com/libhal/libhal-rmd/network)
[![GitHub issues](https://img.shields.io/github/issues/libhal/libhal-rmd.svg)](https://github.com/libhal/libhal-rmd/issues)
[![Latest Version](https://libhal.github.io/libhal-rmd/latest_version.svg)](https://github.com/libhal/libhal-rmd/blob/main/conanfile.py)

libhal device library for the series of the RMD smart motors from
[MyActuator](https://www.myactuator.com/).

## 📚 Software APIs & Usage

To learn about the available drivers and APIs see the
[Doxygen](https://libhal.github.io/libhal-rmd/api)
documentation page or look at the
[`include/libhal-rmd`](https://github.com/libhal/libhal-rmd/tree/main/include/libhal-rmd)
directory.

To see how each driver is used see the
[`demos/`](https://github.com/libhal/libhal-rmd/tree/main/demos) directory.

## 🧰 Setup

Following the
[🚀 Getting Started](https://libhal.github.io/2.1/getting_started/)
instructions.

## 📡 Installing Profiles

The `libhal-lpc40` profiles used for demos. To install them use the following
commands.

```bash
conan config install -sf conan/profiles/ -tf profiles https://github.com/libhal/libhal-armcortex.git
conan config install -sf conan/profiles/ -tf profiles https://github.com/libhal/libhal-lpc40.git
```

## 🏗️ Building Demos

To build demos, start at the root of the repo and execute the following command:

```bash
conan build demos -pr lpc4078 -s build_type=Debug
```

or for the `lpc4074`

```bash
conan build demos -pr lpc4074 -s build_type=Debug
```

## 🔌 Device Wiring & Hookup guide (CAN BUS)

1. Locate the CANTD (CAN Transmit Data) and CANRD (Can Receive Data) pins on
   your microcontroller port.
2. Connect CANTD and CANRD lines to a CAN transceiver.
3. Connect CAN transceiver's CANL and CANH lines and connect them to an motor's
   CANL and CANH lines.
4. Supply adequate power to the CAN transceiver and the smart motor.

## 📦 Adding `libhal-rmd` to your project

Add the following to your `requirements()` method:

```python
    def requirements(self):
        self.requires("libhal-rmd/[^2.0.0]")
```

## Contributing

See [`CONTRIBUTING.md`](CONTRIBUTING.md) for details.

## License

Apache 2.0; see [`LICENSE`](LICENSE) for details.

## Disclaimer

This project is not an official Google project. It is not supported by
Google and Google specifically disclaims all warranties as to its quality,
merchantability, or fitness for a particular purpose.
