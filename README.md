# librmd

[![.github/workflows/presubmit.yml](https://github.com/libhal/librmd/actions/workflows/presubmit.yml/badge.svg?branch=main)](https://github.com/libhal/librmd/actions/workflows/presubmit.yml)

Drivers for the LPC40xx series of microcontrollers conforming to the libhal
interface specification.

# [📚 Software APIs](https://libhal.github.io/librmd/api)

# 📥 Install

## [Install libhal Prerequisites](https://github.com/libhal/libhal/blob/main/docs/prerequisites.md)

## [Install libarmcortex Prerequisites](https://github.com/libhal/libarmcortex/blob/main/docs/prerequisites.md)

## Install using conan via from Conan Center Index

For future use. `librmd` is not currently on the Conan Center Index.

```bash
conan install librmd
```

## Install using conan via libhal-trunk

Trunk represents the latest code on github.

In order to get the latest code remote version of this repo as well as its
dependencies, enter this command to add the `libhal-trunk` remote server to your
list.

This command will insert `libhal-trunk` as the first server to check before
checking the conan center index.
The second command will enable revision mode which is required to use
`libhal-trunk`.

```bash
conan remote add libhal-trunk https://libhal.jfrog.io/artifactory/api/conan/trunk-conan --insert
conan config set general.revisions_enabled=True
```

Now when you run

```
conan install librmd
```

The library will be pulled from the `libhal-trunk`.

## Building the `hello_world` app

The hello world app will send out "Hello, World\n" every second from UART0. To
see the output, use a UART to USB module or a logic analyzer to see the output.
The BAUD rate is set to 38400.

The following commands will create the build folder where the generated build
files will be placed.

```bash
cd hello_world
mkdir build
cd build
```

The following commands will build the `hello_world` app.

```
conan install ..
cmake ..
make
```
