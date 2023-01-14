# libhal-rmd

[![.github/workflows/presubmit.yml](https://github.com/libhal/libhal-rmd/actions/workflows/presubmit.yml/badge.svg?branch=main)](https://github.com/libhal/libhal-rmd/actions/workflows/presubmit.yml)

# [📚 Software APIs](https://libhal.github.io/libhal-rmd/api)

# 📥 Install

## [Install libhal Prerequisites](https://libhal.github.io/prerequisites/)

## Install using conan via from Conan Center Index

For future use. `libhal-rmd` is not currently on the Conan Center Index.

```bash
conan install libhal-rmd
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
conan install libhal-rmd
```

The library will be pulled from the `libhal-trunk`.
