# libhal-rmd

[![✅ Checks](https://github.com/libhal/libhal-rmd/actions/workflows/ci.yml/badge.svg)](https://github.com/libhal/libhal-rmd/actions/workflows/ci.yml)
[![coverage](https://libhal.github.io/libhal-rmd/coverage/coverage.svg)](https://libhal.github.io/libhal-rmd/coverage/)
[![Codacy Badge](https://app.codacy.com/project/badge/Grade/b084e6d5962d49a9afcb275d62cd6586)](https://www.codacy.com/gh/libhal/libhal-rmd/dashboard?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=libhal/libhal-rmd&amp;utm_campaign=Badge_Grade)
[![GitHub stars](https://img.shields.io/github/stars/libhal/libhal-rmd.svg)](https://github.com/libhal/libhal-rmd/stargazers)
[![GitHub forks](https://img.shields.io/github/forks/libhal/libhal-rmd.svg)](https://github.com/libhal/libhal-rmd/network)
[![GitHub issues](https://img.shields.io/github/issues/libhal/libhal-rmd.svg)](https://github.com/libhal/libhal-rmd/issues)
[![Latest Version](https://libhal.github.io/libhal-rmd/latest_version.svg)](https://github.com/libhal/libhal-rmd/blob/main/conanfile.py)
[![ConanCenter Version](https://repology.org/badge/version-for-repo/conancenter/libhal-rmd.svg)](https://conan.io/center/libhal-rmd)


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

## Contributing

See [`CONTRIBUTING.md`](CONTRIBUTING.md) for details.

## License

Apache 2.0; see [`LICENSE`](LICENSE) for details.

## Disclaimer

This project is not an official Google project. It is not supported by
Google and Google specifically disclaims all warranties as to its quality,
merchantability, or fitness for a particular purpose.

