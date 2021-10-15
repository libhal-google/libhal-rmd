from conans import ConanFile


class librmdx_conan(ConanFile):
    name = "librmdx"
    version = "0.0.1"
    license = "Apache License Version 2.0"
    author = "Khalil Estell"
    url = "https://github.com/SJSU-Dev2/librmdx"
    description = "Driver for controlling RMD-X smart servos using libembeddedhal"
    topics = ("can", "canbus", "RMD-X", "hardware", "libembeddedhal")
    exports_sources = "CMakeLists.txt", "include/*"
    no_copy_source = True

    def package(self):
        self.copy("*.hpp")

    def package_id(self):
        self.info.header_only()

    def requirements(self):
        self.requires("libembeddedhal/0.0.1")
