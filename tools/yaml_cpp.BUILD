# -*- python -*-

load(
    "@drake//tools:install.bzl",
    "cmake_config",
    "install",
    "install_cmake_config",
)

package(default_visibility = ["//visibility:public"])

public_headers = glob([
    "include/**/*.h",
])

cc_library(
    name = "yaml_cpp",
    srcs = glob([
        "src/**/*.cpp",
        "src/**/*.h",
    ]),
    hdrs = public_headers,
    includes = ["include"],
)

CMAKE_PACKAGE = "yaml-cpp"

cmake_config(
    package = CMAKE_PACKAGE,
    script = "@drake//tools:yaml_cpp-create-cps.py",
    version_file = "CMakeLists.txt",
)

# Creates rule :install_cmake_config.
install_cmake_config(package = CMAKE_PACKAGE)

install(
    name = "install",
    workspace = CMAKE_PACKAGE,
    targets = [":yaml_cpp"],
    hdrs = public_headers,
    hdr_dest = "include",
    hdr_strip_prefix = ["include"],
    docs = ["LICENSE"],
    deps = [":install_cmake_config"],
)
