# -*- python -*-
# Copyright 2016 Toyota Research Institute.  All rights reserved.

cc_library(
    name = "maliput",
#LATER#    srcs = glob(["**/*.cc"], exclude=["**/test/*.cc"]),
#LATER#    hdrs = glob(["**/*.h"], exclude=["**/test/*.h"]),
    srcs = glob(["geometry_api/*.cc",
                 "monolane/*.cc",
                 "utility/*.cc"], exclude=["**/test/*.cc"]),
    hdrs = glob(["geometry_api/*.h",
                 "monolane/*.h",
                 "utility/*.h"], exclude=["**/test/*.h"]),
###    defines = ["GLM_FORCE_RADIANS"],
###    copts = ["-I/usr/include/ImageMagick"],
###    linkopts = ["-lGLEW", "-lglfw", "-lGL", "-lproj", "-lz",
###                "-lMagickCore",],
    data = glob(["data/**"]),
    deps = ["@boost//:boost"],
    visibility = ["//visibility:public"],
    )

cc_test(
    name = "maliput_gtest",
    srcs = [
        "monolane/test/hodgepodge.cc",
        ],
#LATER#    data = glob(["monolane/test/data/**"]),
    deps = [":maliput", "@gtest//:main",],
    size = "small",
    )

cc_test(
    name = "maliput_obj_gtest",
    size = "small",
    srcs = [
        "utility/test/test_generate_obj.cc",
        ],
    data = glob(["data/**"]),
    deps = [":maliput", "@gtest//:main"])

#LATER#cc_binary(
#LATER#    name = "maliput_gl_gtest",
#LATER#    srcs = [
#LATER#        "render/test/visible_window_test.cc",
#LATER#        "sensor_simulator/test/pcap_emitter.h",
#LATER#        "sensor_simulator/test/pcap_emitter.cc",
#LATER#        "sensor_simulator/test/sensor_manager_test.cc",
#LATER#        ],
#LATER#    data = glob(["data/**"]),
#LATER#    deps = [":maliput", "@gtest//:main"])
#LATER#
#LATER#cc_binary(
#LATER#    name = "performance_test",
#LATER#    srcs = [
#LATER#        "sensor_simulator/test/pcap_emitter.h",
#LATER#        "sensor_simulator/test/pcap_emitter.cc",
#LATER#        "sensor_simulator/test/performance_test.cc",
#LATER#        ],
#LATER#    data = glob(["data/**"]),
#LATER#    linkopts = ["-lboost_program_options"],
#LATER#    deps = [":maliput"])
