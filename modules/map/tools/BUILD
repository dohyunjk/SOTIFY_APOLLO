load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    targets = [
      ":map_tool",
      ":map_xysl",
      ":refresh_default_end_way_point",
      ":sim_map_generator",
      ":adjust_speed_limit",
      ":proto_map_generator",
      ":bin_map_generator",
      ":quaternion_euler",
    ],
    runtime_dest = "modules/map/tools",
)

cc_binary(
    name = "map_tool",
    srcs = ["map_tool.cc"],
    deps = [
        "//cyber/common:file",
        "//cyber/common:log",
        "//modules/common/configs:config_gflags",
        "//modules/map/hdmap:hdmap_util",
        "//modules/map/proto:map_cc_proto",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_binary(
    name = "map_xysl",
    srcs = ["map_xysl.cc"],
    deps = [
        "//modules/common/configs:config_gflags",
        "//modules/common/util",
        "//modules/map/hdmap:hdmap_util",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_binary(
    name = "refresh_default_end_way_point",
    srcs = ["refresh_default_end_way_point.cc"],
    deps = [
        "//cyber/common:file",
        "//cyber/common:log",
        "//modules/map/hdmap:hdmap_util",
        "//modules/routing/proto:poi_cc_proto",
        "//modules/routing/proto:routing_cc_proto",
    ],
)

cc_binary(
    name = "sim_map_generator",
    srcs = ["sim_map_generator.cc"],
    deps = [
        "//cyber/common:file",
        "//cyber/common:log",
        "//modules/common/configs:config_gflags",
        "//modules/common/util:points_downsampler",
        "//modules/map/hdmap:hdmap_util",
        "//modules/map/hdmap/adapter:opendrive_adapter",
        "//modules/map/proto:map_cc_proto",
        "@com_github_gflags_gflags//:gflags",
        "@com_google_absl//absl/strings",
    ],
)

cc_binary(
    name = "adjust_speed_limit",
    srcs = ["adjust_speed_limit.cc"],
    deps = [
        "//cyber/common:file",
        "//cyber/common:log",
        "//modules/common/configs:config_gflags",
        "//modules/map/hdmap:hdmap_util",
        "//modules/map/hdmap/adapter:opendrive_adapter",
        "//modules/map/proto:map_cc_proto",
        "@com_github_gflags_gflags//:gflags",
        "@com_google_absl//absl/strings",
    ],
)

cc_binary(
    name = "proto_map_generator",
    srcs = ["proto_map_generator.cc"],
    deps = [
        "//cyber/common:file",
        "//cyber/common:log",
        "//modules/map/hdmap:hdmap_util",
        "//modules/map/hdmap/adapter:opendrive_adapter",
        "//modules/map/proto:map_cc_proto",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_binary(
    name = "bin_map_generator",
    srcs = ["bin_map_generator.cc"],
    deps = [
        "//cyber/common:file",
        "//cyber/common:log",
        "//modules/map/hdmap:hdmap_util",
        "//modules/map/proto:map_cc_proto",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_binary(
    name = "quaternion_euler",
    srcs = ["quaternion_euler.cc"],
    deps = [
        "//modules/common/math",
        "//modules/common/util",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cpplint()
