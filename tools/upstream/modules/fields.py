"""fields: FIRST field metadata and built-in AprilTag layouts (`prosdk::fields::Field`, `FieldTag`,
`FieldImage`, `GetField(FieldId)`, `GetFields()`, `GetFieldName()`). The 2027 successor of
`AprilTagFieldLayout` / `AprilTagFields` (they no longer exist upstream).

The field JSON files are compiled in as resource sources (rule kind "resource": the code upstream
generates at build time with cmake/scripts/GenResource.cmake). The field images are not vendored
(upstream builds them only into the optional `fieldimages` library, WPILIB_WITH_GUI);
`GetFieldImages()`/`GetFieldImage()` are hand-written in src/prosdk/platform/FieldImages.cpp and
report no image. Field(path)/Serialize(path) read and write files through patch fields/0001."""

NAME = "fields"
SPARSE = ["fields"]

_F = "fields/src/main/native"
_G = "fields/src/generated/main/native"

_IMAGES = "field images (PNG/JPG, 9.3 MB, about 55 MB as a C array): upstream builds them only " \
          "into the optional fieldimages library (WPILIB_WITH_GUI); the robot has no display " \
          "and the flash has no room. GetFieldImages()/GetFieldImage() report no image " \
          "(hand-written src/prosdk/platform/FieldImages.cpp)"

RULES = [
    # public headers (hand-written upstream + the generated FieldId registry)
    dict(kind="header", src=f"{_F}/include", dst="src", include_base="src", include=["*"]),
    dict(kind="header", src=f"{_G}/include", dst="src", include_base="src", include=["*"]),
    # sources (built into libprosdk.a)
    dict(kind="source", src=_F, dst="src/prosdk/impl/fields", include=["cpp/*.cpp"]),
    dict(kind="source", src=_G, dst="src/prosdk/impl/fields/generated",
         include=["cpp/fields/fields.cpp"]),
    # the field JSON files as resource sources: GetResource_<name>_json() in prosdk::fields
    dict(kind="resource", src=f"{_F}/resources/org/wpilib/fields",
         dst="src/prosdk/impl/fields/resources", include=["*.json"],
         resource={"prefix": "FIELDS", "namespace": "wpi::fields"}),
    # host unit tests (Catch2)
    dict(kind="test", src="fields/src/test/native/cpp", dst="test/fields/cpp", include=["*"]),
]

# Not vendored at all, with reasons.
NOT_VENDORED = {
    f"{_F}/resources/org/wpilib/fields/**/*.png": _IMAGES,
    f"{_F}/resources/org/wpilib/fields/**/*.jpg": _IMAGES,
    f"{_G}/cpp/fieldimages/field_images.cpp": "registry of the field images (not vendored, "
                                              "see above); replaced by FieldImages.cpp",
    "fields/src/generate": "Jinja templates, generator input (fields.json) and the official "
                           "AprilTag CSVs; their output (src/generated, the field JSON files) is "
                           "committed upstream and vendored instead",
    "fields/generate_fields.py": "generator of src/generated (output vendored)",
    "fields/convert_apriltag_layouts.py": "converts the official CSVs into the field JSON files "
                                          "(output vendored)",
    "fields/src/test/resources": "custom layout/image files used only by the Java tests",
    "fields/src/main/java": "Java API (no JVM on the MCU)",
    "fields/src/main/python": "Python bindings",
    "fields/{BUILD.bazel,CMakeLists.txt,build.gradle,fields-config.cmake.in,generate_fields.bzl,publish.gradle,robotpy_*.bzl}":
        "upstream build files (Bazel/CMake/Gradle/RobotPy packaging); prosdk builds with tools/build",
    "fields/src/test/native/BUILD.bazel": "upstream build files (Bazel/CMake/Gradle/RobotPy packaging); prosdk builds with tools/build",
    "fields/README.md": "upstream readme of the fields project",
    "fields/src/generated/main/java": "generated Java field registry (no JVM on the MCU)",
    "fields/src/test/java": "Java tests (no JVM on the MCU)",
    "fields/src/test/python": "Python tests",
}
