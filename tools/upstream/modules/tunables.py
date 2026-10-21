"""tunables: the full C++ module including MockTunableBackend and Selectable
(design/reports/telemetry_tunables.md). Python/Java bindings are not vendored."""

NAME = "tunables"
SPARSE = ["tunables"]

_N = "tunables/src/main/native"

RULES = [
    dict(kind="header", src=f"{_N}/include", dst="src", include_base="src", include=["*"]),
    dict(kind="source", src=_N, dst="src/prosdk/impl/tunables", include=["cpp/*.cpp"]),
    dict(kind="test", src="tunables/src/test/native/cpp", dst="test/tunables/cpp",
         include=["*"]),
]
