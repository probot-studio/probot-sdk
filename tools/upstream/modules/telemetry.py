"""telemetry: the full C++ module including the Discard/Multi/Mock backends
(design/reports/telemetry_tunables.md). Python/Java bindings are not vendored."""

NAME = "telemetry"
SPARSE = ["telemetry"]

_N = "telemetry/src/main/native"

RULES = [
    dict(kind="header", src=f"{_N}/include", dst="src", include_base="src", include=["*"]),
    dict(kind="source", src=_N, dst="src/prosdk/impl/telemetry", include=["cpp/*.cpp"]),
    dict(kind="test", src="telemetry/src/test/native/cpp", dst="test/telemetry/cpp",
         include=["*"]),
]
