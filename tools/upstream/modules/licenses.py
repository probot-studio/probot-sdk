"""licenses: upstream license texts, copied verbatim into licenses/ (referenced by NOTICE).
ThirdPartyNotices.txt carries the license of every thirdparty component WPILib ships (LLVM,
JSON for Classic C++, sigslot, double-conversion, hat-trie, nanopb, V8 export-template, Eigen,
GCEM, Sleipnir, ...)."""

NAME = "licenses"
SPARSE = []
TRANSFORMS = {"verbatim": True, "rename": False, "cxx23": False, "arduino_renames": False,
              "guard": False}

RULES = [
    dict(kind="license", src=".", dst="licenses/wpilib",
         include=["LICENSE.md", "ThirdPartyNotices.txt"]),
]
