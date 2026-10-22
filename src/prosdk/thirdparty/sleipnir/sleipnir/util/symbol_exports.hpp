// Copyright (c) Sleipnir contributors

#pragma once

#include <prosdk/platform/config.hpp>  // prosdk: configuration

#ifdef _WIN32
#ifdef _MSC_VER
#pragma warning(disable : 4251)
#endif

#ifdef SLEIPNIR_EXPORTS
#ifdef __GNUC__
#define SLEIPNIR_DLLEXPORT __attribute__((dllexport))
#else
#define SLEIPNIR_DLLEXPORT __declspec(dllexport)
#endif

#elif defined(SLEIPNIR_IMPORTS)

#ifdef __GNUC__
#define SLEIPNIR_DLLEXPORT __attribute__((dllimport))
#else
#define SLEIPNIR_DLLEXPORT __declspec(dllimport)
#endif

#else
#define SLEIPNIR_DLLEXPORT
#endif

#else  // _WIN32

#ifdef SLEIPNIR_EXPORTS
#define SLEIPNIR_DLLEXPORT __attribute__((visibility("default")))
#else
#define SLEIPNIR_DLLEXPORT
#endif

#endif  // _WIN32

// prosdk: explicit template instantiation export helpers. prosdk is a static library and never
// builds on Windows, so the export goes at the declaration (upstream's "DEFAULT" style) and the
// definition takes nothing. The upstream style-detection machinery is removed because it
// dispatches through the token DEFAULT, which Arduino.h defines as a macro. The definitions are
// token-identical to prosdk/util/SymbolExports.hpp (wpiutil/0001), so both headers can be
// included in one translation unit.
//
//   extern template class EXPORT_TEMPLATE_DECLARE(FOO_EXPORT) foo<bar>;
//   template class EXPORT_TEMPLATE_DEFINE(FOO_EXPORT) foo<bar>;
#define EXPORT_TEMPLATE_DECLARE(export) export
#define EXPORT_TEMPLATE_DEFINE(export)
