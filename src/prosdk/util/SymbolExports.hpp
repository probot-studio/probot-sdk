// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#ifdef _WIN32
#ifdef _MSC_VER
#pragma warning(disable : 4251)
#endif

#ifdef WPILIB_EXPORTS
#ifdef __GNUC__
#define WPILIB_DLLEXPORT __attribute__((dllexport))
#else
#define WPILIB_DLLEXPORT __declspec(dllexport)
#endif

#elif defined(WPILIB_IMPORTS)

#ifdef __GNUC__
#define WPILIB_DLLEXPORT __attribute__((dllimport))
#else
#define WPILIB_DLLEXPORT __declspec(dllimport)
#endif

#else
#define WPILIB_DLLEXPORT
#endif

#else  // _WIN32

#ifdef WPILIB_EXPORTS
#define WPILIB_DLLEXPORT __attribute__((visibility("default")))
#else
#define WPILIB_DLLEXPORT
#endif

#endif  // _WIN32

// prosdk: explicit template instantiation export helpers. prosdk is a static library and never
// builds on Windows, so the export goes at the declaration (upstream's "DEFAULT" style) and the
// definition takes nothing. The upstream style-detection machinery is removed because it
// dispatches through the token DEFAULT, which Arduino.h defines as a macro.
//
//   extern template class EXPORT_TEMPLATE_DECLARE(FOO_EXPORT) foo<bar>;
//   template class EXPORT_TEMPLATE_DEFINE(FOO_EXPORT) foo<bar>;
#define EXPORT_TEMPLATE_DECLARE(export) export
#define EXPORT_TEMPLATE_DEFINE(export)
