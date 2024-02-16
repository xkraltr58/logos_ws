#ifndef MIDI_HARDWARE_VISIBILITY_CONTROL_H_
#define MIDI_HARDWARE_VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define MIDI_HARDWARE_EXPORT __attribute__((dllexport))
#define MIDI_HARDWARE_IMPORT __attribute__((dllimport))
#else
#define MIDI_HARDWARE_EXPORT __declspec(dllexport)
#define MIDI_HARDWARE_IMPORT __declspec(dllimport)
#endif
#ifdef MIDI_HARDWARE_BUILDING_DLL
#define MIDI_HARDWARE_PUBLIC MIDI_HARDWARE_EXPORT
#else
#define MIDI_HARDWARE_PUBLIC MIDI_HARDWARE_IMPORT
#endif
#define MIDI_HARDWARE_PUBLIC_TYPE MIDI_HARDWARE_PUBLIC
#define MIDI_HARDWARE_LOCAL
#else
#define MIDI_HARDWARE_EXPORT __attribute__((visibility("default")))
#define MIDI_HARDWARE_IMPORT
#if __GNUC__ >= 4
#define MIDI_HARDWARE_PUBLIC __attribute__((visibility("default")))
#define MIDI_HARDWARE_LOCAL __attribute__((visibility("hidden")))
#else
#define MIDI_HARDWARE_PUBLIC
#define MIDI_HARDWARE_LOCAL
#endif
#define MIDI_HARDWARE_PUBLIC_TYPE
#endif

#endif  // MIDI_HARDWARE_VISIBILITY_CONTROL_H_