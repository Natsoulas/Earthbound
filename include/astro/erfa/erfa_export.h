#ifndef ERFA_EXPORT_H
#define ERFA_EXPORT_H

#ifdef _WIN32
    #ifdef BUILDING_ERFA
        #define ERFA_EXPORT __declspec(dllexport)
    #else
        #define ERFA_EXPORT __declspec(dllimport)
    #endif
#else
    #define ERFA_EXPORT
#endif

#endif // ERFA_EXPORT_H
