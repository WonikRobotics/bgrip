/* Allegro, Copyright 2021 Wonik Robotics Co., Ltd. All rights reserved.
*
* This library is commercial and cannot be redistributed, and/or modified
* WITHOUT ANY ALLOWANCE OR PERMISSION OF Wonik Robotics Co., LTD.
*/

/**
 * @file BGripDef.h
 * @author Wonik Robotics
 * @brief Definitions.
 */
#ifndef __BGRIPDEF_H__
#define __BGRIPDEF_H__

/* DLL export */
#if defined(WIN32) || defined(WINCE)
#    if defined(BGRIP_EXPORTS)
#        define BGRIPEXPORT __declspec(dllexport)
#    elif defined(BGRIP_IMPORTS)
#        define BGRIPEXPORT __declspec(dllimport)
#    else
#        define BGRIPEXPORT
#    endif
#else
#    define BGRIPEXPORT
#endif


#ifdef __cplusplus
#    define BGRIP_EXTERN_C_BEGIN    extern "C" {
#    define BGRIP_EXTERN_C_END    }
#else
#    define BGRIP_EXTERN_C_BEGIN
#    define BGRIP_EXTERN_C_END
#endif

#ifndef DEG2RAD
#define DEG2RAD (3.141592f/180.0f)
#endif

#ifndef RAD2DEG
#define RAD2DEG (180.0f/3.141592f)
#endif

#endif // __BGRIPDEF_H__
