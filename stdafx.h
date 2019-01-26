// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#ifndef STDAFX_H_INCLUDED
#define STDAFX_H_INCLUDED

#if defined(_WINDOWS) || defined(WIN32)

#pragma once

#include <SDKDDKVer.h>

#define WIN32_LEAN_AND_MEAN             // Exclude rarely-used stuff from Windows headers
// Windows Header Files:
#include <windows.h>

//#if defined(_DEBUG)
//#define	_CRTDBG_MAP_ALLOC
//#include <crtdbg.h>
//#define DEBUG_NEW new(_NORMAL_BLOCK, __FILE__, __LINE__)
//#define new DEBUG_NEW
//#endif

#endif

#if defined(__ANDROID__)
#include <android/log.h>
#define LOG_TAG "ORB_SLAM2"
#define LOG(...) __android_log_print(ANDROID_LOG_INFO,LOG_TAG, __VA_ARGS__)
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG,LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR,LOG_TAG, __VA_ARGS__)
#else
#define LOG
#define LOGD
#define LOGE
#endif

#pragma warning(disable: 4819)
#pragma warning(disable: 4996)
#pragma warning(disable: 4244) // conversion from 'float' to 'int', possible loss of data
#pragma warning(disable: 4267) // conversion from 'size_t' to 'int', possible loss of data
#pragma warning(disable: 4305) // truncation from 'double' to 'float'
#pragma warning(disable: 4838) // conversion from 'double' to 'const float' requires a narrowing conversion

#endif // STDAFX_H_INCLUDED