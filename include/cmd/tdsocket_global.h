#pragma once
/*
#include <QtCore/qglobal.h>
#ifndef BUILD_STATIC
# if defined(TDCORE_LIB)
#  define TDCORE_EXPORT Q_DECL_EXPORT
# else
#  define TDCORE_EXPORT Q_DECL_IMPORT
# endif
#else
# define TDCORE_EXPORT
#endif
*/
#if defined(_WIN32) || defined(_MSC_VER)
#ifdef TDSOCKET_LIB  //����ͨ��-DDEEPNETAPI_EXPORT������ñ���
#define TDSOCKET_EXPORT __declspec(dllexport)   
#else
#define TDSOCKET_EXPORT __declspec(dllimport)  
#endif
#elif defined(__linux__) || defined(__APPLE__)  
#ifdef TDSOCKET_LIB
#define TDSOCKET_EXPORT __attribute__ ((visibility ("default")))   //�Կ⿪������˵��ֻϣ����һЩ��Ҫ�����ĺ����������ʹ���ߣ������Ľ������أ������ڵ������ʱ����Ҫ�ڱ���ѡ�������-fvisibility=hidden��ֻ�к���������ǰ����__attribute__ ((visibility ("default")))�Ŀ��Ա�������ʹ�ã�������ȫ�����ء�ͨ���ڱ���ѡ�������-fvisibility=hidden������������������أ�ֻ��__attribute__ ((visibility ("default")))�ĺ����ᱣ��Ĭ�ϣ���ȫ�ֿɼ����������ʱ����-fvisibility=hidden���򲻹ܺ���������ǰ�Ƿ����__attribute__ ((visibility ("default")))������ȫ�ֿɼ�����ΪĬ����default
#else
#define TDSOCKET_EXPORT
#endif
#endif
