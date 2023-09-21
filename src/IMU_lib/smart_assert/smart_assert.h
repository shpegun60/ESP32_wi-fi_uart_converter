/**
 * @file    smart_assert.h
 * @brief   This library for check expressions and send message if expressions is true on RUNTIME and on DEBUG version.
 *          If NDEBUG is not defined some expressions has change to empty define. (programmer must uncommit NDEBUG if code version released)
 *
 *
 * @date
 */

#ifndef __SMART_ASSERT_H_
#define __SMART_ASSERT_H_

#include <assert.h>

// this library must include only .c file!!!!!!!!!!!!!!!!

//   C++ linking for mixed C++/C code
#ifdef __cplusplus
extern "C" {
#endif


//#define NDEBUG


void __M_Assert(const char* expr_str, unsigned char expr, const char* file, int line, const char* msg);
void __M_Error(const char* expr_str, unsigned char expr, const char* file, int line, const char* msg);
void __M_Error_variadic(const char* expr_str, unsigned char expr, const char* file, int line, const char* msg, ...);
void __M_Warning(const char* expr_str, unsigned char expr, const char* file, int line, const char* msg);
void __M_valueObserver(const char* msg, ...);

#ifndef NDEBUG

/*
 * ***********************************************************************************************************************************************
 *  defines for break program and dont save checking when NDEBUG
 * ***********************************************************************************************************************************************
 */

#   define M_Assert_Break(Expr, Msg, afterExpr)\
    do{\
        if (Expr) {\
            __M_Error((#Expr), (Expr), (__FILE__), (__LINE__), (Msg));\
            afterExpr;\
        }\
    }while(0L)


#   define M_Assert_Break_var(Expr, beforeExpr, afterExpr, Msg, arg...)\
    do{\
        if (Expr) {\
            beforeExpr;\
            __M_Error_variadic((#Expr), (Expr), (__FILE__), (__LINE__), (Msg), ##arg);\
            afterExpr;\
        }\
    }while(0L)

/*
 * ***********************************************************************************************************************************************
 *  defines for break program and save checking when NDEBUG (WARNING!!! only after breakExpr saving)
 * ***********************************************************************************************************************************************
 */

#   define M_Assert_BreakSaveCheck(Expr, Msg, afterExpr)\
    do{\
        if (Expr) {\
            __M_Error((#Expr), (Expr), (__FILE__), (__LINE__), (Msg));\
            afterExpr;\
        }\
    }while(0L)

/*
 * ***********************************************************************************************************************************************
 *  defines no break program
 * ***********************************************************************************************************************************************
 */

#   define M_Assert_Warning(Expr, Msg)\
    do{\
        if (Expr) {\
            __M_Warning((#Expr), (Expr), (__FILE__), (__LINE__), (Msg));\
        }\
    }while(0L)

#   define M_Assert_WarningSaveCheck(Expr, Msg, afterExpr)\
    do{\
        if (Expr) {\
            __M_Warning((#Expr), (Expr), (__FILE__), (__LINE__), (Msg));\
            afterExpr;\
        }\
    }while(0L)

#define M_Assert_SafeFunctionCall(foo, ...)\
    do {\
        if(foo) {\
            foo(__VA_ARGS__);\
        } else {\
            __M_Warning((#foo), (0), (__FILE__), (__LINE__), ("NO valid function"));\
        }\
    } while(0L);



#else
#   define M_Assert_Break(Expr, Msg, afterExpr)
#   define M_Assert_Break_par(Expr, beforeExpr, afterExpr, Msg, arg...)

#   define M_Assert_BreakSaveCheck(Expr, Msg, afterExpr)\
    do{\
        if (Expr) {\
            afterExpr;\
        }\
    }while(0L)

#   define M_Assert_Warning(Expr, Msg)

#   define M_Assert_WarningSaveCheck(Expr, Msg, afterExpr)\
    do{\
        if (Expr) {\
            afterExpr;\
        }\
    }while(0L)

#define M_Assert_SafeFunctionCall(foo, ...)\
    do {\
        if(foo) {\
            foo(__VA_ARGS__);\
        }\
    } while(0L);
#endif





#define PRINT_ONCE(...) \
    do{ \
        static int __printed = 0; \
        if(!__printed) { \
            printf(__VA_ARGS__); \
            __printed=1; \
        } \
    }while(0L)


#define TO_TXT(a) #a

#ifdef __cplusplus
}
#endif

#endif // __SMART_ASSERT
