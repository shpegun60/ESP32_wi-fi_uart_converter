#include "smart_assert.h"
#include <stdio.h>
#include <stdarg.h>

void __M_Assert(const char* expr_str, unsigned char expr, const char* file, int line, const char* msg)
{
    if (expr) {
        fprintf(stderr, "Assert failed:\t %s \nExpression:\t%s\nSource:\t\t%s, line %d\n", msg, expr_str,file, line);
        fflush(stderr);
        //printf("Assert failed:\t %s:\nExpected:\t%s\nSource:\t\t%s, line %d\n", msg, expr_str,file, line);
        abort();
    }
}

void __M_Error(const char* expr_str, unsigned char expr, const char* file, int line, const char* msg)
{
    (void)expr;
    fprintf(stderr, "Assert failed:\t %s \nExpression:\t%s\nSource:\t\t%s, line %d\n", msg, expr_str,file, line);
    fflush(stderr);
    //printf("Assert failed:\t %s:\nExpected:\t%s\nSource:\t\t%s, line %d\n", msg, expr_str,file, line);
    abort();
}

void __M_Error_variadic(const char* expr_str, unsigned char expr, const char* file, int line, const char* msg, ...)
{
    (void)expr;

    va_list args;
    va_start(args, msg);

    fprintf(stderr, "\nAssert failed:\t");
    vfprintf(stderr, msg, args);
    fprintf(stderr, "\n");

    fprintf(stderr, "Expression:\t %s \n", expr_str);
    fprintf(stderr, "Source:\t\t %s, line: %d\n", file, line);

    va_end(args);
    fflush(stderr);
    abort();
}

void __M_Warning(const char* expr_str, unsigned char expr, const char* file, int line, const char* msg)
{
    (void)expr;
    fprintf(stderr, "Assert failed:\t %s \nExpression:\t%s\nSource:\t\t%s, line %d\n\n", msg, expr_str,file, line);
    fflush(stderr);
    //printf("Assert failed:\t %s:\nExpected:\t%s\nSource:\t\t%s, line %d\n", msg, expr_str,file, line);
}


void __M_valueObserver(const char* msg, ...)
{
    va_list args;
    va_start(args, msg);
    fprintf(stdout, "\n-----------valueObserver:-----------> \n");
    vfprintf(stdout, msg, args);
    fprintf(stdout, "\n-----------end valueObserver----------- \n");
    fflush(stdout);
    va_end(args);
}


