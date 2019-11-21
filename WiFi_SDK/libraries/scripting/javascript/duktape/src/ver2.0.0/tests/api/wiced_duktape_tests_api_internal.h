/* Header derived from API_TEST_HEADER variable in runtests/runtests.js */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <limits.h>  /* INT_MIN, INT_MAX */
#include "duktape.h"

#define  TEST_SAFE_CALL(func)  do { \
        duk_ret_t _rc; \
        printf("*** %s (duk_safe_call)\n", #func); \
        fflush(stdout); \
        _rc = duk_safe_call(ctx, (func), NULL, 0, 1); \
        printf("==> rc=%d, result='%s'\n", (int) _rc, duk_safe_to_string(ctx, -1)); \
        fflush(stdout); \
        duk_pop(ctx); \
    } while (0)

#define  TEST_PCALL(func)  do { \
        duk_ret_t _rc; \
        printf("*** %s (duk_pcall)\n", #func); \
        fflush(stdout); \
        duk_push_c_function(ctx, (func), 0); \
        _rc = duk_pcall(ctx, 0); \
        printf("==> rc=%d, result='%s'\n", (int) _rc, duk_safe_to_string(ctx, -1)); \
        fflush(stdout); \
        duk_pop(ctx); \
    } while (0)

#line 1
