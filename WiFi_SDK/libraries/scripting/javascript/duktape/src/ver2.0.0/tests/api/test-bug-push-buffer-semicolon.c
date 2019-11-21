#if defined(WICED)
#include "wiced_duktape_tests_api_internal.h"
#endif

/*
 *  https://github.com/svaarala/duktape/issues/500
 */

/*===
still here
===*/

#if defined(WICED)
void wiced_duktape_tests_api_bug_push_buffer_semicolon(duk_context *ctx) {
#else
void test(duk_context *ctx) {
#endif
	(void) (duk_push_buffer(ctx, 123, 0), "dummy");
	printf("still here\n");
}
