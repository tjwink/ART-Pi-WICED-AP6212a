#if defined(WICED)
#include "wiced_duktape_tests_api_internal.h"
#endif

/*===
result: foo123true
===*/

#if defined(WICED)
void wiced_duktape_tests_api_concat(duk_context *ctx) {
#else
void test(duk_context *ctx) {
#endif
	duk_push_string(ctx, "foo");
	duk_push_int(ctx, 123);
	duk_push_true(ctx);
	duk_concat(ctx, 3);

	printf("result: %s\n", duk_get_string(ctx, -1));
	duk_pop(ctx);
}
