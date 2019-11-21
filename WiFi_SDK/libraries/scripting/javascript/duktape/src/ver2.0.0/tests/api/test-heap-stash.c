#if defined(WICED)
#include "wiced_duktape_tests_api_internal.h"
#endif

/*===
top: 0
top: 1
top: 0
value: 123
top: 0
===*/

#if defined(WICED)
void wiced_duktape_tests_api_heap_stash(duk_context *ctx) {
#else
void test(duk_context *ctx) {
#endif
	printf("top: %ld\n", (long) duk_get_top(ctx));
	duk_push_heap_stash(ctx);
	printf("top: %ld\n", (long) duk_get_top(ctx));

	duk_push_int(ctx, 123);
	duk_put_prop_string(ctx, -2, "myvalue");
	duk_pop(ctx);
	printf("top: %ld\n", (long) duk_get_top(ctx));

	duk_push_heap_stash(ctx);
	duk_get_prop_string(ctx, -1, "myvalue");
	printf("value: %ld\n", (long) duk_get_int(ctx, -1));
	duk_pop(ctx);
	duk_pop(ctx);
	printf("top: %ld\n", (long) duk_get_top(ctx));
}
