#if defined(WICED)
#include "wiced_duktape_tests_api_internal.h"
#endif

/*===
Hello world from Ecmascript!
Hello world from C!
===*/

#if defined(WICED)
void wiced_duktape_tests_api_hello_world(duk_context *ctx) {
#else
void test(duk_context *ctx) {
#endif
	duk_push_string(ctx, "print('Hello world from Ecmascript!');");
	duk_eval(ctx);
	printf("Hello world from C!\n");
}
