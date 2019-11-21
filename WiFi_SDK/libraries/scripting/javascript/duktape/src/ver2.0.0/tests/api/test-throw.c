#if defined(WICED)
#include "wiced_duktape_tests_api_internal.h"
#endif

/*===
*** test_basic (duk_safe_call)
==> rc=1, result='throw me'
*** test_return (duk_safe_call)
==> rc=1, result='throw me too'
===*/

static duk_ret_t test_basic(duk_context *ctx, void *udata) {
	(void) udata;

	duk_push_string(ctx, "throw me");
	duk_throw(ctx);
	return 0;
}

static duk_ret_t test_return(duk_context *ctx, void *udata) {
	(void) udata;

	duk_push_string(ctx, "throw me too");
	return duk_throw(ctx);
}

#if defined(WICED)
void wiced_duktape_tests_api_throw(duk_context *ctx) {
#else
void test(duk_context *ctx) {
#endif
	TEST_SAFE_CALL(test_basic);
	TEST_SAFE_CALL(test_return);
}
