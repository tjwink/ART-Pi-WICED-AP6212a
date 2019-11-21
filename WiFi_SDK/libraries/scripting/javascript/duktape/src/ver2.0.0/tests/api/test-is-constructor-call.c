#if defined(WICED)
#include "wiced_duktape_tests_api_internal.h"
#endif

/*===
*** test_1 (duk_safe_call)
duk_is_constructor_call: 0
duk_is_constructor_call: 1
==> rc=0, result='undefined'
===*/

static duk_ret_t my_func(duk_context *ctx) {
	printf("duk_is_constructor_call: %d\n", (int) duk_is_constructor_call(ctx));
	return 0;
}

static duk_ret_t test_1(duk_context *ctx, void *udata) {
	(void) udata;

	duk_push_c_function(ctx, my_func, 0);

	duk_dup(ctx, 0);   /* -> [ func func ] */
	duk_call(ctx, 0);  /* -> [ func ret ] */
	duk_pop(ctx);      /* -> [ func ] */

	duk_dup(ctx, 0);   /* -> [ func func ] */
	duk_new(ctx, 0);   /* -> [ func ret ] */
	duk_pop(ctx);      /* -> [ func ] */

	duk_pop(ctx);

	return 0;
}

#if defined(WICED)
void wiced_duktape_tests_api_is_constructor_call(duk_context *ctx) {
#else
void test(duk_context *ctx) {
#endif
	TEST_SAFE_CALL(test_1);
}
