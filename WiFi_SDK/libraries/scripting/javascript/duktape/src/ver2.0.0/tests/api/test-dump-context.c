#if defined(WICED)
#include "wiced_duktape_tests_api_internal.h"
#endif

/*===
*** test_1 (duk_safe_call)
ctx: top=4, stack=[123,"foo\u1234bar",{foo:123,bar:[1,2,3]},[1,2,3]]
final top: 4
==> rc=0, result='undefined'
===*/

static duk_ret_t test_1(duk_context *ctx, void *udata) {
	(void) udata;

	duk_push_int(ctx, 123);
	duk_eval_string(ctx, "'foo\\u1234bar'");
	duk_eval_string(ctx, "({ foo: 123, bar: [ 1, 2, 3 ]})");
	duk_eval_string(ctx, "([ 1, 2, 3 ])");
	duk_push_context_dump(ctx);
	printf("%s\n", duk_safe_to_string(ctx, -1));
	duk_pop(ctx);
	printf("final top: %ld\n", (long) duk_get_top(ctx));
	return 0;
}

#if defined(WICED)
void wiced_duktape_tests_api_dump_context(duk_context *ctx) {
#else
void test(duk_context *ctx) {
#endif
	TEST_SAFE_CALL(test_1);
}
