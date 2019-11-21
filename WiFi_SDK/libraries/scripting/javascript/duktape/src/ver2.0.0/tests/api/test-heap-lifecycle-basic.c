#if defined(WICED)
#include "wiced_duktape_tests_api_internal.h"
#endif

/*===
duk_create_heap_default() succeeded
concat test: 'foobarquux'
destroyed successfully
still here
===*/

#if defined(WICED)
void wiced_duktape_tests_api_heap_lifecycle_basic(duk_context *ctx) {
#else
void test(duk_context *ctx) {
#endif
	duk_context *new_ctx;

	/*
	 *  Create a new heap with default handlers
	 *
	 *  (wrapper heap is not used in this testcase)
	 */

	new_ctx = duk_create_heap_default();
	if (new_ctx) {
		printf("duk_create_heap_default() succeeded\n");

		duk_push_string(new_ctx, "foo");
		duk_push_string(new_ctx, "bar");
		duk_push_string(new_ctx, "quux");
		duk_concat(new_ctx, 3);
		printf("concat test: '%s'\n", duk_get_string(new_ctx, -1));
	} else {
		printf("context allocation failed\n");
	}

	duk_destroy_heap(new_ctx);
	new_ctx = NULL;
	printf("destroyed successfully\n");

	/*
	 *  NULL free is a no-op
	 */

	duk_destroy_heap(NULL);
	printf("still here\n");
}
