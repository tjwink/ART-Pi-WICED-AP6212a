#if defined(WICED)
#include "wiced_duktape_tests_api_internal.h"
#endif

/*
 *  duk_get_now()
 */

/*===
timestamp is between [2010-01-01,2030-01-01[: yes
===*/

#if defined(WICED)
void wiced_duktape_tests_api_get_now(duk_context *ctx) {
#else
void test(duk_context *ctx) {
#endif
	duk_double_t now;

	now = duk_get_now(ctx);

	/*
	> new Date(2010,1,1).valueOf()
	1264975200000
	> new Date(2030,1,1).valueOf()
	1896127200000
	*/

	printf("timestamp is between [2010-01-01,2030-01-01[: %s\n",
	       now >= 1264975200000.0 && now < 1896127200000.0 ? "yes" : "no");
}
