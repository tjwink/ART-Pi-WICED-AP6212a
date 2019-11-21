#if defined(WICED)
/* Custom provider for getting current time */
extern duk_double_t wiced_duktape_now(duk_context *ctx);
#define DUK_USE_DATE_GET_NOW(ctx)           wiced_duktape_now((ctx))

/* Always in UTC timezone */
#define DUK_USE_DATE_GET_LOCAL_TZOFFSET(d)  0

#undef DUK_USE_OS_STRING
#define DUK_USE_OS_STRING "WICED"
#endif
