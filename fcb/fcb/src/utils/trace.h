#ifndef TRACE_H
#define TRACE_H

#define TRACE_SYNC(__FMT__, ...) do { trace_sync(__FMT__ "\n", __VA_ARGS__);  }while(0)

/**
 * Initialises a FreeRTOS queue & thread to read from the queue.
 *
 * @note must be called before first macro invocation.
 */
int trace_init(void);

/**
 * This function should not be called diretly, use the above macros.
 *
 * This function accepts any number of optional arguments but
 * only the first 6 will actually be used. Arguments to %s must
 * be static strings i.e. the pointer they refer to must point to a
 * valid string.
 *
 * @param fmt a printf format string and any number of non-string arguments
 *
 * @return zero for successful post to queue.
 */
int trace_post(const char * fmt, ...);


/**
 * This functions is as trace_post, except that it prints
 * directly.
 */
int trace_sync(const char * fmt, ...);
#endif /* TRACE_H */
