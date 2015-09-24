#ifndef TRACE_IMPL_H
#define TRACE_IMPL_H

#include "unistd.h"

ssize_t
trace_write(const char* buf __attribute__((unused)), size_t nbyte __attribute__((unused)));

#endif
