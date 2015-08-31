#include "trace.h"
#include "semihosting.h"
#include "unistd.h"

static ssize_t _trace_write_semihosting_debug(const char* buf, size_t nbyte);

ssize_t trace_write(const char* buf __attribute__((unused)), size_t nbyte __attribute__((unused))){
	return _trace_write_semihosting_debug(buf, nbyte);
}


#define TRACE_TMP_ARRAY_SIZE (16)

static ssize_t _trace_write_semihosting_debug(const char* buf, size_t nbyte){
	if (buf[nbyte] == '\0'){
		call_host(SEMIHOSTING_SYS_WRITE0, (void*)buf);
	}
	else {
		char tmp[TRACE_TMP_ARRAY_SIZE];
		size_t togo = nbyte;
		while (togo > 0){
			unsigned int n = ((togo < sizeof(tmp)) ? togo : sizeof(tmp));
			unsigned int i;
			for(i = 0; i < n; ++i, ++buf)
			{
				tmp[i] = *buf;
			}
			tmp[i] = '\0';

			call_host(SEMIHOSTING_SYS_WRITE0, (void*)tmp);
			togo -= n;
		}
	}

	return (ssize_t) nbyte;
}
