/*****************************************************************************
 * @file    trace_impl.c
 * @author  Dragonfly
 * @date    2015-09-01
 * @brief   File contains function to enable printouts in console using semihosting debug.
 ******************************************************************************/

#include "trace.h"
#include "trace_impl.h"
#include "semihosting.h"
#include "unistd.h"

/*
 * Semihosting is a mechanism that enables code running on an ARM target to communicate and use the Input/Output
 * facilities on a host computer that is running a debugger. Semihosting is an output channel that can be used for
 * the trace messages. It comes in two flavours: STDOUT and DEBUG. The DEBUG channel is chosen in this implementation.
 *
 * Choosing between semihosting STDOUT and DEBUG depends on the capabilities of your GDB server, and also on specific needs.
 * It is recommended to test DEBUG first, and if too slow, try STDOUT.
 *
 * Note: Applications buil with semihosting output active normally cannot be executed without the debugger connected and active,
 * since they use BKPT to communicate with the host. However, with a carefully written HardFault_Handler, the semihosting BKPT
 * calls can be processed, making possible to run semihosting applications as standalone, without being terminated with hardware
 * faults.
 *
 */

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

ssize_t trace_write(const char* buf __attribute__((unused)), size_t nbyte __attribute__((unused))){
	return _trace_write_semihosting_debug(buf, nbyte);
}



