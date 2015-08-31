#include "trace.h"
#include "fcb_retval.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "usbd_cdc_if.h"
#include "usbd_def.h"
#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>

/* Private defines */
#ifndef TRACE_PRINTF_TMP_ARRAY_SIZE
#define TRACE_PRINTF_TMP_ARRAY_SIZE (128)
#endif

/* configuration & type declarations */
enum { TRACE_SINK_MSG_SIZE = USB_FS_MAX_PACKET_SIZE /* 64 at time of writing */ };
enum { TRACE_ARGS_MAX = 6 };


/* public function definitions */


int trace_post(const char * fmt, ...) {
    int ret_val = FCB_OK;
    int len;

    static uint8_t trace_out_buf[TRACE_SINK_MSG_SIZE];

    va_list arg_list;

    va_start(arg_list, fmt);
    len = vsnprintf((char*)trace_out_buf,
                   TRACE_SINK_MSG_SIZE,
                   fmt,
                   arg_list);
    va_end(arg_list);


    USBComSendData(trace_out_buf, len);

    return ret_val;
}

int trace_printf(const char* format, ...) {
	int ret;
	va_list argList;

	va_start(argList, format);

	static char buf[TRACE_PRINTF_TMP_ARRAY_SIZE];

	ret = vsnprintf(buf, sizeof(buf), format, argList);
	if (ret > 0)
	{
		ret = trace_write(buf, (size_t)ret);
	}

	va_end(argList);
	return ret;
}

#if TODO
int trace_sync(const char * fmt, ...) {
    uint8_t send_buf[SEND_BUF_LEN] = { (uint8_t) '_'};
    int used_buf = 0;
    int send_buf_len = 0;
    va_list arg_list;

    va_start(arg_list, fmt);
    used_buf = vsnprintf((char*)send_buf, SEND_BUF_LEN, fmt, arg_list);
    va_end(arg_list);
    send_buf_len = (used_buf >= SEND_BUF_LEN) ? SEND_BUF_LEN : used_buf + 1;
    // TODO CDC_Transmit_FS(send_buf, send_buf_len);
    return send_buf_len;
}
#endif
