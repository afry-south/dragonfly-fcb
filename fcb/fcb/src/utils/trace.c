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
/* configuration & type declarations */
enum { TRACE_QUEUE_SIZE = 16 };
enum { TRACE_Q_MSG_SIZE = 32 };
enum { TRACE_SINK_MSG_SIZE = USB_FS_MAX_PACKET_SIZE /* 64 at time of writing */ };
enum { TRACE_ARGS_MAX = 6 };
enum { SEND_BUF_LEN = 64 };

struct trace_msg {
    const char * fmt; /** ptr to printf format string */
    uint32_t args[TRACE_ARGS_MAX]; /* generic arg storage */
};

/* static variable definitions */

static xQueueHandle qh_trace;
static xTaskHandle th_trace = NULL;


/**
 * Forwards the trace to its destination - at the time of writing
 * this is USB.
 */
static void trace_forward(void *);

/* public function definitions */

int trace_init(void) {
    int ret_val = FCB_OK;

    if (sizeof(struct trace_msg) > TRACE_Q_MSG_SIZE) {
        goto Error;
    }

    if (0 == (qh_trace = xQueueCreate(TRACE_QUEUE_SIZE, TRACE_Q_MSG_SIZE))) {
        goto Error;
    }

    if (pdPASS != (ret_val =
                   xTaskCreate((pdTASK_CODE)trace_forward,
                               (signed portCHAR*)"tTrace",
                               configMINIMAL_STACK_SIZE,
                               NULL,
                               1, /* zero is the lowest */
                               &th_trace))) {
        goto Error;
    }

Exit:
    return ret_val;
Error:
    ret_val = FCB_ERR_INIT;
    goto Exit;
}

int trace_post(const char * fmt, ...) {
    int ret_val = FCB_OK;
    int i;
    struct trace_msg msg = { 0 }; /* message to post to q */
    va_list arg_list;
    const char * fmt_ptr = fmt;

    va_start(arg_list, fmt);
    for (i = 0; i <TRACE_ARGS_MAX; i++) {
        va_arg(arg_list,int);
    }
    va_end(arg_list);

    if (pdTRUE != xQueueSend(qh_trace, &msg,portMAX_DELAY)) {
        goto Error;
    }

Exit:
    return ret_val;
Error:
    ret_val = FCB_ERR;
    goto Exit;
}

/* static function definitions */

void trace_forward(void * arg) {
    static uint8_t trace_out_buf[TRACE_SINK_MSG_SIZE];
    struct trace_msg queued_msg; /* message copied from q */

    while (1) {
        int len = 0;
        if (pdTRUE != xQueueReceive(qh_trace,
                                    &queued_msg,
                                    portMAX_DELAY /* in fact, forever see doc */)) {
            goto Error;
        }

        len = snprintf((char*)trace_out_buf,
                       TRACE_SINK_MSG_SIZE,
                       queued_msg.fmt,
                       queued_msg.args[0],
                       queued_msg.args[1],
                       queued_msg.args[2],
                       queued_msg.args[3],
                       queued_msg.args[4],
                       queued_msg.args[5]);

        CDC_Transmit_FS(trace_out_buf,
                        (len < TRACE_SINK_MSG_SIZE) ? len : TRACE_SINK_MSG_SIZE);
    }
Exit:
    return;

Error:
    /* as_todo: write error msg directly to USB (i.e. CDC_Transm ... etc) */
    goto Exit;
}

int trace_sync(const char * fmt, ...) {
    uint8_t send_buf[SEND_BUF_LEN] = { (uint8_t) '_'};
    int used_buf = 0;
    int send_buf_len = 0;
    va_list arg_list;

    va_start(arg_list, fmt);
    used_buf = vsnprintf((char*)send_buf, SEND_BUF_LEN, fmt, arg_list);
    va_end(arg_list);
    send_buf_len = (used_buf >= SEND_BUF_LEN) ? SEND_BUF_LEN : used_buf + 1;
    CDC_Transmit_FS(send_buf, send_buf_len);
    return send_buf_len;
}
