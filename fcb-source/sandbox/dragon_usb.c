#include "usbd_cdc_if.h"

/*
 * This sandbox test sends a character sequence over USB as quickly as
 * possible so we can measure the transmission speed to a PC.
 */

enum { SEND_BUF_LEN = 32 };

void dragon_usb(void) {
	uint8_t send_buf[SEND_BUF_LEN];
	uint8_t i;

	char data = '0';

	for (i = 0; i < SEND_BUF_LEN; i++) {
		send_buf[i] = data + i;
	}

	while (1) {
		CDC_Transmit_FS(send_buf, SEND_BUF_LEN);
	}
}
