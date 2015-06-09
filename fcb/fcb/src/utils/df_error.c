#include "df_error.h"

#include "stm32f3_discovery.h"


void df_error(void) {
    BSP_LED_On(LED10);
    BSP_LED_On(LED3);
    /* TODO in the future this function should accept a text
     * string that could be printed to USB, or elsewhere
     */
    while(1) {
    }
}
