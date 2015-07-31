#include "fcb_error.h"

#include "stm32f3_discovery.h"


void fcb_error(void) {
    BSP_LED_On(LED3);
    BSP_LED_On(LED4);
    BSP_LED_On(LED5);
    BSP_LED_On(LED6);
    BSP_LED_On(LED7);
    BSP_LED_On(LED8);
    BSP_LED_On(LED9);
    BSP_LED_On(LED10);


    /* TODO in the future this function should accept a text
     * string that could be printed to USB, or elsewhere
     */
    while(1) {
    }
}
