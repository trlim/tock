#include <firestorm.h>
#include <tock.h>

#define DRIVER_NUM_ADC 7

int adc_set_callback(subscribe_cb callback, void* callback_args) {
    return subscribe(DRIVER_NUM_ADC, 0, callback, callback_args);
}
int adc_initialize() {
    return command(DRIVER_NUM_ADC, 0, 0);
}
int adc_single_sample(char channel) {
    return command(DRIVER_NUM_ADC, 1, channel);
}
