#pragma once

#define DRIVER_NUM_ADC 7

int adc_set_callback(subscribe_cb callback, void* callback_args);
int adc_initialize();
int adc_single_sample(char channel);
