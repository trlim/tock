#include <firestorm.h>
#include <tock.h>
#include <lps331ap.h>

int lps331ap_set_callback (subscribe_cb callback, void* callback_args) {
    return subscribe(7, 0, callback, callback_args);
}

int lps331ap_get_pressure () {
    return command(7, 0, 0);
}
