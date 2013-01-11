

#ifndef __AP_HAL_VRBRAIN_MAIN_H__
#define __AP_HAL_VRBRAIN_MAIN_H__

#if CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
#define AP_HAL_MAIN() extern "C" {\
    int main (void) {\
	hal.init(0, NULL);			\
        setup();\
        for(;;) loop();\
        return 0;\
    }\
    }
#endif // HAL_BOARD_VRBRAIN

#endif // __AP_HAL_VRBRAIN_MAIN_H__
