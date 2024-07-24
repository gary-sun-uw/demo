#include "interrupt.h"
#include "BMI270_SensorAPI/bmi270.h"
#include "BMI270_SensorAPI/bmi2_defs.h"
#include "util.h"
#include <driverlib.h>

const struct bmi2_sens_int_config feat_int_map[NUM_FEAT]  = { 
    //{ .type = BMI2_ANY_MOTION, .hw_int_pin = BMI2_INT2 }, 
    { .type = BMI2_NO_MOTION, .hw_int_pin = BMI2_INT1 },
    { .type = BMI2_STEP_ACTIVITY, .hw_int_pin = BMI2_INT1 }
};


int8_t setup_features(struct bmi2_dev *bmi2_dev){
    int8_t rslt;
    struct bmi2_sens_config configs[NUM_FEAT] = {
        //{ .type = BMI2_ANY_MOTION },
        { .type = BMI2_NO_MOTION },
        { .type = BMI2_STEP_ACTIVITY } };
    rslt = bmi270_get_sensor_config(configs, NUM_FEAT, bmi2_dev);
    bmi2_error_codes_print_result(rslt);
    if (rslt == BMI2_OK) {

        // /* 1LSB equals 20ms*/
        // configs[0].cfg.any_motion.duration = 0x08;
        // /* 1LSB equals to 0.48mg. Default is 83mg, setting to 50mg. */
        // configs[0].cfg.any_motion.threshold = 0x1FF;

        /* 1LSB equals 20ms*/
        configs[0].cfg.no_motion.duration = 0x10;
        /* 1LSB equals to 0.48mg. Default is 70mg, setting to 50mg. */
        configs[0].cfg.no_motion.threshold = 0x34;

        /* Set new configurations. */
        rslt = bmi270_set_sensor_config(configs, NUM_FEAT, bmi2_dev);
        bmi2_error_codes_print_result(rslt);
    }

    return rslt;
}

int8_t setup_interrupt_pin(struct bmi2_dev *bmi2_dev){
    /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Interrupt pin configuration */
    struct bmi2_int_pin_config pin_config = { 0 };

    rslt = bmi2_get_int_pin_config(&pin_config, bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    if (rslt == BMI2_OK) {
        /* Both interrupt pin configuration */
        pin_config.pin_type = BMI2_INT_BOTH;

        pin_config.pin_cfg[0].input_en = BMI2_INT_INPUT_DISABLE;
        pin_config.pin_cfg[0].lvl = BMI2_INT_ACTIVE_LOW;
        pin_config.pin_cfg[0].od = BMI2_INT_PUSH_PULL;
        pin_config.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE;

        pin_config.pin_cfg[1].input_en = BMI2_INT_INPUT_DISABLE;
        pin_config.pin_cfg[1].lvl = BMI2_INT_ACTIVE_LOW;
        pin_config.pin_cfg[1].od = BMI2_INT_PUSH_PULL;
        pin_config.pin_cfg[1].output_en = BMI2_INT_OUTPUT_ENABLE;

        pin_config.int_latch = BMI2_INT_NON_LATCH;

        rslt = bmi2_set_int_pin_config(&pin_config, bmi2_dev);
        bmi2_error_codes_print_result(rslt);
    }

    return rslt;
}
