#include "encoder.h"
#include "unity_fixture.h"
#include "sdkconfig.h"

TEST_GROUP(ENCODER) ;

static encoder_handle_t handle ;
static encoder_config_t config = {
        .P0_pin = CONFIG_ENCODER_P0_TEST_PIN ,
        .P1_pin = CONFIG_ENCODER_P0_TEST_PIN ,
        .glitch_filter_ns = CONFIG_ENCODER_GLITCH_FILTER ,
        .resolution = CONFIG_ENCODER_0_RESOLUTION ,
        .gear_ratio = (float)CONFIG_ENCODER_0_LIMB_GEAR_RATIO_X / (float)CONFIG_ENCODER_0_LIMB_GEAR_RATIO_Y ,
        .limb_default = CONFIG_ENCODER_0_LIMB_REFERENCE,
        .invert_angle = CONFIG_ENCODER_0_ANGLE_INVERT,
        .lim_low = CONFIG_ENCODER_0_LIMIT_LOW,
        .lim_high = CONFIG_ENCODER_0_LIMIT_HIGH
    } ;

TEST_SETUP(ENCODER) {

}

TEST_TEAR_DOWN(ENCODER) {

}

TEST(ENCODER, Encoder_Initialization) {


    esp_err_t err = encoder_init(&handle, &config);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_NOT_EQUAL(NULL, &handle.pcnt_unit);
}

TEST(ENCODER, Encoder_Run_Short) {

    encoder_init(&handle, &config);
    esp_err_t err = encoder_start(&handle) ;
    TEST_ASSERT_EQUAL(ESP_OK, err) ;

    int pulse_count = 0;
    while (true) {
        TEST_ASSERT_EQUAL(encoder_get_raw_count(&handle, &pulse_count), ESP_OK);
        printf("Pulse count: %d\n", pulse_count);
    }


}