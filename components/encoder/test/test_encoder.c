#include "encoder.h"
#include "unity_fixture.h"
#include "sdkconfig.h"

TEST_GROUP(ENCODER) ;

static encoder_handle_t handle ;
static encoder_config_t config = {
        .P0_pin = CONFIG_ENCODER_P0_TEST_PIN ,
        .P1_pin = CONFIG_ENCODER_P1_TEST_PIN ,
        .glitch_filter_ns = CONFIG_ENCODER_GLITCH_FILTER ,
        .resolution = CONFIG_ENCODER_0_RESOLUTION ,
        .gear_ratio = (float)CONFIG_ENCODER_0_LIMB_GEAR_RATIO_X / (float)CONFIG_ENCODER_0_LIMB_GEAR_RATIO_Y ,
        .limb_default = CONFIG_ENCODER_0_LIMB_REFERENCE,
        .invert_angle = CONFIG_ENCODER_0_ANGLE_INVERT
    } ;

TEST_SETUP(ENCODER) {

}

TEST_TEAR_DOWN(ENCODER) {

}

TEST(ENCODER, Encoder_True_Initialization) {


    esp_err_t err = encoder_init(&handle, &config);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_NOT_EQUAL(NULL, &handle.pcnt_unit);
}

// TEST(ENCODER, Encoder_Test_Initialization) {
//     encoder_config_t virtual_config = {
//         .P0_pin = -1 ,
//         .P1_pin = -1 ,
//         .glitch_filter_ns = CONFIG_ENCODER_GLITCH_FILTER ,
//         .resolution = CONFIG_ENCODER_0_RESOLUTION ,
//         .gear_ratio = (float)CONFIG_ENCODER_0_LIMB_GEAR_RATIO_X / (float)CONFIG_ENCODER_0_LIMB_GEAR_RATIO_Y ,
//         .limb_default = CONFIG_ENCODER_0_LIMB_REFERENCE,
//         .invert_angle = CONFIG_ENCODER_0_ANGLE_INVERT
//     } ;

//     esp_err_t err = encoder_init(&handle, &virtual_config);
//     TEST_ASSERT_EQUAL(ESP_OK, err);
//     TEST_ASSERT_NOT_EQUAL(NULL, &handle.pcnt_unit);
// }




TEST_GROUP(ENCODER_INFINITE) ;
TEST_SETUP(ENCODER_INFINITE) {}
TEST_TEAR_DOWN(ENCODER_INFINITE) {}

TEST(ENCODER_INFINITE, Encoder_Run_Infinite) {

    encoder_init(&handle, &config);
    esp_err_t err = encoder_start(&handle) ;
    TEST_ASSERT_EQUAL(ESP_OK, err) ;

    int pulse_count = 0;
    while (true) {
        TEST_ASSERT_EQUAL(encoder_get_raw_count(&handle, &pulse_count), ESP_OK);
        printf("Pulse count: %d\n", pulse_count);
    }


}