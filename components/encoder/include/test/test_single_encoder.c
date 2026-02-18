#include "encoder.h"
#include "unity.h"


TEST_CASE("Encoder Initialization Test", "[encoder]") {
    encoder_handle_t encoder_handle ;
    encoder_config_t encoder_config = {
        .P0_pin = CONFIG_ENCODER_P0_TEST_PIN ,
        .P1_pin = CONFIG_ENCODER_P1_TEST_PIN ,
        .glitch_filter_ns = CONFIG_ENCODER_GLITCH_FILTER ,
        .resolution = CONFIG_ENCODER_0_RESOLUTION ,
        .gear_ratio = (float)CONFIG_ENCODER_O_GEAR_RATIO_X / (float)CONFIG_ENCODER_O_GEAR_RATIO_Y ,
        .limb_default = CONFIG_ENCODER_0_LIMB_REFERENCE,
        .angle_invert = CONFIG_ENCODER_0_ANGLE_INVERT
    }

    esp_err_t err = encoder_init(&encoder_handle, &encoder_config);
  TEST_ASSERT_EQUAL(ESP_OK, err);
  TEST_ASSERT_NOT_EQUAL(NULL, handle.dev_handle);
}

TEST_CASE("hook up a single encoder as if this was an app_main()", "[encoder]")
{
    encoder_handle_t encoder ;
    encoder_config_t encoder_config = {
        .P0_pin = CONFIG_ENCODER_P0_TEST_PIN ,
        .P1_pin = CONFIG_ENCODER_P1_TEST_PIN ,
        .glitch_filter_ns = CONFIG_ENCODER_GLITCH_FILTER ,
        .resolution = CONFIG_ENCODER_0_RESOLUTION ,
        .gear_ratio = CONFIG_ENCODER_O_GEAR_RATIO ,
    }
    



}