#include "unity.h"
#include "unity_fixture.h"
#include "test_menu.h"
#include "driver/gpio.h"


// NOTE: Define Groups here, and they will be automatically added to the menu
#define TEST_GROUPS \
    X(I2C_Bus)      \
    X(PCA9685)      \
    X(MotorHAT)     \
    X(ADS1015)      \
    X(Driver_WiFi)   \


#define X(g) static void run_##g(void) { RUN_TEST_GROUP(g); }
TEST_GROUPS
#undef X


#define X(g) { #g, run_##g },
static const test_group_t groups[] = {
    TEST_GROUPS
};
#undef X

void app_main(void)
{
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    test_menu_run(groups, sizeof(groups) / sizeof(groups[0]));
}
