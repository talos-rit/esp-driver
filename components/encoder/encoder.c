#include "encoder.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "driver/pulse_cnt.h"

static const char *TAG = "encoder";

esp_err_t encoder_init(encoder_handle_t *handle, encoder_config_t *config)
{
    if (handle == NULL || config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "install pcnt unit");
    pcnt_unit_config_t unit_config = {
        .high_limit = config->lim_high,
        .low_limit = config->lim_low
    };

    esp_err_t ret = pcnt_new_unit(&unit_config, &handle->pcnt_unit);
    if (ret != ESP_OK) {
        return ret;
    }

    ESP_LOGI(TAG, "set glitch filter");
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = config->resolution,
    };
    ret = pcnt_unit_set_glitch_filter(handle->pcnt_unit, &filter_config);
    if (ret != ESP_OK) {
        return ret;
    }

    ESP_LOGI(TAG, "install pcnt channels");
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = config->P0_pin,
        .level_gpio_num = config->P1_pin,
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ret = pcnt_new_channel(handle->pcnt_unit, &chan_a_config, &pcnt_chan_a);
    if (ret != ESP_OK) {
        return ret;
    }

    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = config->P1_pin,
        .level_gpio_num = config->P0_pin,
    };
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ret = pcnt_new_channel(handle->pcnt_unit, &chan_b_config, &pcnt_chan_b);
    if (ret != ESP_OK) {
        return ret;
    }

    ESP_LOGI(TAG, "set edge and level actions for pcnt channels");
    if (config->invert_angle) {
            ret = pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
        if (ret != ESP_OK) {
            return ret;
        }
        ret = pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
        if (ret != ESP_OK) {
            return ret;
        }
    } else {
        ret = pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
        if (ret != ESP_OK) {
            return ret;
        }
        ret = pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
        if (ret != ESP_OK) {
            return ret;
        }
    }
    ret = pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    if (ret != ESP_OK) {
        return ret;
    }

    return ESP_OK ;
}

esp_err_t encoder_start(encoder_handle_t *handle) {

    pcnt_unit_handle_t pcnt_unit = handle->pcnt_unit;

    ESP_LOGI(TAG, "enable pcnt unit");
    esp_err_t ret = pcnt_unit_enable(pcnt_unit);
    if (ret != ESP_OK) {
        return ret;
    }

    ESP_LOGI(TAG, "clear pcnt unit");
    ret = pcnt_unit_clear_count(pcnt_unit);
    if (ret != ESP_OK) {
        return ret;
    }

    ESP_LOGI(TAG, "start pcnt unit");
    ret = pcnt_unit_start(pcnt_unit);
    if (ret != ESP_OK) {
        return ret;
    }

    return ESP_OK ;
}


esp_err_t encoder_get_raw_count(encoder_handle_t *handle, int *pulse_count) {
    return pcnt_unit_get_count(handle->pcnt_unit, pulse_count);
}

esp_err_t encoder_clear_count(encoder_handle_t *handle) {
    return pcnt_unit_clear_count(handle->pcnt_unit);
}

esp_err_t encoder_get_wheel_angle(encoder_handle_t *handle, encoder_config_t *config, float *wheel_angle) {
    
    int pulse_count = 0;
    esp_err_t ret = pcnt_unit_get_count(handle->pcnt_unit, &pulse_count);
    if (ret != ESP_OK) {
        return ret;
    }

    int resolution = config->resolution * 4; // Due to x4 quadrature encoding.

    *wheel_angle = ((float)pulse_count * 360.0f) / resolution ; // Due to 360 degrees in 1 rotation.

    return ESP_OK ;

}

esp_err_t encoder_get_limb_angle(encoder_handle_t *handle, encoder_config_t *config, float *limb_angle) {
    float wheel_angle = 0.0f ;
    esp_err_t ret = encoder_get_wheel_angle(handle, config, &wheel_angle);
    if (ret != ESP_OK) {
        return ret;
    }

    float gear_ratio = config->gear_ratio ;
    *limb_angle = config->limb_default ;

    *limb_angle += wheel_angle / gear_ratio ;

    return ESP_OK ;
}