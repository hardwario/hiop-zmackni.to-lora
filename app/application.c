#include <application.h>
#include <at.h>
#include "lis2dh12.h"

#if 1
#define TIMEOUT_INACTIVITY (12 * 3600 * 1000)
#define TIMEOUT_BEACON (1 * 3600 * 1000)
#define SEND_DATA_INTERVAL (1 * 3600 * 1000)
#define MEASURE_INTERVAL (1 * 60 * 1000)
#else
#define TIMEOUT_INACTIVITY (60 * 1000)
#define TIMEOUT_BEACON (60 * 1000)
#define SEND_DATA_INTERVAL (60 * 1000)
#define MEASURE_INTERVAL (1 * 10 * 1000)
#endif

bc_led_t led;
bc_led_t led2;
bc_button_t button;
bc_cmwx1zzabz_t lora;
bc_tmp112_t tmp112;
bc_dice_t dice;

BC_DATA_STREAM_FLOAT_BUFFER(sm_voltage_buffer, 8)
BC_DATA_STREAM_FLOAT_BUFFER(sm_temperature_buffer, (SEND_DATA_INTERVAL / MEASURE_INTERVAL))
BC_DATA_STREAM_INT_BUFFER(sm_orientation_buffer, 3)

bc_data_stream_t sm_voltage;
bc_data_stream_t sm_temperature;

bc_scheduler_task_id_t battery_measure_task_id;

bool press;

bc_tick_t t_beacon;
bc_tick_t t_inactivity = TIMEOUT_INACTIVITY;

void button_event_handler(bc_button_t *self, bc_button_event_t event, void *event_param)
{
    if (event == BC_BUTTON_EVENT_PRESS)
    {
        bc_atci_printf("$EVENT: \"Press\"");

        bc_led_pulse(&led2, 1000);

        press = true;

        bc_scheduler_plan_now(0);
    }
}

void tmp112_event_handler(bc_tmp112_t *self, bc_tmp112_event_t event, void *event_param)
{
    if (event == BC_TMP112_EVENT_UPDATE)
    {
        float value;

        bc_tmp112_get_temperature_celsius(self, &value);

        bc_data_stream_feed(&sm_temperature, &value);
    }
}

void battery_event_handler(bc_module_battery_event_t event, void *event_param)
{
    if (event == BC_MODULE_BATTERY_EVENT_UPDATE)
    {
        float voltage = NAN;

        bc_module_battery_get_voltage(&voltage);

        bc_data_stream_feed(&sm_voltage, &voltage);
    }
}

void battery_measure_task(void *param)
{
    if (!bc_module_battery_measure())
    {
        bc_scheduler_plan_current_now();
    }
}

void lora_callback(bc_cmwx1zzabz_t *self, bc_cmwx1zzabz_event_t event, void *event_param)
{
    if (event == BC_CMWX1ZZABZ_EVENT_ERROR)
    {
        bc_led_set_mode(&led, BC_LED_MODE_BLINK_FAST);
    }
    else if (event == BC_CMWX1ZZABZ_EVENT_SEND_MESSAGE_START)
    {
        bc_led_set_mode(&led, BC_LED_MODE_ON);

        bc_scheduler_plan_relative(battery_measure_task_id, 20);
    }
    else if (event == BC_CMWX1ZZABZ_EVENT_SEND_MESSAGE_DONE)
    {
        bc_led_set_mode(&led, BC_LED_MODE_OFF);
    }
    else if (event == BC_CMWX1ZZABZ_EVENT_READY)
    {
        bc_led_set_mode(&led, BC_LED_MODE_OFF);
    }
    else if (event == BC_CMWX1ZZABZ_EVENT_JOIN_SUCCESS)
    {
        bc_atci_printf("$JOIN: \"OK\"");
    }
    else if (event == BC_CMWX1ZZABZ_EVENT_JOIN_ERROR)
    {
        bc_atci_printf("$JOIN: \"ERROR\"");
    }
}

bool at_send(void)
{
    t_beacon = 0;

    return true;
}

bool at_status(void)
{
    float value_avg = NAN;

    static const struct {
        bc_data_stream_t *stream;
        const char *name;
        int precision;
    } values[] = {
            {&sm_voltage, "Voltage", 1},
            {&sm_temperature, "Temperature", 1}
    };

    for (size_t i = 0; i < sizeof(values) / sizeof(values[0]); i++)
    {
        value_avg = NAN;

        if (bc_data_stream_get_average(values[i].stream, &value_avg))
        {
            bc_atci_printf("$STATUS: \"%s\",%.*f", values[i].name, values[i].precision, value_avg);
        }
        else
        {
            bc_atci_printf("$STATUS: \"%s\",", values[i].name);
        }
    }

    return true;
}

void application_init(void)
{
    bc_data_stream_init(&sm_voltage, 1, &sm_voltage_buffer);
    bc_data_stream_init(&sm_temperature, 1, &sm_temperature_buffer);

    bc_led_init(&led, BC_GPIO_LED, false, false);
    bc_led_init(&led2, BC_GPIO_P8, false, false);
    bc_led_set_mode(&led, BC_LED_MODE_ON);

    bc_button_init(&button, BC_GPIO_BUTTON, BC_GPIO_PULL_DOWN, false);
    bc_button_set_event_handler(&button, button_event_handler, NULL);

    bc_tmp112_init(&tmp112, BC_I2C_I2C0, 0x49);
    bc_tmp112_set_event_handler(&tmp112, tmp112_event_handler, NULL);
    bc_tmp112_set_update_interval(&tmp112, MEASURE_INTERVAL);

    bc_module_battery_init();
    bc_module_battery_set_event_handler(battery_event_handler, NULL);
    battery_measure_task_id = bc_scheduler_register(battery_measure_task, NULL, 2020);

    bc_dice_init(&dice, BC_DICE_FACE_UNKNOWN);

    bc_cmwx1zzabz_init(&lora, BC_UART_UART1);
    bc_cmwx1zzabz_set_event_handler(&lora, lora_callback, NULL);
    bc_cmwx1zzabz_set_mode(&lora, BC_CMWX1ZZABZ_CONFIG_MODE_ABP);
    bc_cmwx1zzabz_set_class(&lora, BC_CMWX1ZZABZ_CONFIG_CLASS_A);


    at_init(&led, &lora);
    static const bc_atci_command_t commands[] = {
            AT_LORA_COMMANDS,
            {"$SEND", at_send, NULL, NULL, NULL, "Immediately send packet"},
            {"$STATUS", at_status, NULL, NULL, NULL, "Show status"},
            AT_LED_COMMANDS,
            BC_ATCI_COMMAND_CLAC,
            BC_ATCI_COMMAND_HELP
    };
    bc_atci_init(commands, BC_ATCI_COMMANDS_LENGTH(commands));

    if (!lis2dh12_init())
        bc_atci_printf("$ERROR: \"lis2dh12_init\"");

    bc_scheduler_plan_current_relative(10 * 1000);
}

static void send(uint8_t message)
{
    static uint8_t buffer[5];

    memset(buffer, 0, sizeof(buffer));

    buffer[0] = message;

    float voltage_avg = NAN;

    bc_data_stream_get_average(&sm_voltage, &voltage_avg);

    if (!isnan(voltage_avg))
    {
        buffer[1] = voltage_avg * 10.f;
    }

    buffer[2] = 0; // TODO orientation

    float temperature_avg = NAN;

    bc_data_stream_get_average(&sm_temperature, &temperature_avg);

    if (!isnan(temperature_avg))
    {
        int16_t temperature_i16 = (int16_t) (temperature_avg * 10.f);

        buffer[3] = temperature_i16 >> 8;
        buffer[4] = temperature_i16;
    }

    bc_cmwx1zzabz_send_message(&lora, buffer, sizeof(buffer));

    static char tmp[sizeof(buffer) * 2 + 1];

    for (size_t i = 0; i < sizeof(buffer); i++)
    {
        sprintf(tmp + i * 2, "%02x", buffer[i]);
    }

    bc_atci_printf("$SEND: %s", tmp);
}

void application_task(void)
{
    bc_scheduler_plan_current_relative(100);

    if (!bc_cmwx1zzabz_is_ready(&lora))
        return;

    static bc_tick_t t_now;

    t_now = bc_tick_get();

    if (press)
    {
        press = false;

        t_inactivity = t_now + TIMEOUT_INACTIVITY;
        t_beacon = t_now + TIMEOUT_BEACON;

        send(1);

        return;
    }

    if (lis2dh12_irq)
    {
        bc_led_pulse(&led, 10);

        bc_atci_printf("$EVENT: \"Tilt\"");

        lis2dh12_clear_irq();

        t_inactivity = t_now + TIMEOUT_INACTIVITY;
    }

    if (t_now >= t_inactivity)
    {
        t_inactivity = t_now + TIMEOUT_INACTIVITY;
        t_beacon = t_now + TIMEOUT_BEACON;

        send(2);

        return;
    }

    if (t_now >= t_beacon)
    {
        t_beacon = t_now + TIMEOUT_BEACON;

        send(0);

        return;
    }
}
