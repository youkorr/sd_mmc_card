import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.const import (
    CONF_ID,
    CONF_DATA,
    CONF_PATH,
    CONF_CLK_PIN,
    CONF_INPUT,
    CONF_OUTPUT,
    CONF_PULLUP,
    CONF_PULLDOWN,
)
from esphome.core import CORE

CONF_SD_MMC_CARD_ID = "sd_mmc_card_id"
CONF_CMD_PIN = "cmd_pin"
CONF_DATA0_PIN = "data0_pin"
CONF_DATA1_PIN = "data1_pin"
CONF_DATA2_PIN = "data2_pin"
CONF_DATA3_PIN = "data3_pin"
CONF_MODE_1BIT = "mode_1bit"
CONF_POWER_CTRL_PIN = "power_ctrl_pin"

sd_mmc_card_component_ns = cg.esphome_ns.namespace("sd_mmc_card")
SdMmc = sd_mmc_card_component_ns.class_("SdMmc", cg.Component)

def validate_raw_data(value):
    if isinstance(value, str):
        return value.encode("utf-8")
    if isinstance(value, list):
        return cv.Schema([cv.hex_uint8_t])(value)
    raise cv.Invalid(
        "data must either be a string wrapped in quotes or a list of bytes"
    )

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(SdMmc),
        cv.Required(CONF_CLK_PIN): pins.internal_gpio_output_pin_number,
        cv.Required(CONF_CMD_PIN): pins.internal_gpio_output_pin_number,
        cv.Required(CONF_DATA0_PIN): pins.internal_gpio_pin_number({CONF_OUTPUT: True, CONF_INPUT: True}),
        cv.Optional(CONF_DATA1_PIN): pins.internal_gpio_pin_number({CONF_OUTPUT: True, CONF_INPUT: True}),
        cv.Optional(CONF_DATA2_PIN): pins.internal_gpio_pin_number({CONF_OUTPUT: True, CONF_INPUT: True}),
        cv.Optional(CONF_DATA3_PIN): pins.internal_gpio_pin_number({CONF_OUTPUT: True, CONF_INPUT: True}),
        cv.Optional(CONF_MODE_1BIT, default=False): cv.boolean,
        cv.Optional(CONF_POWER_CTRL_PIN) : pins.gpio_pin_schema({
            CONF_OUTPUT: True,
            CONF_PULLUP: False,
            CONF_PULLDOWN: False,
        }),
    }
).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_mode_1bit(config[CONF_MODE_1BIT]))

    cg.add(var.set_clk_pin(config[CONF_CLK_PIN]))
    cg.add(var.set_cmd_pin(config[CONF_CMD_PIN]))
    cg.add(var.set_data0_pin(config[CONF_DATA0_PIN]))

    if (config[CONF_MODE_1BIT] == False):
        cg.add(var.set_data1_pin(config[CONF_DATA1_PIN]))
        cg.add(var.set_data2_pin(config[CONF_DATA2_PIN]))
        cg.add(var.set_data3_pin(config[CONF_DATA3_PIN]))

    if (CONF_POWER_CTRL_PIN in config):
        power_ctrl = await cg.gpio_pin_expression(config[CONF_POWER_CTRL_PIN])
        cg.add(var.set_power_ctrl_pin(power_ctrl))

