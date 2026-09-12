import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID
import re

DEPENDENCIES = ["uart"]

MULTI_CONF = True

CONF_BMS_COUNT = "bms_count"
CONF_UPDATE_INTERVAL = "update_interval"
CONF_SEPLOS_PARSER_ID = "seplos_parser_id"

seplos_parser_ns = cg.esphome_ns.namespace("seplos_parser")

SeplosParser = seplos_parser_ns.class_("SeplosParser", cg.Component)

SENSOR_ID_PATTERN = re.compile(r"^bms(?P<bms_index>\d+)_(?P<metric>[a-zA-Z0-9_]+)$")


def parse_sensor_id(config):
    yaml_id = config[CONF_ID].id
    match = SENSOR_ID_PATTERN.fullmatch(yaml_id)

    if match is None:
        raise cv.Invalid(
            f"id must use the format bms<N>_<metric>, for example bms0_pack_voltage: {yaml_id}"
        )
    return int(match["bms_index"]), match["metric"]

HUB_CHILD_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_SEPLOS_PARSER_ID): cv.use_id(SeplosParser),
    }
)

CONFIG_SCHEMA = (
    cv.Schema({
        cv.GenerateID(): cv.declare_id(SeplosParser),
        cv.Optional(CONF_UPDATE_INTERVAL, default=5): cv.int_,
        cv.Optional(CONF_BMS_COUNT, default=1): cv.int_,
    })
        .extend(cv.COMPONENT_SCHEMA)
        .extend(uart.UART_DEVICE_SCHEMA)
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
#    uart_device = await cg.get_variable(config[CONF_UART_ID])
    cg.add(var.set_update_interval(config[CONF_UPDATE_INTERVAL]))
    cg.add(var.set_bms_count(config[CONF_BMS_COUNT]))
