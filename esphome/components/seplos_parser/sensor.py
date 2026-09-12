import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import ICON_EMPTY, UNIT_EMPTY
from . import HUB_CHILD_SCHEMA, CONF_SEPLOS_PARSER_ID, parse_sensor_id

DEPENDENCIES = ["seplos_parser"]

CONFIG_SCHEMA = (
    sensor.sensor_schema(
      unit_of_measurement=UNIT_EMPTY, icon=ICON_EMPTY, accuracy_decimals=2
    )
    .extend(HUB_CHILD_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    paren = await cg.get_variable(config[CONF_SEPLOS_PARSER_ID])
    var = await sensor.new_sensor(config)
    bms_index, metric = parse_sensor_id(config)

    cg.add(paren.register_sensor(var, bms_index, metric))
