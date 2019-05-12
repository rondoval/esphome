import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import core
from esphome.components import i2c, sensor
from esphome.const import CONF_DURATION, CONF_GAS_RESISTANCE, CONF_HEATER, \
    CONF_HUMIDITY, CONF_ID, CONF_IIR_FILTER, CONF_OVERSAMPLING, CONF_PRESSURE, \
    CONF_TEMPERATURE, UNIT_OHM, ICON_GAS_CYLINDER, UNIT_CELSIUS, \
    ICON_THERMOMETER, UNIT_HECTOPASCAL, ICON_GAUGE, ICON_WATER_PERCENT, UNIT_PERCENT

DEPENDENCIES = ['i2c']

bme680_ns = cg.esphome_ns.namespace('bme680')

BME680BsecComponent = bme680_ns.class_('BME680BsecComponent', cg.PollingComponent, i2c.I2CDevice)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(BME680BsecComponent),
    cv.Optional(CONF_TEMPERATURE):
        sensor.sensor_schema(UNIT_CELSIUS, ICON_THERMOMETER, 1),
    cv.Optional(CONF_PRESSURE):
        sensor.sensor_schema(UNIT_HECTOPASCAL, ICON_GAUGE, 1),
    cv.Optional(CONF_HUMIDITY):
        sensor.sensor_schema(UNIT_PERCENT, ICON_WATER_PERCENT, 1),
    cv.Optional(CONF_GAS_RESISTANCE):
        sensor.sensor_schema(UNIT_OHM, ICON_GAS_CYLINDER, 1),
    cv.Optional(CONF_IAQ):
        sensor.sensor_schema(UNIT_EMPTY, ICON_GAS_CYLINDER, 1),
    cv.Optional(CONF_BREATHING_VOC):
        sensor.sensor_schema(UNIT_PARTS_PER_MILLION, ICON_GAS_CYLINDER, 1),
    cv.Optional(CONF_CO2_EQUIVALENT):
        sensor.sensor_schema(UNIT_PARTS_PER_MILLION, ICON_GAS_CYLINDER, 1),
}).extend(cv.polling_component_schema('60s')).extend(i2c.i2c_device_schema(0x76))


def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    yield cg.register_component(var, config)
    yield i2c.register_i2c_device(var, config)

    if CONF_TEMPERATURE in config:
        conf = config[CONF_TEMPERATURE]
        sens = yield sensor.new_sensor(conf)
        cg.add(var.set_temperature_sensor(sens))

    if CONF_PRESSURE in config:
        conf = config[CONF_PRESSURE]
        sens = yield sensor.new_sensor(conf)
        cg.add(var.set_pressure_sensor(sens))

    if CONF_HUMIDITY in config:
        conf = config[CONF_HUMIDITY]
        sens = yield sensor.new_sensor(conf)
        cg.add(var.set_humidity_sensor(sens))

    if CONF_GAS_RESISTANCE in config:
        conf = config[CONF_GAS_RESISTANCE]
        sens = yield sensor.new_sensor(conf)
        cg.add(var.set_gas_resistance_sensor(sens))

    if CONF_IAQ in config:
        conf = config[CONF_IAQ]
        sens = yield sensor.new_sensor(conf)
        cg.add(var.set_iaq_sensor(sens))

    if CONF_BREATHING_VOC in config:
        conf = config[CONF_BREATHING_VOC]
        sens = yield sensor.new_sensor(conf)
        cg.add(var.set_breath_voc_equivalent_sensor(sens))

    if CONF_CO2_EQUIVALENT in config:
        conf = config[CONF_CO2_EQUIVALENT]
        sens = yield sensor.new_sensor(conf)
        cg.add(var.set_co2_equivalent_sensor(sens))
