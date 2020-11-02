#include "bme680_bsec.h"
#include "esphome/core/log.h"

namespace esphome {
namespace bme680 {

static const char *TAG = "bme680_bsec.sensor";


//TODO set state/get state - save to SPIFFS
// BSEC config: 3.3V, 3s, 4d
const uint8_t bsec_config_iaq[454] =
     {3,7,4,1,61,0,0,0,0,0,0,0,174,1,0,0,48,0,1,0,0,192,168,71,64,49,119,76,0,0,225,68,137,65,0,63,205,204,204,62,0,0,64,63,205,204,204,62,0,0,0,0,0,80,5,95,0,0,0,0,0,0,0,0,28,0,2,0,0,244,1,225,0,25,0,0,128,64,0,0,32,65,144,1,0,0,112,65,0,0,0,63,16,0,3,0,10,215,163,60,10,215,35,59,10,215,35,59,9,0,5,0,0,0,0,0,1,88,0,9,0,229,208,34,62,0,0,0,0,0,0,0,0,218,27,156,62,225,11,67,64,0,0,160,64,0,0,0,0,0,0,0,0,94,75,72,189,93,254,159,64,66,62,160,191,0,0,0,0,0,0,0,0,33,31,180,190,138,176,97,64,65,241,99,190,0,0,0,0,0,0,0,0,167,121,71,61,165,189,41,192,184,30,189,64,12,0,10,0,0,0,0,0,0,0,0,0,229,0,254,0,2,1,5,48,117,100,0,44,1,112,23,151,7,132,3,197,0,92,4,144,1,64,1,64,1,144,1,48,117,48,117,48,117,48,117,100,0,100,0,100,0,48,117,48,117,48,117,100,0,100,0,48,117,48,117,100,0,100,0,100,0,100,0,48,117,48,117,48,117,100,0,100,0,100,0,48,117,48,117,100,0,100,0,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,255,255,255,255,255,255,255,255,220,5,220,5,220,5,255,255,255,255,255,255,220,5,220,5,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,44,1,0,0,0,0,253,65,0,0};

void BME680BsecComponent::setup() {
	BME680Component::setup();
//bsec_library_return_t status=
	bsec_init();
	bsec_version_t version;
	bsec_get_version(&version);
//  ESP_LOGCONFIG(TAG, "BSEC lib version: %d.%d.%d.%d", bme680.version.major, bme680.version.minor, bme680.version.major_bugfix, version.minor_bugfix);

	// set BSEC lib configuration
	uint8_t workBuffer[BSEC_MAX_PROPERTY_BLOB_SIZE];
	//status =
  bsec_set_configuration(bsec_config_iaq, BSEC_MAX_PROPERTY_BLOB_SIZE, workBuffer, sizeof(workBuffer));

	// setup virtual sensors
	bsec_sensor_configuration_t bsecSensList[] {
		{BSEC_SAMPLE_RATE_LP, BSEC_OUTPUT_RAW_PRESSURE},
		{BSEC_SAMPLE_RATE_LP, BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE},
		{BSEC_SAMPLE_RATE_LP, BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY},
		{BSEC_SAMPLE_RATE_ULP, BSEC_OUTPUT_RAW_GAS},
		{BSEC_SAMPLE_RATE_ULP, BSEC_OUTPUT_IAQ},
		{BSEC_SAMPLE_RATE_ULP, BSEC_OUTPUT_CO2_EQUIVALENT},
		{BSEC_SAMPLE_RATE_ULP, BSEC_OUTPUT_BREATH_VOC_EQUIVALENT}
	};
	bsec_sensor_configuration_t sensorSettings[BSEC_MAX_PHYSICAL_SENSOR];
	uint8_t nSensorSettings = BSEC_MAX_PHYSICAL_SENSOR;

	//status =
  bsec_update_subscription(bsecSensList, sizeof(bsecSensList), sensorSettings, &nSensorSettings);
}

void BME680BsecComponent::update() {
  // check if it is time to call the algorithm
	int64_t call_time_ms = get_time_ms();
	if (call_time_ms >= next_call) {

		int64_t call_time_ns = call_time_ms * int64_t(1000000);

		bsec_bme_settings_t settings;
		bsec_library_return_t status = bsec_sensor_control(call_time_ns, &settings);
		if (status < BSEC_OK) {
      this->status_set_warning();
			return;
    }
    
		next_call = settings.next_call / int64_t(1000000); // Convert from ns to ms

		this->set_run_gas_(settings.run_gas);
		this->set_humidity_oversampling((BME680Oversampling)settings.humidity_oversampling);
		this->set_temperature_oversampling((BME680Oversampling)settings.temperature_oversampling);
		this->set_pressure_oversampling((BME680Oversampling)settings.pressure_oversampling);
    this->set_heater(settings.heater_temperature, settings.heating_duration);

    this->setup_control_registers_();

    uint8_t meas_control = 0;  // No need to fetch, we're setting all fields
    meas_control |= (this->temperature_oversampling_ & 0b111) << 5;
    meas_control |= (this->pressure_oversampling_ & 0b111) << 2;
    meas_control |= 0b01;  // forced mode, triggers measurement
    if (!this->write_byte(0x74, meas_control)) {
      this->status_set_warning();
      return;
    }
    /* Wait for measurement to complete */
    this->set_timeout("data", this->calc_meas_duration_(), [this,call_time_ns,settings]() { this->read_process_data(call_time_ns, settings); });
  }
}

void BME680BsecComponent::read_process_data(const int64_t curr_time_ns, const bsec_bme_settings_t settings) {
  BME680Component::read_data_();

  bsec_input_t inputs[BSEC_MAX_PHYSICAL_SENSOR]; // Temp, Pres, Hum & Gas
  uint8_t nInputs = 0;

  if (settings.process_data & BSEC_PROCESS_TEMPERATURE) {
    inputs[nInputs].sensor_id = BSEC_INPUT_TEMPERATURE;
    inputs[nInputs].signal = temperature_sensor_->raw_state; //todo wrong - should not publish
    inputs[nInputs].time_stamp = curr_time_ns;
    nInputs++;
  }
  if (settings.process_data & BSEC_PROCESS_HUMIDITY) {
    inputs[nInputs].sensor_id = BSEC_INPUT_HUMIDITY;
    inputs[nInputs].signal = humidity_sensor_->raw_state;
    inputs[nInputs].time_stamp = curr_time_ns;
    nInputs++;
  }
  if (settings.process_data & BSEC_PROCESS_PRESSURE) {
    inputs[nInputs].sensor_id = BSEC_INPUT_PRESSURE;
    inputs[nInputs].signal = pressure_sensor_->raw_state;
    inputs[nInputs].time_stamp = curr_time_ns;
    nInputs++;
  }
  if (settings.process_data & BSEC_PROCESS_GAS) {
    inputs[nInputs].sensor_id = BSEC_INPUT_GASRESISTOR;
    inputs[nInputs].signal = gas_resistance_sensor_->raw_state;
    inputs[nInputs].time_stamp = curr_time_ns;
    nInputs++;
  }

  if (nInputs > 0) {
    uint8_t nOutputs = BSEC_NUMBER_OUTPUTS;
    bsec_output_t _outputs[BSEC_NUMBER_OUTPUTS];

    //status =
    bsec_do_steps(inputs, nInputs, _outputs, &nOutputs);
  //  if (status != BSEC_OK)
    //  return false;

    this->status_clear_warning();

    if (nOutputs > 0) {
      for (uint8_t i = 0; i < nOutputs; i++) {
        switch (_outputs[i].sensor_id) {
          sensor::Sensor *breath_voc_equivalent_sensor_;
          case BSEC_OUTPUT_IAQ:
            if (this->iaq_sensor_ != nullptr)
              this->iaq_sensor_->publish_state(_outputs[i].signal);
            break;
          case BSEC_OUTPUT_CO2_EQUIVALENT:
            if (this->co2_equivalent_sensor_ != nullptr)
              this->co2_equivalent_sensor_->publish_state(_outputs[i].signal);
            break;
          case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT:
            if (this->breath_voc_equivalent_sensor_ != nullptr)
              this->breath_voc_equivalent_sensor_->publish_state(_outputs[i].signal);
            break;
          case BSEC_OUTPUT_RAW_PRESSURE:
            if (this->pressure_sensor_ != nullptr)
              this->pressure_sensor_->publish_state(_outputs[i].signal);
            break;
          case BSEC_OUTPUT_RAW_GAS:
            if (this->gas_resistance_sensor_ != nullptr)
              this->gas_resistance_sensor_->publish_state(_outputs[i].signal);
            break;
          case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE:
            if (this->temperature_sensor_ != nullptr)
              this->temperature_sensor_->publish_state(_outputs[i].signal);
            break;
          case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY:
            if (this->humidity_sensor_ != nullptr)
              this->humidity_sensor_->publish_state(_outputs[i].signal);
            break;
          default:
            break;
        }
      }
    }
  }
}

int64_t BME680BsecComponent::get_time_ms() {
	int64_t time_ms = millis();

	if (last_time > time_ms) { // An overflow occured
		last_time = time_ms;
		millis_overflow_counter++;
	}

	return time_ms + ((int64_t)millis_overflow_counter << 32);
}

}  // namespace bme680
}  // namespace esphome
