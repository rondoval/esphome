#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/bme680/bme680.h"
#include "bsec_datatypes.h"
#include "bsec_interface.h"

namespace esphome {
namespace bme680 {

class BME680BsecComponent : public BME680Component {
 public:
   void set_iaq_sensor(sensor::Sensor *iaq_sensor) { iaq_sensor_ = iaq_sensor; }
   void set_co2_equivalent_sensor(sensor::Sensor *co2_equivalent_sensor) { co2_equivalent_sensor_ = co2_equivalent_sensor; }
   void set_breath_voc_equivalent_sensor(sensor::Sensor *breath_voc_equivalent_sensor) { breath_voc_equivalent_sensor_ = breath_voc_equivalent_sensor; }

  // ========== INTERNAL METHODS ==========
  // (In most use cases you won't need these)
  void setup() override;
  void update() override;

 protected:
   void read_process_data(int64_t curr_time_ns, bsec_bme_settings_t settings);
   int64_t next_call;			// Stores the time when the algorithm has to be called next in ms

 	// Global variables to help create a millisecond timestamp that doesn't overflow every 51 days.
 	// If it overflows, it will have a negative value. Something that should never happen.
 	uint32_t millis_overflow_counter {0};
 	uint32_t last_time {0};

  int64_t get_time_ms();

  sensor::Sensor *iaq_sensor_;
  sensor::Sensor *co2_equivalent_sensor_;
  sensor::Sensor *breath_voc_equivalent_sensor_;
};

}  // namespace bme680
}  // namespace esphome
