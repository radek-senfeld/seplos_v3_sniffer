#include "seplos_parser.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include <sstream>
#include <unordered_map>

namespace esphome {
namespace seplos_parser {

static const char *TAG = "seplos_parser.component";

void SeplosParser::setup() {
  buffer.reserve(128);

  // Initialisierung der Sensorvektoren
  std::vector<std::vector<sensor::Sensor *> *> sensor_vectors = {
      &pack_voltage_,
      &current_,
      &remaining_capacity_,
      &total_capacity_,
      &total_discharge_capacity_,
      &soc_,
      &soh_,
      &cycle_count_,
      &average_cell_voltage_,
      &average_cell_temp_,
      &max_cell_voltage_,
      &min_cell_voltage_,
      &delta_cell_voltage_,
      &max_cell_temp_,
      &min_cell_temp_,
      &maxdiscurt_,
      &maxchgcurt_,
      &cell_1_,
      &cell_2_,
      &cell_3_,
      &cell_4_,
      &cell_5_,
      &cell_6_,
      &cell_7_,
      &cell_8_,
      &cell_9_,
      &cell_10_,
      &cell_11_,
      &cell_12_,
      &cell_13_,
      &cell_14_,
      &cell_15_,
      &cell_16_,
      &cell_temp_1_,
      &cell_temp_2_,
      &cell_temp_3_,
      &cell_temp_4_,
      &case_temp_,
      &power_temp_,
      &power_};

  for (auto *vec : sensor_vectors) {
    vec->resize(bms_count_, nullptr);
  }

  std::vector<std::vector<text_sensor::TextSensor *> *> text_sensor_vectors = {
      &system_status_,
      &active_balancing_cells_,
      &cell_temperature_alarms_,
      &cell_voltage_alarms_,
      &FET_status_,
      &active_alarms_,
      &active_protections_};

  for (auto *vec : text_sensor_vectors) {
    vec->resize(bms_count_, nullptr);
  }

  // Zuordnung der Sensornamen zu den jeweiligen Vektoren
  std::unordered_map<std::string, std::vector<sensor::Sensor *> *> sensor_map =
      {{"pack_voltage", &pack_voltage_},
       {"current", &current_},
       {"remaining_capacity", &remaining_capacity_},
       {"total_capacity", &total_capacity_},
       {"total_discharge_capacity", &total_discharge_capacity_},
       {"soc", &soc_},
       {"soh", &soh_},
       {"cycle_count", &cycle_count_},
       {"average_cell_voltage", &average_cell_voltage_},
       {"average_cell_temp", &average_cell_temp_},
       {"max_cell_voltage", &max_cell_voltage_},
       {"min_cell_voltage", &min_cell_voltage_},
       {"delta_cell_voltage", &delta_cell_voltage_},
       {"max_cell_temp", &max_cell_temp_},
       {"min_cell_temp", &min_cell_temp_},
       {"maxdiscurt", &maxdiscurt_},
       {"maxchgcurt", &maxchgcurt_},
       {"cell_1", &cell_1_},
       {"cell_2", &cell_2_},
       {"cell_3", &cell_3_},
       {"cell_4", &cell_4_},
       {"cell_5", &cell_5_},
       {"cell_6", &cell_6_},
       {"cell_7", &cell_7_},
       {"cell_8", &cell_8_},
       {"cell_9", &cell_9_},
       {"cell_10", &cell_10_},
       {"cell_11", &cell_11_},
       {"cell_12", &cell_12_},
       {"cell_13", &cell_13_},
       {"cell_14", &cell_14_},
       {"cell_15", &cell_15_},
       {"cell_16", &cell_16_},
       {"cell_temp_1", &cell_temp_1_},
       {"cell_temp_2", &cell_temp_2_},
       {"cell_temp_3", &cell_temp_3_},
       {"cell_temp_4", &cell_temp_4_},
       {"case_temp", &case_temp_},
       {"power_temp", &power_temp_},
       {"power", &power_}};

  std::unordered_map<std::string, std::vector<text_sensor::TextSensor *> *>
      text_sensor_map = {{"system_status", &system_status_},
                         {"active_balancing_cells", &active_balancing_cells_},
                         {"cell_temperature_alarms", &cell_temperature_alarms_},
                         {"cell_voltage_alarms", &cell_voltage_alarms_},
                         {"FET_status", &FET_status_},
                         {"active_alarms", &active_alarms_},
                         {"active_protections", &active_protections_}};

  for (const auto &registration : sensor_registrations_) {
    auto entry = sensor_map.find(registration.metric);
    if (registration.bms_index < 0 || registration.bms_index >= bms_count_) {
      ESP_LOGW(TAG, "Sensor has invalid BMS index: %d", registration.bms_index);
    } else if (entry == sensor_map.end()) {
      ESP_LOGW(TAG, "Unknown sensor metric: %s", registration.metric.c_str());
    } else {
      (*entry->second)[registration.bms_index] = registration.sensor;
    }
  }

  for (const auto &registration : text_sensor_registrations_) {
    auto entry = text_sensor_map.find(registration.metric);
    if (registration.bms_index < 0 || registration.bms_index >= bms_count_) {
      ESP_LOGW(TAG, "Text sensor has invalid BMS index: %d", registration.bms_index);
    } else if (entry == text_sensor_map.end()) {
      ESP_LOGW(TAG, "Unknown text sensor metric: %s", registration.metric.c_str());
    } else {
      (*entry->second)[registration.bms_index] = registration.sensor;
    }
  }
}

void SeplosParser::loop() {
  while (available()) {
    uint8_t byte = read();
    buffer.push_back(byte);

    if (buffer.size() > 100) {
      buffer.erase(buffer.begin());
    }

    if (buffer.size() >= 5) {
      if (!is_valid_header()) {
        buffer.erase(buffer.begin());
        continue;
      }

      size_t expected_length = get_expected_length();
      if (buffer.size() >= expected_length) {
        if (validate_crc(expected_length)) {
          process_packet(expected_length);
          // buffer.clear();
          buffer.erase(buffer.begin(), buffer.begin() + expected_length);
          return; // Nach dem Verarbeiten eines Pakets direkt aus der loop()
                  // aussteigen
        } else {
          buffer.erase(buffer.begin());
        }
      }
    }
  }
}

bool SeplosParser::is_valid_header() {
  return ((buffer[0] >= 0x01 && buffer[0] <= 0x10 && buffer[1] == 0x04 &&
           (buffer[2] == 0x24 || buffer[2] == 0x34)) ||
          (buffer[0] >= 0x01 && buffer[0] <= 0x10 && buffer[1] == 0x01 &&
           buffer[2] == 0x12));
}
size_t SeplosParser::get_expected_length() {
  // +3 Header, +2 CRC, =+5
  if (buffer[2] == 0x24) {
    return 41;
  } // (0x24) 36+5=41
  if (buffer[2] == 0x34) {
    return 57;
  } // (0x34) 52+5=57
  if (buffer[1] == 0x01 && buffer[2] == 0x12) {
    return 23;
  }         // (0x12) 18+5=23
  return 0; // If an invalid packet arrives
}
bool SeplosParser::validate_crc(size_t length) {
  uint16_t received_crc = (buffer[length - 1] << 8) | buffer[length - 2];
  uint16_t calculated_crc = calculate_modbus_crc(buffer, length - 2);
  return received_crc == calculated_crc;
}

void SeplosParser::process_packet(size_t length) {
  int bms_index = buffer[0] - 0x01;
  if (bms_index < 0 || bms_index >= bms_count_) {
    ESP_LOGW("seplos", "Ungültige BMS-ID: %d", buffer[0]);
    return;
  }

  auto publish_sensor = [&](const std::vector<sensor::Sensor *> &sensor_vec, float value) {
    if (sensor_vec[bms_index]) {
      sensor_vec[bms_index]->publish_state(value);
    }
  };

  auto publish_text = [&](const std::vector<text_sensor::TextSensor *> &sensor_vec, const std::string &value) {
    if (sensor_vec[bms_index]) {
      sensor_vec[bms_index]->publish_state(value);
    }
  };

  auto get_16bit = [&](int index) -> uint16_t {
    return (buffer[index] << 8) | buffer[index + 1];
  };

  if (buffer[2] == 0x24) { // 36-Byte-Paket
    float voltage = get_16bit(3) / 100.0f;
    float current = static_cast<int16_t>(get_16bit(5)) / 100.0f;
    float power = voltage * current;

    publish_sensor(pack_voltage_, voltage);
    publish_sensor(current_, current);
    publish_sensor(power_, power);
    publish_sensor(remaining_capacity_, get_16bit(7) / 100.0f);
    publish_sensor(total_capacity_, get_16bit(9) / 100.0f);
    publish_sensor(total_discharge_capacity_, get_16bit(11) / 0.1f);
    publish_sensor(soc_, get_16bit(13) / 10.0f);
    publish_sensor(soh_, get_16bit(15) / 10.0f);
    publish_sensor(cycle_count_, get_16bit(17));
    publish_sensor(average_cell_voltage_, get_16bit(19) / 1000.0f);
    publish_sensor(average_cell_temp_, get_16bit(21) / 10.0f - 273.15f);
    publish_sensor(max_cell_voltage_, get_16bit(23) / 1000.0f);
    publish_sensor(min_cell_voltage_, get_16bit(25) / 1000.0f);
    publish_sensor(delta_cell_voltage_, static_cast<float>(get_16bit(23) - get_16bit(25)));
    publish_sensor(max_cell_temp_, get_16bit(27) / 10.0f - 273.15f);
    publish_sensor(min_cell_temp_, get_16bit(29) / 10.0f - 273.15f);
    publish_sensor(maxdiscurt_, get_16bit(33) / 1.0f);
    publish_sensor(maxchgcurt_, get_16bit(35) / 1.0f);
  }

  if (buffer[2] == 0x34) {
    const std::vector<sensor::Sensor *>* cells[] = {
        &cell_1_, &cell_2_, &cell_3_, &cell_4_, &cell_5_, &cell_6_, &cell_7_, &cell_8_,
        &cell_9_, &cell_10_, &cell_11_, &cell_12_, &cell_13_, &cell_14_, &cell_15_, &cell_16_};
    for (int i = 0; i < 16; i++) {
      publish_sensor(*cells[i], get_16bit(3 + i * 2) / 1000.0f);
    }

    const std::vector<sensor::Sensor *>* temps[] = {&cell_temp_1_, &cell_temp_2_, &cell_temp_3_, &cell_temp_4_};
    for (int i = 0; i < 4; i++) {
      publish_sensor(*temps[i], get_16bit(35 + i * 2) / 10.0f - 273.15f);
    }

    publish_sensor(case_temp_, get_16bit(51) / 10.0f - 273.15f);
    publish_sensor(power_temp_, get_16bit(53) / 10.0f - 273.15f);
  }

  if (buffer[2] == 0x12) {
    if (!should_update(bms_index))
      return;

    std::string active_alarms, active_protections, system_status_str, fet_status_str;
    std::string volt_str, temp_str, balancing_str, high_volt_str, high_temp_str;

    auto append_str = [](std::string &out, const char* msg) {
      if (!out.empty()) out += ", ";
      out += msg;
    };

    auto append_bits = [](uint8_t byte, int offset, std::string &out) {
      for (int i = 0; i < 8; i++) {
        if (byte & (1 << i)) {
          if (!out.empty()) out += ", ";
          out += std::to_string(i + offset);
        }
      }
    };

    append_bits(buffer[3], 1, volt_str);
    append_bits(buffer[4], 9, volt_str);
    append_bits(buffer[5], 1, high_volt_str);
    append_bits(buffer[6], 9, high_volt_str);
    if (!volt_str.empty() && !high_volt_str.empty()) volt_str += " | " + high_volt_str;
    else volt_str += high_volt_str;

    append_bits(buffer[7], 1, temp_str);
    append_bits(buffer[8], 1, high_temp_str);
    if (!temp_str.empty() && !high_temp_str.empty()) temp_str += " | " + high_temp_str;
    else temp_str += high_temp_str;

    append_bits(buffer[9], 1, balancing_str);
    append_bits(buffer[10], 9, balancing_str);

    if (buffer[11] & 0x01) append_str(system_status_str, "Discharge");
    if (buffer[11] & 0x02) append_str(system_status_str, "Charge");
    if (buffer[11] & 0x04) append_str(system_status_str, "Floating Charge");
    if (buffer[11] & 0x08) append_str(system_status_str, "Full Charge");
    if (buffer[11] & 0x10) append_str(system_status_str, "Standby Mode");
    if (buffer[11] & 0x20) append_str(system_status_str, "Turn Off");

    if (buffer[12] & 0x01) append_str(active_alarms, "Cell High Voltage Alarm");
    if (buffer[12] & 0x02) append_str(active_protections, "Cell Over Voltage Protection");
    if (buffer[12] & 0x04) append_str(active_alarms, "Cell Low Voltage Alarm");
    if (buffer[12] & 0x08) append_str(active_protections, "Cell Under Voltage Protection");
    if (buffer[12] & 0x10) append_str(active_alarms, "Pack High Voltage Alarm");
    if (buffer[12] & 0x20) append_str(active_protections, "Pack Over Voltage Protection");
    if (buffer[12] & 0x40) append_str(active_alarms, "Pack Low Voltage Alarm");
    if (buffer[12] & 0x80) append_str(active_protections, "Pack Under Voltage Protection");

    if (buffer[13] & 0x01) append_str(active_alarms, "Charge High Temperature Alarm");
    if (buffer[13] & 0x02) append_str(active_protections, "Charge High Temperature Protection");
    if (buffer[13] & 0x04) append_str(active_alarms, "Charge Low Temperature Alarm");
    if (buffer[13] & 0x08) append_str(active_protections, "Charge Under Temperature Protection");
    if (buffer[13] & 0x10) append_str(active_alarms, "Discharge High Temperature Alarm");
    if (buffer[13] & 0x20) append_str(active_protections, "Discharge Over Temperature Protection");
    if (buffer[13] & 0x40) append_str(active_alarms, "Discharge Low Temperature Alarm");
    if (buffer[13] & 0x80) append_str(active_protections, "Discharge Under Temperature Protection");

    if (buffer[14] & 0x01) append_str(active_alarms, "High Environment Temperature Alarm");
    if (buffer[14] & 0x02) append_str(active_protections, "Over Environment Temperature Protection");
    if (buffer[14] & 0x04) append_str(active_alarms, "Low Environment Temperature Alarm");
    if (buffer[14] & 0x08) append_str(active_protections, "Under Environment Temperature Protection");
    if (buffer[14] & 0x10) append_str(active_alarms, "High Power Temperature Alarm");
    if (buffer[14] & 0x20) append_str(active_protections, "Over Power Temperature Protection");
    if (buffer[14] & 0x40) append_str(active_alarms, "Cell Temperature Low Heating");

    if (buffer[15] & 0x01) append_str(active_alarms, "Charge Current Alarm");
    if (buffer[15] & 0x02) append_str(active_protections, "Charge Over Current Protection");
    if (buffer[15] & 0x04) append_str(active_protections, "Charge Second Level Current Protection");
    if (buffer[15] & 0x08) append_str(active_alarms, "Discharge Current Alarm");
    if (buffer[15] & 0x10) append_str(active_protections, "Discharge Over Current Protection");
    if (buffer[15] & 0x20) append_str(active_protections, "Discharge Second Level Over Current Protection");
    if (buffer[15] & 0x40) append_str(active_protections, "Output Short Circuit Protection");

    if (buffer[16] & 0x01) append_str(active_alarms, "Output Short Latch Up");
    if (buffer[16] & 0x04) append_str(active_alarms, "Second Charge Latch Up");
    if (buffer[16] & 0x08) append_str(active_alarms, "Second Discharge Latch Up");

    if (buffer[17] & 0x04) append_str(active_alarms, "SOC Alarm");
    if (buffer[17] & 0x08) append_str(active_protections, "SOC Protection");
    if (buffer[17] & 0x10) append_str(active_alarms, "Cell Difference Alarm");

    if (buffer[18] & 0x01) append_str(fet_status_str, "Discharge FET On");
    if (buffer[18] & 0x02) append_str(fet_status_str, "Charge FET On");
    if (buffer[18] & 0x04) append_str(fet_status_str, "Current Limiting FET On");
    if (buffer[18] & 0x08) append_str(fet_status_str, "Heating On");

    if (buffer[19] & 0x01) append_str(active_alarms, "Low SOC Alarm");
    if (buffer[19] & 0x02) append_str(active_alarms, "Intermittent Charge");
    if (buffer[19] & 0x04) append_str(active_alarms, "External Switch Conrol");
    if (buffer[19] & 0x08) append_str(active_alarms, "Static Standy Sleep Mode");
    if (buffer[19] & 0x10) append_str(active_alarms, "History Data Recording");
    if (buffer[19] & 0x20) append_str(active_protections, "Under SOC Protections");
    if (buffer[19] & 0x40) append_str(active_alarms, "Active Limited Current");
    if (buffer[19] & 0x80) append_str(active_alarms, "Passive Limited Current");

    if (buffer[20] & 0x01) append_str(active_protections, "NTC Fault");
    if (buffer[20] & 0x02) append_str(active_protections, "AFE Fault");
    if (buffer[20] & 0x04) append_str(active_protections, "Charge Mosfet Fault");
    if (buffer[20] & 0x08) append_str(active_protections, "Discharge Mosfet Fault");
    if (buffer[20] & 0x10) append_str(active_protections, "Cell Fault");
    if (buffer[20] & 0x20) append_str(active_protections, "Break Line Fault");
    if (buffer[20] & 0x40) append_str(active_protections, "Key Fault");
    if (buffer[20] & 0x80) append_str(active_protections, "Aerosol Alarm");

    publish_text(cell_voltage_alarms_, volt_str);
    publish_text(cell_temperature_alarms_, temp_str);
    publish_text(active_balancing_cells_, balancing_str);
    publish_text(system_status_, system_status_str);
    publish_text(FET_status_, fet_status_str);
    publish_text(active_alarms_, active_alarms);
    publish_text(active_protections_, active_protections);
  }
}

const uint16_t crc_table[256] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241, 0xC601,
    0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440, 0xCC01, 0x0CC0,
    0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40, 0x0A00, 0xCAC1, 0xCB81,
    0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841, 0xD801, 0x18C0, 0x1980, 0xD941,
    0x1B00, 0xDBC1, 0xDA81, 0x1A40, 0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01,
    0x1DC0, 0x1C80, 0xDC41, 0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0,
    0x1680, 0xD641, 0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081,
    0x1040, 0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441, 0x3C00,
    0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41, 0xFA01, 0x3AC0,
    0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840, 0x2800, 0xE8C1, 0xE981,
    0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41, 0xEE01, 0x2EC0, 0x2F80, 0xEF41,
    0x2D00, 0xEDC1, 0xEC81, 0x2C40, 0xE401, 0x24C0, 0x2580, 0xE541, 0x2700,
    0xE7C1, 0xE681, 0x2640, 0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0,
    0x2080, 0xE041, 0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281,
    0x6240, 0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41, 0xAA01,
    0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840, 0x7800, 0xB8C1,
    0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41, 0xBE01, 0x7EC0, 0x7F80,
    0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40, 0xB401, 0x74C0, 0x7580, 0xB541,
    0x7700, 0xB7C1, 0xB681, 0x7640, 0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101,
    0x71C0, 0x7080, 0xB041, 0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0,
    0x5280, 0x9241, 0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481,
    0x5440, 0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841, 0x8801,
    0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40, 0x4E00, 0x8EC1,
    0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41, 0x4400, 0x84C1, 0x8581,
    0x4540, 0x8701, 0x47C0, 0x4680, 0x8641, 0x8201, 0x42C0, 0x4380, 0x8341,
    0x4100, 0x81C1, 0x8081, 0x4040};

uint16_t SeplosParser::calculate_modbus_crc(const std::vector<uint8_t> &data,
                                            size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; i++) {
    uint8_t index = crc ^ data[i];
    crc = (crc >> 8) ^ crc_table[index];
  }
  return crc;
}

void SeplosParser::dump_config() {
  for (int i = 0; i < bms_count_; i++) {
    last_updates_[i] = millis();
    // ESP_LOGD("SeplosParser", "Initialisiere Timer für BMS %d: %u", i,
    // last_updates_[i]);
  }
  for (auto *sensor : this->sensors_) {
    LOG_SENSOR("  ", "Sensor", sensor);
  }

  for (auto *text_sensor : this->text_sensors_) {
    LOG_TEXT_SENSOR("  ", "Text sensor", text_sensor);
  }

  //    for(auto *binary_sensor : this->binary_sensors_){
  //        LOG_BINARY_SENSOR("  ", "Binary sensor", binary_sensor);
  //    }
}
void SeplosParser::set_bms_count(int bms_count) {
  this->bms_count_ = bms_count;       // Wert speichern
  last_updates_.resize(bms_count, 0); // Dynamische Größe
  ESP_LOGI("SeplosParser", "BMS Count gesetzt auf: %d", bms_count);
}
void SeplosParser::set_update_interval(int update_interval) {
  this->update_interval_ = update_interval * 1000;
  ESP_LOGI("SeplosParser", "update interval: %d", update_interval);
}
bool SeplosParser::should_update(int bms_index) {
  if (bms_index < 0 || bms_index >= bms_count_) {
    // ESP_LOGW("SeplosParser", "Ungültiger BMS-Index: %d (max: %d)", bms_index,
    // bms_count_);
    return false; // Ungültiger Index
  }

  uint32_t now = millis();
  // ESP_LOGD("SeplosParser", "BMS %d: now=%u, last_update=%u, interval=%u",
  //           bms_index, now, last_updates_[bms_index], update_interval_);
  if (now - last_updates_[bms_index] >= update_interval_) {
    last_updates_[bms_index] =
        now; // Setze den Timer für dieses BMS-Gerät zurück
    // ESP_LOGD("SeplosParser", "Update für BMS %d durchgeführt", bms_index);
    return true;
  }
  // ESP_LOGD("SeplosParser", "Kein Update für BMS %d nötig", bms_index);
  return false;
}

} // namespace seplos_parser
} // namespace esphome
