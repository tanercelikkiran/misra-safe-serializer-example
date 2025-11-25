
#include "SafeSerializer.h"

#include <cstdint>
#include <cstddef>
#include <algorithm>
#include <type_traits>
#include <array>


// ==========================================
// 2. USER STRUCTS
// ==========================================

enum FlightPhase_e : uint8_t { FLIGHT_PHASE_PREFLIGHT = 0, FLIGHT_PHASE_CRUISE = 4, FLIGHT_PHASE_SHUTDOWN = 9 };
enum SystemHealth_e : uint8_t { SYSTEM_STATUS_OK = 0, SYSTEM_STATUS_FAIL = 2 };
enum NavSource_e : uint8_t { NAV_SOURCE_GPS = 0 };
enum GearStatus_e : uint8_t { GEAR_UP_LOCKED = 0 };

struct SubSystemData {
    uint16_t subId;
    float temperature;

    bool deserialize(const uint8_t* buffer, size_t max_len, size_t& consumed) {
        size_t local_offset = 0;
        // For a nested call, the offset is reset (from the child buffer perspective) or the main buffer is passed.
        // In this structure, we use the main buffer + offset pointer logic, so local_offset starts at 0 here,
        // because the pointer is already offset.
        bool res = deserialize_from_buffer(buffer, max_len, local_offset, subId, temperature);
        consumed = local_offset;
        return res;
    }

    bool serialize(uint8_t* buffer, size_t max_len, size_t& consumed) const {
        size_t local_offset = 0;
        // For a nested call, the offset is reset (from the child buffer perspective) or the main buffer is passed.
        // In this structure, we use the main buffer + offset pointer logic, so local_offset starts at 0 here,
        // because the pointer is already offset.
        bool res = serialize_to_buffer(buffer, max_len, local_offset, subId, temperature);
        consumed = local_offset;
        return res;
    }

    size_t trueSize() const {
        // Ignores padding: returns sizeof(uint16_t) + sizeof(uint8_t) = 3.
		return calculate_packed_size(subId, temperature);
    }
};

struct DO178C_FlightData_t {
    // --- HEADER & IDENTIFICATION ---
    uint32_t    packet_sequence_id;
    double      system_timestamp_sec;
    uint16_t    aircraft_id;
    uint8_t     software_version_major;
    uint8_t     software_version_minor;

    // --- SYSTEM STATE & FLAGS ---
    FlightPhase_e current_flight_phase;
    SystemHealth_e master_system_health;
    uint8_t     is_autopilot_engaged;
    uint8_t     is_autothrottle_armed;
    uint8_t     is_weight_on_wheels;

    SubSystemData sub_system_data;

    // --- NAVIGATION DATA ---
    double      latitude_deg;
    double      longitude_deg;
    double      altitude_baro_ft;
    double      altitude_radio_ft;
    double      altitude_gps_ft;
    float       pos_accuracy_h_m;
    float       pos_accuracy_v_m;
    NavSource_e active_nav_source;
    uint8_t     visible_satellites;
    uint16_t    waypoint_index;

    // --- FLIGHT DYNAMICS ---
    double      pitch_angle_deg;
    double      roll_angle_deg;
    double      heading_mag_deg;
    double      heading_true_deg;
    double      track_angle_deg;
    float       drift_angle_deg;
    float       pitch_rate_deg_s;
    float       roll_rate_deg_s;
    float       yaw_rate_deg_s;

    // --- SPEEDS ---
    float       airspeed_indicated_kts;
    float       airspeed_true_kts;
    float       ground_speed_kts;
    float       mach_number;
    float       vertical_speed_fpm;
    float       accel_normal_g;
    float       accel_lateral_g;
    float       accel_longitudinal_g;
    float       angle_of_attack_deg;
    float       sideslip_angle_deg;
    float       flight_path_angle_deg;

    // --- ENGINE 1 ---
    float       eng1_n1_percent;
    float       eng1_n2_percent;
    float       eng1_egt_c;
    float       eng1_fuel_flow_kg_h;
    float       eng1_oil_press_psi;
    float       eng1_oil_temp_c;
    float       eng1_vibration_ips;
    float       eng1_throttle_cmd_pct;
    uint8_t     eng1_fire_warning;
    uint8_t     eng1_reverser_deployed;

    // --- ENGINE 2 ---
    float       eng2_n1_percent;
    float       eng2_n2_percent;
    float       eng2_egt_c;
    float       eng2_fuel_flow_kg_h;
    float       eng2_oil_press_psi;
    float       eng2_oil_temp_c;
    float       eng2_vibration_ips;
    float       eng2_throttle_cmd_pct;
    uint8_t     eng2_fire_warning;
    uint8_t     eng2_reverser_deployed;

    // --- FUEL ---
    float       fuel_qty_left_kg;
    float       fuel_qty_right_kg;
    float       fuel_qty_center_kg;
    float       fuel_qty_total_kg;
    float       fuel_temp_c;
    uint8_t     fuel_pump_l_on;
    uint8_t     fuel_pump_r_on;

    // --- ELECTRICAL ---
    float       dc_bus_main_volts;
    float       dc_bus_main_amps;
    float       bat_1_volts;
    float       bat_1_amps;
    float       ac_bus_freq_hz;
    float       gen_1_load_pct;
    float       gen_2_load_pct;
    uint8_t     ext_power_available;

    // --- HYDRAULIC ---
    float       hyd_press_sys_a_psi;
    float       hyd_press_sys_b_psi;
    float       hyd_qty_sys_a_pct;
    float       hyd_qty_sys_b_pct;
    float       brake_pressure_psi;
    float       cabin_pressure_psi;
    float       cabin_altitude_ft;
    float       cabin_rate_fpm;

    // --- FLIGHT CONTROLS ---
    float       aileron_pos_l_deg;
    float       aileron_pos_r_deg;
    float       elevator_pos_l_deg;
    float       elevator_pos_r_deg;
    float       rudder_pos_deg;
    float       flap_handle_pos;
    float       flap_actual_pos_l;
    float       flap_actual_pos_r;
    float       spoiler_pos_pct;
    float       trim_stab_units;
    float       trim_aileron_units;
    float       trim_rudder_units;

    // --- LANDING GEAR ---
    GearStatus_e gear_nose_status;
    GearStatus_e gear_main_l_status;
    GearStatus_e gear_main_r_status;
    float       brake_temp_l_c;
    float       brake_temp_r_c;
    float       tire_pressure_nose_psi;

    // --- ENV ---
    float       oat_c;
    float       tat_c;
    float       wind_speed_kts;
    float       wind_direction_deg;
    float       air_density_ratio;
    uint8_t     ice_detected;

    // --- AP TARGETS ---
    int32_t     ap_target_alt_ft;
    int16_t     ap_target_speed_kts;
    int16_t     ap_target_heading_deg;
    int16_t     ap_target_vs_fpm;
    double      fms_dist_to_dest_nm;
    double      fms_ete_dest_sec;
    float       fms_x_track_error_nm;
    float       fms_req_nav_perf_nm;

    // --- DIAG ---
    uint32_t    crc32_checksum;
    uint16_t    frame_counter;
    uint8_t     cpu_load_percent;
    uint8_t     num_active_faults;
    uint32_t    bit_status_word;

    // --- FULL DESERIALIZATION METHOD ---
    bool deserialize(const uint8_t* buffer, size_t max_len, size_t& consumed) {
        LOG_INFO("DO178C_FlightData_t deserialization START. Available Buffer: %zu bytes", max_len);

        bool result = deserialize_from_buffer(
            buffer, max_len, consumed,
            // 1. Header
            packet_sequence_id, system_timestamp_sec, aircraft_id,
            software_version_major, software_version_minor,
            // 2. State
            current_flight_phase, master_system_health,
            is_autopilot_engaged, is_autothrottle_armed, is_weight_on_wheels,
            // 3. SubSystem (Nested)
            sub_system_data,
            // 4. Nav
            latitude_deg, longitude_deg, altitude_baro_ft, altitude_radio_ft,
            altitude_gps_ft, pos_accuracy_h_m, pos_accuracy_v_m,
            active_nav_source, visible_satellites, waypoint_index,
            // 5. Dynamics
            pitch_angle_deg, roll_angle_deg, heading_mag_deg, heading_true_deg,
            track_angle_deg, drift_angle_deg, pitch_rate_deg_s, roll_rate_deg_s,
            yaw_rate_deg_s,
            // 6. Speed
            airspeed_indicated_kts, airspeed_true_kts, ground_speed_kts,
            mach_number, vertical_speed_fpm, accel_normal_g, accel_lateral_g,
            accel_longitudinal_g, angle_of_attack_deg, sideslip_angle_deg,
            flight_path_angle_deg,
            // 7. Engine 1
            eng1_n1_percent, eng1_n2_percent, eng1_egt_c, eng1_fuel_flow_kg_h,
            eng1_oil_press_psi, eng1_oil_temp_c, eng1_vibration_ips,
            eng1_throttle_cmd_pct, eng1_fire_warning, eng1_reverser_deployed,
            // 8. Engine 2
            eng2_n1_percent, eng2_n2_percent, eng2_egt_c, eng2_fuel_flow_kg_h,
            eng2_oil_press_psi, eng2_oil_temp_c, eng2_vibration_ips,
            eng2_throttle_cmd_pct, eng2_fire_warning, eng2_reverser_deployed,
            // 9. Fuel
            fuel_qty_left_kg, fuel_qty_right_kg, fuel_qty_center_kg,
            fuel_qty_total_kg, fuel_temp_c, fuel_pump_l_on, fuel_pump_r_on,
            // 10. Electrical
            dc_bus_main_volts, dc_bus_main_amps, bat_1_volts, bat_1_amps,
            ac_bus_freq_hz, gen_1_load_pct, gen_2_load_pct, ext_power_available,
            // 11. Hydraulic
            hyd_press_sys_a_psi, hyd_press_sys_b_psi, hyd_qty_sys_a_pct,
            hyd_qty_sys_b_pct, brake_pressure_psi, cabin_pressure_psi,
            cabin_altitude_ft, cabin_rate_fpm,
            // 12. Controls
            aileron_pos_l_deg, aileron_pos_r_deg, elevator_pos_l_deg,
            elevator_pos_r_deg, rudder_pos_deg, flap_handle_pos,
            flap_actual_pos_l, flap_actual_pos_r, spoiler_pos_pct,
            trim_stab_units, trim_aileron_units, trim_rudder_units,
            // 13. Gear
            gear_nose_status, gear_main_l_status, gear_main_r_status,
            brake_temp_l_c, brake_temp_r_c, tire_pressure_nose_psi,
            // 14. Env
            oat_c, tat_c, wind_speed_kts, wind_direction_deg,
            air_density_ratio, ice_detected,
            // 15. AP Targets
            ap_target_alt_ft, ap_target_speed_kts, ap_target_heading_deg,
            ap_target_vs_fpm, fms_dist_to_dest_nm, fms_ete_dest_sec,
            fms_x_track_error_nm, fms_req_nav_perf_nm,
            // 16. Diag
            crc32_checksum, frame_counter, cpu_load_percent, num_active_faults,
            bit_status_word
        );

        LOG_INFO("DO178C_FlightData_t deserialization END (result=%s, consumed=%zu bytes)", result ? "OK" : "FAIL", consumed);
        return result;
    }

    bool serialize(uint8_t* buffer, size_t max_len, size_t& consumed) const {
        LOG_INFO("DO178C_FlightData_t serialization START. Available Buffer: %zu bytes", max_len);
        bool result = serialize_to_buffer(
            buffer, max_len, consumed,
            // 1. Header
            packet_sequence_id, system_timestamp_sec, aircraft_id,
            software_version_major, software_version_minor,
            // 2. State
            current_flight_phase, master_system_health,
            is_autopilot_engaged, is_autothrottle_armed, is_weight_on_wheels,
            // 3. SubSystem (Nested)
            sub_system_data,
            // 4. Nav
            latitude_deg, longitude_deg, altitude_baro_ft, altitude_radio_ft,
            altitude_gps_ft, pos_accuracy_h_m, pos_accuracy_v_m,
            active_nav_source, visible_satellites, waypoint_index,
            // 5. Dynamics
            pitch_angle_deg, roll_angle_deg, heading_mag_deg, heading_true_deg,
            track_angle_deg, drift_angle_deg, pitch_rate_deg_s, roll_rate_deg_s,
            yaw_rate_deg_s,
            // 6. Speed
            airspeed_indicated_kts, airspeed_true_kts, ground_speed_kts,
            mach_number, vertical_speed_fpm, accel_normal_g, accel_lateral_g,
            accel_longitudinal_g, angle_of_attack_deg, sideslip_angle_deg,
            flight_path_angle_deg,
            // 7. Engine 1
            eng1_n1_percent, eng1_n2_percent, eng1_egt_c, eng1_fuel_flow_kg_h,
            eng1_oil_press_psi, eng1_oil_temp_c, eng1_vibration_ips,
            eng1_throttle_cmd_pct, eng1_fire_warning, eng1_reverser_deployed,
            // 8. Engine 2
            eng2_n1_percent, eng2_n2_percent, eng2_egt_c, eng2_fuel_flow_kg_h,
            eng2_oil_press_psi, eng2_oil_temp_c, eng2_vibration_ips,
            eng2_throttle_cmd_pct, eng2_fire_warning, eng2_reverser_deployed,
            // 9. Fuel
            fuel_qty_left_kg, fuel_qty_right_kg, fuel_qty_center_kg,
            fuel_qty_total_kg, fuel_temp_c, fuel_pump_l_on, fuel_pump_r_on,
            // 10. Electrical
            dc_bus_main_volts, dc_bus_main_amps, bat_1_volts, bat_1_amps,
            ac_bus_freq_hz, gen_1_load_pct, gen_2_load_pct, ext_power_available,
            // 11. Hydraulic
            hyd_press_sys_a_psi, hyd_press_sys_b_psi, hyd_qty_sys_a_pct,
            hyd_qty_sys_b_pct, brake_pressure_psi, cabin_pressure_psi,
            cabin_altitude_ft, cabin_rate_fpm,
            // 12. Controls
            aileron_pos_l_deg, aileron_pos_r_deg, elevator_pos_l_deg,
            elevator_pos_r_deg, rudder_pos_deg, flap_handle_pos,
            flap_actual_pos_l, flap_actual_pos_r, spoiler_pos_pct,
            trim_stab_units, trim_aileron_units, trim_rudder_units,
            // 13. Gear
            gear_nose_status, gear_main_l_status, gear_main_r_status,
            brake_temp_l_c, brake_temp_r_c, tire_pressure_nose_psi,
            // 14. Env
            oat_c, tat_c, wind_speed_kts, wind_direction_deg,
            air_density_ratio, ice_detected,
            // 15. AP Targets
            ap_target_alt_ft, ap_target_speed_kts, ap_target_heading_deg,
            ap_target_vs_fpm, fms_dist_to_dest_nm, fms_ete_dest_sec,
            fms_x_track_error_nm, fms_req_nav_perf_nm,
            // 16. Diag
            crc32_checksum, frame_counter, cpu_load_percent, num_active_faults,
            bit_status_word
        );
		return result;
	}

    size_t trueSize() const {
        return calculate_packed_size(
            // 1. Header
            packet_sequence_id, system_timestamp_sec, aircraft_id,
            software_version_major, software_version_minor,
            // 2. State
            current_flight_phase, master_system_health,
            is_autopilot_engaged, is_autothrottle_armed, is_weight_on_wheels,
            // 3. SubSystem (Nested)
            sub_system_data,
            // 4. Nav
            latitude_deg, longitude_deg, altitude_baro_ft, altitude_radio_ft,
            altitude_gps_ft, pos_accuracy_h_m, pos_accuracy_v_m,
            active_nav_source, visible_satellites, waypoint_index,
            // 5. Dynamics
            pitch_angle_deg, roll_angle_deg, heading_mag_deg, heading_true_deg,
            track_angle_deg, drift_angle_deg, pitch_rate_deg_s, roll_rate_deg_s,
            yaw_rate_deg_s,
            // 6. Speed
            airspeed_indicated_kts, airspeed_true_kts, ground_speed_kts,
            mach_number, vertical_speed_fpm, accel_normal_g, accel_lateral_g,
            accel_longitudinal_g, angle_of_attack_deg, sideslip_angle_deg,
            flight_path_angle_deg,
            // 7. Engine 1
            eng1_n1_percent, eng1_n2_percent, eng1_egt_c, eng1_fuel_flow_kg_h,
            eng1_oil_press_psi, eng1_oil_temp_c, eng1_vibration_ips,
            eng1_throttle_cmd_pct, eng1_fire_warning, eng1_reverser_deployed,
            // 8. Engine 2
            eng2_n1_percent, eng2_n2_percent, eng2_egt_c, eng2_fuel_flow_kg_h,
            eng2_oil_press_psi, eng2_oil_temp_c, eng2_vibration_ips,
            eng2_throttle_cmd_pct, eng2_fire_warning, eng2_reverser_deployed,
            // 9. Fuel
            fuel_qty_left_kg, fuel_qty_right_kg, fuel_qty_center_kg,
            fuel_qty_total_kg, fuel_temp_c, fuel_pump_l_on, fuel_pump_r_on,
            // 10. Electrical
            dc_bus_main_volts, dc_bus_main_amps, bat_1_volts, bat_1_amps,
            ac_bus_freq_hz, gen_1_load_pct, gen_2_load_pct, ext_power_available,
            // 11. Hydraulic
            hyd_press_sys_a_psi, hyd_press_sys_b_psi, hyd_qty_sys_a_pct,
            hyd_qty_sys_b_pct, brake_pressure_psi, cabin_pressure_psi,
            cabin_altitude_ft, cabin_rate_fpm,
            // 12. Controls
            aileron_pos_l_deg, aileron_pos_r_deg, elevator_pos_l_deg,
            elevator_pos_r_deg, rudder_pos_deg, flap_handle_pos,
            flap_actual_pos_l, flap_actual_pos_r, spoiler_pos_pct,
            trim_stab_units, trim_aileron_units, trim_rudder_units,
            // 13. Gear
            gear_nose_status, gear_main_l_status, gear_main_r_status,
            brake_temp_l_c, brake_temp_r_c, tire_pressure_nose_psi,
            // 14. Env
            oat_c, tat_c, wind_speed_kts, wind_direction_deg,
            air_density_ratio, ice_detected,
            // 15. AP Targets
            ap_target_alt_ft, ap_target_speed_kts, ap_target_heading_deg,
            ap_target_vs_fpm, fms_dist_to_dest_nm, fms_ete_dest_sec,
            fms_x_track_error_nm, fms_req_nav_perf_nm,
            // 16. Diag
            crc32_checksum, frame_counter, cpu_load_percent, num_active_faults,
			bit_status_word);
    }
};

// ==========================================
// 3. TEST HARNESS (MAIN)
// ==========================================

// Util test function: Float comparison with epsilon
bool is_close(double a, double b, double epsilon = 0.001) {
    return std::fabs(a - b) < epsilon;
}

int main() {
#ifdef TEST_ENV
    LOG_INFO("========================================");
    LOG_INFO("DO-178C Flight Data Serialization Test (SAFE MODE)");
    LOG_INFO("========================================");

    // 1. VERİ HAZIRLIĞI
    DO178C_FlightData_t originalData = {};

    originalData.packet_sequence_id = 0xDEADBEEF;
    originalData.system_timestamp_sec = 12345.6789;
    originalData.aircraft_id = 0x1234;
    originalData.software_version_major = 1;
    originalData.software_version_minor = 42;

    originalData.current_flight_phase = FLIGHT_PHASE_CRUISE;
    originalData.master_system_health = SYSTEM_STATUS_OK;
    originalData.is_autopilot_engaged = true;
    originalData.is_autothrottle_armed = true;
    originalData.is_weight_on_wheels = false;

    originalData.sub_system_data.subId = 101;
    originalData.sub_system_data.temperature = 35.7f;

    originalData.latitude_deg = 40.7128;
    originalData.longitude_deg = -74.0060;
    originalData.altitude_baro_ft = 35000.0;
    originalData.altitude_radio_ft = 500.0;
    originalData.altitude_gps_ft = 35020.0;
    originalData.pos_accuracy_h_m = 5.5f;
    originalData.pos_accuracy_v_m = 3.2f;
    originalData.active_nav_source = NAV_SOURCE_GPS;
    originalData.visible_satellites = 12;
    originalData.waypoint_index = 42;

    originalData.pitch_angle_deg = 2.5;
    originalData.roll_angle_deg = 0.0;
    originalData.heading_mag_deg = 180.0;
    originalData.heading_true_deg = 181.5;
    originalData.track_angle_deg = 181.3;
    originalData.drift_angle_deg = 0.2f;
    originalData.pitch_rate_deg_s = 0.1f;
    originalData.roll_rate_deg_s = 0.0f;
    originalData.yaw_rate_deg_s = 0.05f;

    originalData.airspeed_indicated_kts = 450.0f;
    originalData.airspeed_true_kts = 485.0f;
    originalData.ground_speed_kts = 490.0f;
    originalData.mach_number = 0.72f;
    originalData.vertical_speed_fpm = -100.0f;
    originalData.accel_normal_g = 1.05f;
    originalData.accel_lateral_g = 0.02f;
    originalData.accel_longitudinal_g = 0.0f;
    originalData.angle_of_attack_deg = 2.3f;
    originalData.sideslip_angle_deg = -0.1f;
    originalData.flight_path_angle_deg = -0.8f;

    originalData.eng1_n1_percent = 85.5f;
    originalData.eng1_n2_percent = 92.3f;
    originalData.eng1_egt_c = 620.0f;
    originalData.eng1_fuel_flow_kg_h = 4800.0f;
    originalData.eng1_oil_press_psi = 55.0f;
    originalData.eng1_oil_temp_c = 75.5f;
    originalData.eng1_vibration_ips = 0.15f;
    originalData.eng1_throttle_cmd_pct = 85.0f;
    originalData.eng1_fire_warning = false;
    originalData.eng1_reverser_deployed = false;

    originalData.eng2_n1_percent = 85.3f;
    originalData.eng2_n2_percent = 92.1f;
    originalData.eng2_egt_c = 615.0f;
    originalData.eng2_fuel_flow_kg_h = 4780.0f;
    originalData.eng2_oil_press_psi = 55.2f;
    originalData.eng2_oil_temp_c = 75.2f;
    originalData.eng2_vibration_ips = 0.14f;
    originalData.eng2_throttle_cmd_pct = 84.8f;
    originalData.eng2_fire_warning = false;
    originalData.eng2_reverser_deployed = false;

    originalData.fuel_qty_left_kg = 18500.0f;
    originalData.fuel_qty_right_kg = 18600.0f;
    originalData.fuel_qty_center_kg = 12000.0f;
    originalData.fuel_qty_total_kg = 49100.0f;
    originalData.fuel_temp_c = -45.0f;
    originalData.fuel_pump_l_on = true;
    originalData.fuel_pump_r_on = true;

    originalData.dc_bus_main_volts = 28.5f;
    originalData.dc_bus_main_amps = 150.0f;
    originalData.bat_1_volts = 28.2f;
    originalData.bat_1_amps = 5.0f;
    originalData.ac_bus_freq_hz = 400.0f;
    originalData.gen_1_load_pct = 75.0f;
    originalData.gen_2_load_pct = 78.0f;
    originalData.ext_power_available = false;


    originalData.hyd_press_sys_a_psi = 3000.0f;
    originalData.hyd_press_sys_b_psi = 3050.0f;
    originalData.hyd_qty_sys_a_pct = 95.0f;
    originalData.hyd_qty_sys_b_pct = 98.0f;
    originalData.brake_pressure_psi = 0.0f;
    originalData.cabin_pressure_psi = 10.2f;
    originalData.cabin_altitude_ft = 8000.0f;
    originalData.cabin_rate_fpm = -50.0f;

    originalData.aileron_pos_l_deg = 0.0f;
    originalData.aileron_pos_r_deg = 0.0f;
    originalData.elevator_pos_l_deg = -1.5f;
    originalData.elevator_pos_r_deg = -1.5f;
    originalData.rudder_pos_deg = 0.2f;
    originalData.flap_handle_pos = 0.0f;
    originalData.flap_actual_pos_l = 0.0f;
    originalData.flap_actual_pos_r = 0.0f;
    originalData.spoiler_pos_pct = 0.0f;
    originalData.trim_stab_units = 5.5f;
    originalData.trim_aileron_units = 0.0f;
    originalData.trim_rudder_units = 0.0f;

    originalData.gear_nose_status = GEAR_UP_LOCKED;
    originalData.gear_main_l_status = GEAR_UP_LOCKED;
    originalData.gear_main_r_status = GEAR_UP_LOCKED;
    originalData.brake_temp_l_c = 50.0f;
    originalData.brake_temp_r_c = 52.0f;
    originalData.tire_pressure_nose_psi = 0.0f;

    originalData.oat_c = -56.0f;
    originalData.tat_c = -10.0f;
    originalData.wind_speed_kts = 15.0f;
    originalData.wind_direction_deg = 270.0f;
    originalData.air_density_ratio = 0.38f;
    originalData.ice_detected = false;

    originalData.ap_target_alt_ft = 35000;
    originalData.ap_target_speed_kts = 450;
    originalData.ap_target_heading_deg = 180;
    originalData.ap_target_vs_fpm = 0;
    originalData.fms_dist_to_dest_nm = 1250.5;
    originalData.fms_ete_dest_sec = 9500.0;
    originalData.fms_x_track_error_nm = 0.05f;
    originalData.fms_req_nav_perf_nm = 0.5f;

    originalData.crc32_checksum = 0xAABBCCDD;
    originalData.frame_counter = 12345;
    originalData.cpu_load_percent = 65;
    originalData.num_active_faults = 0;
    originalData.bit_status_word = 0x00000001;

	// 2. SERIALIZATION
    LOG_INFO("[STEP 1] Serializing Flight Data...");
	constexpr size_t MAX_BUFFER_SIZE = 2048;
	uint8_t serializedBuffer[MAX_BUFFER_SIZE] = {};
	size_t bufPos = 0;

	bool serResult = originalData.serialize(serializedBuffer, MAX_BUFFER_SIZE, bufPos);

    if (serResult) {
        LOG_INFO("Serialization SUCCESS. Total Bytes Written: %zu", bufPos);
        debug_hex_dump(serializedBuffer, bufPos);
    }
    else {
        LOG_ERROR("Serialization FAILED.");
        return -1;
	}

    // 3. DESERIALIZATION
    LOG_INFO("[STEP 2] Deserializing Flight Data...");
    DO178C_FlightData_t deserializedData = {};
    size_t consumed = 0;

	// We reset bufPos to 0 to read from the start of the buffer
    bool result = deserializedData.deserialize(serializedBuffer, bufPos, consumed);

    // 4. VERIFICATION
    LOG_INFO("[STEP 3] Verifying Data Integrity...");
    if (result) {
        bool allMatch = true;

        if (originalData.packet_sequence_id != deserializedData.packet_sequence_id) {
            LOG_ERROR("MISMATCH: packet_sequence_id"); allMatch = false;
        }
        if (!is_close(originalData.latitude_deg, deserializedData.latitude_deg)) {
            LOG_ERROR("MISMATCH: latitude_deg"); allMatch = false;
        }
        if (originalData.sub_system_data.subId != deserializedData.sub_system_data.subId) {
            LOG_ERROR("MISMATCH: sub_system_data.subId"); allMatch = false;
        }

        if (allMatch) {
            LOG_INFO("SUCCESS: All critical fields match!");
        }
        else {
            LOG_ERROR("FAILURE: Data corruption detected.");
        }
    }
    else {
        LOG_ERROR("Deserialization returned FALSE.");
    }

#endif
    return 0;
}