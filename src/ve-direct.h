#pragma once
#include "configuration.h"
#include <sstream>
#include <iomanip>
#include <map>
#include <charconv>

namespace ve
{
enum class command : uint8_t {
    enter_boot = 0x0,
    zero = 0x0,
    ping = 0x1,
    reserved_2 = 0x2,
    app_version = 0x3,
    product_id = 0x4,
    reserved_5 = 0x5,
    restart = 0x6,
    get = 0x7,
    set = 0x8,
    reserved_9 = 0x9,
    async = 0xa,
    reserved_b = 0xb,
    reserved_c = 0xc,
    reserved_d = 0xd,
    reserved_e = 0xe,
    reserved_f = 0xf
};

enum class response : uint8_t {
    reserved_0 = 0x0,
    zero = 0x0,
    done = 0x1,
    reserved_2 = 0x2,
    unknown = 0x3,
    error = 0x4,
    ping = 0x5,
    reserved_6 = 0x6,
    get = 0x7,
    set = 0x8,
    reserved_9 = 0x9,
    async = 0xa,
    reserved_b = 0xb,
    reserved_c = 0xc,
    reserved_d = 0xd,
    reserved_e = 0xe,
    reserved_f = 0xf
};

// Comparison commands and responses raw
inline bool operator==(command c, response r) {
    return static_cast<uint8_t>(c) == static_cast<uint8_t>(r);
}
inline bool operator==(response r, command c) {
    return c == r;
}

enum class id : uint16_t {
    zero = 0x0000,
    // Product Information registers
    product_id = 0x0100,
    group_id = 0x0104,
    serial_number = 0x010A,
    model_name = 0x010B,
    capabilities = 0x0140,
    // Generic device control registers 
    device_mode = 0x0200,
    device_state = 0x0201,
    remote_control_conf = 0x0202,
    device_off_reason_8 = 0x0205,
    device_off_reason_32 = 0x0207,
    // Battery settings registers
    battery_safe_mode= 0xEDFF,
    battery_adaptive_mode = 0xEDFE,
    battery_automatic_eq_mode = 0xEDFD,
    battery_bulk_time_limit = 0xEDFC,
    battery_absorption_time_limit = 0xEDFB,
    battery_absorption_voltage = 0xEDF7,
    battery_float_voltage = 0xEDF6,
    battery_equalisation_voltage = 0xEDF4,
    battery_temperature_compensation = 0xEDF2,
    battery_type = 0xEDF1,
    battery_max_current = 0xEDF0,
    battery_voltage = 0xEDEF,
    battery_temperature = 0xEDEC,
    battery_voltage_setting = 0xEDEA,
    battery_bms_present = 0xEDE8,
    battery_tail_current = 0xEDE7,
    battery_low_temp_charge_current = 0xEDE6,
    battery_auto_equalise_stop_voltage = 0xEDE5,
    battery_equalisation_current_level = 0xEDE4,
    battery_equalisation_duration = 0xEDE3,
    battery_rebulk_voltage_offset = 0xED2E,
    battery_low_temp_level = 0xEDE0,
    battery_voltage_compensation = 0xEDCA,
    // 2-wire BMS input - MPPT RS models only
    remote_input_mode_conf = 0xD0C0,
    two_wire_bms_input_states = 0xD01F,
    // Charger data registers
    charger_max_current = 0xEDDF,
    yield_system = 0xEDDD,
    yield_user = 0xEDDC,
    charger_temp_internal = 0xEDDB,
    charger_error_code = 0xEDDA,
    charger_current = 0xEDD7,
    charger_voltage = 0xEDD5,
    charger_state_additional_info = 0xEDD4,
    yield_today = 0xEDD3,
    power_max_today = 0xEDD2,
    yield_yesterday = 0xEDD1,
    power_max_yesterday = 0xEDD0,
    voltage_settings_range = 0xEDCE,
    history_version = 0xEDCD,
    streetlight_version = 0xEDCC,
    equalise_current_max = 0xEDC7,
    equalise_voltage_max = 0xEDC6,
    adjustable_voltage_min = 0x2211,
    adjustable_voltage_max = 0x2212,
    // DC channel registers - MPPT RS models only
    battery_ripple_voltage = 0xED8B,
    battery_voltage_repl_0xEDD5 = 0xED8D,
    battery_current_repl_0xEDD7 = 0xED8F,
    // Solar panel data registers
    num_of_mppt_trackers = 0x0244,
    panel_current_max = 0xEDBF,
    panel_power = 0xEDBC,
    panel_voltage = 0xEDBB,
    panel_current = 0xEDBD,
    panel_voltage_max = 0xEDB8,
    tacker_mode = 0xEDB3,
    panel_starting_voltage = 0xEDB2,
    panel_input_resistance = 0xEDB1,
    // Solar panel data individual MPPT trackers registers - MPPT RS models only 
    // TODO: add tracker specific values
    // Panel power (see 0xEDBC) 0xECCC 0xECDC 0xECEC 0xECFC
    // Panel voltage (see 0xEDBB) 0xECCB 0xECDB 0xECEB 0xECFB
    // Panel current (see 0xEDBD) 0xECCD 0xECDD 0xECED 0xECFD
    // Tracker mode (see 0xEDB3) 0xECC3 0xECD3 0xECE3 0xECF3
    // Load output data/settings registers
    load_current = 0xEDAD,
    load_offset_voltage = 0xEDAC,
    load_output_control = 0xEDAB,
    load_output_voltage = 0xEDA9,
    load_output_state = 0xEDA8,
    load_switch_high_level = 0xED9D,
    load_switch_low_level = 0xED9C,
    load_output_off_reason = 0xED91,
    load_AES_timer = 0xED90,
    // TOOD: everything from 'Relay settings registers'
};

struct flags {
    uint8_t unkown_id : 1;
    uint8_t not_supported : 1;
    uint8_t parameter_error : 1;
};

union flags_union {
    uint8_t byte;
    flags bits;
    flags_union() : byte(0) {}
};

enum class data_type {
    none,
    sint8,
    sint16,
    sint32,
    uint8,
    uint16,
    uint32,
    string
};

struct id_metadata {
    data_type type;
    float scale;
    const char* unit;
};

const std::map<const id,const id_metadata> id_metadata_map = {
    {id::zero,                                  {data_type::none, 0.0, ""}},
    {id::product_id,                            {data_type::uint32, 0.0 , ""}},
    {id::group_id,                              {data_type::uint8, 0.0, ""}},
    {id::serial_number,                         {data_type::string, 0.0, ""}},
    {id::model_name,                            {data_type::string, 0.0, ""}},
    {id::capabilities,                          {data_type::uint32, 0.0, ""}},
    // Generic device control registers 
    {id::device_mode,                           {data_type::uint8, 0.0, ""}},
    {id::device_state,                          {data_type::uint8, 0.0, ""}},
    {id::remote_control_conf,                   {data_type::uint32, 0.0, ""}},
    {id::device_off_reason_8,                   {data_type::uint8, 0.0, ""}},
    {id::device_off_reason_32,                  {data_type::uint32, 0.0, ""}},
    // Battery settings registers
    {id::battery_safe_mode,                     {data_type::uint8, 0.0, ""}},
    {id::battery_adaptive_mode,                 {data_type::uint8, 0.0, ""}},
    {id::battery_automatic_eq_mode,             {data_type::uint8, 0.0, ""}},
    {id::battery_bulk_time_limit,               {data_type::uint16, 0.01, "hours"}},
    {id::battery_absorption_time_limit,         {data_type::uint16, 0.01, "hours"}},
    {id::battery_absorption_voltage,            {data_type::uint16, 0.01, "V"}},
    {id::battery_float_voltage,                 {data_type::uint16, 0.01, "V"}},
    {id::battery_equalisation_voltage,          {data_type::uint16, 0.01, "V"}},
    {id::battery_temperature_compensation,      {data_type::sint16, 0.01, "mV/K"}},
    {id::battery_type,                          {data_type::uint8, 1.0, ""}},
    {id::battery_max_current,                   {data_type::uint16, 0.1, "A"}},
    {id::battery_voltage,                       {data_type::uint8, 1.0, "V"}},
    {id::battery_temperature,                   {data_type::uint16, 0.01, "K"}},
    {id::battery_voltage_setting,               {data_type::uint8, 1.0, "V"}},
    {id::battery_bms_present,                   {data_type::uint8, 0.0, ""}},
    {id::battery_tail_current,                  {data_type::uint16, 0.1, "A"}}, // No unit in documentation
    {id::battery_low_temp_charge_current,       {data_type::uint16, 0.1, "A"}},
    {id::battery_auto_equalise_stop_voltage,    {data_type::uint8, 0.0, ""}},
    {id::battery_equalisation_current_level,    {data_type::uint8, 1.0, "%"}},
    {id::battery_equalisation_duration,         {data_type::uint16, 0.01, "hours"}},
    {id::battery_rebulk_voltage_offset,         {data_type::uint16, 0.01, "V"}},
    {id::battery_low_temp_level,                {data_type::sint16, 0.01, "°C"}},
    {id::battery_voltage_compensation,          {data_type::uint16, 0.01, "V"}},
    // Charger data registers
    //TODO: implement all values we need
    {id::panel_voltage,                         {data_type::uint16, 0.01, "V"}},
    {id::panel_power,                           {data_type::uint32, 0.01, "W"}},
    {id::yield_today,                           {data_type::uint16, 0.01, "kWH"}}, // This was uint32 up to firmware version 1.12, safe to ignore i hope, why would you change this, save 2 bytes for a binary incompatibility? at least v1.12 is really ancient
    {id::yield_system,                          {data_type::uint32, 0.01, "kWH"}}, // but this stayed uint32 ...
    {id::charger_voltage,                       {data_type::uint16, 0.01, "V"}},
    {id::charger_current,                       {data_type::uint16, 0.1, "A"}}
};

const id_metadata* get_id_metadata(const id id);

static_assert(sizeof(flags_union) == sizeof(uint8_t), "Flags union/struct should be at most 1 byte");

struct VEValue
{
    data_type type;
    union {
        int8_t sint8_value;
        uint8_t uint8_value;
        int16_t sint16_value;
        uint16_t uint16_value;
        int32_t sint32_value;
        uint32_t uint32_value;
    };
    std::string str_value;

    VEValue() : type(data_type::none), sint32_value(0) {}

    // Templated constructor for initializing with a specific type and value
    // TODO: add string type handling (or not? check in the docs if setting strings is even needed)
    // TODO: rework the type handling, this feels bad :/
    template<data_type T, typename ValueType>
    VEValue(ValueType value) : type(T) {
        set_value<T>(value);
    }

    // Overloaded constructor for uint16_t
    VEValue(uint16_t value) : type(data_type::uint16), uint16_value(value) {}

    // Overloaded constructor for uint8_t
    VEValue(uint8_t value) : type(data_type::uint8), uint8_value(value) {}

    // Overloaded constructor for sint8_t
    VEValue(int8_t value) : type(data_type::sint8), sint8_value(value) {}

    // Overloaded constructor for sint16_t
    VEValue(int16_t value) : type(data_type::sint16), sint16_value(value) {}

    // Overloaded constructor for sint32_t
    VEValue(int32_t value) : type(data_type::sint32), sint32_value(value) {}

    // Overloaded constructor for uint32_t
    VEValue(uint32_t value) : type(data_type::uint32), uint32_value(value) {}

    // Helper method to set the appropriate union member based on the enum type
    template<data_type T, typename ValueType>
    void set_value(ValueType value) {
        // Implementation will be based on the type provided
        if (T == data_type::sint8) {
            sint8_value = value;
        } else if (T == data_type::uint8) {
            uint8_value = value;
        } else if (T == data_type::sint16) {
            sint16_value = value;
        } else if (T == data_type::uint16) {
            uint16_value = value;
        } else if (T == data_type::sint32) {
            sint32_value = value;
        } else if (T == data_type::uint32) {
            uint32_value = value;
        }
    }
};

class VEMessage
{
    struct command_t {
        ve::command code = ve::command::zero;
        ve::id id = ve::id::zero;
        ve::VEValue value = VEValue();
        ve::flags_union flags = ve::flags_union();
        uint8_t checksum = 0x55;
        command_t(ve::command c, ve::id i, ve::VEValue v = VEValue(), ve::flags_union f = ve::flags_union())
            : code(c), id(i), value(v), flags(f) {}
        command_t() {}
    } command;
    struct {
        ve::response code = ve::response::zero;
        ve::id id = ve::id::zero;
        ve::VEValue value = VEValue();
        ve::flags_union flags = ve::flags_union();
        uint8_t checksum = 0x55;
    } response;
    template <typename T>
    void update_checksum(T val) {
        using view_t = uint8_t[sizeof(T)];
        const view_t& view = *(view_t*)&val;
        for(const uint8_t& byte : view) {
            command.checksum -= byte;
        }
    }
    template <typename T ,size_t N>
    void update_checksum(const T(&val)[N]) {
        for(const T& v : val) {
            update_checksum(v);
        }
    }
    template <typename T>
    void update_checksum(const T *val ,size_t len) {
        for(int i=0; i < len; i++){
            update_checksum(val[i]);
        }
    }
    template <typename T>
    bool msg_append_hex(T val, size_t width) {
        uint32_t val_le;
        switch (sizeof(T)) {
            case 1:
                val_le = static_cast<uint8_t>(val);
                break;
            case 2:
                val_le = __builtin_bswap16(static_cast<uint16_t>(val));
                break;
            case 4:
                val_le = __builtin_bswap32(static_cast<uint32_t>(val));
                break;
            default:
                return false;
        }
        // Allocate buffer with space for null terminator
        char buffer[width + 1];
        int written = snprintf(buffer, sizeof(buffer), "%0*X", static_cast<int>(width), val_le);

        // Check if snprintf succeeded, both should not occur
        if (written < 0 || written > static_cast<int>(width)) {
            return false;
        }

        // Append the exact number of requested characters
        hex_command.append(buffer, width);
        return true;
    }
    template <typename T>
    bool msg_append_hex(T val) {
        return msg_append_hex(val, sizeof(T)*2);
    }
    template <typename T>
    bool msg_append_with_checksum(T val,size_t width) {
        bool ret = msg_append_hex(val,width);
        if(ret) {
            update_checksum(val);
        }
        return ret;
    }
    template <typename T>
    bool msg_append_with_checksum(T val) {
        bool ret = msg_append_hex(val);
        if(ret) {
            update_checksum(val);
        }
        return ret;
    }
    template <typename T>
    bool msg_decode_hex(T& val, size_t offs, size_t width) {
        int val_le{};
        std::string sub;
        if ( (offs + width) > hex_response.size()) return false;
        sub = hex_response.substr(offs, width);
        val_le = std::stoi(sub, nullptr,16);
        //TODO: handle conversion error?
        //TODO: test if endian swap works as intended (for every size!)
        switch(sizeof(T)) {
            case 1:
                val = static_cast<T>(val_le);
                break;
            case 2:
                val = static_cast<T>(__builtin_bswap16(val_le));
                break;
            case 4:
                val = static_cast<T>(__builtin_bswap32(val_le));
                break;
            default:
                return false;
        }
        return true;
    }
    template <typename T>
    bool msg_decode_hex(T& val, size_t offs) {
        return msg_decode_hex(val, offs, sizeof(T)*2);
    }
public:
    std::string hex_command;
    std::string hex_response;
    VEMessage();
    VEMessage(ve::command c, ve::id i, ve::VEValue v = VEValue(), ve::flags_union f = ve::flags_union());
    bool msg_generate();
    bool msg_generate(ve::command c, ve::id i, ve::VEValue v = VEValue(), ve::flags_union f = ve::flags_union());
    bool msg_decode();
    bool msg_decode(const std::string& msg);
    void resp_debug();
    bool resp_check();
    uint32_t get_resp_value();
};

class VEDirect
{
    HardwareSerial *ve_serial;
public:
    VEDirect();
    void debug(VEMessage &vemessage);
    void debug_next(VEMessage &vemessage);
    void discard();
    void command_product_id();
    void command_get();
    void send(VEMessage& vemessage);
    bool receive_next(VEMessage& vemessage);
    bool receive_response(VEMessage& vemessage);
    void generate_send(VEMessage& vemessage);
    void send(const std::string& message);
    bool get_value(ve::id id, int32_t& int_value);
};

}// namespace ve
