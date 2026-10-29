#include "ve-direct.h"
#include <cstdint>

ve::VEDirect::VEDirect()
    : ve_serial(&Serial1)
{
    ve_serial->setRxBufferSize(SERIAL_BUFFER_SIZE);
    ve_serial->begin(KB_VED_BAUD, SERIAL_8N1, KB_VED_RXPIN, KB_VED_TXPIN);
    ve_serial->setRxTimeout(8);
}

void ve::VEDirect::debug(ve::VEMessage &vemessage)
{
    LOG_DEBUG("VEDirect debug M[ %s ] > R[ %s ]\n", vemessage.hex_command.c_str(),vemessage.hex_response.c_str());
}

void ve::VEDirect::debug_next(ve::VEMessage &vemessage)
{
    if(receive_next(vemessage)) {
        debug(vemessage);
    } else {
        LOG_DEBUG("VEDirect no valid message received!\n");
    }
}

bool ve::VEDirect::receive_next(ve::VEMessage &vemessage)
{
    //LOG_DEBUG("VEDirect receive_next\n");
    vemessage.hex_response.clear();
    bool is_hex_command = false;
    int missed = 0;
    do { //TODO: this read blocks until answer is received.
         // we should probably implement this with a timeout to not have the whole device blocked by a broken mppt (connection)?
        unsigned char received = ve_serial->read();
        // Check if message starts with : , which marks the start of a hex message
        if (received == ':') {
            vemessage.hex_response.push_back(received);
            is_hex_command = true;
            continue;
        // messages end with a newline
        } else if (received == 0xFF && missed < 1) {
            // Wait for a small processing delay before we get a reponse
            delay(5);
            missed++;
            continue;
        } else if (is_hex_command) {
            if (received == '\n') {
                return true;
            // put only hex characters
            } else if (isxdigit(received)) {
                vemessage.hex_response.push_back(received);
            } else if (received == 0xFF && missed < 3) {
                // Some devices seem to be slow to respond even in the middle of a hex message
                // seems like they dont prepare the message, but build it on the fly? wierd for these short messages....
                // So we wait for them, but only 3 x 10 ms max
                delay(5);
                missed++;
                continue;
            } else { // recieved garbage inside hex command
                LOG_DEBUG("VEDirect error in hex command: %c / 0x%02X  missed: %d\n", received, received,missed);
                return false;
            }
        } else { // reset in case we recieve non hex
            //LOG_DEBUG("VEDirect skipping char: %c / 0x%02X\n", received, received);
        }
    // we force read the first char and wait for full hex packet
    } while (ve_serial->available() > 0 || is_hex_command);
    return false;
}

void ve::VEDirect::discard()
{
    while (int bytes = ve_serial->available()) {
        LOG_DEBUG("VEDirect discarding %d bytes\n", bytes);
        for(int i = 0;i<bytes;i++) ve_serial->read();
    }
}

void ve::VEDirect::send(const std::string& message)
{
    LOG_DEBUG("VEMessage sending message: %s\n", message.c_str());
    ve_serial->write(message.c_str(),message.length());
    ve_serial->write('\n');
}

void ve::VEDirect::send(VEMessage &vemessage)
{
    send(vemessage.hex_command);
}

void ve::VEDirect::generate_send(VEMessage &vemessage)
{
    vemessage.msg_generate();
    send(vemessage.hex_command);
}

bool ve::VEDirect::receive_response(ve::VEMessage &vemessage) {
    //Every attempt is with 15ms max delay + processing. so we should be done in max ~50ms
    for(int i = 0; i<3;i++) {
        if(receive_next(vemessage)){
            if(vemessage.msg_decode()){
                if(vemessage.resp_check()){
                    return true;
                } else {
                    LOG_DEBUG("VEDirect failed to check response match at attempt %d\n",i+1);
                    debug(vemessage);
                }
            } else {
                LOG_DEBUG("VEDirect failed to decode at attempt %d\n",i+1);
                debug(vemessage);
            }
        } else {
            LOG_DEBUG("VEDirect failed to recieve at attempt %d\n",i+1);
            debug(vemessage);
        }
    }
    return false;
}

uint32_t ve::VEMessage::get_resp_value() {
    return response.value.uint32_value;
}

bool ve::VEDirect::get_value(ve::id id, int32_t& int_value) {
    ve::VEMessage msg(ve::command::get, id);
    discard();
    generate_send(msg);
    if(receive_response(msg)) {
        debug(msg);
        msg.resp_debug();
        int_value = msg.get_resp_value();
        return true;
    }
    return false;
}

const ve::id_metadata* ve::get_id_metadata(const ve::id id) {
    auto it = id_metadata_map.find(id);
    if (it != id_metadata_map.end()) {
        return &it->second;
    } else {
        return nullptr;
    }
}

ve::VEMessage::VEMessage()
    : command(),
      response()
{
}

ve::VEMessage::VEMessage(ve::command code, ve::id id, ve::VEValue value, ve::flags_union flags)
    : command(code,id,value,flags),
      response()
{
}

bool ve::VEMessage::msg_generate(ve::command code, ve::id id, ve::VEValue value, ve::flags_union flags){
    this->command.code = code;
    this->command.id = id;
    this->command.value = value;
    this->command.flags = flags;
    return msg_generate();
}

bool ve::VEMessage::msg_generate() {
    hex_command = ":";
    //TODO: parametrize options
    msg_append_with_checksum(command.code,1);
    switch(command.code) {
        case command::enter_boot:
            //TODO
            break;
        case command::ping:
        case command::app_version:
        case command::product_id:
        case command::restart:
            break;
        case command::get:
        case command::set:
            msg_append_with_checksum(command.id);
            msg_append_with_checksum(command.flags.byte);
            break;
        default:
            return false;
    }
    // TODO: this should be redundant for uint8_t ?
    if(command.code == command::set) {
        const id_metadata* meta = get_id_metadata(command.id);
        if (meta) {
            switch(meta->type) {
                case data_type::sint8:
                    msg_append_with_checksum(command.value.sint8_value);
                    break;
                case data_type::sint16:
                    msg_append_with_checksum(command.value.sint16_value);
                    break;
                case data_type::sint32:
                    msg_append_with_checksum(command.value.sint32_value);
                    break;
                case data_type::uint8:
                    msg_append_with_checksum(command.value.uint8_value);
                    break;
                case data_type::uint16:
                    msg_append_with_checksum(command.value.uint16_value);
                    break;
                case data_type::uint32:
                    msg_append_with_checksum(command.value.uint32_value);
                    break;
                case data_type::string:
                    //TODO: implement string type handling
                    return false;
                case data_type::none:
                default:
                    return false;
            }
        } else {
            return false;
        }
    }
    command.checksum %= 256;
    msg_append_hex(command.checksum);
    //hex_command += '\n';
    return true;
}

bool ve::VEMessage::msg_decode(const std::string& msg) {
    hex_response = msg;
    return msg_decode();
}

bool ve::VEMessage::msg_decode() {
    //TODO: reset checksum?
    if(hex_response.size() < 2) {
        LOG_DEBUG("VEDirect decode : command to short %d\n", hex_response.size());
        return false;
    }
    if (hex_response[0] != ':') {
        LOG_DEBUG("VEDirect decocde: no ':'\n");
        return false;
    }
    msg_decode_hex(response.code,1,1);
    // LOG_DEBUG("VEDirect decocde: reponse %02X\n",(uint8_t)response.code);
    switch(response.code) {
        case response::done:
            //TODO: response depends on command, so should parse that
            // but this also means reponses have to be in order, as else we would not be able to match the response to a command
            // or there can only be one command send at a time?
            break;
        case response::unknown:
            //TODO: reponse data is the unknown command
            break;
        case response::error:
            //TODO: find out the response size/data for errors
            break;
        case response::ping:
            //TODO: parse this command. It has a wierd format. why are they suddenly trying to save bits? :D
            break;
        case response::get:
        case response::set:
        case response::async: { //TODO: how to handle async messages? like get/set, but different?
            if(hex_response.size() < 8) {
                LOG_DEBUG("VEDirect decode : command to short %d\n", hex_response.size());
                return false;
            }
            msg_decode_hex(response.id,2);
            // LOG_DEBUG("VEDirect decocde: id %04X\n",(uint16_t)response.id);
            msg_decode_hex(response.flags.byte,6);
            // LOG_DEBUG("VEDirect decocde: flags %02X\n",(uint8_t)response.flags.byte);
            const id_metadata* meta = get_id_metadata(response.id);
            if (meta) {
                bool ret = false;
                switch(meta->type) {
                    case data_type::sint8:
                        ret = msg_decode_hex(response.value.sint8_value, 8);
                        break;
                    case data_type::sint16:
                        ret = msg_decode_hex(response.value.sint16_value, 8);
                        break;
                    case data_type::sint32:
                        ret = msg_decode_hex(response.value.sint32_value, 8);
                        break;
                    case data_type::uint8:
                        ret = msg_decode_hex(response.value.uint8_value, 8);
                        break;
                    case data_type::uint16:
                        ret = msg_decode_hex(response.value.uint16_value, 8);
                        break;
                    case data_type::uint32:
                        ret = msg_decode_hex(response.value.uint32_value, 8);
                        break;
                    case data_type::string:
                        //TODO: implement string type handling
                        return false;
                    case data_type::none:
                    default:
                        return false;
                }
                // LOG_DEBUG("VEDirect decocde: value %08X\n",response.value.uint32_value);
                return ret;
            } else {
                return false;
            }
            //TODO: Implement
            break;
        }
        default:
            return false;
    }
    //TODO: implement flag handling
    //TODO: implement checksum check
    return false;
}
void ve::VEMessage::resp_debug() {
    LOG_DEBUG("debugging response of type 0x%05X\n",(uint8_t)response.code);
    switch(response.code) {
        case response::done:
        case response::unknown:
        case response::error:
        case response::ping:
            break;
        case response::get:
        case response::set:
        case response::async: { //TODO: how to handle async messages? like get/set, but different?
            const id_metadata* meta = get_id_metadata(response.id);
            if (meta) {
                switch(response.id) {
                    case id::battery_max_current:
                        LOG_DEBUG("Battery max current is %f %s\n", response.value.uint16_value * meta->scale , meta->unit);
                        break;
                    case id::battery_voltage:
                        LOG_DEBUG("Battery voltage is %f %s\n", response.value.uint8_value * meta->scale , meta->unit);
                        break;
                    case id::charger_voltage:
                        LOG_DEBUG("Charger voltage is %f %s\n", response.value.uint16_value * meta->scale , meta->unit);
                        break;
                    case id::charger_current:
                        LOG_DEBUG("Charger current is %f %s\n", response.value.uint16_value * meta->scale , meta->unit);
                        break;
                    default:
                        LOG_DEBUG("Unknown id 0x%04X\n", (uint16_t)response.id);
                }
            } else {
                LOG_DEBUG("No metadata for id 0x%04X\n", (uint16_t)response.id);
            }
            break;
        }
        default:
            return;
    }
}

bool ve::VEMessage::resp_check() {
    switch(command.code) {
        case command::zero:
        case command::ping:
        case command::app_version:
        case command::product_id:
        case command::restart:
            return false;
            break;
        case command::get:
        case command::set: {
            if(command.code == response.code && command.id == response.id) {
                return true;
            } else {
                return false;
            }
            break;
        }
        case command::async:
        default:
            return false;
    }
}

// (0x55 − (0x7+0xDB+0xED) ) & 0xff
