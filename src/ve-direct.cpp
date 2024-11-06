#include "ve-direct.h"

ve::VEDirect::VEDirect()
    : ve_serial(&Serial1)
{
    ve_serial->setRxBufferSize(SERIAL_BUFFER_SIZE);
    ve_serial->begin(KB_VED_BAUD, SERIAL_8N1, KB_VED_RXPIN, KB_VED_TXPIN);
}
void ve::VEDirect::debug()
{
    bool is_hex_msg = false;
    while (ve_serial->available()) {
        unsigned char received = ve_serial->read();
        // Check if message starts with : , which marks the start of a hex message
        if (received == ':') {
            bufferIndex = 0;
            buffer[bufferIndex++] = received;
            is_hex_msg = true;
            continue;
        // messages end with a newline
        } else if (received == '\n') {
            // terminate string with null character
            buffer[bufferIndex] = '\0';
            if (is_hex_msg) {
                LOG_DEBUG("VEDirect Message: %s\n", &buffer);
                // reset, as we don't know if there is plain ascii between the hex messages
                is_hex_msg = false;
            }
            bufferIndex = 0;
        // put only hex characters in the buffer (if we have enough space)
        } else if ( isxdigit(received) && (bufferIndex < KB_VED_BUFFER_SIZE - 1)) {
            buffer[bufferIndex++] = received;
        } else { // reset in case we recieve non hex
            is_hex_msg = false;
            bufferIndex = 0;
        }
    }
}

// void ve::VEDirect::command_get(uint16_t id,uint8_t flags)
// {
//     uint16_t chksum = 0x55;
//     char input[2 + ve::command_length(ve::command::get) * 2 + 2 + 1];
//     std::snprintf(input,sizeof(input),":%01x%02x%02x%02x00", ve::command::get,0,0,0);
// }

// void ve::VEDirect::command_product_id()
// {
//     uint16_t chksum = 0x55;
//     char input[2 + ve::command_length(ve::command::product_id) * 2 + 2 + 1 + 1];
//     chksum -= (uint16_t)ve::command::product_id;
//     chksum = chksum % 256;
//     std::snprintf(input,sizeof(input),":%01x%02x\n", ve::command::product_id,chksum);
//     LOG_DEBUG("VEDirect command: %s\n", input);
//     ve_serial->write(input,sizeof(input));
// }

void ve::VEDirect::send(const std::string& message)
{
    ve_serial->write(message.c_str(),message.length());
}

void ve::VEDirect::send(VEMessage &vemessage)
{
    send(vemessage.get_hex_msg());
}

void ve::VEDirect::generate_send(VEMessage &vemessage)
{
    vemessage.msg_generate();
    send(vemessage.get_hex_msg());
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
    : command(command::zero),
      id(id::zero),
      flags(),
      hex_msg()
{
}

bool ve::VEMessage::msg_generate() {
    hex_msg = ":";
    //TODO: parametrize options
    command = command::set;
    id = id::battery_max_current;
    value = VEValue(uint16_t(0x004F));
    msg_append_with_checksum(command,1);
    switch(command) {
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
            msg_append_with_checksum(id);
            msg_append_with_checksum(flags.byte);
            break;
        default:
            return false;
    }
    // TODO: this should be redundant for uint8_t ?
    if(command == command::set) {
        const id_metadata* meta = get_id_metadata(id);
        if (meta) {
            switch(meta->type) {
                case data_type::sint8:
                    msg_append_with_checksum(value.sint8_value);
                    break;
                case data_type::uint16:
                    msg_append_with_checksum(value.uint16_value);
                    break;
                default:
                    return false;
            }
        } else {
            return false;
        }
    }
    checksum %= 256;
    msg_append_hex(checksum);
    hex_msg += '\n';
    return true;
}

bool ve::VEMessage::msg_decode(const std::string& msg) {
    return false;
}

// (0x55 − (0x7+0xDB+0xED) ) & 0xff
