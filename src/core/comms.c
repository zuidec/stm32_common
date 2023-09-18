#include "core/comms.h"
#include "core/uart.h"
#include "core/crc8.h"

#define PACKET_BUFFER_LENGTH    (8)

typedef enum comms_state_t  {
    CommsState_length,
    CommsState_payload,
    CommsState_crc,
} comms_state_t;

static uint8_t data_byte_count = 0;
static comms_state_t state = CommsState_length;

static comms_packet_t temporary_packet = {
    .length = 0,
    .payload = {0},
    .crc = 0
};
static comms_packet_t re_tx_packet = {
    .length = 0,
    .payload = {0},
    .crc = 0
};
static comms_packet_t ack_packet = {
    .length = 0,
    .payload = {0},
    .crc = 0
};
static comms_packet_t last_tx_packet = {
    .length = 0,
    .payload = {0},
    .crc = 0
};
static comms_packet_t packet_buffer[PACKET_BUFFER_LENGTH];
static uint32_t packet_read_index = 0;
static uint32_t packet_write_index = 0;
static uint32_t packet_buffer_mask = PACKET_BUFFER_LENGTH -1;

static bool comms_is_re_tx_packet(const comms_packet_t* packet) {
    if(packet->length!=1)   {
        return false;
    }
    if(packet->payload[0]!=PACKET_RE_TX_PAYLOAD)    {
        return false;
    }
     for (uint8_t i = 1; i < PACKET_PAYLOAD_LENGTH; i++) {
        if(packet->payload[i]!=0xff)    {
            return false;
        }
    }
    return true;
}

static bool comms_is_ack_packet(const comms_packet_t* packet) {
    if(packet->length!=1)   {
        return false;
    }
    if(packet->payload[0]!=PACKET_ACK_PAYLOAD)    {
        return false;
    }
     for (uint8_t i = 1; i < PACKET_PAYLOAD_LENGTH; i++) {
        if(packet->payload[i]!=0xff)    {
            return false;
        }
    }
    return true;
}

static void comms_packet_copy(const comms_packet_t* source, comms_packet_t* destination)  {
    destination->length = source->length;

    for (uint8_t i = 1; i < PACKET_PAYLOAD_LENGTH; i++) {
        destination->payload[i] = source->payload[i];
    }

    destination->crc = source->crc;

}

void comms_setup(void)  {

    // Initialize the retransmit packet structure
    re_tx_packet.length = 1;
    re_tx_packet.payload[0] = PACKET_RE_TX_PAYLOAD;
    for (uint8_t i = 1; i < PACKET_PAYLOAD_LENGTH; i++) {
        re_tx_packet.payload[i] = 0xff;
    }
    re_tx_packet.crc = comms_compute_crc(&re_tx_packet);

    // Initialize the ack packet structure
    ack_packet.length = 1;
    ack_packet.payload[0] = PACKET_ACK_PAYLOAD;
    for (uint8_t i = 1; i < PACKET_PAYLOAD_LENGTH; i++) {
        ack_packet.payload[i] = 0xff;
    }
    ack_packet.crc = comms_compute_crc(&ack_packet);

}

void comms_update(void) {
    while (uart_data_available())   {
        switch (state)  {
            case CommsState_length:  {
                temporary_packet.length = uart_read_byte();
                state = CommsState_payload;
                break;
            }

            case CommsState_payload:    {
                temporary_packet.payload[data_byte_count++] = uart_read_byte();
                if(data_byte_count >= PACKET_PAYLOAD_LENGTH )   {
                    data_byte_count = 0;
                    state = CommsState_crc;
                }
                break;
            }

            case CommsState_crc:    {
                temporary_packet.crc = uart_read_byte();
                
                if(temporary_packet.crc != comms_compute_crc(&temporary_packet))    {
                    comms_write(&re_tx_packet);
                    state = CommsState_length;
                    break;
                }
                if(comms_is_re_tx_packet(&temporary_packet))    {
                    comms_write(&last_tx_packet);
                    state = CommsState_length;
                    break;
                }
                if(comms_is_ack_packet(&temporary_packet))  {
                    state = CommsState_length;
                    break;
                }
                
                uint32_t next_write_index = (packet_write_index+ 1 ) & packet_buffer_mask;
                if(next_write_index==packet_read_index) {
                    __asm__("BKPT #0");
                }
                comms_packet_copy(&temporary_packet, &packet_buffer[packet_write_index]);
                packet_write_index = next_write_index;
                comms_write(&ack_packet);
                state = CommsState_length;
                break;
            }
            default:    {
                state = CommsState_length;
                break;
            }
        }

    }

}

bool comms_packets_available(void)  {

    return packet_read_index != packet_write_index;
}

void comms_write(comms_packet_t* packet)    {
    uart_write((uint8_t*)packet, PACKET_LENGTH);
    comms_packet_copy(packet, &last_tx_packet );

}

void comms_read(comms_packet_t* packet) {
    comms_packet_copy(&packet_buffer[packet_read_index & packet_buffer_mask], packet);
    packet_read_index = (packet_read_index + 1) & packet_buffer_mask;

}

uint8_t comms_compute_crc(comms_packet_t* packet)   {
    return crc8((uint8_t*)packet, (PACKET_LENGTH - PACKET_CRC_BYTES));
}

