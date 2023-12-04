#include <ArtNet.h>

using namespace art_net;

namespace art_net {
    OpCode art_net_get_packet_op_code(ArtNetBasePacket *data) {
        uint16_t opCode = (((uint16_t)data->OpCodeHi) << 8) | data->OpCodeLo;
        return (OpCode) opCode;
    }

    void ArtNet::sendPollReply(uint32_t dstIP, uint16_t dstPort) {
        ArtNetPollReplyPacket replyPacket;

        memset(&replyPacket, 0, sizeof(replyPacket));

        memcpy(replyPacket.ID, ART_NET_ID, sizeof(ART_NET_ID));
        replyPacket.OpCodeHi = ((uint16_t)OpCode::PollReply >> 8);
        replyPacket.OpCodeLo = ((uint16_t)OpCode::PollReply & 0xFF);

        memcpy(replyPacket.ip, &ip, sizeof(replyPacket.ip));

        replyPacket.port_l = 0x36;
        replyPacket.port_h = 0x19;

        replyPacket.ver_h = 0x0;
        replyPacket.ver_l = 14U;

        replyPacket.net_sw = net;
        replyPacket.sub_sw = subnet;

        replyPacket.oem_h = 0;
        replyPacket.oem_l = 0xFF;

        memcpy(replyPacket.short_name, ART_NET_SHORT_NAME, sizeof(ART_NET_SHORT_NAME));
        memcpy(replyPacket.long_name, ART_NET_LONG_NAME, sizeof(ART_NET_LONG_NAME));

        memcpy(replyPacket.node_report, "#0001 [0000] OK", 16);

        replyPacket.num_ports_h = 0;
        replyPacket.num_ports_l = ART_NET_OUTPUT_UNIVERSE_COUNT;

        for (uint8_t i = 0; i < NUM_POLLREPLY_PUBLIC_PORT_LIMIT; i++) {
            if (i < ART_NET_OUTPUT_UNIVERSE_COUNT) {
                // TODO: Add support for input
                replyPacket.sw_out[i] = i;
                replyPacket.port_types[i] = 0b10100000;
                replyPacket.good_output[i] = 0b10000000;
            }
        }

        memcpy(replyPacket.mac, mac, sizeof(mac));

        replyPacket.status_2 = 0b00001110;

        sendPacketFunc(dstIP, dstPort, (uint8_t*) &replyPacket, sizeof(replyPacket));
    }

    void ArtNet::setSendPacketCallback(std::function<void(uint32_t, uint16_t, uint8_t*, uint32_t)> func) {
        sendPacketFunc = func;
    }

    void ArtNet::setDmxDataCallback(std::function<void(uint8_t, uint8_t, uint8_t*, uint16_t)> func) {
        dmxDataCallback = func;
    }

    void ArtNet::onDmxPacket(ArtNetDmxDataPacket *packet) {
        if (packet->Net != net) {
            return;
        } 

        if (subnet != packet->SubUni >> 4) {
            return;
        }

        uint8_t universe = packet->SubUni & 0x0F;

        if (universe >= ART_NET_OUTPUT_UNIVERSE_COUNT) {
            return;
        }

        if (packet->Sequence > 0) {
            if (packet->Sequence <= (0xF + 1) && receiveSequence[universe] >= (0xFF - 1 - 0xF)) {
                receiveSequence[universe] = packet->Sequence;
            } else if (receiveSequence[universe] > packet->Sequence) {
                return;
            } else {
                receiveSequence[universe] = packet->Sequence;
            }
        }

        uint16_t dataLength = (uint16_t)packet->LengthHi << 8;
        dataLength |= packet->LengthLo;

        if (dataLength > 512) {
            return;
        }

        dmxDataCallback(universe, 0, packet->Data, dataLength);
    }

    PacketParseStatus ArtNet::onPacketReceived(uint32_t remoteIP, uint16_t remotePort, uint8_t *data, uint32_t size) {
        if (size < sizeof(ArtNetBasePacket)) {
            return PacketParseStatus::BadSize;
        }

        if (strncmp(ART_NET_ID, (const char*)data, sizeof(ART_NET_ID)) != 0) {
            return PacketParseStatus::BadId;
        }

        ArtNetBasePacket *basePacket = (ArtNetBasePacket*) data;

        switch (art_net_get_packet_op_code(basePacket)) {
            case OpCode::Address:
            case OpCode::Input:
            case OpCode::Poll: {
                sendPollReply(remoteIP, remotePort);
                return PacketParseStatus::Success;
            }
            case OpCode::Dmx: {
                onDmxPacket((ArtNetDmxDataPacket*) basePacket);
                return PacketParseStatus::Success;
            }
            default: {
                return PacketParseStatus::BadOpCode;
            }
        };

        return PacketParseStatus::Invalid;
    }
}