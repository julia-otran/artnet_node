#include <Arduino.h>

#define ART_NET_ID "Art-Net"

#ifndef ART_NET_OUTPUT_UNIVERSE_COUNT
#define ART_NET_OUTPUT_UNIVERSE_COUNT 4
#endif

#ifndef ART_NET_SHORT_NAME
#define ART_NET_SHORT_NAME "ESP-ArtNet"
#endif

#ifndef ART_NET_LONG_NAME
#define ART_NET_LONG_NAME "ESP ArtNet to DMX512 Node"
#endif

namespace art_net {
    enum class PacketParseStatus : int8_t {
        BadSize = -1,
        BadId = -2,
        BadOpCode = -3,
        Invalid = -120,
        Success = 0
    };

    static constexpr uint8_t NUM_PIXELS_PER_UNIV {170};
    static constexpr size_t NUM_POLLREPLY_PUBLIC_PORT_LIMIT {4};

    enum class OpCode : uint16_t {
        // Device Discovery
        Poll = 0x2000,
        PollReply = 0x2100,
        // Device Configuration
        Address = 0x6000,
        Input = 0x7000,
        IpProg = 0xF800,
        IpProgReply = 0xF900,
        Command = 0x2400,
        // Streaming Control
        Dmx = 0x5000,
        Nzs = 0x5100,
        Sync = 0x5200,
        // RDM
        TodRequest = 0x8000,
        TodData = 0x8100,
        TodControl = 0x8200,
        Rdm = 0x8300,
        RdmSub = 0x8400,
        // Time-Keeping
        TimeCode = 0x9700,
        TimeSync = 0x9800,
        // Triggering
        Trigger = 0x9900,
        // Diagnostics
        DiagData = 0x2300,
        // File Transfer
        FirmwareMaster = 0xF200,
        FirmwareReply = 0xF300,
        Directory = 0x9A00,
        DirectoryReply = 0x9B00,
        FileTnMaster = 0xF400,
        FileFnMaster = 0xF500,
        FileFnReply = 0xF600,
        // N/A
        NA = 0x0000,
    };

    typedef struct ArtNetBasePacket {
        char ID[8];
        uint8_t OpCodeLo;
        uint8_t OpCodeHi;
    } ArtNetBasePacket;

    typedef struct ArtNetPollReplyPacket {
        char ID[8];
        uint8_t OpCodeLo;
        uint8_t OpCodeHi;
        uint8_t ip[4];
        uint8_t port_l;
        uint8_t port_h;
        uint8_t ver_h;
        uint8_t ver_l;
        uint8_t net_sw;
        uint8_t sub_sw;
        uint8_t oem_h;
        uint8_t oem_l;
        uint8_t ubea_ver;
        uint8_t status_1;
        uint8_t esta_man_l;
        uint8_t esta_man_h;
        uint8_t short_name[18];
        uint8_t long_name[64];
        uint8_t node_report[64];
        uint8_t num_ports_h;
        uint8_t num_ports_l;
        uint8_t port_types[NUM_POLLREPLY_PUBLIC_PORT_LIMIT];
        uint8_t good_input[NUM_POLLREPLY_PUBLIC_PORT_LIMIT];
        uint8_t good_output[NUM_POLLREPLY_PUBLIC_PORT_LIMIT];
        uint8_t sw_in[NUM_POLLREPLY_PUBLIC_PORT_LIMIT];
        uint8_t sw_out[NUM_POLLREPLY_PUBLIC_PORT_LIMIT];
        uint8_t sw_video;
        uint8_t sw_macro;
        uint8_t sw_remote;
        uint8_t spare[3];
        uint8_t style;
        uint8_t mac[6];
        uint8_t bind_ip[4];
        uint8_t bind_index;
        uint8_t status_2;
        uint8_t filler[26];
        uint8_t b[239];
    } ArtNetPollReplyPacket;

    typedef struct ArtNetDmxDataPacket {
        char ID[8];
        uint8_t OpCodeLo;
        uint8_t OpCodeHi;
        uint8_t ProtVerHi, ProtVerLo;
        uint8_t Sequence;
        uint8_t Physical;
        uint8_t SubUni;
        uint8_t Net;
        uint8_t LengthHi;
        uint8_t LengthLo;
        uint8_t Data[512];
    } ArtNetDmxDataPacket;

    class ArtNet {
        public:
            uint8_t net, subnet, mac[6], receiveSequence[ART_NET_OUTPUT_UNIVERSE_COUNT];
            uint32_t ip;
            void setSendPacketCallback(std::function<void(uint32_t, uint16_t, uint8_t*, uint32_t)> func);
            void setDmxDataCallback(std::function<void(uint8_t, uint8_t, uint8_t*, uint16_t)> func);
            PacketParseStatus onPacketReceived(uint32_t remoteIP, uint16_t remotePort, uint8_t *data, uint32_t size);
        private:
            std::function<void(uint32_t, uint16_t, uint8_t*, uint32_t)> sendPacketFunc;
            std::function<void(uint8_t, uint8_t, uint8_t*, uint16_t)> dmxDataCallback;
            void sendPollReply(uint32_t dstIP, uint16_t dstPort);
            void onDmxPacket(ArtNetDmxDataPacket *packet);
    };
}
