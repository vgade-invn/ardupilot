
#include "AP_Networking_LatencyTest.h"

#if AP_NETWORKING_LATENCYTEST_ENABLED
#include <GCS_MAVLink/GCS.h>

#ifndef AP_NETWORKING_LATENCYTEST_DEFAULT_PORT
#define AP_NETWORKING_LATENCYTEST_DEFAULT_PORT   5555
#endif

uint32_t AP_Networking_LatencyTest::latencyTest_last_rx_data = 0;
const AP_Param::GroupInfo AP_Networking_LatencyTest::var_info[] = {

    // @Param: PORT
    // @DisplayName: Port
    // @Description: Port
    // @Range: 1 65535
    // @User: Standard
    AP_GROUPINFO("PORT", 1, AP_Networking_LatencyTest, _eth.port, AP_NETWORKING_LATENCYTEST_DEFAULT_PORT),

    AP_GROUPINFO("DST_ADDR0", 2,  AP_Networking_LatencyTest,    _eth.ip[0],   255),
    AP_GROUPINFO("DST_ADDR1", 3,  AP_Networking_LatencyTest,    _eth.ip[1],   255),
    AP_GROUPINFO("DST_ADDR2", 4,  AP_Networking_LatencyTest,    _eth.ip[2],   255),
    AP_GROUPINFO("DST_ADDR3", 5,  AP_Networking_LatencyTest,    _eth.ip[3],   255),
    
    AP_GROUPEND
};

extern const AP_HAL::HAL& hal;

/// Constructor
AP_Networking_LatencyTest::AP_Networking_LatencyTest(AP_Networking &front, AP_Networking::AP_Networking_State &state,  AP_Networking_Params &params) :
    AP_Networking_Backend(front, state, params)
{
    AP_Param::setup_object_defaults(this, var_info);
    _state.var_info = var_info;
}

void AP_Networking_LatencyTest::init()
{
    if (_eth.pcb == nullptr) {
        _eth.pcb = udp_new_ip_type(IPADDR_TYPE_ANY);
        if (_eth.pcb != nullptr) {
            // set up RX callback
            udp_recv(_eth.pcb, AP_Networking_LatencyTest::latencytest_recv_callback, nullptr);
        }
    }

}

void AP_Networking_LatencyTest::update()
{
    if (_eth.pcb == nullptr) {
        // init failure or disabled
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _stats.update_last_ms < 1000) {
        // run this update() at 1Hz
        return;
    }
    _stats.update_last_ms = now_ms;

    // TODO: do we need to wrap this WITH_SEMAPHORE?
    const uint32_t last_rx_data = latencyTest_last_rx_data;

    if (last_rx_data == 0) {
        // We've never received a packet.
        // Wait 5 seconds before starting and then send a start packet
        // every second if we have a valid dest IP address
        if (now_ms > 5*1000 &&
            _eth.ip4_addr.addr != 0 &&
            _eth.ip4_addr.addr != 0xFFFFFFFF &&
            now_ms - _stats.start_ms >= 1000) {
            _stats.start_ms = now_ms;

            GCS_SEND_TEXT(MAV_SEVERITY_INFO,"NET: LatencyTest START %u", (unsigned)_stats.start_ms);

            // send initial packet
            LatencyTestPacket_t pkt {};
            pkt.magic = MAGIC_VALUE;
            pkt.data = 1;

            IP4_ADDR(&_eth.ip4_addr, _eth.ip[0], _eth.ip[1], _eth.ip[2], _eth.ip[3]);
            AP_Networking::send_udp(_eth.pcb, _eth.ip4_addr, _eth.port.get(), (uint8_t*)&pkt, sizeof(pkt));
        }
        
    } else if (now_ms - _stats.gcs_send_ms >= 1000) {
        // TODO: handle (last_rx_data < _stats.last_rx_data)
        const uint32_t delta = (last_rx_data - _stats.last_rx_data);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,"NET: %.3fms: cnt:%u cnt/s:%u lag:%.3f",
            (double)(_stats.start_ms * 0.001f),
            (unsigned)last_rx_data,
            (unsigned)delta,
            (double)(delta * 0.001f));

        _stats.last_rx_data = last_rx_data;
    }
}

void AP_Networking_LatencyTest::latencytest_recv_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
    struct netif *netif = ip_current_input_netif();
    (void)netif;
    struct LatencyTestPacket_t *pkt = (struct LatencyTestPacket_t *)p->payload;


    // TODO: use arg to pass the expected port
    // if (port != _eth.port.get()) {
    // if (port != AP_NETWORKING_LATENCYTEST_DEFAULT_PORT) {
    //     return;
    // }

    if (p == nullptr) {
        return;
    }
    
    if (pkt->magic == AP_Networking_LatencyTest::MAGIC_VALUE && p->len == sizeof(LatencyTestPacket_t)) {
  //if (pkt->magic == AP_Networking_LatencyTest::MAGIC_VALUE) {
        latencyTest_last_rx_data = pkt->data;
        pkt->data = pkt->data + 1;
        udp_sendto(pcb, p, &pcb->remote_ip, port);
    }
    pbuf_free(p);
}

#endif // AP_NETWORKING_LATENCYTEST_ENABLED

