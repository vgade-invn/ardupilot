
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>

#include "AP_CANManager.h"
#include "AP_CANTester.h"
#include "AP_CANManager.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <stdio.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_UAVCAN/AP_UAVCAN.h>
#include <uavcan/protocol/dynamic_node_id_client.hpp>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo CANTester::var_info[] = {
    AP_GROUPINFO("ID", 1, CANTester, _test_id, 0),
    AP_GROUPINFO("LPR8", 2, CANTester, _loop_rate, 500),
};

#define debug_can(level_debug, fmt, args...) do { AP::can().log(level_debug, _driver_index,  fmt, #args); } while (0)

bool CANTester::add_interface(AP_HAL::CANIface* can_iface) {
    if (_num_ifaces >= MAX_NUMBER_OF_CAN_INTERFACES) {
        debug_can(AP_CANManager::LOG_ERROR, "CANTester: Max Number of CanIfaces exceeded\n\r");
    }

    _can_ifaces[_num_ifaces] = can_iface;

    if (_can_ifaces[_num_ifaces] == nullptr) {
        debug_can(AP_CANManager::LOG_ERROR, "CANTester: CAN driver not found\n\r");
        return false;
    }

    if (!_can_ifaces[_num_ifaces]->is_initialized()) {
        debug_can(AP_CANManager::LOG_ERROR, "CANTester: Driver not initialized\n\r");
        return false;
    }

    _num_ifaces++;
    return true;
}

void CANTester::init(uint8_t driver_index, bool enable_filters)
{
    _driver_index = driver_index;

    // Reset Test mask
    _test_id.set_and_save(0);

    debug_can(AP_CANManager::LOG_DEBUG, "CANTester: starting init\n\r");

    if (_initialized) {
        debug_can(AP_CANManager::LOG_ERROR, "CANTester: already initialized\n\r");
        return;
    }

    if (_can_ifaces[0] == nullptr) {
        debug_can(AP_CANManager::LOG_ERROR, "CANTester: Interface not found\n\r");
        return;
    }

    // kick start tester thread
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&CANTester::main_thread, void), "can_tester", 8192, AP_HAL::Scheduler::PRIORITY_CAN, 1)) {
        debug_can(AP_CANManager::LOG_ERROR, "CANTester: couldn't create thread\n\r");
        return;
    }

    _initialized = true;

    debug_can(AP_CANManager::LOG_DEBUG, "CANTester: init done\n\r");

    return;
}

// write frame on CAN bus
bool CANTester::write_frame(uint8_t iface, AP_HAL::CanFrame &out_frame, uint64_t timeout)
{
    if (!_can_ifaces[iface]->set_event_handle(&_event_handle)) {
        debug_can(AP_CANManager::LOG_ERROR, "CANTester: Cannot add event handle\n\r");
        return false;
    }
    // wait for space in buffer to send command
    bool read_select = false;
    bool write_select = true;
    out_frame.id += iface;
    int ret = _can_ifaces[iface]->select(read_select, write_select, &out_frame, AP_HAL::micros64() + timeout);
    if (ret < 0 || !write_select) {
        // delay if no space is available to send
        return false;
    }
    // send frame and return success
    return (_can_ifaces[iface]->send(out_frame, AP_HAL::micros() + 1000, AP_HAL::CANIface::AbortOnError) == 1);
}

// read frame on CAN bus, returns true on success
bool CANTester::read_frame(uint8_t iface, AP_HAL::CanFrame &recv_frame, uint64_t timeout)
{
    if (!_can_ifaces[iface]->set_event_handle(&_event_handle)) {
        debug_can(AP_CANManager::LOG_ERROR, "CANTester: Cannot add event handle\n\r");
        return false;
    }
    // wait for space in buffer to read
    bool read_select = true;
    bool write_select = false;
    int ret = _can_ifaces[iface]->select(read_select, write_select, nullptr, AP_HAL::micros64() + timeout);
    if (ret < 0 || !read_select) {
        // return false if no data is available to read
        return false;
    }
    uint64_t time;
    AP_HAL::CANIface::CanIOFlags flags {};

    // read frame and return success
    return (_can_ifaces[iface]->receive(recv_frame, time, flags) == 1);
}

void CANTester::main_thread() {
    while(true) {
        switch (_test_id)
        {
        case CANTester::TEST_LOOPBACK:
            if (_can_ifaces[1] != nullptr) {
                gcs().send_text(MAV_SEVERITY_ALERT, "********Running Loopback Test*******");
                if (test_loopback(_loop_rate)) {
                    gcs().send_text(MAV_SEVERITY_ALERT, "********Loopback Test Pass*******");
                } else {
                    gcs().send_text(MAV_SEVERITY_ALERT, "********Loopback Test Fail*******");
                }
            } else {
                gcs().send_text(MAV_SEVERITY_ALERT, "Can't do Loopback Test with single iface");
            }
            break;
        case CANTester::TEST_BUSOFF_RECOVERY:
            if (_can_ifaces[1] != nullptr) {
                gcs().send_text(MAV_SEVERITY_ALERT, "********Running Busoff Recovery Test********");
                if (test_busoff_recovery()) {
                    gcs().send_text(MAV_SEVERITY_ALERT, "********Busoff Recovery Test Pass********");
                } else {
                    gcs().send_text(MAV_SEVERITY_ALERT, "********Busoff Recovery Test Fail********");
                }
            } else {
                gcs().send_text(MAV_SEVERITY_ALERT, "Can't do Busoff Recovery Test with single iface");
            }
            break;
        case CANTester::TEST_UAVCAN_DNA:
            if (_can_ifaces[1] == nullptr) {
                gcs().send_text(MAV_SEVERITY_ALERT, "********Running UAVCAN DNA Test********");
                if (test_uavcan_dna()) {
                    gcs().send_text(MAV_SEVERITY_ALERT, "********UAVCAN DNA Test Pass********");
                } else {
                    gcs().send_text(MAV_SEVERITY_ALERT, "********UAVCAN DNA Test Fail********");
                }
            } else {
                gcs().send_text(MAV_SEVERITY_ALERT, "Only one iface needs to be set for UAVCAN_DNA_TEST");
            }
            break;
        default:
            break;
        }
        hal.scheduler->delay(1000);
    }
}

/*****************************************
 *           Loopback Test               *
 * ***************************************/

#define NUM_LOOPBACK_RUNS 1000UL
#define LOOPBACK_MAGIC 0x34567819UL

bool CANTester::test_loopback(uint32_t loop_rate) {
    AP_HAL::CanFrame frame;
    uint32_t num_loops = NUM_LOOPBACK_RUNS;
    memset(&_loopback_stats[0], 0, sizeof(_loopback_stats[0]));
    memset(&_loopback_stats[1], 0, sizeof(_loopback_stats[1]));
    while(num_loops--) {
        for (uint8_t i = 0; i < 2; i++) {
            create_loopback_frame(_loopback_stats[0], frame);
            if (write_frame(i, frame, loop_rate)) {
                _loopback_stats[i].num_tx++;
            } else {
                _loopback_stats[i].failed_tx++;
            }

            reset_frame(frame);
            if (read_frame((i+1)%2, frame, loop_rate)) {
                check_loopback_frame(_loopback_stats[i], frame);
                _loopback_stats[i].num_rx++;
            }
        }
        hal.scheduler->delay_microseconds(loop_rate);
    }
    for (uint8_t i = 0; i < _num_ifaces; i++) {
        gcs().send_text(MAV_SEVERITY_ALERT, "Loopback Test Results %d->%d:", i, (i+1)%2);
        gcs().send_text(MAV_SEVERITY_ALERT, "num_tx: %lu, failed_tx: %lu",
                                _loopback_stats[i].num_tx, _loopback_stats[i].failed_tx);
        gcs().send_text(MAV_SEVERITY_ALERT, "num_rx: %lu, bad_rx_data: %lu, bad_rx_seq: %lu", 
                                _loopback_stats[i].num_rx, _loopback_stats[i].bad_rx_data, _loopback_stats[i].bad_rx_seq);
        if (_loopback_stats[i].num_rx < 0.9f * _loopback_stats[i].num_tx) {
            return false;
        }
    }
    return true;
}

void CANTester::reset_frame(AP_HAL::CanFrame& frame) {
    frame.id = 0;
    memset(frame.data, 0, sizeof(frame.data));
    frame.dlc = 0;
    
}

void CANTester::create_loopback_frame(CANTester::loopback_stats_s &stats, AP_HAL::CanFrame& frame) {
    frame.id = 13 | AP_HAL::CanFrame::FlagEFF;
    frame.dlc = AP_HAL::CanFrame::MaxDataLen;
    memcpy(frame.data, &stats.tx_seq, sizeof(stats.tx_seq));
    uint32_t loopback_magic = LOOPBACK_MAGIC;
    memcpy(&frame.data[4], &loopback_magic, sizeof(loopback_magic));
    stats.tx_seq++;
}


void CANTester::check_loopback_frame(CANTester::loopback_stats_s &stats, AP_HAL::CanFrame& frame) {
    if (frame.id != (13 | AP_HAL::CanFrame::FlagEFF) ||
        frame.id != (12 | AP_HAL::CanFrame::FlagEFF)) {
        //frame not sent by us
        return;
    }
    if (frame.dlc != AP_HAL::CanFrame::MaxDataLen) {
        stats.bad_rx_data++;
    }

    uint32_t loopback_magic = LOOPBACK_MAGIC;
    if (memcmp(&frame.data[4], &loopback_magic, sizeof(loopback_magic)) != 0) {
        stats.bad_rx_data++;
        return;
    }

    uint32_t valid_seq = stats.next_valid_seq;
    uint32_t curr_seq = (*((uint32_t*)frame.data));
    if (valid_seq > curr_seq) {
        stats.bad_rx_seq++;
    }
    valid_seq  = curr_seq + 1;
}

/*****************************************
 *         Busoff Recovery Test          *
 * ***************************************/
bool CANTester::test_busoff_recovery()
{
    uint32_t num_busoff_runs = 100000;
    uint64_t timestamp;
    AP_HAL::CANIface::CanIOFlags flags;
    AP_HAL::CanFrame bo_frame;
    bo_frame.id = (12 | AP_HAL::CanFrame::FlagEFF);
    memset(bo_frame.data, 0xA, sizeof(bo_frame.data));
    bo_frame.dlc = AP_HAL::CanFrame::MaxDataLen;
    bool bus_off_detected = false;
    // Bus Fault can be introduced by shorting CANH and CANL
    gcs().send_text(MAV_SEVERITY_ERROR, "Introduce Bus Off Fault on the bus.");
    while (num_busoff_runs--) {
        if (bus_off_detected) {
            break;
        }
        //Spam the bus with same frame
        _can_ifaces[0]->send(bo_frame, AP_HAL::micros()+1000, 0);
        _can_ifaces[1]->receive(bo_frame, timestamp, flags);
        _can_ifaces[1]->send(bo_frame, AP_HAL::micros()+1000, 0);
        _can_ifaces[0]->receive(bo_frame, timestamp, flags);
        bus_off_detected = _can_ifaces[0]->is_busoff() || _can_ifaces[1]->is_busoff();
        hal.scheduler->delay_microseconds(50);
    }
    if (!bus_off_detected) {
        gcs().send_text(MAV_SEVERITY_ERROR, "BusOff not detected on the bus");
        return false;
    }
    gcs().send_text(MAV_SEVERITY_ERROR, "BusOff detected remove Fault.");
    hal.scheduler->delay(1000);
    gcs().send_text(MAV_SEVERITY_ERROR, "Running Loopback test.");
    //Send Dummy Frames to clear the error
    _can_ifaces[0]->send(bo_frame, AP_HAL::micros()+1000, 0);
    bo_frame.id += 1;
    _can_ifaces[1]->send(bo_frame, AP_HAL::micros()+1000, 0);
    //Flush the CAN bus Rx Buffer
    _can_ifaces[0]->flush();
    _can_ifaces[1]->flush();

    return test_loopback(_loop_rate);
}

/*****************************************
 *             UAVCAN DNA Test           *
 * ***************************************/

bool CANTester::test_uavcan_dna() {
    
    uavcan::CanIfaceMgr _uavcan_iface_mgr {};

    if (!_uavcan_iface_mgr.add_interface(_can_ifaces[0])) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Failed to add iface");
        return false;
    }

    uavcan::Node<0> node(_uavcan_iface_mgr, uavcan::SystemClock::instance(), _node_allocator);
    node.setName("org.ardupilot.dnatest");

    uavcan::protocol::HardwareVersion hw_version;
    const uint8_t uid_buf_len = hw_version.unique_id.capacity();
    uint8_t uid_len = uid_buf_len;
    uint8_t unique_id[uid_buf_len];

    if (hal.util->get_system_id_unformatted(unique_id, uid_len)) {
        unique_id[uid_len - 1] -= 5;
        uavcan::copy(unique_id, unique_id + uid_len, hw_version.unique_id.begin());
    }

    node.setHardwareVersion(hw_version); // Copying the value to the node's internals

    /*
     * Starting the node normally, in passive mode (i.e. without node ID assigned).
     */
    const int node_start_res = node.start();
    if (node_start_res < 0)
    {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Failed to start the node");
        return false;
    }

    /*
     * Initializing the dynamic node ID allocation client.
     * By default, the client will use TransferPriority::OneHigherThanLowest for communications with the allocator;
     * this can be overriden through the third argument to the start() method.
     */
    uavcan::DynamicNodeIDClient client(node);
    int expected_node_id = 100;
    int client_start_res = client.start(node.getHardwareVersion().unique_id,    // USING THE SAME UNIQUE ID AS ABOVE
                                        expected_node_id);
    if (client_start_res < 0)
    {
        gcs().send_text(MAV_SEVERITY_ALERT,"Failed to start the dynamic node");
    }

    /*
     * Waiting for the client to obtain for us a node ID.
     * This may take a few seconds.
     */
    gcs().send_text(MAV_SEVERITY_ALERT, "Allocation is in progress");
    uint32_t num_runs = 50;
    while (!client.isAllocationComplete() && num_runs--) {
        const int res = node.spin(uavcan::MonotonicDuration::fromMSec(200));    // Spin duration doesn't matter
        if (res < 0)
        {
            gcs().send_text(MAV_SEVERITY_ALERT, "Transient failure");
        }
    }
    gcs().send_text(MAV_SEVERITY_ALERT, "Dynamic node ID %d allocated node ID %d", 
                                        int(client.getAllocatedNodeID().get()), 
                                        int(client.getAllocatorNodeID().get()));
    if (client.getAllocatedNodeID().get() != expected_node_id) {
        gcs().send_text(MAV_SEVERITY_ALERT, "Unexpected Node Id, expected %d", expected_node_id);
        return false;
    }
    return true;
}