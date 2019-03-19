/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  AP_Periph can support
 */
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "AP_Periph.h"
#include "hal.h"
#include <canard.h>
#include <uavcan/protocol/dynamic_node_id/Allocation.h>
#include <uavcan/protocol/NodeStatus.h>
#include <uavcan/protocol/RestartNode.h>
#include <uavcan/protocol/GetNodeInfo.h>
#include <uavcan/protocol/file/BeginFirmwareUpdate.h>
#include <ardupilot/bus/I2CReqAnnounce.h>
#include <ardupilot/bus/I2CAnnounce.h>
#include <stdio.h>
#include <ardupilot/bus/I2C.h>
#include <AP_HAL_ChibiOS/hwdef/common/stm32_util.h>

#include "i2c.h"

extern const AP_HAL::HAL &hal;

static CanardInstance canard;
static uint32_t canard_memory_pool[2048/4];
static const uint8_t PreferredNodeID = 7; // CANARD_BROADCAST_NODE_ID;

// can config for 1MBit
static uint32_t baudrate = 1000000U;

static const CANConfig cancfg = {
    CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
    CAN_BTR_SJW(0) | CAN_BTR_TS2(2-1) |
    CAN_BTR_TS1(15-1) | CAN_BTR_BRP((STM32_PCLK1/18)/baudrate - 1)
};

#define APP_VERSION_MAJOR                                           1
#define APP_VERSION_MINOR                                           0
#define APP_NODE_NAME                                               "org.ardupilot.ap_periph"

/*
 * Variables used for dynamic node ID allocation.
 * RTFM at http://uavcan.org/Specification/6._Application_level_functions/#dynamic-node-id-allocation
 */
static uint32_t send_next_node_id_allocation_request_at_ms; ///< When the next node ID allocation request should be sent
static uint8_t node_id_allocation_unique_id_offset;         ///< Depends on the stage of the next request

/*
 * Node status variables
 */
static uavcan_protocol_NodeStatus node_status;


/**
 * Returns a pseudo random float in the range [0, 1].
 */
static float getRandomFloat(void)
{
    return float(get_random16()) / 0xFFFF;
}


/*
  get cpu unique ID
 */
static void readUniqueID(uint8_t* out_uid)
{
    uint8_t len = UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_UNIQUE_ID_MAX_LENGTH;
    memset(out_uid, 0, len);
    hal.util->get_system_id_unformatted(out_uid, len);
}


/*
  handle a GET_NODE_INFO request
 */
static void handle_get_node_info(CanardInstance* ins,
                                 CanardRxTransfer* transfer)
{
    uint8_t buffer[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE] {};
    uavcan_protocol_GetNodeInfoResponse pkt {};

    node_status.uptime_sec = AP_HAL::millis() / 1000U;

    pkt.status = node_status;
    pkt.software_version.major = APP_VERSION_MAJOR;
    pkt.software_version.minor = APP_VERSION_MINOR;

    readUniqueID(pkt.hardware_version.unique_id);

    char name[strlen(APP_NODE_NAME)+1];
    strcpy(name, APP_NODE_NAME);
    pkt.name.len = strlen(APP_NODE_NAME);
    pkt.name.data = (uint8_t *)name;

    uint16_t total_size = uavcan_protocol_GetNodeInfoResponse_encode(&pkt, buffer);

    const int16_t resp_res = canardRequestOrRespond(ins,
                                                    transfer->source_node_id,
                                                    UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE,
                                                    UAVCAN_PROTOCOL_GETNODEINFO_ID,
                                                    &transfer->transfer_id,
                                                    transfer->priority,
                                                    CanardResponse,
                                                    &buffer[0],
                                                    total_size);
    if (resp_res <= 0) {
        printf("Could not respond to GetNodeInfo: %d\n", resp_res);
    }
}


static void handle_begin_firmware_update(CanardInstance* ins, CanardRxTransfer* transfer)
{
    // instant reboot, with backup register used to give bootloader
    // the node_id we rely on the caller re-sending the firmware
    // update request to the bootloader
    set_fast_reboot((rtc_boot_magic)(RTC_BOOT_CANBL | canardGetLocalNodeID(ins)));
    NVIC_SystemReset();
}

/*
  we have received a I2CReqAnnounce, which means another node wants us
  to announce the I2C buses we have available
 */
static void handle_i2c_req_announce(CanardInstance* ins, CanardRxTransfer* transfer)
{
    ardupilot_bus_I2CAnnounce pkt;
    uint8_t busnum = 1;
    pkt.bus_list.len = 1;
    pkt.bus_list.data = &busnum;

    uint8_t buffer[ARDUPILOT_BUS_I2CANNOUNCE_MAX_SIZE];
    uint16_t total_size = ardupilot_bus_I2CAnnounce_encode(&pkt, buffer);

    canardBroadcast(ins,
                    ARDUPILOT_BUS_I2CANNOUNCE_SIGNATURE,
                    ARDUPILOT_BUS_I2CANNOUNCE_ID,
                    &transfer->transfer_id,
                    transfer->priority,
                    &buffer[0],
                    total_size);
    
}

/*
  we have received a I2C service request to perform an I2C operation
 */
static void handle_i2c_request(CanardInstance* ins, CanardRxTransfer* transfer)
{

    ardupilot_bus_I2CRequest req;
    uint8_t arraybuf[ARDUPILOT_BUS_I2C_REQUEST_BUFFER_MAX_LENGTH];
    uint8_t *arraybuf_ptr = arraybuf;
    if (ardupilot_bus_I2CRequest_decode(transfer, transfer->payload_len, &req, &arraybuf_ptr) < 0) {
        printf("I2C: decode failed\n");
    }

    uint8_t reply_data[req.numread] {};

    bool ok = false;

    if (req.bus == 0) {
        ok = i2c_transfer(req.address, req.buffer.len?req.buffer.data:NULL, req.buffer.len, reply_data, req.numread);
        //printf("i2c_transfer ad:0x%02x ok=%u len=%u recv_len=%u\n", req.address, ok, req.buffer.len, req.numread);
    }

    ardupilot_bus_I2CResponse pkt;
    pkt.result = ok?ARDUPILOT_BUS_I2C_RESPONSE_I2C_OK:ARDUPILOT_BUS_I2C_RESPONSE_I2C_ERR;
    pkt.buffer.len = ok?req.numread:0;
    pkt.buffer.data = reply_data;

    uint8_t buffer[ARDUPILOT_BUS_I2C_RESPONSE_MAX_SIZE];
    uint16_t total_size = ardupilot_bus_I2CResponse_encode(&pkt, buffer);

    canardRequestOrRespond(ins,
                           transfer->source_node_id,
                           ARDUPILOT_BUS_I2C_SIGNATURE,
                           ARDUPILOT_BUS_I2C_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           &buffer[0],
                           total_size);
}

static void handle_allocation_response(CanardInstance* ins, CanardRxTransfer* transfer)
{
    // Rule C - updating the randomized time interval
    send_next_node_id_allocation_request_at_ms =
        AP_HAL::millis() + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS +
        (uint32_t)(getRandomFloat() * UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);

    if (transfer->source_node_id == CANARD_BROADCAST_NODE_ID)
    {
        printf("Allocation request from another allocatee\n");
        node_id_allocation_unique_id_offset = 0;
        return;
    }

    // Copying the unique ID from the message
    static const uint8_t UniqueIDBitOffset = 8;
    uint8_t received_unique_id[UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_UNIQUE_ID_MAX_LENGTH];
    uint8_t received_unique_id_len = 0;
    for (; received_unique_id_len < (transfer->payload_len - (UniqueIDBitOffset / 8U)); received_unique_id_len++) {
        assert(received_unique_id_len < UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_UNIQUE_ID_MAX_LENGTH);
        const uint8_t bit_offset = (uint8_t)(UniqueIDBitOffset + received_unique_id_len * 8U);
        (void) canardDecodeScalar(transfer, bit_offset, 8, false, &received_unique_id[received_unique_id_len]);
    }

    // Obtaining the local unique ID
    uint8_t my_unique_id[UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_UNIQUE_ID_MAX_LENGTH];
    readUniqueID(my_unique_id);

    // Matching the received UID against the local one
    if (memcmp(received_unique_id, my_unique_id, received_unique_id_len) != 0) {
        printf("Mismatching allocation response\n");
        node_id_allocation_unique_id_offset = 0;
        return;         // No match, return
    }

    if (received_unique_id_len < UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_UNIQUE_ID_MAX_LENGTH) {
        // The allocator has confirmed part of unique ID, switching to the next stage and updating the timeout.
        node_id_allocation_unique_id_offset = received_unique_id_len;
        send_next_node_id_allocation_request_at_ms -= UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS;

        printf("Matching allocation response: %d\n", received_unique_id_len);
    } else {
        // Allocation complete - copying the allocated node ID from the message
        uint8_t allocated_node_id = 0;
        (void) canardDecodeScalar(transfer, 0, 7, false, &allocated_node_id);
        assert(allocated_node_id <= 127);

        canardSetLocalNodeID(ins, allocated_node_id);
        printf("Node ID allocated: %d\n", allocated_node_id);
    }
}

/**
 * This callback is invoked by the library when a new message or request or response is received.
 */
static void onTransferReceived(CanardInstance* ins,
                               CanardRxTransfer* transfer)
{
    /*
     * Dynamic node ID allocation protocol.
     * Taking this branch only if we don't have a node ID, ignoring otherwise.
     */
    if (canardGetLocalNodeID(ins) == CANARD_BROADCAST_NODE_ID) {
        if (transfer->transfer_type == CanardTransferTypeBroadcast &&
            transfer->data_type_id == UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID) {
            handle_allocation_response(ins, transfer);
        }
        return;
    }

    switch (transfer->data_type_id) {
    case UAVCAN_PROTOCOL_GETNODEINFO_ID:
        handle_get_node_info(ins, transfer);
        break;

    case UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_ID:
        handle_begin_firmware_update(ins, transfer);
        break;

    case ARDUPILOT_BUS_I2CREQANNOUNCE_ID:
        printf("got I2CReqAnnounce\n");
        handle_i2c_req_announce(ins, transfer);
        break;

    case ARDUPILOT_BUS_I2C_ID:
        handle_i2c_request(ins, transfer);
        break;

    case UAVCAN_PROTOCOL_RESTARTNODE_ID:
        printf("RestartNode\n");
        hal.scheduler->delay(10);
        NVIC_SystemReset();
        break;
    }
}


/**
 * This callback is invoked by the library when it detects beginning of a new transfer on the bus that can be received
 * by the local node.
 * If the callback returns true, the library will receive the transfer.
 * If the callback returns false, the library will ignore the transfer.
 * All transfers that are addressed to other nodes are always ignored.
 */
static bool shouldAcceptTransfer(const CanardInstance* ins,
                                 uint64_t* out_data_type_signature,
                                 uint16_t data_type_id,
                                 CanardTransferType transfer_type,
                                 uint8_t source_node_id)
{
    (void)source_node_id;

    if (canardGetLocalNodeID(ins) == CANARD_BROADCAST_NODE_ID)
    {
        /*
         * If we're in the process of allocation of dynamic node ID, accept only relevant transfers.
         */
        if ((transfer_type == CanardTransferTypeBroadcast) &&
            (data_type_id == UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID))
        {
            *out_data_type_signature = UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE;
            return true;
        }
        return false;
    }

    switch (data_type_id) {
    case UAVCAN_PROTOCOL_GETNODEINFO_ID:
        *out_data_type_signature = UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE;
        return true;
    case UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_ID:
        *out_data_type_signature = UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_SIGNATURE;
        return true;
    case UAVCAN_PROTOCOL_RESTARTNODE_ID:
        *out_data_type_signature = UAVCAN_PROTOCOL_RESTARTNODE_SIGNATURE;
        return true;
    case ARDUPILOT_BUS_I2C_ID:
        *out_data_type_signature = ARDUPILOT_BUS_I2C_SIGNATURE;
        return true;
    case ARDUPILOT_BUS_I2CREQANNOUNCE_ID:
        *out_data_type_signature = ARDUPILOT_BUS_I2CREQANNOUNCE_SIGNATURE;
        return true;
    default:
        break;
    }

    return false;
}

static void processTx(void)
{
    for (const CanardCANFrame* txf = NULL; (txf = canardPeekTxQueue(&canard)) != NULL;) {
        CANTxFrame txmsg {};
        txmsg.DLC = txf->data_len;
        memcpy(txmsg.data8, txf->data, 8);
        txmsg.EID = txf->id & CANARD_CAN_EXT_ID_MASK;
        txmsg.IDE = 1;
        txmsg.RTR = 0;
        if (canTransmit(&CAND1, CAN_ANY_MAILBOX, &txmsg, TIME_IMMEDIATE) == MSG_OK) {
            canardPopTxQueue(&canard);
        } else {
            // Timeout - just exit and try again later
            return;
        }
    }
}

static void processRx(void)
{
    CANRxFrame rxmsg {};
    while (canReceive(&CAND1, CAN_ANY_MAILBOX, &rxmsg, TIME_IMMEDIATE) == MSG_OK) {
        CanardCANFrame rx_frame {};

        palToggleLine(HAL_GPIO_PIN_LED);

        const uint64_t timestamp = AP_HAL::micros64();
        memcpy(rx_frame.data, rxmsg.data8, 8);
        rx_frame.data_len = rxmsg.DLC;
        if(rxmsg.IDE) {
            rx_frame.id = CANARD_CAN_FRAME_EFF | rxmsg.EID;
        } else {
            rx_frame.id = rxmsg.SID;
        }
        canardHandleRxFrame(&canard, &rx_frame, timestamp);
    }
}

static uint16_t pool_peak_percent(void)
{
    const CanardPoolAllocatorStatistics stats = canardGetPoolAllocatorStatistics(&canard);
    const uint16_t peak_percent = (uint16_t)(100U * stats.peak_usage_blocks / stats.capacity_blocks);
    return peak_percent;
}

/**
 * This function is called at 1 Hz rate from the main loop.
 */
static void process1HzTasks(uint64_t timestamp_usec)
{
    /*
     * Purging transfers that are no longer transmitted. This will occasionally free up some memory.
     */
    canardCleanupStaleTransfers(&canard, timestamp_usec);

    /*
     * Printing the memory usage statistics.
     */
    {
        /*
         * The recommended way to establish the minimal size of the memory pool is to stress-test the application and
         * record the worst case memory usage.
         */
        if (pool_peak_percent() > 70) {
            printf("WARNING: ENLARGE MEMORY POOL\n");
        }
    }

    /*
     * Transmitting the node status message periodically.
     */
    {
        uint8_t buffer[UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE];
        node_status.uptime_sec = AP_HAL::millis() / 1000U;

        node_status.vendor_specific_status_code = hal.util->available_memory();

        uint32_t len = uavcan_protocol_NodeStatus_encode(&node_status, buffer);

        static uint8_t transfer_id;  // Note that the transfer ID variable MUST BE STATIC (or heap-allocated)!

        const int16_t bc_res = canardBroadcast(&canard,
                                               UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE,
                                               UAVCAN_PROTOCOL_NODESTATUS_ID,
                                               &transfer_id,
                                               CANARD_TRANSFER_PRIORITY_LOW,
                                               buffer,
                                               len);
        if (bc_res <= 0) {
            printf("broadcast fail %d\n", bc_res);
        } else {
            //printf("broadcast node status OK\n");
        }
    }

    node_status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
}

/*
  wait for dynamic allocation of node ID
 */
static void can_wait_node_id(void)
{
    uint8_t node_id_allocation_transfer_id = 0;

    while (canardGetLocalNodeID(&canard) == CANARD_BROADCAST_NODE_ID)
    {
        printf("Waiting for dynamic node ID allocation... (pool %u)\n", pool_peak_percent());

        send_next_node_id_allocation_request_at_ms =
            AP_HAL::millis() + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS +
            (uint32_t)(getRandomFloat() * UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);

        while ((AP_HAL::millis() < send_next_node_id_allocation_request_at_ms) &&
               (canardGetLocalNodeID(&canard) == CANARD_BROADCAST_NODE_ID))
        {
            processTx();
            processRx();
            canardCleanupStaleTransfers(&canard, AP_HAL::micros64());
        }

        if (canardGetLocalNodeID(&canard) != CANARD_BROADCAST_NODE_ID)
        {
            break;
        }

        // Structure of the request is documented in the DSDL definition
        // See http://uavcan.org/Specification/6._Application_level_functions/#dynamic-node-id-allocation
        uint8_t allocation_request[CANARD_CAN_FRAME_MAX_DATA_LEN - 1];
        allocation_request[0] = (uint8_t)(PreferredNodeID << 1U);

        if (node_id_allocation_unique_id_offset == 0)
        {
            allocation_request[0] |= 1;     // First part of unique ID
        }

        uint8_t my_unique_id[UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_UNIQUE_ID_MAX_LENGTH];
        readUniqueID(my_unique_id);

        static const uint8_t MaxLenOfUniqueIDInRequest = 6;
        uint8_t uid_size = (uint8_t)(UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_UNIQUE_ID_MAX_LENGTH - node_id_allocation_unique_id_offset);
        if (uid_size > MaxLenOfUniqueIDInRequest)
        {
            uid_size = MaxLenOfUniqueIDInRequest;
        }

        // Paranoia time
        assert(node_id_allocation_unique_id_offset < UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_UNIQUE_ID_MAX_LENGTH);
        assert(uid_size <= MaxLenOfUniqueIDInRequest);
        assert(uid_size > 0);
        assert((uid_size + node_id_allocation_unique_id_offset) <= UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_UNIQUE_ID_MAX_LENGTH);

        memmove(&allocation_request[1], &my_unique_id[node_id_allocation_unique_id_offset], uid_size);

        // Broadcasting the request
        const int16_t bcast_res = canardBroadcast(&canard,
                                                  UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE,
                                                  UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID,
                                                  &node_id_allocation_transfer_id,
                                                  CANARD_TRANSFER_PRIORITY_LOW,
                                                  &allocation_request[0],
                                                  (uint16_t) (uid_size + 1));
        if (bcast_res < 0)
        {
            printf("Could not broadcast ID allocation req; error %d\n", bcast_res);
        }

        // Preparing for timeout; if response is received, this value will be updated from the callback.
        node_id_allocation_unique_id_offset = 0;
    }

    printf("Dynamic node ID allocation complete [%d]\n", canardGetLocalNodeID(&canard));
}

void AP_Periph_FW::can_start()
{
    node_status.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    node_status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_INITIALIZATION;
    node_status.uptime_sec = AP_HAL::millis() / 1000U;

    canStart(&CAND1, &cancfg);

    canardInit(&canard, (uint8_t *)canard_memory_pool, sizeof(canard_memory_pool),
               onTransferReceived, shouldAcceptTransfer, NULL);

    canardSetLocalNodeID(&canard, PreferredNodeID);

    // wait for dynamic node ID allocation
    can_wait_node_id();
}


void AP_Periph_FW::can_update()
{
    static uint32_t last_1Hz_ms;
    uint32_t now = AP_HAL::millis();
    if (now - last_1Hz_ms >= 1000) {
        last_1Hz_ms = now;
        process1HzTasks(AP_HAL::micros64());
    }
    processTx();
    processRx();
}
