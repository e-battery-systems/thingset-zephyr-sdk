/*
 * Copyright (c) The ThingSet Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <inttypes.h>
#include <stdlib.h>

#include <zephyr/ztest.h>

#include <thingset.h>
#include <thingset/can.h>
#include <thingset/sdk.h>

#define CONFIG_BMS_IC_SWITCHES

#define MAX_COLD_SWAP_DEVICES    8
#define COLD_SWAP_CAN_ID_NOT_SET 0xff

#define APP_ID_MEAS_PACK_VOLTAGE  0x71
#define APP_ID_MEAS_STACK_VOLTAGE 0x72
#define APP_ID_MEAS_BMS_STATE     0x7F
#define APP_ID_MEAS_ERROR_FLAGS   0x7E

#define TMP_ID        0x1
#define TMP_SUBSET_ID 0x1

static const struct device *can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));

static struct can_frame report_frames[] = {
    {
        .id = 0x1D008002, /* msg 0x0, first frame, seq 0x0 */
        .flags = CAN_FRAME_IDE,
        .data = { 0x1F, 0x18, 0x31, 0xAE, 0x18, 0x71, 0xFA, 0x42 },
        .dlc = 8,
    },
    {
        .id = 0x1D009102, /* msg 0x0, frame, seq 0x1 */
        .flags = CAN_FRAME_IDE,
        .data = { 0x55, 0x85, 0x1E, 0x18, 0x72, 0xFA, 0x42, 0x52 },
        .dlc = 8,
    },
    {
        .id = 0x1D009202, /* msg 0x0, frame, seq 0x2 */
        .flags = CAN_FRAME_IDE,
        .data = { 0xA3, 0xD7, 0x18, 0x73, 0xFA, 0xC2, 0x9D, 0x2E },
        .dlc = 8,
    },
    {
        .id = 0x1D009302, /* msg 0x0, frame, seq 0x3 */
        .flags = CAN_FRAME_IDE,
        .data = { 0x14, 0x18, 0x74, 0x82, 0xFA, 0x41, 0xA3, 0x99 },
        .dlc = 8,
    },
    {
        .id = 0x1D009402, /* msg 0x0, frame, seq 0x4 */
        .flags = CAN_FRAME_IDE,
        .data = { 0xA0, 0xFA, 0x41, 0xA2, 0xCC, 0xD0, 0x18, 0x75 },
        .dlc = 8,
    },
    {
        .id = 0x1D009502, /* msg 0x0, frame, seq 0x5 */
        .flags = CAN_FRAME_IDE,
        .data = { 0xFA, 0x42, 0x34, 0x99, 0xA0, 0x18, 0x77, 0xFA },
        .dlc = 8,
    },
    {
        .id = 0x1D009602, /* msg 0x0, frame, seq 0x6 */
        .flags = CAN_FRAME_IDE,
        .data = { 0x42, 0x51, 0x66, 0x68, 0x18, 0x7C, 0xFA, 0x42 },
        .dlc = 8,
    },
    {
        .id = 0x1D009702, /* msg 0x0, frame, seq 0x7 */
        .flags = CAN_FRAME_IDE,
        .data = { 0x96, 0x97, 0x9A, 0x18, 0x7E, 0x00, 0x18, 0x7F },
        .dlc = 8,
    },
    {
        .id = 0x1D009802, /* msg 0x0, frame, seq 0x8 */
        .flags = CAN_FRAME_IDE,
        .data = { 0x01, 0x18, 0x80, 0x90, 0xFA, 0x40, 0x56, 0x14 },
        .dlc = 8,
    },
    {
        .id = 0x1D009902, /* msg 0x0, frame, seq 0x9 */
        .flags = CAN_FRAME_IDE,
        .data = { 0x7C, 0xFA, 0x40, 0x56, 0x45, 0xA2, 0xFA, 0x40 },
        .dlc = 8,
    },
    {
        .id = 0x1D009A02, /* msg 0x0, frame, seq 0xA */
        .flags = CAN_FRAME_IDE,
        .data = { 0x56, 0x24, 0xDE, 0xFA, 0x40, 0x55, 0xA1, 0xCB },
        .dlc = 8,
    },
    {
        .id = 0x1D009B02, /* msg 0x0, frame, seq 0xB */
        .flags = CAN_FRAME_IDE,
        .data = { 0xFA, 0x40, 0x55, 0xD2, 0xF2, 0xFA, 0x40, 0x57 },
        .dlc = 8,
    },
    {
        .id = 0x1D009C02, /* msg 0x0, frame, seq 0xC */
        .flags = CAN_FRAME_IDE,
        .data = { 0x1A, 0xA0, 0xFA, 0x40, 0x55, 0xB2, 0x2E, 0xFA },
        .dlc = 8,
    },
    {
        .id = 0x1D009D02, /* msg 0x0, frame, seq 0xD */
        .flags = CAN_FRAME_IDE,
        .data = { 0x40, 0x56, 0x04, 0x19, 0xFA, 0x40, 0x56, 0x87 },
        .dlc = 8,
    },
    {
        .id = 0x1D009E02, /* msg 0x0, frame, seq 0xE */
        .flags = CAN_FRAME_IDE,
        .data = { 0x2C, 0xFA, 0x40, 0x56, 0x56, 0x05, 0xFA, 0x40 },
        .dlc = 8,
    },
    {
        .id = 0x1D009F02, /* msg 0x0, frame, seq 0xF */
        .flags = CAN_FRAME_IDE,
        .data = { 0x56, 0xD9, 0x17, 0xFA, 0x40, 0x57, 0x0A, 0x3E },
        .dlc = 8,
    },
    {
        .id = 0x1D009002, /* msg 0x0, frame, seq 0x10 */
        .flags = CAN_FRAME_IDE,
        .data = { 0xFA, 0x40, 0x56, 0xC8, 0xB5, 0xFA, 0x40, 0x56 },
        .dlc = 8,
    },
    {
        .id = 0x1D009102, /* msg 0x0, frame, seq 0x11 */
        .flags = CAN_FRAME_IDE,
        .data = { 0x35, 0x40, 0xFA, 0x40, 0x56, 0x24, 0xDE, 0xFA },
        .dlc = 8,
    },
    {
        .id = 0x1D009202, /* msg 0x0, frame, seq 0x12 */
        .flags = CAN_FRAME_IDE,
        .data = { 0x40, 0x56, 0x45, 0xA2, 0x18, 0x81, 0xFA, 0x40 },
        .dlc = 8,
    },
    {
        .id = 0x1D009302, /* msg 0x0, frame, seq 0x13 */
        .flags = CAN_FRAME_IDE,
        .data = { 0x56, 0x4E, 0xD9, 0x18, 0x82, 0xFA, 0x40, 0x55 },
        .dlc = 8,
    },
    {
        .id = 0x1D008003, /* msg 0x0, first frame, seq 0x0 */
        .flags = CAN_FRAME_IDE,
        .data = { 0x1F, 0x18, 0x31, 0xAE, 0x18, 0x71, 0xFA, 0x42 },
        .dlc = 8,
    },
    {
        .id = 0x1D009103, /* msg 0x0, frame, seq 0x1 */
        .flags = CAN_FRAME_IDE,
        .data = { 0x55, 0x85, 0x1E, 0x18, 0x72, 0xFA, 0x42, 0x52 },
        .dlc = 8,
    },
    {
        .id = 0x1D009402, /* msg 0x0, frame, seq 0x14 */
        .flags = CAN_FRAME_IDE,
        .data = { 0xA1, 0xCB, 0x18, 0x83, 0xFA, 0x40, 0x57, 0x1A },
        .dlc = 8,
    },
    {
        .id = 0x1D00A502, /* msg 0x0, last frame, seq 0x15 */
        .flags = CAN_FRAME_IDE,
        .data = { 0xA0, 0x18, 0x84, 0x00 },
        .dlc = 4,
    },
    {
        .id = 0x1D009203, /* msg 0x0, frame, seq 0x2 */
        .flags = CAN_FRAME_IDE,
        .data = { 0xA3, 0xD7, 0x18, 0x73, 0xFA, 0xC2, 0x9D, 0x2E },
        .dlc = 8,
    },
    {
        .id = 0x1D009303, /* msg 0x0, frame, seq 0x3 */
        .flags = CAN_FRAME_IDE,
        .data = { 0x14, 0x18, 0x74, 0x82, 0xFA, 0x41, 0xA3, 0x99 },
        .dlc = 8,
    },
    {
        .id = 0x1D009403, /* msg 0x0, frame, seq 0x4 */
        .flags = CAN_FRAME_IDE,
        .data = { 0xA0, 0xFA, 0x41, 0xA2, 0xCC, 0xD0, 0x18, 0x75 },
        .dlc = 8,
    },
    {
        .id = 0x1D009503, /* msg 0x0, frame, seq 0x5 */
        .flags = CAN_FRAME_IDE,
        .data = { 0xFA, 0x42, 0x34, 0x99, 0xA0, 0x18, 0x77, 0xFA },
        .dlc = 8,
    },
    {
        .id = 0x1D009603, /* msg 0x0, frame, seq 0x6 */
        .flags = CAN_FRAME_IDE,
        .data = { 0x42, 0x51, 0x66, 0x68, 0x18, 0x7C, 0xFA, 0x42 },
        .dlc = 8,
    },
    {
        .id = 0x1D009703, /* msg 0x0, frame, seq 0x7 */
        .flags = CAN_FRAME_IDE,
        .data = { 0x96, 0x97, 0x9A, 0x18, 0x7E, 0x00, 0x18, 0x7F },
        .dlc = 8,
    },
    {
        .id = 0x1D009803, /* msg 0x0, frame, seq 0x8 */
        .flags = CAN_FRAME_IDE,
        .data = { 0x01, 0x18, 0x80, 0x90, 0xFA, 0x40, 0x56, 0x14 },
        .dlc = 8,
    },
    {
        .id = 0x1D009903, /* msg 0x0, frame, seq 0x9 */
        .flags = CAN_FRAME_IDE,
        .data = { 0x7C, 0xFA, 0x40, 0x56, 0x45, 0xA2, 0xFA, 0x40 },
        .dlc = 8,
    },
    {
        .id = 0x1D009A03, /* msg 0x0, frame, seq 0xA */
        .flags = CAN_FRAME_IDE,
        .data = { 0x56, 0x24, 0xDE, 0xFA, 0x40, 0x55, 0xA1, 0xCB },
        .dlc = 8,
    },
    {
        .id = 0x1D009B03, /* msg 0x0, frame, seq 0xB */
        .flags = CAN_FRAME_IDE,
        .data = { 0xFA, 0x40, 0x55, 0xD2, 0xF2, 0xFA, 0x40, 0x57 },
        .dlc = 8,
    },
    {
        .id = 0x1D009C03, /* msg 0x0, frame, seq 0xC */
        .flags = CAN_FRAME_IDE,
        .data = { 0x1A, 0xA0, 0xFA, 0x40, 0x55, 0xB2, 0x2E, 0xFA },
        .dlc = 8,
    },
    {
        .id = 0x1D009D03, /* msg 0x0, frame, seq 0xD */
        .flags = CAN_FRAME_IDE,
        .data = { 0x40, 0x56, 0x04, 0x19, 0xFA, 0x40, 0x56, 0x87 },
        .dlc = 8,
    },
    {
        .id = 0x1D009E03, /* msg 0x0, frame, seq 0xE */
        .flags = CAN_FRAME_IDE,
        .data = { 0x2C, 0xFA, 0x40, 0x56, 0x56, 0x05, 0xFA, 0x40 },
        .dlc = 8,
    },
    {
        .id = 0x1D009F03, /* msg 0x0, frame, seq 0xF */
        .flags = CAN_FRAME_IDE,
        .data = { 0x56, 0xD9, 0x17, 0xFA, 0x40, 0x57, 0x0A, 0x3E },
        .dlc = 8,
    },
    {
        .id = 0x1D009003, /* msg 0x0, frame, seq 0x10 */
        .flags = CAN_FRAME_IDE,
        .data = { 0xFA, 0x40, 0x56, 0xC8, 0xB5, 0xFA, 0x40, 0x56 },
        .dlc = 8,
    },
    {
        .id = 0x1D009103, /* msg 0x0, frame, seq 0x11 */
        .flags = CAN_FRAME_IDE,
        .data = { 0x35, 0x40, 0xFA, 0x40, 0x56, 0x24, 0xDE, 0xFA },
        .dlc = 8,
    },
    {
        .id = 0x1D009203, /* msg 0x0, frame, seq 0x12 */
        .flags = CAN_FRAME_IDE,
        .data = { 0x40, 0x56, 0x45, 0xA2, 0x18, 0x81, 0xFA, 0x40 },
        .dlc = 8,
    },
    {
        .id = 0x1D009303, /* msg 0x0, frame, seq 0x13 */
        .flags = CAN_FRAME_IDE,
        .data = { 0x56, 0x4E, 0xD9, 0x18, 0x82, 0xFA, 0x40, 0x55 },
        .dlc = 8,
    },
    {
        .id = 0x1D009403, /* msg 0x0, frame, seq 0x14 */
        .flags = CAN_FRAME_IDE,
        .data = { 0xA1, 0xCB, 0x18, 0x83, 0xFA, 0x40, 0x57, 0x1A },
        .dlc = 8,
    },
    {
        .id = 0x1D00A503, /* msg 0x0, last frame, seq 0x15 */
        .flags = CAN_FRAME_IDE,
        .data = { 0xA0, 0x18, 0x84, 0x00 },
        .dlc = 4,
    },
};

/**
 * Possible BMS states
 */
enum bms_state
{
    BMS_STATE_OFF,      ///< Off state (charging and discharging disabled)
    BMS_STATE_CHG,      ///< Charging state (discharging disabled)
    BMS_STATE_DIS,      ///< Discharging state (charging disabled)
    BMS_STATE_NORMAL,   ///< Normal operating mode (both charging and discharging enabled)
    BMS_STATE_SHUTDOWN, ///< BMS starting shutdown sequence
};

struct bms_values
{
    uint8_t can_id;
    /** Current state of the battery */
    enum bms_state state;
    /** Battery internal stack voltage (V) */
    float total_voltage;
#ifdef CONFIG_BMS_IC_SWITCHES
    /** Battery external pack voltage (V) */
    float external_voltage;
#endif
    /** BMS errors stored as BMS_ERR_* flags */
    uint32_t error_flags;
    int64_t last_update;
};

struct can_report
{
    size_t len;
    uint8_t source_addr;
    uint8_t buf[256];
};

static struct bms_values tmp_bms_values;
static struct thingset_context ts_local;
struct thingset_data_object data_objects[] = {
    THINGSET_GROUP(THINGSET_ID_ROOT, TMP_ID, "tmp", THINGSET_NO_CALLBACK),
    THINGSET_ITEM_UINT8(TMP_ID, 0x2, "xCanId", &tmp_bms_values.can_id, THINGSET_ANY_RW,
                        TMP_SUBSET_ID),
    THINGSET_ITEM_UINT8(TMP_ID, APP_ID_MEAS_BMS_STATE, "xBmsState",
                        (uint8_t *)&tmp_bms_values.state, THINGSET_ANY_RW, TMP_SUBSET_ID),
    THINGSET_ITEM_UINT32(TMP_ID, APP_ID_MEAS_ERROR_FLAGS, "xErrorFlags",
                         &tmp_bms_values.error_flags, THINGSET_ANY_RW, TMP_SUBSET_ID),
    THINGSET_ITEM_FLOAT(TMP_ID, APP_ID_MEAS_PACK_VOLTAGE, "xPackVoltage_V",
                        &tmp_bms_values.total_voltage, 2, THINGSET_ANY_RW, TMP_SUBSET_ID),
#ifdef CONFIG_BMS_IC_SWITCHES
    THINGSET_ITEM_FLOAT(TMP_ID, APP_ID_MEAS_STACK_VOLTAGE, "xStackVoltage_V",
                        &tmp_bms_values.external_voltage, 2, THINGSET_ANY_RW, TMP_SUBSET_ID),
#endif
};
static struct bms_values bms_values[MAX_COLD_SWAP_DEVICES] = {
    [0 ... MAX_COLD_SWAP_DEVICES - 1] = { .can_id = COLD_SWAP_CAN_ID_NOT_SET,
                                          .state = BMS_STATE_OFF,
                                          .total_voltage = 0.0f,
#ifdef CONFIG_BMS_IC_SWITCHES
                                          .external_voltage = 0.0f,
#endif
                                          .error_flags = 0 }
};

K_MSGQ_DEFINE(can_msgq, sizeof(struct can_report), MAX_COLD_SWAP_DEVICES * 2, 1);

ZTEST(thingset_can, test_receive_packetized_long_report)
{
    for (size_t i = 0; i < ARRAY_SIZE(bms_values); i++) {
        zassert_equal(bms_values[i].can_id, COLD_SWAP_CAN_ID_NOT_SET);
    }

    /* start sending can frames */
    for (int i = 0; i < ARRAY_SIZE(report_frames); i++) {
        int err = can_send(can_dev, &report_frames[i], K_MSEC(10), NULL, NULL);
        zassert_equal(err, 0, "can_send failed: %d", err);
    }

    /* wait for receiving can messages */
    k_sleep(K_MSEC(1000));

    struct can_report can_report_buffer;
    while (k_msgq_get(&can_msgq, &can_report_buffer, K_MSEC(1000)) == 0) {
        printf("src can id 0x%x length %" PRIdMAX "\n", can_report_buffer.source_addr,
               can_report_buffer.len);
        for (size_t i = 0; i < ARRAY_SIZE(bms_values); i++) {
            if (bms_values[i].can_id == COLD_SWAP_CAN_ID_NOT_SET) {
                bms_values[i].can_id = can_report_buffer.source_addr;
            }

            if (bms_values[i].can_id == can_report_buffer.source_addr) {
                int status = thingset_import_report(&ts_local, can_report_buffer.buf,
                                                    can_report_buffer.len, THINGSET_WRITE_MASK,
                                                    THINGSET_BIN_IDS_VALUES, TS_ID_SUBSET_LIVE);
                if (status != 0) {
                    printf("Importing data failed with ThingSet response code 0x%X\n", -status);
                }
                printf("[%" PRIdMAX "].total_voltage %f\n", i,
                       (double)tmp_bms_values.total_voltage);
#ifdef CONFIG_BMS_IC_SWITCHES
                printf("[%" PRIdMAX "].external_voltage %f\n", i,
                       (double)tmp_bms_values.external_voltage);
#endif
                printf("[%" PRIdMAX "].state 0x%x\n", i, tmp_bms_values.state);
                printf("[%" PRIdMAX "].error_flags 0x%x\n", i, tmp_bms_values.error_flags);

                int64_t actual_time = k_uptime_get();

                bms_values[i].total_voltage = tmp_bms_values.total_voltage;
                bms_values[i].state = tmp_bms_values.state;
                bms_values[i].error_flags = tmp_bms_values.error_flags;
                bms_values[i].last_update = actual_time;
#ifdef CONFIG_BMS_IC_SWITCHES
                bms_values[i].external_voltage = tmp_bms_values.external_voltage;
#endif
                break;
            }
        }
    }

    /* check if the values are the ones which were sent via can */
    for (size_t i = 0; i < 2; i++) {
        zassert_equal(bms_values[i].can_id, i + 2);
        zassert_within((double)bms_values[i].total_voltage, 53.379997, 0.001);
#ifdef CONFIG_BMS_IC_SWITCHES
        zassert_within((double)bms_values[i].external_voltage, 52.66, 0.001);
#endif
        zassert_equal(bms_values[i].state, 1);
        zassert_equal(bms_values[i].error_flags, 0);
    }
}

static void report_rx_callback(const uint8_t *buf, size_t len, uint8_t source_addr)
{

    struct can_report can_report_buffer = { 0 };
    if (len < sizeof(can_report_buffer)) {
        memcpy(can_report_buffer.buf, buf, len);
        can_report_buffer.len = len;
        can_report_buffer.source_addr = source_addr;
        int err = k_msgq_put(&can_msgq, &can_report_buffer, K_NO_WAIT);
        if (err) {
            printf("Unable to add can frame to buffer: %d\n", err);
            k_msgq_purge(&can_msgq);
        }
    }
}

static void *thingset_can_setup(void)
{
    int err;

    size_t data_objects_size = ARRAY_SIZE(data_objects);
    thingset_init(&ts_local, data_objects, data_objects_size);

    zassert_true(device_is_ready(can_dev), "CAN device not ready");

    (void)can_stop(can_dev);

    err = can_set_mode(can_dev, CAN_MODE_LOOPBACK);
    zassert_equal(err, 0, "failed to set loopback mode (err %d)", err);

    err = can_start(can_dev);
    zassert_equal(err, 0, "failed to start CAN controller (err %d)", err);

    /* wait for address claiming to finish */
    k_sleep(K_MSEC(1000));

    thingset_can_set_report_rx_callback(report_rx_callback);

    return NULL;
}

ZTEST_SUITE(thingset_can, NULL, thingset_can_setup, NULL, NULL, NULL);
