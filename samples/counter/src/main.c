/*
 * Copyright (c) The ThingSet Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>

#include "thingset/can.h"
#include "thingset/sdk.h"
#include <thingset.h>

static uint32_t counter;
static float pi = 3.14F;

#define APP_ID_SAMPLE          0x05
#define APP_ID_SAMPLE_RCOUNTER 0x050
#define APP_ID_SAMPLE_WCOUNTER 0x051
#define APP_ID_SAMPLE_CPI      0x052

THINGSET_ADD_GROUP(TS_ID_ROOT, APP_ID_SAMPLE, "Sample", THINGSET_NO_CALLBACK);

THINGSET_ADD_ITEM_UINT32(APP_ID_SAMPLE, APP_ID_SAMPLE_RCOUNTER, "rCounter", &counter,
                         THINGSET_ANY_R, TS_SUBSET_LIVE | TS_SUBSET_SUMMARY);

THINGSET_ADD_ITEM_UINT32(APP_ID_SAMPLE, APP_ID_SAMPLE_WCOUNTER, "wCounter", &counter,
                         THINGSET_ANY_RW, TS_SUBSET_LIVE | TS_SUBSET_SUMMARY);

THINGSET_ADD_ITEM_FLOAT(APP_ID_SAMPLE, APP_ID_SAMPLE_CPI, "cPi", &pi, 2, THINGSET_ANY_R,
                        TS_SUBSET_LIVE | TS_SUBSET_SUMMARY);

static void report_rx_callback(uint16_t data_id, const uint8_t *value, size_t value_len,
                               uint8_t source_addr)
{
    printf("data_id 0x%X src_addr 0x%X length %u raw ", data_id, source_addr, value_len);
    for (size_t i = 0; i < value_len; i++) {
        printf("%02x", *(value + i));
        if (i == 0 && value_len != 1) {
            printf(" ");
        }
    }

    printf(" ");

    const struct thingset_data_object *obj = thingset_get_object_by_id(&ts, data_id);

    struct thingset_data_object object = {};
    thingset_bin_test(&ts, data_id, value, value_len, &object);
    printf("data %X ", *object.data.u32);

    printf("type %u ", obj->type);

    if (obj->type == THINGSET_TYPE_U32) {
        size_t offset = 0;
        if (value_len > 1) {
            offset++;
        }
        uint32_t tmp = sys_be32_to_cpu(*(uint32_t *)(value + offset)
                                       << ((sizeof(uint32_t) - (value_len - offset)) * 8));
        printf("value %u\n", tmp);
    }
    else if (obj->type == THINGSET_TYPE_F32) {
        uint32_t tmp = sys_be32_to_cpu(*(uint32_t *)(value + 1));
        printf("value %0.2f\n", *(float *)&tmp);
    }
    else {
        printf("not supported!\n");
    }
}

int main(void)
{
    thingset_can_set_report_rx_callback(report_rx_callback);

    while (true) {
        counter++;
        k_sleep(K_SECONDS(1));
    }

    return 0;
}
