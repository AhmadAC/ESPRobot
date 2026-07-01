#include "ble_manager.h"
#include "esp_log.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "servo_controller.h"
#include <string.h>

static const char *TAG = "BLE_MGR";

// RX Callback logic that catches strings written from a Phone Bluetooth App
static int ble_rx_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    char buf[128] = {0};
    int len = OS_MBUF_PKTLEN(ctxt->om);
    if (len > 0 && len < sizeof(buf)) {
        os_mbuf_copydata(ctxt->om, 0, len, buf);
        // Trim standard trailing newlines
        for(int i = 0; i < len; i++) {
            if(buf[i] == '\r' || buf[i] == '\n') buf[i] = '\0';
        }
        ESP_LOGI(TAG, "BLE UART RX Trigger: %s", buf);
        
        // Pass the generic string command into the servo backend
        servo_set_action(buf);
    }
    return 0;
}

// -------------------------------------------------------------
// Static Definitions for C++ Compliance
// -------------------------------------------------------------
// In C++, we must declare these UUID structs statically rather 
// than using inline macros so the compiler can assign them 
// permanent memory addresses (lvalues).
static const ble_uuid16_t svc_uuid = BLE_UUID16_INIT(0xABF0);
static const ble_uuid16_t chr_uuid = BLE_UUID16_INIT(0xABF1);

// Standalone Characteristics Array (Nordic UART RX mimic)
static const struct ble_gatt_chr_def gatt_chrs[] = {
    {
        .uuid = (const ble_uuid_t *)&chr_uuid,
        .access_cb = ble_rx_cb,
        .arg = nullptr,
        .descriptors = nullptr,
        .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
        .min_key_size = 0,
        .val_handle = nullptr
    },
    {} // C++ Zero-initialize termination marker
};

// Standalone Minimalistic BLE Server Tree
static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = (const ble_uuid_t *)&svc_uuid,
        .includes = nullptr,
        .characteristics = gatt_chrs
    },
    {} // C++ Zero-initialize termination marker
};
// -------------------------------------------------------------

static void ble_app_on_sync(void) {
    uint8_t own_addr_type;
    ble_hs_id_infer_auto(0, &own_addr_type);
    
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (uint8_t *)"ESPRobot";
    fields.name_len = 8;
    fields.name_is_complete = 1;
    ble_gap_adv_set_fields(&fields);

    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    
    ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, NULL, NULL);
    ESP_LOGI(TAG, "BLE UART Advertising Started (Connect via NRFConnect or LightBlue)");
}

void ble_host_task(void *param) {
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void ble_manager_init() {
    nimble_port_init();
    ble_svc_gap_device_name_set("ESPRobot");
    ble_svc_gap_init();
    ble_gatts_count_cfg(gatt_svcs);
    ble_gatts_add_svcs(gatt_svcs);
    ble_hs_cfg.sync_cb = ble_app_on_sync;
    nimble_port_freertos_init(ble_host_task);
    ESP_LOGI(TAG, "BLE Manager Initialization Finished.");
}