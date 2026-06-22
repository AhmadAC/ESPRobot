#pragma once

void wifi_manager_init();
char* wifi_scan_networks_json();
void wifi_save_credentials(const char* ssid, const char* pass);