idf_component_register(SRCS "main.cpp"
                            "wifi_manager.cpp"
                            "sensor_monitor.cpp"
                            "servo_controller.cpp"
                            "web_server.cpp"
                            "ble_manager.cpp"
                            "audio_player.cpp"
                    INCLUDE_DIRS ".")

target_add_binary_data(${COMPONENT_LIB} "../Sound/dog-barking.wav" BINARY)