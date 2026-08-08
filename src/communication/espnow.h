#ifndef ESPNOW_H
#define ESPNOW_H

#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

#include "communication/espnow_protocol.h"

class ESPNow
{
    public:
        ESPNow();
        
        bool begin();
        bool send(const uint8_t* data, size_t len);
        bool send_attitude(const attitude_data_packet_t & attitude_data);

        bool is_armed() const;
        bool is_reset() const;
        bool is_throttle_up() const;
        bool is_throttle_down() const;
        bool is_direction_forward() const;
        bool is_direction_backward() const;
        bool is_direction_left() const;
        bool is_direction_right() const;
        void reset_command();
        const command_data_packet_t & get_command_data() const;

        bool is_heartbeat_recent(uint32_t timeout_ms) const;    

        void register_recv_callback(esp_now_recv_cb_t callback);
        void register_send_callback(esp_now_send_cb_t callback);

        static void on_data_sent(const wifi_tx_info_t *info, esp_now_send_status_t status);
        static void on_data_recv(const esp_now_recv_info_t *info, const uint8_t *data, int data_len);

        bool register_peer();

    private:
        // replace with controller's mac address
        uint8_t controller_address[6] = {0xAC, 0xA7, 0x04, 0xBB, 0x6D, 0x64};

        command_data_packet_t command_data;
        heartbeat_data_packet_t heartbeat_data;

        uint32_t last_heartbeat_receive_time_us;

        static ESPNow * active_instance;

        bool add_peer(const esp_now_peer_info_t* peerInfo);
};

#endif /* ESPNOW_H */
