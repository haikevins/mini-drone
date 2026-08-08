#include "communication/espnow.h"

ESPNow * ESPNow::active_instance = nullptr;

ESPNow::ESPNow() :
    command_data{false, false, false, false, false, false, false, false},
    heartbeat_data{0u},

    last_heartbeat_receive_time_us(0u)
{}

bool ESPNow::begin()
{
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    if (esp_now_init() != ESP_OK)
    {
        return false;
    }

    active_instance = this;
    register_send_callback(ESPNow::on_data_sent);
    register_recv_callback(ESPNow::on_data_recv);
    
    return true;
}

bool ESPNow::send(const uint8_t* data, size_t len)
{
    if (esp_now_send(controller_address, data, len) == ESP_OK)
    {
        return true;
    }

    return false;
}

bool ESPNow::send_attitude(const attitude_data_packet_t & attitude_data)
{
    return send(reinterpret_cast<const uint8_t*>(&attitude_data), sizeof(attitude_data));
}

bool ESPNow::is_armed() const
{
    return command_data.arm;
}

bool ESPNow :: is_reset() const
{
    return command_data.reset;
}

bool ESPNow :: is_throttle_up() const
{
    return command_data.throttle_up;
}

bool ESPNow :: is_throttle_down() const
{
    return command_data.throttle_down;
}

bool ESPNow :: is_direction_forward() const
{
    return command_data.direction_forward;
}

bool ESPNow :: is_direction_backward() const
{
    return command_data.direction_backward;
}

bool ESPNow :: is_direction_left() const
{
    return command_data.direction_left;
}

bool ESPNow :: is_direction_right() const
{
    return command_data.direction_right;
}

void ESPNow :: reset_command()
{
    command_data.arm = false;
    command_data.reset = false;

    command_data.throttle_up = false;
    command_data.throttle_down = false;

    command_data.direction_forward = false;
    command_data.direction_backward = false;
    command_data.direction_left = false;
    command_data.direction_right = false;
}

const command_data_packet_t & ESPNow::get_command_data() const
{
    return command_data;
}

bool ESPNow::is_heartbeat_recent(uint32_t timeout_us) const
{
    return (micros() - last_heartbeat_receive_time_us) <= timeout_us;
}

void ESPNow::register_recv_callback(esp_now_recv_cb_t callback)
{
    esp_now_register_recv_cb(callback);
}

void ESPNow::register_send_callback(esp_now_send_cb_t callback)
{
    esp_now_register_send_cb(callback);
}

void ESPNow::on_data_sent(const wifi_tx_info_t *info, esp_now_send_status_t status)
{
    (void)info;
    (void)status;
}

void ESPNow::on_data_recv(const esp_now_recv_info_t *info, const uint8_t *data, int data_len)
{
    (void)info;

    if ((active_instance == nullptr) || (data == nullptr))
    {
        return;
    }

    ESPNow * self = active_instance;
    const uint32_t now = micros();

    if (data_len == static_cast<int>(sizeof(self->command_data)))
    {
        memcpy(&self->command_data, data, sizeof(self->command_data));
        return;
    }

    if (data_len == static_cast<int>(sizeof(heartbeat_data_packet_t)))
    {
        memcpy(&self->heartbeat_data, data, sizeof(self->heartbeat_data));
        self->last_heartbeat_receive_time_us = now;
        return;
    }
}

bool ESPNow::register_peer()
{
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, controller_address, 6);

    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    const bool success = add_peer(&peerInfo);

    return success;
}

bool ESPNow::add_peer(const esp_now_peer_info_t* peerInfo)
{
    if (esp_now_add_peer(peerInfo) != ESP_OK)
    {
        return false;
    }

    return true;
}
