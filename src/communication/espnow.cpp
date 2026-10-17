#include "communication/espnow.h"

ESPNow *ESPNow::active_instance = nullptr;

// ESP-NOW callbacks run in the Wi-Fi task. Snapshot the shared command/timestamps
// under a short critical section instead of reading a half-updated packet.
static portMUX_TYPE s_rx_mux = portMUX_INITIALIZER_UNLOCKED;

ESPNow::ESPNow()
    : command_data{false, false, false, false, false, false, false, false}, heartbeat_data{0u},

      last_heartbeat_receive_time_us(0u), last_command_receive_time_us(0u), have_heartbeat(false), have_command(false)
{
}

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

bool ESPNow::send(const uint8_t *data, size_t len)
{
    if (esp_now_send(controller_address, data, len) == ESP_OK)
    {
        return true;
    }

    return false;
}

bool ESPNow::send_attitude(const attitude_data_packet_t &attitude_data)
{
    return (send(reinterpret_cast<const uint8_t *>(&attitude_data), sizeof(attitude_data)));
}

bool ESPNow::send_status(const flight_status_packet_t &status_data)
{
    return (send(reinterpret_cast<const uint8_t *>(&status_data), sizeof(status_data)));
}

ESPNowRxSnapshot ESPNow::get_rx_snapshot() const
{
    ESPNowRxSnapshot out{};
    portENTER_CRITICAL(&s_rx_mux);
    out.command = command_data;
    out.last_command_us = last_command_receive_time_us;
    out.last_heartbeat_us = last_heartbeat_receive_time_us;
    out.have_command = (have_command == true);
    out.have_heartbeat = (have_heartbeat == true);
    portEXIT_CRITICAL(&s_rx_mux);
    return out;
}

bool ESPNow::is_fresh(uint32_t now_us, uint32_t last_us, bool seen, uint32_t timeout_us)
{
    return ((seen == true) && (uint32_t(now_us - last_us) <= timeout_us));
}

bool ESPNow::is_armed() const
{
    return (get_rx_snapshot().command.arm == true);
}

bool ESPNow ::is_reset() const
{
    return get_rx_snapshot().command.reset;
}

bool ESPNow ::is_throttle_up() const
{
    return get_rx_snapshot().command.throttle_up;
}

bool ESPNow ::is_throttle_down() const
{
    return get_rx_snapshot().command.throttle_down;
}

bool ESPNow ::is_direction_forward() const
{
    return get_rx_snapshot().command.direction_forward;
}

bool ESPNow ::is_direction_backward() const
{
    return get_rx_snapshot().command.direction_backward;
}

bool ESPNow ::is_direction_left() const
{
    return get_rx_snapshot().command.direction_left;
}

bool ESPNow ::is_direction_right() const
{
    return get_rx_snapshot().command.direction_right;
}

void ESPNow ::reset_command()
{
    portENTER_CRITICAL(&s_rx_mux);
    command_data.arm = false;
    command_data.reset = false;

    command_data.throttle_up = false;
    command_data.throttle_down = false;

    command_data.direction_forward = false;
    command_data.direction_backward = false;
    command_data.direction_left = false;
    command_data.direction_right = false;
    portEXIT_CRITICAL(&s_rx_mux);
}

command_data_packet_t ESPNow::get_command_data() const
{
    return get_rx_snapshot().command;
}

bool ESPNow::is_heartbeat_recent(uint32_t timeout_us) const
{
    const ESPNowRxSnapshot snap = get_rx_snapshot();
    return (is_fresh(micros(), snap.last_heartbeat_us, snap.have_heartbeat, timeout_us) == true);
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
    if ((active_instance == nullptr) || (data == nullptr) || (info == nullptr) || (info->src_addr == nullptr))
    {
        return;
    }

    ESPNow *self = active_instance;

    // Accept control packets ONLY from the configured remote's STA MAC.
    if (memcmp(info->src_addr, self->controller_address, 6) != 0)
    {
        return;
    }

    const uint32_t now = micros();
    if (data_len == static_cast<int>(sizeof(command_data_packet_t)))
    {
        command_data_packet_t cmd{};
        memcpy(&cmd, data, sizeof(cmd));
        portENTER_CRITICAL(&s_rx_mux);
        self->command_data = cmd;
        self->last_command_receive_time_us = now;
        self->have_command = true;
        portEXIT_CRITICAL(&s_rx_mux);
        return;
    }

    if (data_len == static_cast<int>(sizeof(heartbeat_data_packet_t)))
    {
        heartbeat_data_packet_t hb{};
        memcpy(&hb, data, sizeof(hb));
        portENTER_CRITICAL(&s_rx_mux);
        self->heartbeat_data = hb;
        self->last_heartbeat_receive_time_us = now;
        self->have_heartbeat = true;
        portEXIT_CRITICAL(&s_rx_mux);
        return;
    }
}

bool ESPNow::register_peer()
{
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, controller_address, 6);

    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    const bool success = (add_peer(&peerInfo) == true);

    return (success == true);
}

bool ESPNow::add_peer(const esp_now_peer_info_t *peerInfo)
{
    if (esp_now_add_peer(peerInfo) != ESP_OK)
    {
        return false;
    }

    return true;
}
