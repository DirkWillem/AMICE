#include "can_bus_status.h"

#include <cmath>

#include <canspec/era/era_matcher.h>

namespace can {

BusStatus::BusStatus()
    : m_last_calculation{0}, m_last_msg_timestamp{0}, m_n_recv_msgs{0}, m_n_recv_bits{0},
      m_bus_load{0}, m_msg_frequency{0}, m_bus_info{} {}

void BusStatus::CalculateBusStatus(const Store &store, uint32_t t) {
    const auto dt = static_cast<float>(t - m_last_calculation);

    // Calculates bus load for the last period
    m_msg_frequency = roundf(m_n_recv_msgs / dt);
    m_bus_load = roundf(m_n_recv_bits / dt);

    // Check for bus info
    m_bus_info.is_stella_era = era::CANBusMatcher::Match(store);
}

void BusStatus::ReceivedMessage(uint32_t n_bits) {
    m_n_recv_bits += 8*n_bits + 44 + (34 + 8*n_bits - 1)/4;
    m_n_recv_msgs++;
}

uint32_t BusStatus::bus_load() const {
    return m_bus_load;
}

uint32_t BusStatus::message_frequency() const {
    return m_msg_frequency;
}

BusInfo BusStatus::bus_info() const {
    return m_bus_info;
}

}