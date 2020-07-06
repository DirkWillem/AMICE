#ifndef AMICE_CAN_BUS_STATUS_H
#define AMICE_CAN_BUS_STATUS_H

#include <atomic>
#include <cstdint>

#include "can_store.h"

namespace can {

/**
 * Contains information on the CAN bus status
 */
struct BusInfo {
    /** Whether the CAN bus AMICE is connected to, is Stella Era's CAN bus */
    bool is_stella_era;
};

/**
 * Class which keeps track of the CAN bus status
 */
class BusStatus {
public:
    BusStatus();

    /**
     * Calculates the bus status
     * @param store CAN store
     * @param t Current timestamp
     */
    void CalculateBusStatus(const Store& store, uint32_t t);

    /**
     * Updates the bus status monitor that a message was received
     * @param n_bits Number of bits received
     */
    void ReceivedMessage(uint32_t n_bits);

    /**
     * Returns the bus load
     * @return Bus load
     */
    [[nodiscard]] uint32_t bus_load() const;

    /**
     * Returns the current CAN bus load
     */
    [[nodiscard]] uint32_t message_frequency() const;

    /**
     * Returns the current bus info
     * @return CAN bus info
     */
    [[nodiscard]] BusInfo bus_info() const;
private:
    uint32_t m_last_calculation;
    uint32_t m_last_msg_timestamp;
    std::atomic<uint32_t> m_n_recv_msgs;
    std::atomic<uint32_t> m_n_recv_bits;

    uint32_t m_bus_load;
    uint32_t m_msg_frequency;
    BusInfo m_bus_info;
};

}


#endif //AMICE_CAN_BUS_STATUS_H
