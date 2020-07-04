#ifndef AMICE_EVENT_GROUP_H
#define AMICE_EVENT_GROUP_H

#include <type_traits>
#include <optional>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>

namespace core {

namespace internal {

template<auto V>
struct ValidateEvent {
    static_assert(sizeof(V) == sizeof(uint8_t), "Underlying event value must be uint8_t");
    static_assert(static_cast<uint8_t>(V) < 24, "Event may not be larger than 24");
};

}

template<auto... Es>
/**
 * Represents a list of event list events
 * @tparam Es Events
 */
struct GroupEvents : public internal::ValidateEvent<Es> ... {
    /** Event bits */
    static constexpr uint8_t Bits = ((1u << static_cast<uint8_t>(Es)) | ... | 0);

};

template<typename E>
/**
 * Represents an event group
 * @tparam E Event type
 */
class EventGroup {
public:
    /**
     * Contains a snapshot of the set bits in the event group
     */
    struct Snapshot {
    protected:
        friend class EventGroup<E>;

        /**
         * Constructor
         * @param bits Event bits
         */
        explicit Snapshot(uint32_t bits)
                : bits{bits} {}

    public:
        /**
         * Returns whether the given event has been raised in this snapshot
         * @param e Event to check
         * @return Whether the event was raised
         */
        [[nodiscard]] bool operator&(E e) const {
            const auto bitmask = 1u << static_cast<uint8_t>(e);
            return (bits & bitmask) == bitmask;
        }

    private:
        uint32_t bits;
    };

    using Event = E;

    static_assert(sizeof(E) == sizeof(uint8_t), "Underlying type of an event group event must be uint8_t");

    /** Constructor */
    EventGroup()
            : m_group{} {
        m_handle = xEventGroupCreateStatic(&m_group);
    }

    template<typename EE>
    /**
     * Raises an event
     * @tparam EE Type to pass, either the event type, or a GroupEvents
     * @param event Event or GroupEvents to raise
     */
    void Raise(EE event) {
        if constexpr (std::is_same_v<EE, E>) {
            xEventGroupSetBits(m_handle, 1u << static_cast<uint8_t>(event));
        } else {
            xEventGroupSetBits(m_handle, EE::Bits);
        }
    }

    template<typename EE>
    /**
     * Raises an event, version with an interface similar to EventQueue
     * @tparam EE Type to pass, either the event type, or a GroupEvents
     * @param event Event or GroupEvents to raise
     */
    [[nodiscard]] bool Raise(EE event, uint32_t) {
        Raise<EE>(event);
        return true;
    }

    template<typename EE>
    /**
     * Resets an event
     * @tparam EE Type to pass, either the event type, or a GroupEvents
     * @param event Event or GroupEvents to reset
     */
    void Reset(EE event) {
        if constexpr (std::is_same_v<EE, E>) {
            xEventGroupClearBits(m_handle, 1u << static_cast<uint8_t>(event));
        } else {
            (void) event;
            xEventGroupClearBits(m_handle, EE::Bits);
        }
    }

    /**
     * Waits until the given event was raised
     * @param event Event to check for
     * @param timeout_ms Timeout in milliseconds
     * @param reset_on_success Whether the event should be reset on a successful await
     * @return Whether the await returns because of the events being raised (instead of a timeout)
     */
    bool Await(E event, uint32_t timeout_ms = 5000, bool reset_on_success = true) const {
        const auto bitmask = 1u << static_cast<uint8_t>(event);
        // Enter wait mode
        auto result = xEventGroupWaitBits(
                m_handle,
                bitmask,
                reset_on_success,
                pdTRUE,
                pdMS_TO_TICKS(timeout_ms));

        // Return whether the event bits were set
        return (result & bitmask) == bitmask;
    }

    template<typename G>
    /**
     * Waits until all of the given event have been raised
     * @tparam G Events to check for
     * @param timeout_ms Timeout in milliseconds
     * @param reset_on_success Whether the event should be reset on a successful await
     * @return Whether the await returns because of the events being raised (instead of a timeout)
     */
    bool AwaitAll(G, uint32_t timeout_ms = 5000, bool reset_on_success = true) const {
        const auto bitmask = G::Bits;
        // Enter wait mode
        auto result = xEventGroupWaitBits(
                m_handle,
                bitmask,
                reset_on_success,
                pdTRUE,
                pdMS_TO_TICKS(timeout_ms));

        // Return whether the event bits were set
        return (result & bitmask) == bitmask;
    }

    template<typename G>
    /**
     * Waits until any of the given event have been raised
     * @tparam G Events to check for
     * @param timeout_ms Timeout in milliseconds
     * @param reset_on_success Whether the event should be reset on a successful await
     * @return Snapshot of the event group at the time of returning, or std::nullopt on timeout
     */
    std::optional<Snapshot> AwaitAny(G, uint32_t timeout_ms = 5000, bool reset_on_success = true) const {
        const auto bitmask = G::Bits;
        // Enter wait mode
        auto result = xEventGroupWaitBits(
                m_handle,
                bitmask,
                reset_on_success,
                pdFALSE,
                pdMS_TO_TICKS(timeout_ms));

        // Return whether the event bits were set
        if ((result & bitmask) != 0) {
            return Snapshot{result};
        } else {
            return std::nullopt;
        }
    }

private:
    StaticEventGroup_t m_group;
    EventGroupHandle_t m_handle;

};

}

#endif //AMICE_EVENT_GROUP_H
