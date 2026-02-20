
#pragma once

#include <Arduino.h>
#include <std_msgs/msg/bool.h>
#include "../types.hpp"

struct LedSubConfig {
    uint8_t     pin;
    const char* topic_name;
};

struct LedSubState {
    std_msgs__msg__Bool msg = {};
    uint8_t             pin = 0;
};

/// Generic LED callback — reads pin from LedSubState stored alongside msg.
inline void ledCallback(const void* msg_in) {
    // msg_in points to LedSubState.msg, LedSubState.pin follows in memory
    auto* state = reinterpret_cast<const LedSubState*>(msg_in);
    digitalWrite(state->pin, state->msg.data ? HIGH : LOW);
}

inline SubscriptionEntry makeLedSubscription(
        const LedSubConfig& cfg,
        LedSubState* state)
{
    state->pin = cfg.pin;
    pinMode(cfg.pin, OUTPUT);
    digitalWrite(cfg.pin, LOW);

    return SubscriptionEntry{
        .sub          = rcl_get_zero_initialized_subscription(),
        .msg          = &state->msg,
        .type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        .topic_name   = cfg.topic_name,
        .callback     = ledCallback,
        .best_effort  = false,   // reliable for LED commands
    };
}