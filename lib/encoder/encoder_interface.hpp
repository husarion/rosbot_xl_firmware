#pragma once

#include <stdint.h>
#include <math.h>

struct EncoderData {
    float position = 0.0f;   // [rad]
    float velocity = 0.0f;   // [rad/s]
};

class EncoderInterface {
public:
    virtual ~EncoderInterface() = default;

    virtual void init() = 0;
    virtual void update() = 0;
    virtual void reset() = 0;
    virtual const EncoderData getData() const { return data_; }
    virtual const char* name() const = 0;

protected:
    EncoderData data_ = {};
};