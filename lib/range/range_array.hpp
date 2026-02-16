#pragma once

#include "range_interface.hpp"
#include <cstdint>

static constexpr uint8_t MAX_SENSORS = 4;

struct RangesData {
    float range[MAX_SENSORS] = {};
    uint8_t count = 0;
};

/// Holds N independent RangeInterface pointers.
class RangeArray {
public:
    RangeArray() = default;
    RangeArray(RangeInterface** sensors, uint8_t count);

    void init();
    void update();

    const RangesData getData() const { return data_; }
    uint8_t count() const { return count_; }
    bool isAvailable() const { return count_ > 0; }

private:
    RangeInterface** sensors_ = nullptr;
    uint8_t       count_   = 0;
    RangesData    data_    = {};
};

extern RangeArray g_ranges;