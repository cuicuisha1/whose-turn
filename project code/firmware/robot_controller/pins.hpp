#pragma once
#include <stdint.h>
namespace robot {
constexpr uint8_t PIN_PAN=10;
constexpr uint8_t PIN_TILT=11;
static_assert(PIN_PAN==10 && PIN_TILT==11,"Do not change the head pin assignment");
}
