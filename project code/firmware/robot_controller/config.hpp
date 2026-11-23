#pragma once
#include "src/controller.hpp"
namespace robot {
inline Config productionConfig() {
  Config c;
  // Commissioning records: leave false until the actual head system is tested.
  c.powerWiringVerified=false; // Tethered power, fuses, ground, USB/backfeed.
  c.servoLimitsVerified=false; // Actual head travel, pulse limits and support.
  c.panMinUs=1400;c.panMaxUs=1600;
  c.tiltMinUs=1400;c.tiltMaxUs=1600;
  return c;
}
}
