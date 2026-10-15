#include "csgrs.hpp"
#include <cassert>
#include <string>

int main() {
  using Region = csgrs::CurveRegion<csgrs::F64>;
  const auto square = Region::square(1);
  try {
    const auto invalid = square.translate(0, 0, 1);
    (void)invalid;
    assert(false);
  } catch (const csgrs::error &error) {
    assert(error.status() == CSGRS_STATUS_UNSUPPORTED);
    assert(std::string(error.what()).find("XY plane") != std::string::npos);
  }
  try {
    const auto invalid = Region::square(0).scale(0, 1, 1);
    (void)invalid;
    assert(false);
  } catch (const csgrs::error &error) {
    assert(error.status() == CSGRS_STATUS_VALIDATION_FAILED);
  }
  const auto reflected = square.rotate(180, 0, 0);
  const auto bounds = reflected.bounding_box().value();
  assert(bounds.mins.y == -1 && bounds.maxs.y == 0 && bounds.maxs.x == 1);
  const auto restored = Region::square(0).union_with(reflected);
  assert(restored.bounding_box().value().mins.y == -1);
}
