#include "csgrs.hpp"
#include <cassert>
#include <limits>
#include <string>

int main() {
  using Region = csgrs::CurveRegion<csgrs::F64>;
  try {
    const auto invalid = Region::rectangle(0, -1);
    (void)invalid;
    assert(false);
  } catch (const csgrs::error &error) {
    assert(error.status() == CSGRS_STATUS_VALIDATION_FAILED);
    assert(std::string(error.what()).find("height") != std::string::npos);
  }
  try {
    const auto invalid = Region::circle(1, std::numeric_limits<size_t>::max());
    (void)invalid;
    assert(false);
  } catch (const csgrs::error &error) {
    assert(error.status() == CSGRS_STATUS_RESOURCE_EXHAUSTED);
  }
  const auto empty = Region::square(0);
  assert(!empty.bounding_box().has_value());
  const auto restored = empty.union_with(Region::square(1));
  assert(restored.bounding_box().value().maxs.x == 1);
}
