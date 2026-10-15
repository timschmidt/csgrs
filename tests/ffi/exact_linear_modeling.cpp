#include "csgrs.hpp"
#include <cassert>
#include <string>

int main() {
  using Region = csgrs::CurveRegion<csgrs::I128>;
  const csgrs_i128_t width{0, UINT64_C(9007199254740993)};
  const auto region = Region::rectangle(width, {0, 1});
  try {
    const auto failed = region.extrude({0, 0});
    (void)failed;
    assert(false);
  } catch (const csgrs::error &error) {
    assert(error.status() == CSGRS_STATUS_VALIDATION_FAILED);
    assert(std::string(error.what()).find("nonzero") != std::string::npos);
  }
  const auto solid = region.extrude({0, 2});
  const auto bounds = solid.bounding_box().value();
  assert(bounds.maxs.x.hi == 0 && bounds.maxs.x.lo == width.lo);
  assert(bounds.maxs.y.lo == 1 && bounds.maxs.z.lo == 2);
}
