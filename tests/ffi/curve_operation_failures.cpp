#include "csgrs.hpp"
#include <cassert>
#include <limits>
#include <string>

template <class Action>
void fails(Action action, csgrs_status_t status, const char *message) {
  try {
    action();
    assert(false);
  } catch (const csgrs::error &error) {
    assert(error.status() == status);
    assert(std::string(error.what()).find(message) != std::string::npos);
  }
}

int main() {
  using Region = csgrs::CurveRegion<csgrs::F64>;
  const auto square = Region::square(2);
  const auto restored = square.rotate(0, 0, 13).rotate(0, 0, -13);
  fails([&] { square.union_with(restored); }, CSGRS_STATUS_PREDICATE_UNDECIDED, "blocked by Ordering");
  fails([&] { square.difference(restored); }, CSGRS_STATUS_PREDICATE_UNDECIDED, "blocked by Ordering");
  fails([&] { square.intersection(restored); }, CSGRS_STATUS_PREDICATE_UNDECIDED, "blocked by Ordering");
  fails([&] { square.xor_with(restored); }, CSGRS_STATUS_PREDICATE_UNDECIDED, "blocked by Ordering");
  fails([&] { square.revolve(360, 1); }, CSGRS_STATUS_VALIDATION_FAILED, "segments must not be less than 2");
  const auto large = Region::square(std::numeric_limits<double>::max()).scale(2, 2, 1);
  fails([&] { large.region_profiles(); }, CSGRS_STATUS_VALIDATION_FAILED, "finite projection point is not finite");
  const auto empty = square.difference(square);
  assert(!empty.bounding_box().has_value());
  const auto recovered = empty.union_with(square);
  const auto bounds = recovered.bounding_box().value();
  assert(bounds.mins.x == 0 && bounds.mins.y == 0 && bounds.maxs.x == 2 && bounds.maxs.y == 2);
}
