#include "csgrs.hpp"
#include <cassert>
#include <iostream>
#include <string>

int main() {
  using Mesh = csgrs::TriangleMesh<csgrs::F64>;
  const auto cube = Mesh::cube(1);
  const auto restored = cube.rotate(0, 0, 13).rotate(0, 0, -13);
  const auto operations = {&Mesh::union_with, &Mesh::difference,
                           &Mesh::intersection, &Mesh::xor_with};
  unsigned structured = 0;
  for (const auto operation : operations) {
    try {
      const auto failed = (cube.*operation)(restored);
      (void)failed;
      assert(false && "failed Boolean published geometry");
    } catch (const csgrs::error &error) {
      assert(std::string(error.what()).find("predicate could not be decided") != std::string::npos);
      structured += error.status() == CSGRS_STATUS_PREDICATE_UNDECIDED;
    }
  }
  std::cout << "C++ undecided Boolean statuses: " << structured << "/4" << std::endl;
  assert(structured == 4);
  const auto empty = cube.difference(cube);
  assert(!empty.bounding_box().has_value());
  const auto recovered = empty.union_with(cube);
  const auto bounds = recovered.bounding_box().value();
  assert(bounds.mins.x == 0 && bounds.maxs.x == 1);
  std::cout << "C++ typed Boolean failures, empty result and recovery passed\n";
}
