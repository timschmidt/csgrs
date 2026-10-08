#ifndef EXACTCORE_HYPER_COMPARISON_EMPTY_CIRCLE_COMPLEX_H
#define EXACTCORE_HYPER_COMPARISON_EMPTY_CIRCLE_COMPLEX_H

#include <cstddef>
#include <cstdint>
#include <vector>

#include <CORE/CORE.h>

namespace exactcore_hyper_comparison {

// Enumerate every nondegenerate point triple whose open circumcircle contains
// no other input point. Cocircular triples are retained, so the result is a
// Delaunay cell complex rather than a tie-broken triangulation.
inline std::vector<std::uint32_t> exhaustive_empty_circle_complex(
    const std::vector<CORE::Expr> &x,
    const std::vector<CORE::Expr> &y) {
  std::vector<std::uint32_t> cells;
  if (x.size() != y.size())
    return cells;

  const std::size_t point_count = x.size();
  for (std::size_t i = 0; i + 2 < point_count; ++i) {
    for (std::size_t j = i + 1; j + 1 < point_count; ++j) {
      for (std::size_t k = j + 1; k < point_count; ++k) {
        const CORE::Expr orientation =
            (x[j] - x[i]) * (y[k] - y[i]) -
            (y[j] - y[i]) * (x[k] - x[i]);
        const int winding = CORE::sign(orientation);
        if (winding == 0)
          continue;

        bool empty = true;
        for (std::size_t m = 0; m < point_count && empty; ++m) {
          if (m == i || m == j || m == k)
            continue;

          const CORE::Expr ax = x[i] - x[m];
          const CORE::Expr ay = y[i] - y[m];
          const CORE::Expr bx = x[j] - x[m];
          const CORE::Expr by = y[j] - y[m];
          const CORE::Expr cx = x[k] - x[m];
          const CORE::Expr cy = y[k] - y[m];
          const CORE::Expr a_squared = ax * ax + ay * ay;
          const CORE::Expr b_squared = bx * bx + by * by;
          const CORE::Expr c_squared = cx * cx + cy * cy;
          const CORE::Expr determinant =
              a_squared * (bx * cy - by * cx) -
              b_squared * (ax * cy - ay * cx) +
              c_squared * (ax * by - ay * bx);

          empty = CORE::sign(determinant) != winding;
        }

        if (empty) {
          cells.push_back(static_cast<std::uint32_t>(i));
          cells.push_back(static_cast<std::uint32_t>(j));
          cells.push_back(static_cast<std::uint32_t>(k));
        }
      }
    }
  }
  return cells;
}

} // namespace exactcore_hyper_comparison

#endif
