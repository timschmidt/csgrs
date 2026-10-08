#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <new>
#include <vector>

#define CORE_LEVEL 3
#include <CORE/CORE.h>
#include <CORE/geometry2d.h>
#include <CORE/geometry3d.h>

#undef double
#undef long

#include <gmp.h>
#include <mpfr.h>

#include "exactcore_memory.h"

/*
 * This translation unit is linked only for the `memory-profile` Cargo feature.
 * It deliberately interposes C++ allocation and installs GMP allocation hooks;
 * keeping it out of ordinary builds prevents instrumentation from perturbing
 * the Criterion latency campaign.
 */

namespace {

const std::uint64_t MEMORY_MAGIC = UINT64_C(0x45434d454d4f5259);

struct alignas(std::max_align_t) AllocationHeader {
  std::size_t size;
  std::uint64_t magic;
  bool counted;
};

std::atomic<bool> tracking_enabled(false);
std::atomic<bool> gmp_hooks_installed(false);
std::atomic<std::uint64_t> allocation_count(0);
std::atomic<std::uint64_t> reallocation_count(0);
std::atomic<std::uint64_t> deallocation_count(0);
std::atomic<std::uint64_t> allocated_bytes(0);
std::atomic<std::uint64_t> reallocated_bytes(0);
std::atomic<std::uint64_t> deallocated_bytes(0);
std::atomic<std::uint64_t> live_bytes(0);
std::atomic<std::uint64_t> peak_live_bytes(0);

void update_peak(std::uint64_t candidate) {
  std::uint64_t peak = peak_live_bytes.load(std::memory_order_relaxed);
  while (candidate > peak &&
         !peak_live_bytes.compare_exchange_weak(
             peak, candidate, std::memory_order_relaxed,
             std::memory_order_relaxed)) {
  }
}

void record_new_allocation(std::size_t size) {
  allocation_count.fetch_add(1, std::memory_order_relaxed);
  allocated_bytes.fetch_add(size, std::memory_order_relaxed);
  const std::uint64_t live =
      live_bytes.fetch_add(size, std::memory_order_relaxed) + size;
  update_peak(live);
}

void record_new_reallocation(std::size_t size) {
  reallocation_count.fetch_add(1, std::memory_order_relaxed);
  reallocated_bytes.fetch_add(size, std::memory_order_relaxed);
  const std::uint64_t live =
      live_bytes.fetch_add(size, std::memory_order_relaxed) + size;
  update_peak(live);
}

void record_resize(std::size_t old_size, std::size_t new_size) {
  if (new_size >= old_size) {
    const std::uint64_t live =
        live_bytes.fetch_add(new_size - old_size, std::memory_order_relaxed) +
        new_size - old_size;
    update_peak(live);
  } else {
    live_bytes.fetch_sub(old_size - new_size, std::memory_order_relaxed);
  }
}

void record_deallocation(std::size_t size, bool count_event) {
  live_bytes.fetch_sub(size, std::memory_order_relaxed);
  if (count_event) {
    deallocation_count.fetch_add(1, std::memory_order_relaxed);
    deallocated_bytes.fetch_add(size, std::memory_order_relaxed);
  }
}

void *tracked_allocate(std::size_t requested) {
  const std::size_t size = requested == 0 ? 1 : requested;
  if (size > std::numeric_limits<std::size_t>::max() -
                 sizeof(AllocationHeader)) {
    return 0;
  }
  void *raw = std::malloc(sizeof(AllocationHeader) + size);
  if (!raw)
    return 0;
  AllocationHeader *header = static_cast<AllocationHeader *>(raw);
  header->size = size;
  header->magic = MEMORY_MAGIC;
  header->counted = tracking_enabled.load(std::memory_order_relaxed);
  if (header->counted)
    record_new_allocation(size);
  return header + 1;
}

AllocationHeader *allocation_header(void *pointer) {
  if (!pointer)
    return 0;
  AllocationHeader *header = static_cast<AllocationHeader *>(pointer) - 1;
  if (header->magic != MEMORY_MAGIC)
    std::abort();
  return header;
}

void tracked_deallocate(void *pointer) {
  if (!pointer)
    return;
  AllocationHeader *header = allocation_header(pointer);
  if (header->counted) {
    record_deallocation(
        header->size, tracking_enabled.load(std::memory_order_relaxed));
  }
  header->magic = 0;
  std::free(header);
}

void *tracked_reallocate(void *pointer, std::size_t requested) {
  if (!pointer)
    return tracked_allocate(requested);
  const std::size_t size = requested == 0 ? 1 : requested;
  if (size > std::numeric_limits<std::size_t>::max() -
                 sizeof(AllocationHeader)) {
    return 0;
  }
  AllocationHeader *old_header = allocation_header(pointer);
  const std::size_t old_size = old_header->size;
  const bool was_counted = old_header->counted;
  AllocationHeader *new_header = static_cast<AllocationHeader *>(
      std::realloc(old_header, sizeof(AllocationHeader) + size));
  if (!new_header)
    return 0;

  const bool enabled = tracking_enabled.load(std::memory_order_relaxed);
  const bool counted = was_counted || enabled;
  new_header->size = size;
  new_header->magic = MEMORY_MAGIC;
  new_header->counted = counted;
  if (was_counted) {
    record_resize(old_size, size);
    if (enabled) {
      reallocation_count.fetch_add(1, std::memory_order_relaxed);
      reallocated_bytes.fetch_add(size, std::memory_order_relaxed);
    }
  } else if (enabled) {
    record_new_reallocation(size);
  }
  return new_header + 1;
}

void *gmp_allocate(std::size_t size) { return tracked_allocate(size); }

void *gmp_reallocate(void *pointer, std::size_t, std::size_t size) {
  return tracked_reallocate(pointer, size);
}

void gmp_deallocate(void *pointer, std::size_t) {
  tracked_deallocate(pointer);
}

void install_gmp_hooks() {
  bool expected = false;
  if (gmp_hooks_installed.compare_exchange_strong(
          expected, true, std::memory_order_relaxed,
          std::memory_order_relaxed)) {
    mp_set_memory_functions(gmp_allocate, gmp_reallocate, gmp_deallocate);
  }
}

#if defined(__GNUC__) || defined(__clang__)
__attribute__((constructor(101))) void install_gmp_hooks_before_static_objects() {
  install_gmp_hooks();
}
#endif

CORE::BigInt integer(std::int64_t value) {
  return CORE::BigInt(static_cast<long>(value));
}

CORE::Expr expression(std::int64_t value) {
  return CORE::Expr(CORE::BigRat(integer(value)));
}

::Point2d point2(const std::int64_t *values) {
  return ::Point2d(expression(values[0]), expression(values[1]));
}

::Point3d point3(const std::int64_t *values) {
  return ::Point3d(expression(values[0]), expression(values[1]),
                   expression(values[2]));
}

#if defined(__GNUC__) || defined(__clang__)
template <typename T> inline void observe(const T &value) {
  __asm__ __volatile__("" : : "g"(&value) : "memory");
}
#else
template <typename T> inline void observe(const T &value) {
  volatile const T *observed = &value;
  (void)observed;
  std::atomic_signal_fence(std::memory_order_seq_cst);
}
#endif

std::size_t output_capacity(std::size_t cases, std::uint64_t repetitions) {
  if (cases == 0 || repetitions == 0)
    return 0;
  if (repetitions > std::numeric_limits<std::size_t>::max() / cases)
    throw std::bad_alloc();
  return cases * static_cast<std::size_t>(repetitions);
}

} // namespace

void *operator new(std::size_t size) {
  void *pointer = tracked_allocate(size);
  if (!pointer)
    throw std::bad_alloc();
  return pointer;
}

void *operator new[](std::size_t size) {
  void *pointer = tracked_allocate(size);
  if (!pointer)
    throw std::bad_alloc();
  return pointer;
}

void operator delete(void *pointer) noexcept { tracked_deallocate(pointer); }
void operator delete[](void *pointer) noexcept { tracked_deallocate(pointer); }

void *operator new(std::size_t size, const std::nothrow_t &) noexcept {
  return tracked_allocate(size);
}

void *operator new[](std::size_t size, const std::nothrow_t &) noexcept {
  return tracked_allocate(size);
}

void operator delete(void *pointer, const std::nothrow_t &) noexcept {
  tracked_deallocate(pointer);
}

void operator delete[](void *pointer, const std::nothrow_t &) noexcept {
  tracked_deallocate(pointer);
}

struct ec_memory_fixture {
  virtual ~ec_memory_fixture() {}
  virtual std::uint64_t run(std::uint64_t repetitions,
                            bool materialize_outputs) = 0;
};

namespace {

struct LinePointCase {
  ::Line2d line;
  ::Point2d query;

  explicit LinePointCase(const std::int64_t *coordinates)
      : line(point2(coordinates), point2(coordinates + 2)),
        query(point2(coordinates + 4)) {}
};

class LinePointFixture : public ec_memory_fixture {
public:
  LinePointFixture(const std::int64_t *coordinates, std::size_t count) {
    cases_.reserve(count);
    for (std::size_t index = 0; index < count; ++index)
      cases_.emplace_back(coordinates + index * 6);
  }

  std::uint64_t run(std::uint64_t repetitions,
                    bool materialize_outputs) override {
    std::uint64_t checksum = 0;
    if (materialize_outputs) {
      std::vector<int> outputs;
      outputs.reserve(output_capacity(cases_.size(), repetitions));
      for (std::uint64_t repetition = 0; repetition < repetitions;
           ++repetition) {
        for (std::size_t index = 0; index < cases_.size(); ++index) {
          const int result = cases_[index].line.orientation(cases_[index].query);
          outputs.push_back(result);
          checksum += static_cast<std::uint64_t>(result + 2);
        }
      }
      observe(outputs);
    } else {
      for (std::uint64_t repetition = 0; repetition < repetitions;
           ++repetition) {
        for (std::size_t index = 0; index < cases_.size(); ++index) {
          const int result = cases_[index].line.orientation(cases_[index].query);
          observe(result);
          checksum += static_cast<std::uint64_t>(result + 2);
        }
      }
    }
    observe(checksum);
    return checksum;
  }

private:
  std::vector<LinePointCase> cases_;
};

struct TrianglePointCase {
  ::Triangle3d triangle;
  ::Point3d query;

  explicit TrianglePointCase(const std::int64_t *coordinates)
      : triangle(point3(coordinates), point3(coordinates + 3),
                 point3(coordinates + 6)),
        query(point3(coordinates + 9)) {}
};

class TrianglePointFixture : public ec_memory_fixture {
public:
  TrianglePointFixture(const std::int64_t *coordinates, std::size_t count) {
    cases_.reserve(count);
    for (std::size_t index = 0; index < count; ++index)
      cases_.emplace_back(coordinates + index * 12);
  }

  std::uint64_t run(std::uint64_t repetitions,
                    bool materialize_outputs) override {
    std::uint64_t checksum = 0;
    if (materialize_outputs) {
      std::vector<int> outputs;
      outputs.reserve(output_capacity(cases_.size(), repetitions));
      for (std::uint64_t repetition = 0; repetition < repetitions;
           ++repetition) {
        for (std::size_t index = 0; index < cases_.size(); ++index) {
          const int result = cases_[index].triangle.contains(cases_[index].query);
          outputs.push_back(result);
          checksum += static_cast<std::uint64_t>(result);
        }
      }
      observe(outputs);
    } else {
      for (std::uint64_t repetition = 0; repetition < repetitions;
           ++repetition) {
        for (std::size_t index = 0; index < cases_.size(); ++index) {
          const bool result =
              cases_[index].triangle.contains(cases_[index].query);
          observe(result);
          checksum += static_cast<std::uint64_t>(result);
        }
      }
    }
    observe(checksum);
    return checksum;
  }

private:
  std::vector<TrianglePointCase> cases_;
};

struct TrianglePairCase {
  ::Triangle3d left;
  ::Triangle3d right;

  explicit TrianglePairCase(const std::int64_t *coordinates)
      : left(point3(coordinates), point3(coordinates + 3),
             point3(coordinates + 6)),
        right(point3(coordinates + 9), point3(coordinates + 12),
              point3(coordinates + 15)) {}
};

class TrianglePairFixture : public ec_memory_fixture {
public:
  TrianglePairFixture(const std::int64_t *coordinates, std::size_t count) {
    cases_.reserve(count);
    for (std::size_t index = 0; index < count; ++index)
      cases_.emplace_back(coordinates + index * 18);
  }

  std::uint64_t run(std::uint64_t repetitions,
                    bool materialize_outputs) override {
    std::uint64_t checksum = 0;
    if (materialize_outputs) {
      std::vector<int> outputs;
      outputs.reserve(output_capacity(cases_.size(), repetitions));
      for (std::uint64_t repetition = 0; repetition < repetitions;
           ++repetition) {
        for (std::size_t index = 0; index < cases_.size(); ++index) {
          const int result =
              cases_[index].left.intersects(cases_[index].right);
          outputs.push_back(result);
          checksum += static_cast<std::uint64_t>(result + 2);
        }
      }
      observe(outputs);
    } else {
      for (std::uint64_t repetition = 0; repetition < repetitions;
           ++repetition) {
        for (std::size_t index = 0; index < cases_.size(); ++index) {
          const int result =
              cases_[index].left.intersects(cases_[index].right);
          observe(result);
          checksum += static_cast<std::uint64_t>(result + 2);
        }
      }
    }
    observe(checksum);
    return checksum;
  }

private:
  std::vector<TrianglePairCase> cases_;
};

} // namespace

extern "C" int ec_memory_tracking_init(void) {
  install_gmp_hooks();
  return gmp_hooks_installed.load(std::memory_order_relaxed) ? 0 : -1;
}

extern "C" void ec_memory_tracking_enable(int enabled) {
  tracking_enabled.store(enabled != 0, std::memory_order_seq_cst);
}

extern "C" void ec_memory_tracking_reset(void) {
  allocation_count.store(0, std::memory_order_relaxed);
  reallocation_count.store(0, std::memory_order_relaxed);
  deallocation_count.store(0, std::memory_order_relaxed);
  allocated_bytes.store(0, std::memory_order_relaxed);
  reallocated_bytes.store(0, std::memory_order_relaxed);
  deallocated_bytes.store(0, std::memory_order_relaxed);
  peak_live_bytes.store(live_bytes.load(std::memory_order_relaxed),
                        std::memory_order_relaxed);
}

extern "C" void ec_memory_tracking_snapshot(ec_memory_stats *output) {
  if (!output)
    return;
  output->allocation_count = allocation_count.load(std::memory_order_relaxed);
  output->reallocation_count =
      reallocation_count.load(std::memory_order_relaxed);
  output->deallocation_count =
      deallocation_count.load(std::memory_order_relaxed);
  output->allocated_bytes = allocated_bytes.load(std::memory_order_relaxed);
  output->reallocated_bytes =
      reallocated_bytes.load(std::memory_order_relaxed);
  output->deallocated_bytes =
      deallocated_bytes.load(std::memory_order_relaxed);
  output->live_bytes = live_bytes.load(std::memory_order_relaxed);
  output->peak_live_bytes = peak_live_bytes.load(std::memory_order_relaxed);
}

extern "C" void ec_memory_release_caches(void) { mpfr_free_cache(); }

extern "C" ec_memory_fixture *
ec_memory_fixture_new(int workload, const std::int64_t *coordinates,
                      std::size_t case_count) {
  if (!coordinates && case_count != 0)
    return 0;
  try {
    switch (workload) {
    case EC_MEMORY_LINE_POINT:
      return new LinePointFixture(coordinates, case_count);
    case EC_MEMORY_TRIANGLE_POINT:
      return new TrianglePointFixture(coordinates, case_count);
    case EC_MEMORY_TRIANGLE_PAIR:
      return new TrianglePairFixture(coordinates, case_count);
    default:
      return 0;
    }
  } catch (...) {
    return 0;
  }
}

extern "C" std::uint64_t
ec_memory_fixture_run(ec_memory_fixture *fixture, std::uint64_t repetitions,
                      int materialize_outputs) {
  if (!fixture)
    return 0;
  return fixture->run(repetitions, materialize_outputs != 0);
}

extern "C" void ec_memory_fixture_free(ec_memory_fixture *fixture) {
  delete fixture;
}
