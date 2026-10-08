#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <new>
#include <string>
#include <vector>

#define CORE_LEVEL 3
#include <CORE/CORE.h>
#include <CORE/ComplexT.h>
#include <CORE/geometry2d.h>
#include <CORE/geometry3d.h>
#include <CORE/linearAlgebraT.h>
#include <CORE/poly/Curves.h>
#include <CORE/poly/Poly.h>
#include <CORE/poly/Sturm.h>

#undef double
#undef long

#include "empty_circle_complex.h"
#include "exactcore_bench.h"

/*
 * This file is deliberately benchmark-specific. The correctness oracle keeps
 * its one-shot C ABI, while these fixtures retain native inputs and execute an
 * operation many times behind one FFI call. That separates adapter overhead
 * from exactCore's native operation cost without exposing legacy C++ types to
 * Rust.
 */

struct ec_benchmark_fixture {
  virtual ~ec_benchmark_fixture() {}
  virtual void run_many(std::uint64_t iterations) = 0;
};

namespace {

#if defined(__GNUC__) || defined(__clang__)
inline void benchmark_memory_barrier() {
  __asm__ __volatile__("" : : : "memory");
}

template <typename T> inline void benchmark_observe(const T &value) {
  __asm__ __volatile__("" : : "g"(&value) : "memory");
}
#else
inline void benchmark_memory_barrier() {
  std::atomic_signal_fence(std::memory_order_seq_cst);
}

template <typename T> inline void benchmark_observe(const T &value) {
  volatile const T *observed = &value;
  (void)observed;
  std::atomic_signal_fence(std::memory_order_seq_cst);
}
#endif

template <typename Operation>
class CallableFixture : public ec_benchmark_fixture {
public:
  explicit CallableFixture(const Operation &operation)
      : operation_(operation) {}

  void run_many(std::uint64_t iterations) override {
    for (std::uint64_t index = 0; index < iterations; ++index) {
      benchmark_memory_barrier();
      operation_();
      benchmark_memory_barrier();
    }
  }

private:
  Operation operation_;
};

template <typename Operation>
ec_benchmark_fixture *fixture(const Operation &operation) {
  return new CallableFixture<Operation>(operation);
}

bool is(const std::string &id, const char *expected) { return id == expected; }

bool begins_with(const std::string &id, const char *prefix) {
  const std::size_t length = std::strlen(prefix);
  return id.size() >= length && id.compare(0, length, prefix) == 0;
}

int indexed_suffix(const std::string &id, const char *prefix, int maximum) {
  if (!begins_with(id, prefix))
    return -1;
  const std::string suffix = id.substr(std::strlen(prefix));
  if (suffix.size() != 1 || suffix[0] < '0' || suffix[0] > '9')
    return -1;
  const int value = suffix[0] - '0';
  return value <= maximum ? value : -1;
}

CORE::BigInt integer(std::int64_t value) {
  return CORE::BigInt(static_cast<long>(value));
}

CORE::BigRat rational(std::int64_t numerator, std::int64_t denominator) {
  CORE::BigRat value(integer(numerator), integer(denominator));
  value.canonicalize();
  return value;
}

CORE::Expr expression(std::int64_t numerator, std::int64_t denominator = 1) {
  return CORE::Expr(rational(numerator, denominator));
}

::Point2d point2(std::int64_t x, std::int64_t y) {
  return ::Point2d(expression(x), expression(y));
}

::Point3d point3(std::int64_t x, std::int64_t y, std::int64_t z) {
  return ::Point3d(expression(x), expression(y), expression(z));
}

template <typename T>
CORE::Polynomial<T> polynomial(const std::vector<std::int64_t> &coefficients) {
  if (coefficients.empty())
    return CORE::Polynomial<T>();
  std::vector<T> converted(coefficients.size());
  for (std::size_t index = 0; index < coefficients.size(); ++index) {
    converted[index] = T(integer(coefficients[index]));
  }
  return CORE::Polynomial<T>(static_cast<int>(converted.size() - 1),
                             &converted[0]);
}

CORE::BiPoly<CORE::BigInt>
bivariate(const std::vector<std::int64_t> &coefficients, std::size_t x_count,
          std::size_t y_count) {
  std::vector<CORE::Polynomial<CORE::BigInt>> rows;
  rows.reserve(y_count);
  for (std::size_t y = 0; y < y_count; ++y) {
    std::vector<CORE::BigInt> row(x_count);
    for (std::size_t x = 0; x < x_count; ++x) {
      row[x] = integer(coefficients[x * y_count + y]);
    }
    rows.push_back(
        CORE::Polynomial<CORE::BigInt>(static_cast<int>(x_count - 1), &row[0]));
  }
  return CORE::BiPoly<CORE::BigInt>(rows);
}

template <typename T>
VectorT<T> vector(const std::vector<std::int64_t> &values) {
  std::vector<T> converted(values.size());
  for (std::size_t index = 0; index < values.size(); ++index) {
    converted[index] = T(integer(values[index]));
  }
  return VectorT<T>(static_cast<int>(values.size()), &converted[0]);
}

template <typename T>
MatrixT<T> matrix(const std::vector<std::int64_t> &values, int dimension) {
  std::vector<T> converted(values.size());
  for (std::size_t index = 0; index < values.size(); ++index) {
    converted[index] = T(integer(values[index]));
  }
  return MatrixT<T>(dimension, dimension, &converted[0]);
}

struct RationalState {
  CORE::BigRat left;
  CORE::BigRat right;

  RationalState()
      : left(rational(123456789, 1000003)),
        right(rational(-987654321, 1000033)) {}
};

ec_benchmark_fixture *rational_fixture(const std::string &id) {
  if (!begins_with(id, "rational."))
    return 0;
  const std::shared_ptr<RationalState> state(new RationalState());
  if (is(id, "rational.add")) {
    return fixture([state]() {
      const CORE::BigRat result = state->left + state->right;
      benchmark_observe(result);
    });
  }
  if (is(id, "rational.subtract")) {
    return fixture([state]() {
      const CORE::BigRat result = state->left - state->right;
      benchmark_observe(result);
    });
  }
  if (is(id, "rational.multiply")) {
    return fixture([state]() {
      const CORE::BigRat result = state->left * state->right;
      benchmark_observe(result);
    });
  }
  if (is(id, "rational.divide")) {
    return fixture([state]() {
      const CORE::BigRat result = state->left / state->right;
      benchmark_observe(result);
    });
  }
  if (is(id, "rational.negate")) {
    return fixture([state]() {
      const CORE::BigRat result = -state->left;
      benchmark_observe(result);
    });
  }
  if (is(id, "rational.absolute")) {
    return fixture([state]() {
      CORE::BigRat result;
      result.abs(state->left);
      benchmark_observe(result);
    });
  }
  if (is(id, "rational.reciprocal")) {
    return fixture([state]() {
      const CORE::BigRat result = state->left.reciprocal();
      benchmark_observe(result);
    });
  }
  if (is(id, "rational.compare")) {
    return fixture([state]() {
      const int result = state->left < state->right
                             ? -1
                             : (state->left > state->right ? 1 : 0);
      benchmark_observe(result);
    });
  }
  if (is(id, "rational.parts")) {
    return fixture([state]() {
      const CORE::BigInt numerator = state->left.numerator();
      const CORE::BigInt denominator = state->left.denominator();
      benchmark_observe(numerator);
      benchmark_observe(denominator);
    });
  }
  return 0;
}

struct ExprState {
  CORE::Expr unary;
  CORE::Expr left;
  CORE::Expr right;

  ExprState()
      : unary(expression(1, 3)), left(expression(7, 3)),
        right(expression(5, 11)) {}
};

ec_benchmark_fixture *expr_fixture(const std::string &id) {
  if (!begins_with(id, "real."))
    return 0;
  const std::shared_ptr<ExprState> state(new ExprState());

  if (is(id, "real.negate"))
    return fixture([state]() {
      const CORE::Expr result = -state->unary;
      benchmark_observe(result);
    });
  if (is(id, "real.absolute"))
    return fixture([state]() {
      const CORE::Expr result = CORE::abs(state->unary);
      benchmark_observe(result);
    });
  if (is(id, "real.sqrt"))
    return fixture([state]() {
      const CORE::Expr result = sqrt(state->unary);
      benchmark_observe(result);
    });
  if (is(id, "real.cbrt"))
    return fixture([state]() {
      const CORE::Expr result = cbrt(state->unary);
      benchmark_observe(result);
    });
  if (is(id, "real.root_n"))
    return fixture([state]() {
      const CORE::Expr result = root(state->unary, 5);
      benchmark_observe(result);
    });
  if (is(id, "real.exp"))
    return fixture([state]() {
      const CORE::Expr result = exp(state->unary);
      benchmark_observe(result);
    });
  if (is(id, "real.ln"))
    return fixture([state]() {
      const CORE::Expr result = log(state->unary);
      benchmark_observe(result);
    });
  if (is(id, "real.log2"))
    return fixture([state]() {
      const CORE::Expr result = log2(state->unary);
      benchmark_observe(result);
    });
  if (is(id, "real.log10"))
    return fixture([state]() {
      const CORE::Expr result = log10(state->unary);
      benchmark_observe(result);
    });
  if (is(id, "real.sin"))
    return fixture([state]() {
      const CORE::Expr result = sin(state->unary);
      benchmark_observe(result);
    });
  if (is(id, "real.cos"))
    return fixture([state]() {
      const CORE::Expr result = cos(state->unary);
      benchmark_observe(result);
    });
  if (is(id, "real.tan"))
    return fixture([state]() {
      const CORE::Expr result = tan(state->unary);
      benchmark_observe(result);
    });
  if (is(id, "real.asin"))
    return fixture([state]() {
      const CORE::Expr result = asin(state->unary);
      benchmark_observe(result);
    });
  if (is(id, "real.acos"))
    return fixture([state]() {
      const CORE::Expr result = acos(state->unary);
      benchmark_observe(result);
    });
  if (is(id, "real.atan"))
    return fixture([state]() {
      const CORE::Expr result = atan(state->unary);
      benchmark_observe(result);
    });
  if (is(id, "real.square"))
    return fixture([state]() {
      const CORE::Expr result = state->unary * state->unary;
      benchmark_observe(result);
    });
  if (is(id, "real.exp2"))
    return fixture([state]() {
      const CORE::Expr result = exp2(state->unary);
      benchmark_observe(result);
    });
  if (is(id, "real.exp10"))
    return fixture([state]() {
      const CORE::Expr result = exp10(state->unary);
      benchmark_observe(result);
    });
  if (is(id, "real.cot"))
    return fixture([state]() {
      const CORE::Expr result = cot(state->unary);
      benchmark_observe(result);
    });
  if (is(id, "real.add"))
    return fixture([state]() {
      const CORE::Expr result = state->left + state->right;
      benchmark_observe(result);
    });
  if (is(id, "real.subtract"))
    return fixture([state]() {
      const CORE::Expr result = state->left - state->right;
      benchmark_observe(result);
    });
  if (is(id, "real.multiply"))
    return fixture([state]() {
      const CORE::Expr result = state->left * state->right;
      benchmark_observe(result);
    });
  if (is(id, "real.divide"))
    return fixture([state]() {
      const CORE::Expr result = state->left / state->right;
      benchmark_observe(result);
    });
  if (is(id, "real.powi"))
    return fixture([state]() {
      const CORE::Expr result = pow(state->left, 7L);
      benchmark_observe(result);
    });
  if (is(id, "real.pi"))
    return fixture([]() {
      const CORE::Expr result = CORE::pi();
      benchmark_observe(result);
    });
  if (is(id, "real.e"))
    return fixture([]() {
      const CORE::Expr result = CORE::e();
      benchmark_observe(result);
    });
  if (is(id, "real.sign"))
    return fixture([state]() {
      const int result = CORE::sign(sin(state->unary));
      benchmark_observe(result);
    });
  if (is(id, "real.floor"))
    return fixture([state]() {
      const CORE::BigInt result = CORE::floor(state->left);
      benchmark_observe(result);
    });
  if (is(id, "real.ceil"))
    return fixture([state]() {
      const CORE::BigInt result = CORE::ceil(state->left);
      benchmark_observe(result);
    });
  return 0;
}

struct ComplexState {
  ComplexT<CORE::BigRat> left;
  ComplexT<CORE::BigRat> right;

  ComplexState()
      : left(rational(7, 13), rational(-11, 17)),
        right(rational(19, 23), rational(29, 31)) {}
};

ec_benchmark_fixture *complex_fixture(const std::string &id) {
  if (!begins_with(id, "complex."))
    return 0;
  const std::shared_ptr<ComplexState> state(new ComplexState());
  if (is(id, "complex.add"))
    return fixture([state]() {
      const ComplexT<CORE::BigRat> result = state->left + state->right;
      benchmark_observe(result);
    });
  if (is(id, "complex.subtract"))
    return fixture([state]() {
      const ComplexT<CORE::BigRat> result = state->left - state->right;
      benchmark_observe(result);
    });
  if (is(id, "complex.multiply"))
    return fixture([state]() {
      const ComplexT<CORE::BigRat> result = state->left * state->right;
      benchmark_observe(result);
    });
  if (is(id, "complex.divide"))
    return fixture([state]() {
      const ComplexT<CORE::BigRat> result = state->left / state->right;
      benchmark_observe(result);
    });
  if (is(id, "complex.norm_squared"))
    return fixture([state]() {
      const CORE::BigRat result = state->left.re() * state->left.re() +
                                  state->left.im() * state->left.im();
      benchmark_observe(result);
    });
  return 0;
}

struct VectorState {
  VectorT<CORE::BigRat> left;
  VectorT<CORE::BigRat> right;

  VectorState(const std::vector<std::int64_t> &left_values,
              const std::vector<std::int64_t> &right_values)
      : left(vector<CORE::BigRat>(left_values)),
        right(vector<CORE::BigRat>(right_values)) {}
};

ec_benchmark_fixture *vector_fixture(const std::string &id) {
  int dimension = 0;
  if (begins_with(id, "vector2."))
    dimension = 2;
  if (begins_with(id, "vector3."))
    dimension = 3;
  if (begins_with(id, "vector4."))
    dimension = 4;
  if (dimension == 0)
    return 0;

  std::vector<std::int64_t> left;
  std::vector<std::int64_t> right;
  if (dimension == 2) {
    left = std::vector<std::int64_t>{3, -4};
    right = std::vector<std::int64_t>{7, 11};
  } else if (dimension == 3) {
    left = std::vector<std::int64_t>{3, -4, 5};
    right = std::vector<std::int64_t>{7, 11, -13};
  } else {
    left = std::vector<std::int64_t>{3, -4, 5, -6};
    right = std::vector<std::int64_t>{7, 11, -13, 17};
  }
  const std::shared_ptr<VectorState> state(new VectorState(left, right));
  const std::string suffix = id.substr(8);
  if (suffix == "add")
    return fixture([state]() {
      const VectorT<CORE::BigRat> result = state->left + state->right;
      benchmark_observe(result);
    });
  if (suffix == "subtract")
    return fixture([state]() {
      const VectorT<CORE::BigRat> result = state->left - state->right;
      benchmark_observe(result);
    });
  if (suffix == "dot")
    return fixture([state]() {
      CORE::BigRat result(0);
      for (int index = 0; index < state->left.dimension(); ++index) {
        result += state->left[index] * state->right[index];
      }
      benchmark_observe(result);
    });
  if (suffix == "cross" && dimension == 3)
    return fixture([state]() {
      const VectorT<CORE::BigRat> result = state->left.cross(state->right);
      benchmark_observe(result);
    });
  if (suffix == "wedge" && dimension == 2)
    return fixture([state]() {
      const CORE::BigRat result =
          state->left[0] * state->right[1] - state->left[1] * state->right[0];
      benchmark_observe(result);
    });
  if (suffix == "norm") {
    const std::shared_ptr<std::vector<CORE::Expr>> expr_values(
        new std::vector<CORE::Expr>());
    for (std::size_t index = 0; index < left.size(); ++index) {
      expr_values->push_back(expression(left[index]));
    }
    return fixture([expr_values]() {
      CORE::Expr squared(0);
      for (std::size_t index = 0; index < expr_values->size(); ++index) {
        squared += (*expr_values)[index] * (*expr_values)[index];
      }
      const CORE::Expr result = sqrt(squared);
      benchmark_observe(result);
    });
  }
  return 0;
}

struct MatrixState {
  MatrixT<CORE::BigRat> left;
  MatrixT<CORE::BigRat> right;
  int dimension;

  MatrixState(const std::vector<std::int64_t> &left_values,
              const std::vector<std::int64_t> &right_values,
              int matrix_dimension)
      : left(matrix<CORE::BigRat>(left_values, matrix_dimension)),
        right(matrix<CORE::BigRat>(right_values, matrix_dimension)),
        dimension(matrix_dimension) {}
};

ec_benchmark_fixture *matrix_fixture(const std::string &id) {
  int dimension = 0;
  if (begins_with(id, "matrix3."))
    dimension = 3;
  if (begins_with(id, "matrix4."))
    dimension = 4;
  if (dimension == 0)
    return 0;

  const std::vector<std::int64_t> left =
      dimension == 3 ? std::vector<std::int64_t>{2, -1, 3, 4, 0, 5, -2, 7, 1}
                     : std::vector<std::int64_t>{2, 1, 0, 3, -1, 4,  2, 0,
                                                 5, 0, 3, 1, 2,  -2, 1, 6};
  const std::vector<std::int64_t> right =
      dimension == 3 ? std::vector<std::int64_t>{1, 2, 0, -3, 4, 1, 5, -2, 6}
                     : std::vector<std::int64_t>{1,  0, 2, -1, 3, 1, 0, 4,
                                                 -2, 5, 1, 0,  0, 2, 3, 1};
  const std::shared_ptr<MatrixState> state(
      new MatrixState(left, right, dimension));
  const std::string suffix = id.substr(8);
  if (suffix == "add")
    return fixture([state]() {
      const MatrixT<CORE::BigRat> result = state->left + state->right;
      benchmark_observe(result);
    });
  if (suffix == "subtract")
    return fixture([state]() {
      const MatrixT<CORE::BigRat> result = state->left - state->right;
      benchmark_observe(result);
    });
  if (suffix == "multiply")
    return fixture([state]() {
      const MatrixT<CORE::BigRat> result = state->left * state->right;
      benchmark_observe(result);
    });
  if (suffix == "transpose")
    return fixture([state]() {
      const MatrixT<CORE::BigRat> result = transpose(state->left);
      benchmark_observe(result);
    });
  if (suffix == "determinant")
    return fixture([state]() {
      const CORE::BigRat result = state->left.determinant();
      benchmark_observe(result);
    });
  if (suffix == "inverse")
    return fixture([state]() {
      MatrixT<CORE::BigRat> adjugate(state->dimension);
      const CORE::BigRat determinant = state->left.bareissInverse(&adjugate);
      benchmark_observe(determinant);
      benchmark_observe(adjugate);
    });
  return 0;
}

struct ThreePoints2 {
  ::Point2d a;
  ::Point2d b;
  ::Point2d c;

  explicit ThreePoints2(const std::int64_t *coordinates)
      : a(point2(coordinates[0], coordinates[1])),
        b(point2(coordinates[2], coordinates[3])),
        c(point2(coordinates[4], coordinates[5])) {}
};

ec_benchmark_fixture *orientation2_fixture(const std::int64_t *coordinates) {
  const std::shared_ptr<ThreePoints2> state(new ThreePoints2(coordinates));
  return fixture([state]() {
    const int result = ::orientation2d(state->a, state->b, state->c);
    benchmark_observe(result);
  });
}

ec_benchmark_fixture *area2_fixture(const std::int64_t *coordinates) {
  const std::shared_ptr<ThreePoints2> state(new ThreePoints2(coordinates));
  return fixture([state]() {
    const CORE::Expr result = ::area(state->a, state->b, state->c);
    benchmark_observe(result);
  });
}

ec_benchmark_fixture *between2_fixture(const std::int64_t *coordinates) {
  const std::shared_ptr<ThreePoints2> state(new ThreePoints2(coordinates));
  return fixture([state]() {
    const bool result = ::between(state->a, state->b, state->c);
    benchmark_observe(result);
  });
}

struct Lines2 {
  ::Point2d a;
  ::Point2d b;
  ::Point2d c;
  ::Point2d d;
  ::Line2d first;
  ::Line2d second;

  explicit Lines2(const std::int64_t *coordinates)
      : a(point2(coordinates[0], coordinates[1])),
        b(point2(coordinates[2], coordinates[3])),
        c(point2(coordinates[4], coordinates[5])),
        d(point2(coordinates[6], coordinates[7])), first(a, b), second(c, d) {}
};

ec_benchmark_fixture *line2_relation_fixture(int operation,
                                             const std::int64_t *coordinates) {
  const std::shared_ptr<Lines2> state(new Lines2(coordinates));
  switch (operation) {
  case 0:
    return fixture([state]() {
      const int result = state->first.orientation(state->c);
      benchmark_observe(result);
    });
  case 1:
    return fixture([state]() {
      const bool result = state->first.contains(state->c);
      benchmark_observe(result);
    });
  case 2:
    return fixture([state]() {
      const bool result = state->first.isParallel(state->second);
      benchmark_observe(result);
    });
  case 3:
    return fixture([state]() {
      const int result = state->first.intersects(state->second);
      benchmark_observe(result);
    });
  case 4:
    return fixture([state]() {
      const bool result = state->first.isCoincident(state->second);
      benchmark_observe(result);
    });
  case 5:
    return fixture([state]() {
      const bool result = state->first.isVertical();
      benchmark_observe(result);
    });
  case 6:
    return fixture([state]() {
      const bool result = state->first.isHorizontal();
      benchmark_observe(result);
    });
  default:
    return 0;
  }
}

ec_benchmark_fixture *
line2_intersection_fixture(const std::int64_t *coordinates) {
  const std::shared_ptr<Lines2> state(new Lines2(coordinates));
  return fixture([state]() {
    ::GeomObj *object = state->first.intersection(state->second);
    if (object != 0) {
      ::Point2d *point = dynamic_cast<::Point2d *>(object);
      if (point != 0) {
        const CORE::Expr x = point->X();
        const CORE::Expr y = point->Y();
        benchmark_observe(x);
        benchmark_observe(y);
      }
      delete object;
    }
    benchmark_observe(object);
  });
}

struct Segments2 {
  ::Point2d a;
  ::Point2d b;
  ::Point2d c;
  ::Point2d d;
  ::Segment2d first;
  ::Segment2d second;

  explicit Segments2(const std::int64_t *coordinates)
      : a(point2(coordinates[0], coordinates[1])),
        b(point2(coordinates[2], coordinates[3])),
        c(point2(coordinates[4], coordinates[5])),
        d(point2(coordinates[6], coordinates[7])), first(a, b), second(c, d) {}
};

ec_benchmark_fixture *
segment2_relation_fixture(int operation, const std::int64_t *coordinates) {
  const std::shared_ptr<Segments2> state(new Segments2(coordinates));
  switch (operation) {
  case 0:
    return fixture([state]() {
      const bool result = state->first.contains(state->c);
      benchmark_observe(result);
    });
  case 1:
    return fixture([state]() {
      const int result = state->first.intersects(state->second);
      benchmark_observe(result);
    });
  case 2:
    return fixture([state]() {
      const bool result = state->first.isCoincident(state->second);
      benchmark_observe(result);
    });
  case 3:
    return fixture([state]() {
      const bool result = state->first.isParallel(state->second);
      benchmark_observe(result);
    });
  default:
    return 0;
  }
}

struct Incircle2State {
  ::Point2d a;
  ::Point2d b;
  ::Point2d c;
  ::Point2d query;
  ::Circle2d circle;

  explicit Incircle2State(const std::int64_t *coordinates)
      : a(point2(coordinates[0], coordinates[1])),
        b(point2(coordinates[2], coordinates[3])),
        c(point2(coordinates[4], coordinates[5])),
        query(point2(coordinates[6], coordinates[7])), circle(a, b, c) {}
};

ec_benchmark_fixture *incircle2_fixture(const std::int64_t *coordinates) {
  const std::shared_ptr<Incircle2State> state(new Incircle2State(coordinates));
  return fixture([state]() {
    const int result = state->circle.side_of(state->query);
    benchmark_observe(result);
  });
}

struct CircleLine2State {
  ::Point2d center;
  ::Point2d a;
  ::Point2d b;
  ::Circle2d circle;
  ::Line2d line;

  CircleLine2State(std::int64_t radius, const std::int64_t *coordinates)
      : center(point2(coordinates[0], coordinates[1])),
        a(point2(coordinates[2], coordinates[3])),
        b(point2(coordinates[4], coordinates[5])),
        circle(center, expression(radius)), line(a, b) {}
};

ec_benchmark_fixture *circle_line2_fixture(std::int64_t radius,
                                           const std::int64_t *coordinates) {
  const std::shared_ptr<CircleLine2State> state(
      new CircleLine2State(radius, coordinates));
  return fixture([state]() {
    const CORE::Expr distance = state->circle.distance(state->line);
    const int result = CORE::sign(distance);
    benchmark_observe(result);
  });
}

struct CircleSegment2State {
  ::Point2d center;
  ::Point2d a;
  ::Point2d b;
  ::Segment2d segment;
  CORE::Expr radius;

  CircleSegment2State(std::int64_t radius_value,
                      const std::int64_t *coordinates)
      : center(point2(coordinates[0], coordinates[1])),
        a(point2(coordinates[2], coordinates[3])),
        b(point2(coordinates[4], coordinates[5])), segment(a, b),
        radius(expression(radius_value)) {}
};

ec_benchmark_fixture *circle_segment2_fixture(std::int64_t radius,
                                              const std::int64_t *coordinates) {
  const std::shared_ptr<CircleSegment2State> state(
      new CircleSegment2State(radius, coordinates));
  return fixture([state]() {
    const CORE::Expr distance =
        state->segment.distance(state->center) - state->radius;
    const int result = CORE::sign(distance);
    benchmark_observe(result);
  });
}

struct CircleDistance2State {
  ::Point2d first_center;
  ::Point2d second_center;
  ::Circle2d first;
  ::Circle2d second;

  CircleDistance2State(std::int64_t first_radius, std::int64_t second_radius,
                       const std::int64_t *coordinates)
      : first_center(point2(coordinates[0], coordinates[1])),
        second_center(point2(coordinates[2], coordinates[3])),
        first(first_center, expression(first_radius)),
        second(second_center, expression(second_radius)) {}
};

ec_benchmark_fixture *
circle_distance2_fixture(int operation, std::int64_t first_radius,
                         std::int64_t second_radius,
                         const std::int64_t *coordinates) {
  const std::shared_ptr<CircleDistance2State> state(
      new CircleDistance2State(first_radius, second_radius, coordinates));
  if (operation == 0)
    return fixture([state]() {
      const CORE::Expr result = state->first.distance(state->second_center);
      benchmark_observe(result);
    });
  if (operation == 1)
    return fixture([state]() {
      const CORE::Expr result = state->first.distance(state->second);
      benchmark_observe(result);
    });
  return 0;
}

ec_benchmark_fixture *point2_distance_fixture(const std::int64_t *coordinates) {
  const std::shared_ptr<::Point2d> first(
      new ::Point2d(point2(coordinates[0], coordinates[1])));
  const std::shared_ptr<::Point2d> second(
      new ::Point2d(point2(coordinates[2], coordinates[3])));
  return fixture([first, second]() {
    const CORE::Expr result = first->distance(*second);
    benchmark_observe(result);
  });
}

ec_benchmark_fixture *
line2_point_distance_fixture(const std::int64_t *coordinates) {
  const std::shared_ptr<Lines2> state(new Lines2(coordinates));
  return fixture([state]() {
    const CORE::Expr result = state->first.distance(state->c);
    benchmark_observe(result);
  });
}

ec_benchmark_fixture *
segment2_point_distance_fixture(const std::int64_t *coordinates) {
  const std::shared_ptr<Segments2> state(new Segments2(coordinates));
  return fixture([state]() {
    const CORE::Expr result = state->first.distance(state->c);
    benchmark_observe(result);
  });
}

ec_benchmark_fixture *geometry2_fixture(const std::string &id) {
  static const std::int64_t triangle[] = {0, 0, 13, 2, 3, 17};
  static const std::int64_t between[] = {0, 0, 13, 2, 6, 1};
  static const std::int64_t lines[] = {0, 0, 13, 2, 3, 17, 19, -7};
  static const std::int64_t incircle[] = {0, 0, 13, 2, 3, 17, 4, 4};
  static const std::int64_t circle_line[] = {0, 0, -10, 7, 10, 7};
  static const std::int64_t circle_point[] = {0, 0, 13, 7};
  static const std::int64_t circle_circle[] = {0, 0, 17, 4};
  static const std::int64_t points[] = {0, 0, 3, 4};
  static const std::int64_t point_distance[] = {0, 0, 13, 2, 3, 17, 0, 0};

  if (is(id, "geometry2.orientation"))
    return orientation2_fixture(triangle);
  if (is(id, "geometry2.area"))
    return area2_fixture(triangle);
  if (is(id, "geometry2.between"))
    return between2_fixture(between);
  const int line_operation = indexed_suffix(id, "geometry2.line_relation_", 6);
  if (line_operation >= 0)
    return line2_relation_fixture(line_operation, lines);
  if (is(id, "geometry2.line_intersection"))
    return line2_intersection_fixture(lines);
  const int segment_operation =
      indexed_suffix(id, "geometry2.segment_relation_", 3);
  if (segment_operation >= 0)
    return segment2_relation_fixture(segment_operation, lines);
  if (is(id, "geometry2.incircle"))
    return incircle2_fixture(incircle);
  if (is(id, "geometry2.circle_line"))
    return circle_line2_fixture(5, circle_line);
  if (is(id, "geometry2.circle_segment"))
    return circle_segment2_fixture(5, circle_line);
  if (is(id, "geometry2.circle_point_distance")) {
    return circle_distance2_fixture(0, 5, 0, circle_point);
  }
  if (is(id, "geometry2.circle_circle_distance")) {
    return circle_distance2_fixture(1, 5, 7, circle_circle);
  }
  if (is(id, "geometry2.point_distance"))
    return point2_distance_fixture(points);
  if (is(id, "geometry2.line_point_distance")) {
    return line2_point_distance_fixture(point_distance);
  }
  if (is(id, "geometry2.segment_point_distance")) {
    return segment2_point_distance_fixture(point_distance);
  }
  return 0;
}

struct FourPoints3 {
  ::Point3d a;
  ::Point3d b;
  ::Point3d c;
  ::Point3d d;

  explicit FourPoints3(const std::int64_t *coordinates)
      : a(point3(coordinates[0], coordinates[1], coordinates[2])),
        b(point3(coordinates[3], coordinates[4], coordinates[5])),
        c(point3(coordinates[6], coordinates[7], coordinates[8])),
        d(point3(coordinates[9], coordinates[10], coordinates[11])) {}
};

ec_benchmark_fixture *orientation3_fixture(const std::int64_t *coordinates) {
  const std::shared_ptr<FourPoints3> state(new FourPoints3(coordinates));
  return fixture([state]() {
    const int result = ::orientation3d(state->a, state->b, state->c, state->d);
    benchmark_observe(result);
  });
}

ec_benchmark_fixture *volume3_fixture(const std::int64_t *coordinates) {
  const std::shared_ptr<FourPoints3> state(new FourPoints3(coordinates));
  return fixture([state]() {
    const CORE::Expr result =
        CORE::Expr(::orientation3d(state->a, state->b, state->c, state->d)) *
        ::volume(state->a, state->b, state->c, state->d);
    benchmark_observe(result);
  });
}

struct Lines3 {
  ::Point3d a;
  ::Point3d b;
  ::Point3d c;
  ::Point3d d;
  ::Line3d first;
  ::Line3d second;

  explicit Lines3(const std::int64_t *coordinates)
      : a(point3(coordinates[0], coordinates[1], coordinates[2])),
        b(point3(coordinates[3], coordinates[4], coordinates[5])),
        c(point3(coordinates[6], coordinates[7], coordinates[8])),
        d(point3(coordinates[9], coordinates[10], coordinates[11])),
        first(a, b), second(c, d) {}
};

ec_benchmark_fixture *line3_relation_fixture(int operation,
                                             const std::int64_t *coordinates) {
  const std::shared_ptr<Lines3> state(new Lines3(coordinates));
  switch (operation) {
  case 0:
    return fixture([state]() {
      const bool result = state->first.contains(state->c);
      benchmark_observe(result);
    });
  case 1:
    return fixture([state]() {
      const bool result = state->first.isParallel(state->second);
      benchmark_observe(result);
    });
  case 2:
    return fixture([state]() {
      const bool result = state->first.isSkew(state->second);
      benchmark_observe(result);
    });
  case 3:
    return fixture([state]() {
      const int result = state->first.intersects(state->second);
      benchmark_observe(result);
    });
  case 4:
    return fixture([state]() {
      const bool result = state->first.isCoincident(state->second);
      benchmark_observe(result);
    });
  default:
    return 0;
  }
}

struct Segments3 {
  ::Point3d a;
  ::Point3d b;
  ::Point3d c;
  ::Point3d d;
  ::Segment3d first;
  ::Segment3d second;

  explicit Segments3(const std::int64_t *coordinates)
      : a(point3(coordinates[0], coordinates[1], coordinates[2])),
        b(point3(coordinates[3], coordinates[4], coordinates[5])),
        c(point3(coordinates[6], coordinates[7], coordinates[8])),
        d(point3(coordinates[9], coordinates[10], coordinates[11])),
        first(a, b), second(c, d) {}
};

ec_benchmark_fixture *
segment3_relation_fixture(int operation, const std::int64_t *coordinates) {
  const std::shared_ptr<Segments3> state(new Segments3(coordinates));
  switch (operation) {
  case 0:
    return fixture([state]() {
      const bool result = state->first.contains(state->c);
      benchmark_observe(result);
    });
  case 1:
    return fixture([state]() {
      const int result = state->first.intersects(state->second);
      benchmark_observe(result);
    });
  case 2:
    return fixture([state]() {
      const bool result = state->first.isCoincident(state->second);
      benchmark_observe(result);
    });
  case 3:
    return fixture([state]() {
      const bool result = state->first.isCoplanar(state->second);
      benchmark_observe(result);
    });
  default:
    return 0;
  }
}

struct Plane3State {
  ::Point3d a;
  ::Point3d b;
  ::Point3d c;
  ::Point3d d;
  ::Point3d e;
  ::Point3d f;
  ::Plane3d first;
  ::Line3d line;
  ::Segment3d segment;
  ::Plane3d second;

  explicit Plane3State(const std::int64_t *coordinates)
      : a(point3(coordinates[0], coordinates[1], coordinates[2])),
        b(point3(coordinates[3], coordinates[4], coordinates[5])),
        c(point3(coordinates[6], coordinates[7], coordinates[8])),
        d(point3(coordinates[9], coordinates[10], coordinates[11])),
        e(point3(coordinates[12], coordinates[13], coordinates[14])),
        f(point3(coordinates[15], coordinates[16], coordinates[17])),
        first(a, b, c), line(d, e), segment(d, e), second(d, e, f) {}
};

ec_benchmark_fixture *plane3_relation_fixture(int operation,
                                              const std::int64_t *coordinates) {
  const std::shared_ptr<Plane3State> state(new Plane3State(coordinates));
  switch (operation) {
  case 0:
    return fixture([state]() {
      const bool result = state->first.contains(state->d);
      benchmark_observe(result);
    });
  case 1:
    return fixture([state]() {
      const CORE::Expr value = state->first.apply(state->d);
      const int result = CORE::sign(value);
      benchmark_observe(result);
    });
  case 2:
    return fixture([state]() {
      const bool result = state->first.contains(state->line);
      benchmark_observe(result);
    });
  case 3:
    return fixture([state]() {
      const int result = state->first.intersects(state->line);
      benchmark_observe(result);
    });
  case 4:
    return fixture([state]() {
      const bool result = state->first.contains(state->segment);
      benchmark_observe(result);
    });
  case 5:
    return fixture([state]() {
      const int result = state->first.intersects(state->segment);
      benchmark_observe(result);
    });
  case 6:
    return fixture([state]() {
      const bool result = state->first.isParallel(state->second);
      benchmark_observe(result);
    });
  case 7:
    return fixture([state]() {
      const int result = state->first.intersects(state->second);
      benchmark_observe(result);
    });
  default:
    return 0;
  }
}

struct Triangle3State {
  ::Point3d a;
  ::Point3d b;
  ::Point3d c;
  ::Point3d d;
  ::Point3d e;
  ::Point3d f;
  ::Triangle3d first;
  ::Segment3d segment;
  ::Line3d line;
  ::Triangle3d second;

  explicit Triangle3State(const std::int64_t *coordinates)
      : a(point3(coordinates[0], coordinates[1], coordinates[2])),
        b(point3(coordinates[3], coordinates[4], coordinates[5])),
        c(point3(coordinates[6], coordinates[7], coordinates[8])),
        d(point3(coordinates[9], coordinates[10], coordinates[11])),
        e(point3(coordinates[12], coordinates[13], coordinates[14])),
        f(point3(coordinates[15], coordinates[16], coordinates[17])),
        first(a, b, c), segment(d, e), line(d, e), second(d, e, f) {}
};

ec_benchmark_fixture *
triangle3_relation_fixture(int operation, const std::int64_t *coordinates) {
  const std::shared_ptr<Triangle3State> state(new Triangle3State(coordinates));
  switch (operation) {
  case 0:
    return fixture([state]() {
      const bool result = state->first.isCoplanar(state->d);
      benchmark_observe(result);
    });
  case 1:
    return fixture([state]() {
      const bool result = state->first.contains(state->d);
      benchmark_observe(result);
    });
  case 2:
    return fixture([state]() {
      const bool result = state->first.isOnEdge(state->d);
      benchmark_observe(result);
    });
  case 3:
    return fixture([state]() {
      const bool result = state->first.inside(state->d);
      benchmark_observe(result);
    });
  case 4:
    return fixture([state]() {
      const bool result = state->first.do_intersect(state->segment);
      benchmark_observe(result);
    });
  case 5:
    return fixture([state]() {
      const bool result = state->first.do_intersect(state->line);
      benchmark_observe(result);
    });
  case 6:
    return fixture([state]() {
      const bool result = state->first.do_intersect(state->second);
      benchmark_observe(result);
    });
  default:
    return 0;
  }
}

ec_benchmark_fixture *point3_distance_fixture(const std::int64_t *coordinates) {
  const std::shared_ptr<::Point3d> first(
      new ::Point3d(point3(coordinates[0], coordinates[1], coordinates[2])));
  const std::shared_ptr<::Point3d> second(
      new ::Point3d(point3(coordinates[3], coordinates[4], coordinates[5])));
  return fixture([first, second]() {
    const CORE::Expr result = first->distance(*second);
    benchmark_observe(result);
  });
}

ec_benchmark_fixture *
line3_point_distance_fixture(const std::int64_t *coordinates) {
  const std::shared_ptr<Lines3> state(new Lines3(coordinates));
  return fixture([state]() {
    const CORE::Expr result = state->first.distance(state->c);
    benchmark_observe(result);
  });
}

ec_benchmark_fixture *
segment3_point_distance_fixture(const std::int64_t *coordinates) {
  const std::shared_ptr<Segments3> state(new Segments3(coordinates));
  return fixture([state]() {
    const CORE::Expr result = state->first.distance(state->c);
    benchmark_observe(result);
  });
}

ec_benchmark_fixture *
plane3_point_distance_fixture(const std::int64_t *coordinates) {
  const std::shared_ptr<Plane3State> state(new Plane3State(coordinates));
  return fixture([state]() {
    const CORE::Expr result = state->first.distance(state->d);
    benchmark_observe(result);
  });
}

ec_benchmark_fixture *geometry3_fixture(const std::string &id) {
  static const std::int64_t tetrahedron[] = {0, 0,  0,  13, 2, 1,
                                             3, 17, -2, 4,  5, 19};
  static const std::int64_t lines[] = {0, 0, 0, 13, 2, 1, 3, 17, -2, 4, 5, 19};
  static const std::int64_t plane[] = {0, 0, 0,  13, 0, 0,  0, 17, 0,
                                       4, 5, -2, 4,  5, 19, 0, 0,  7};
  static const std::int64_t points[] = {0, 0, 0, 3, 4, 12};
  static const std::int64_t line_distance[] = {0, 0,  0,  13, 2, 1,
                                               3, 17, -2, 0,  0, 0};
  static const std::int64_t plane_distance[] = {0, 0, 0,  13, 0, 0, 0, 17, 0,
                                                3, 4, 12, 0,  0, 0, 0, 0,  0};
  static const std::int64_t triangle[] = {0, 0, 0,  13, 0, 0,  0,  17, 0,
                                          3, 4, -2, 3,  4, 19, 19, 19, 3};

  if (is(id, "geometry3.orientation"))
    return orientation3_fixture(tetrahedron);
  if (is(id, "geometry3.volume"))
    return volume3_fixture(tetrahedron);
  const int line_operation = indexed_suffix(id, "geometry3.line_relation_", 4);
  if (line_operation >= 0)
    return line3_relation_fixture(line_operation, lines);
  const int segment_operation =
      indexed_suffix(id, "geometry3.segment_relation_", 3);
  if (segment_operation >= 0)
    return segment3_relation_fixture(segment_operation, lines);
  const int plane_operation =
      indexed_suffix(id, "geometry3.plane_relation_", 7);
  if (plane_operation >= 0)
    return plane3_relation_fixture(plane_operation, plane);
  if (is(id, "geometry3.point_distance"))
    return point3_distance_fixture(points);
  if (is(id, "geometry3.line_point_distance")) {
    return line3_point_distance_fixture(line_distance);
  }
  if (is(id, "geometry3.segment_point_distance")) {
    return segment3_point_distance_fixture(line_distance);
  }
  if (is(id, "geometry3.plane_point_distance")) {
    return plane3_point_distance_fixture(plane_distance);
  }
  const int triangle_operation =
      indexed_suffix(id, "geometry3.triangle_relation_", 6);
  if (triangle_operation >= 0) {
    return triangle3_relation_fixture(triangle_operation, triangle);
  }
  return 0;
}

struct PolynomialState {
  CORE::Polynomial<CORE::BigInt> left;
  CORE::Polynomial<CORE::BigInt> right;
  CORE::BigInt x;

  PolynomialState()
      : left(
            polynomial<CORE::BigInt>(std::vector<std::int64_t>{-6, 11, -6, 1})),
        right(polynomial<CORE::BigInt>(std::vector<std::int64_t>{2, -3, 1})),
        x(integer(7)) {}
};

ec_benchmark_fixture *polynomial_fixture(const std::string &id) {
  if (!begins_with(id, "polynomial."))
    return 0;
  const std::shared_ptr<PolynomialState> state(new PolynomialState());
  if (is(id, "polynomial.evaluate"))
    return fixture([state]() {
      const CORE::BigInt result = state->left.eval(state->x);
      benchmark_observe(result);
    });
  const int binary_operation = indexed_suffix(id, "polynomial.binary_", 4);
  switch (binary_operation) {
  case 0:
    return fixture([state]() {
      const CORE::Polynomial<CORE::BigInt> result = state->left + state->right;
      const CORE::BigInt evaluated = result.eval(state->x);
      benchmark_observe(evaluated);
    });
  case 1:
    return fixture([state]() {
      const CORE::Polynomial<CORE::BigInt> result = state->left - state->right;
      const CORE::BigInt evaluated = result.eval(state->x);
      benchmark_observe(evaluated);
    });
  case 2:
    return fixture([state]() {
      const CORE::Polynomial<CORE::BigInt> result = state->left * state->right;
      const CORE::BigInt evaluated = result.eval(state->x);
      benchmark_observe(evaluated);
    });
  case 3:
    return fixture([state]() {
      CORE::Polynomial<CORE::BigInt> result = state->left;
      result.pseudoRemainder(state->right);
      const CORE::BigInt evaluated = result.eval(state->x);
      benchmark_observe(evaluated);
    });
  case 4:
    return fixture([state]() {
      const CORE::Polynomial<CORE::BigInt> result =
          composeHorner(state->left, state->right);
      const CORE::BigInt evaluated = result.eval(state->x);
      benchmark_observe(evaluated);
    });
  default:
    break;
  }
  if (is(id, "polynomial.derivative"))
    return fixture([state]() {
      CORE::Polynomial<CORE::BigInt> result = state->left;
      result.differentiate();
      result.differentiate();
      const CORE::BigInt evaluated = result.eval(state->x);
      benchmark_observe(evaluated);
    });
  if (is(id, "polynomial.resultant"))
    return fixture([state]() {
      const CORE::BigInt result = CORE::res(state->left, state->right);
      benchmark_observe(result);
    });
  if (is(id, "polynomial.discriminant"))
    return fixture([state]() {
      const CORE::BigInt result = CORE::disc(state->left);
      benchmark_observe(result);
    });
  if (is(id, "polynomial.gcd"))
    return fixture([state]() {
      const CORE::Polynomial<CORE::BigInt> result =
          CORE::gcd(state->left, state->right);
      benchmark_observe(result);
    });
  if (is(id, "polynomial.square_free"))
    return fixture([state]() {
      CORE::Polynomial<CORE::BigInt> result = state->left;
      result.sqFreePart();
      benchmark_observe(result);
    });
  if (is(id, "polynomial.root_count"))
    return fixture([state]() {
      const CORE::Sturm<CORE::BigInt> roots(state->left);
      const int result = roots.numberOfRoots();
      benchmark_observe(result);
    });
  if (is(id, "polynomial.root_count_interval"))
    return fixture([state]() {
      const CORE::Sturm<CORE::BigInt> roots(state->left);
      const int result =
          roots.numberOfRoots(CORE::BigFloat(static_cast<long>(0)),
                              CORE::BigFloat(static_cast<long>(4)));
      benchmark_observe(result);
    });
  if (is(id, "polynomial.root_isolation"))
    return fixture([state]() {
      const CORE::Sturm<CORE::BigInt> roots(state->left);
      CORE::BFVecInterval intervals;
      roots.isolateRoots(intervals);
      benchmark_observe(intervals);
    });
  return 0;
}

struct BivariateState {
  CORE::BiPoly<CORE::BigInt> left;
  CORE::BiPoly<CORE::BigInt> right;
  CORE::BigInt x;
  CORE::BigInt y;

  BivariateState()
      : left(bivariate(std::vector<std::int64_t>{0, 1, -1, 0}, 2, 2)),
        right(bivariate(std::vector<std::int64_t>{-1, 1, 1, 0}, 2, 2)),
        x(integer(3)), y(integer(7)) {}
};

ec_benchmark_fixture *bivariate_fixture(const std::string &id) {
  if (!begins_with(id, "bivariate."))
    return 0;
  const std::shared_ptr<BivariateState> state(new BivariateState());
  if (is(id, "bivariate.evaluate"))
    return fixture([state]() {
      const CORE::BigInt result =
          state->left.eval<CORE::BigInt>(state->x, state->y);
      benchmark_observe(result);
    });
  if (is(id, "bivariate.resultant"))
    return fixture([state]() {
      const CORE::Polynomial<CORE::BigInt> resultant =
          CORE::resY(state->left, state->right);
      const CORE::BigInt result = resultant.eval(state->y);
      benchmark_observe(result);
    });
  return 0;
}

struct DelaunayState {
  std::vector<CORE::Expr> x;
  std::vector<CORE::Expr> y;

  DelaunayState() : x(6), y(6) {
    static const std::int64_t points[][2] = {{0, 0}, {6, 0},  {7, 4},
                                             {3, 7}, {-2, 4}, {2, 3}};
    for (std::size_t index = 0; index < 6; ++index) {
      x[index] = expression(points[index][0]);
      y[index] = expression(points[index][1]);
    }
  }
};

ec_benchmark_fixture *delaunay_fixture(const std::string &id) {
  if (!is(id, "triangulation.delaunay_complex"))
    return 0;
  const std::shared_ptr<DelaunayState> state(new DelaunayState());
  return fixture([state]() {
    const std::vector<std::uint32_t> triangles =
        exactcore_hyper_comparison::exhaustive_empty_circle_complex(state->x,
                                                                    state->y);
    benchmark_observe(triangles);
  });
}

ec_benchmark_fixture *curve_fixture(const std::string &id) {
  static const std::int64_t line_points[] = {0, 0, 13, 2};
  static const std::int64_t line_query[] = {0, 0, 13, 2, 4, 4, 0, 0};
  static const std::int64_t intersections[] = {0, 0, 13, 2, 3, 17, 19, -7};
  static const std::int64_t circle_line[] = {0, 0, -10, 7, 10, 7};
  static const std::int64_t circle_point[] = {0, 0, 13, 7};
  static const std::int64_t circle_circle[] = {0, 0, 12, 0};

  if (is(id, "curve.line_length"))
    return point2_distance_fixture(line_points);
  if (is(id, "curve.line_side"))
    return line2_relation_fixture(0, line_query);
  if (is(id, "curve.line_contains_point")) {
    return segment2_relation_fixture(0, line_query);
  }
  if (is(id, "curve.line_intersection_topology")) {
    return segment2_relation_fixture(1, intersections);
  }
  if (is(id, "curve.line_intersection_witness")) {
    return line2_intersection_fixture(intersections);
  }
  if (is(id, "curve.segment_dispatch")) {
    return segment2_relation_fixture(1, intersections);
  }
  if (is(id, "curve.supporting_line_circle")) {
    return circle_line2_fixture(5, circle_line);
  }
  if (is(id, "curve.circle_point_distance")) {
    return circle_distance2_fixture(0, 5, 0, circle_point);
  }
  if (is(id, "curve.circle_circle_relation")) {
    return circle_distance2_fixture(1, 5, 5, circle_circle);
  }
  return 0;
}

ec_benchmark_fixture *mesh_fixture(const std::string &id) {
  static const std::int64_t point[] = {0, 0, 0, 13, 2, 0, 3, 17, 0,
                                       4, 4, 0, 0,  0, 0, 0, 0,  0};
  static const std::int64_t boundary[] = {0,  0, 0, 13, 2, 0, 3, 17, 0,
                                          13, 2, 0, 0,  0, 0, 0, 0,  0};
  static const std::int64_t intersection[] = {0, 0, 0,  13, 2, 0, 3, 17, 0,
                                              4, 4, -7, 4,  4, 7, 9, 4,  0};

  if (is(id, "mesh.plane_point_classification")) {
    return plane3_relation_fixture(1, point);
  }
  if (is(id, "mesh.triangle_contains_point")) {
    return triangle3_relation_fixture(1, point);
  }
  if (is(id, "mesh.triangle_contains_point_strictly")) {
    return triangle3_relation_fixture(3, point);
  }
  if (is(id, "mesh.triangle_boundary_point")) {
    return triangle3_relation_fixture(2, boundary);
  }
  if (is(id, "mesh.triangle_triangle_intersection")) {
    return triangle3_relation_fixture(6, intersection);
  }
  return 0;
}

ec_benchmark_fixture *path_fixture(const std::string &id) {
  static const std::int64_t line_points[] = {-2, 3, 6, -3};
  static const std::int64_t axis[] = {0, 4, 12, 4, 0, 0, 0, 0};
  static const std::int64_t endpoints[] = {-2, 3, 6, -3, 6, -3, -2, 3};
  static const std::int64_t order[] = {0, 0, 2, 0, 8, 0};
  static const std::int64_t circle_point[] = {0, 0, 3, 4};
  static const std::int64_t circle_segment[] = {0, 0, -10, 0, 10, 0};
  static const std::int64_t circle_circle[] = {0, 0, 12, 0};

  if (is(id, "path.line_length"))
    return point2_distance_fixture(line_points);
  if (is(id, "path.line_axis_classification")) {
    return line2_relation_fixture(6, axis);
  }
  if (is(id, "path.line_endpoint_equality")) {
    return segment2_relation_fixture(2, endpoints);
  }
  if (is(id, "path.line_parameter_order"))
    return between2_fixture(order);
  if (is(id, "path.circle_point_membership")) {
    return circle_distance2_fixture(0, 5, 0, circle_point);
  }
  if (is(id, "path.circle_segment_intersection")) {
    return circle_segment2_fixture(5, circle_segment);
  }
  if (is(id, "path.circle_circle_relation")) {
    return circle_distance2_fixture(1, 5, 5, circle_circle);
  }
  return 0;
}

ec_benchmark_fixture *make_fixture(const std::string &id) {
  if (ec_benchmark_fixture *result = rational_fixture(id))
    return result;
  if (ec_benchmark_fixture *result = expr_fixture(id))
    return result;
  if (ec_benchmark_fixture *result = complex_fixture(id))
    return result;
  if (ec_benchmark_fixture *result = vector_fixture(id))
    return result;
  if (ec_benchmark_fixture *result = matrix_fixture(id))
    return result;
  if (ec_benchmark_fixture *result = geometry2_fixture(id))
    return result;
  if (ec_benchmark_fixture *result = geometry3_fixture(id))
    return result;
  if (ec_benchmark_fixture *result = polynomial_fixture(id))
    return result;
  if (ec_benchmark_fixture *result = bivariate_fixture(id))
    return result;
  if (ec_benchmark_fixture *result = delaunay_fixture(id))
    return result;
  if (ec_benchmark_fixture *result = curve_fixture(id))
    return result;
  if (ec_benchmark_fixture *result = mesh_fixture(id))
    return result;
  if (ec_benchmark_fixture *result = path_fixture(id))
    return result;
  return 0;
}

} // namespace

extern "C" ec_benchmark_fixture *
ec_benchmark_fixture_new(const char *operation_id) {
  if (operation_id == 0)
    return 0;
  try {
    return make_fixture(std::string(operation_id));
  } catch (...) {
    return 0;
  }
}

extern "C" int ec_benchmark_fixture_run(ec_benchmark_fixture *fixture,
                                        std::uint64_t iterations) {
  if (fixture == 0)
    return -1;
  try {
    fixture->run_many(iterations);
    return 0;
  } catch (...) {
    return -2;
  }
}

extern "C" void ec_benchmark_fixture_free(ec_benchmark_fixture *fixture) {
  delete fixture;
}
