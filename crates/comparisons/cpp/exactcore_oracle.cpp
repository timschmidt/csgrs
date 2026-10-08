#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#define CORE_LEVEL 3
#include <CORE/CORE.h>
#include <CORE/ComplexT.h>
#include <CORE/IntervalT.h>
#include <CORE/geometry2d.h>
#include <CORE/geometry3d.h>
#include <CORE/linearAlgebraT.h>
#include <CORE/poly/Curves.h>
#include <CORE/poly/Descartes.h>
#include <CORE/poly/Poly.h>
#include <CORE/poly/Sturm.h>

#undef double
#undef long

#include "empty_circle_complex.h"
#include "exactcore_oracle.h"

namespace {

template <typename T>
std::string render(const T &value) {
    std::ostringstream stream;
    stream << value;
    return stream.str();
}

int write_string(const std::string &value, char *out, std::size_t out_len) {
    if (out == 0 || out_len == 0 || value.size() + 1 > out_len) {
        return -2;
    }
    std::memcpy(out, value.c_str(), value.size() + 1);
    return static_cast<int>(value.size());
}

CORE::BigRat rational(const char *text) {
    CORE::BigRat value(text);
    value.canonicalize();
    return value;
}

CORE::Expr expression(const char *text) {
    return CORE::Expr(rational(text));
}

CORE::Expr integer_expression(std::int64_t value) {
    return CORE::Expr(CORE::BigInt(std::to_string(value)));
}

::Point2d point2(const std::int64_t *coordinates) {
    return ::Point2d(integer_expression(coordinates[0]), integer_expression(coordinates[1]));
}

::Point3d point3(const std::int64_t *coordinates) {
    return ::Point3d(
        integer_expression(coordinates[0]),
        integer_expression(coordinates[1]),
        integer_expression(coordinates[2]));
}

template <typename NT>
CORE::Polynomial<NT> polynomial(const std::int64_t *coefficients, std::size_t count) {
    if (count == 0) {
        return CORE::Polynomial<NT>();
    }
    std::vector<NT> converted(count);
    for (std::size_t index = 0; index < count; ++index) {
        converted[index] = NT(CORE::BigInt(std::to_string(coefficients[index])));
    }
    return CORE::Polynomial<NT>(static_cast<int>(count - 1), &converted[0]);
}

CORE::BiPoly<CORE::BigInt> bivariate(
    const std::int64_t *coefficients,
    std::size_t x_count,
    std::size_t y_count) {
    std::vector<CORE::Polynomial<CORE::BigInt> > rows;
    rows.reserve(y_count);
    for (std::size_t y = 0; y < y_count; ++y) {
        std::vector<CORE::BigInt> x_coefficients(x_count);
        for (std::size_t x = 0; x < x_count; ++x) {
            x_coefficients[x] = CORE::BigInt(std::to_string(coefficients[x * y_count + y]));
        }
        rows.push_back(CORE::Polynomial<CORE::BigInt>(
            static_cast<int>(x_count - 1), &x_coefficients[0]));
    }
    return CORE::BiPoly<CORE::BigInt>(rows);
}

template <typename T>
double numeric(const T &value) {
    return value.doubleValue();
}

double numeric(const CORE::Expr &value) {
    return value.doubleValue();
}

double numeric(const CORE::BigRat &value) {
    return value.doubleValue();
}

double numeric(const CORE::BigInt &value) {
    return value.doubleValue();
}

}  // namespace

extern "C" int ec_rational_binary(
    int operation,
    const char *left,
    const char *right,
    char *out,
    std::size_t out_len) {
    try {
        const CORE::BigRat a = rational(left);
        const CORE::BigRat b = rational(right);
        CORE::BigRat result;
        switch (operation) {
            case 0: result = a + b; break;
            case 1: result = a - b; break;
            case 2: result = a * b; break;
            case 3:
                if (b == 0) return -3;
                result = a / b;
                break;
            default: return -1;
        }
        result.canonicalize();
        return write_string(render(result), out, out_len);
    } catch (...) {
        return -4;
    }
}

extern "C" int ec_rational_unary(
    int operation,
    const char *text,
    char *out,
    std::size_t out_len) {
    try {
        const CORE::BigRat value = rational(text);
        CORE::BigRat result;
        switch (operation) {
            case 0: result = -value; break;
            case 1: result.abs(value); break;
            case 2:
                if (value == 0) return -3;
                result = value.reciprocal();
                break;
            default: return -1;
        }
        result.canonicalize();
        return write_string(render(result), out, out_len);
    } catch (...) {
        return -4;
    }
}

extern "C" int ec_rational_compare(const char *left, const char *right) {
    try {
        const CORE::BigRat a = rational(left);
        const CORE::BigRat b = rational(right);
        return a < b ? -1 : (a > b ? 1 : 0);
    } catch (...) {
        return -2;
    }
}

extern "C" int ec_rational_parts(
    const char *text,
    char *numerator,
    std::size_t numerator_len,
    char *denominator,
    std::size_t denominator_len) {
    try {
        const CORE::BigRat value = rational(text);
        const int numerator_status =
            write_string(render(value.numerator()), numerator, numerator_len);
        if (numerator_status < 0) return numerator_status;
        const int denominator_status =
            write_string(render(value.denominator()), denominator, denominator_len);
        return denominator_status < 0 ? denominator_status : 0;
    } catch (...) {
        return -4;
    }
}

extern "C" double ec_expr_unary(int operation, const char *text, std::uint32_t root_degree) {
    try {
        const CORE::Expr value = expression(text);
        CORE::Expr result;
        switch (operation) {
            case 0: result = -value; break;
            case 1: result = CORE::abs(value); break;
            case 2: result = sqrt(value); break;
            case 3: result = cbrt(value); break;
            case 4:
                if (root_degree == 0) return std::numeric_limits<double>::quiet_NaN();
                result = root(value, root_degree);
                break;
            case 5: result = exp(value); break;
            case 6: result = log(value); break;
            case 7: result = log2(value); break;
            case 8: result = log10(value); break;
            case 9: result = sin(value); break;
            case 10: result = cos(value); break;
            case 11: result = tan(value); break;
            case 12: result = asin(value); break;
            case 13: result = acos(value); break;
            case 14: result = atan(value); break;
            case 15: result = value * value; break;
            case 16: result = exp2(value); break;
            case 17: result = exp10(value); break;
            case 18: result = cot(value); break;
            default: return std::numeric_limits<double>::quiet_NaN();
        }
        return result.doubleValue();
    } catch (...) {
        return std::numeric_limits<double>::quiet_NaN();
    }
}

extern "C" double ec_expr_binary(
    int operation,
    const char *left,
    const char *right,
    std::int64_t integer_parameter) {
    try {
        const CORE::Expr a = expression(left);
        const CORE::Expr b = expression(right);
        CORE::Expr result;
        switch (operation) {
            case 0: result = a + b; break;
            case 1: result = a - b; break;
            case 2: result = a * b; break;
            case 3: result = a / b; break;
            case 4: result = pow(a, static_cast<long>(integer_parameter)); break;
            default: return std::numeric_limits<double>::quiet_NaN();
        }
        return result.doubleValue();
    } catch (...) {
        return std::numeric_limits<double>::quiet_NaN();
    }
}

extern "C" double ec_expr_constant(int operation) {
    try {
        switch (operation) {
            case 0: return CORE::pi().doubleValue();
            case 1: return CORE::e().doubleValue();
            default: return std::numeric_limits<double>::quiet_NaN();
        }
    } catch (...) {
        return std::numeric_limits<double>::quiet_NaN();
    }
}

extern "C" int ec_expr_sign_unary(int operation, const char *text, std::uint32_t root_degree) {
    const double value = ec_expr_unary(operation, text, root_degree);
    if (std::isnan(value)) return -2;
    return value < 0.0 ? -1 : (value > 0.0 ? 1 : 0);
}

extern "C" int ec_expr_floor_ceil(
    int operation,
    const char *text,
    char *out,
    std::size_t out_len) {
    try {
        const CORE::Expr value = expression(text);
        const CORE::BigInt result = operation == 0 ? CORE::floor(value) : CORE::ceil(value);
        return write_string(render(result), out, out_len);
    } catch (...) {
        return -4;
    }
}

extern "C" int ec_complex_binary(
    int operation,
    const char *ar,
    const char *ai,
    const char *br,
    const char *bi,
    char *real_out,
    std::size_t real_len,
    char *imag_out,
    std::size_t imag_len) {
    try {
        const ComplexT<CORE::BigRat> a(rational(ar), rational(ai));
        const ComplexT<CORE::BigRat> b(rational(br), rational(bi));
        ComplexT<CORE::BigRat> result;
        switch (operation) {
            case 0: result = a + b; break;
            case 1: result = a - b; break;
            case 2: result = a * b; break;
            case 3:
                if (b == CORE::BigRat(0)) return -3;
                result = a / b;
                break;
            default: return -1;
        }
        const int real_status = write_string(render(result.re()), real_out, real_len);
        if (real_status < 0) return real_status;
        const int imaginary_status = write_string(render(result.im()), imag_out, imag_len);
        return imaginary_status < 0 ? imaginary_status : 0;
    } catch (...) {
        return -4;
    }
}

extern "C" int ec_complex_norm_squared(
    const char *real,
    const char *imag,
    char *out,
    std::size_t out_len) {
    try {
        const CORE::BigRat re = rational(real);
        const CORE::BigRat im = rational(imag);
        return write_string(render(re * re + im * im), out, out_len);
    } catch (...) {
        return -4;
    }
}

extern "C" double ec_vector_dot(
    std::uint32_t dimension,
    const std::int64_t *left,
    const std::int64_t *right) {
    try {
        std::vector<CORE::BigRat> a(dimension), b(dimension);
        for (std::uint32_t index = 0; index < dimension; ++index) {
            a[index] = CORE::BigRat(static_cast<long>(left[index]));
            b[index] = CORE::BigRat(static_cast<long>(right[index]));
        }
        const VectorT<CORE::BigRat> va(static_cast<int>(dimension), &a[0]);
        const VectorT<CORE::BigRat> vb(static_cast<int>(dimension), &b[0]);
        CORE::BigRat result(0);
        for (std::uint32_t index = 0; index < dimension; ++index) {
            result += va[static_cast<int>(index)] * vb[static_cast<int>(index)];
        }
        return numeric(result);
    } catch (...) {
        return std::numeric_limits<double>::quiet_NaN();
    }
}

extern "C" int ec_vector_binary(
    int operation,
    std::uint32_t dimension,
    const std::int64_t *left,
    const std::int64_t *right,
    double *out) {
    try {
        std::vector<CORE::BigRat> a(dimension), b(dimension);
        for (std::uint32_t index = 0; index < dimension; ++index) {
            a[index] = CORE::BigRat(static_cast<long>(left[index]));
            b[index] = CORE::BigRat(static_cast<long>(right[index]));
        }
        const VectorT<CORE::BigRat> va(static_cast<int>(dimension), &a[0]);
        const VectorT<CORE::BigRat> vb(static_cast<int>(dimension), &b[0]);
        const VectorT<CORE::BigRat> result = operation == 0 ? va + vb : va - vb;
        for (std::uint32_t index = 0; index < dimension; ++index) {
            out[index] = numeric(result[static_cast<int>(index)]);
        }
        return 0;
    } catch (...) {
        return -1;
    }
}

extern "C" int ec_vector_cross3(
    const std::int64_t *left,
    const std::int64_t *right,
    double *out) {
    try {
        VectorT<CORE::BigRat> a(
            CORE::BigRat(static_cast<long>(left[0])),
            CORE::BigRat(static_cast<long>(left[1])),
            CORE::BigRat(static_cast<long>(left[2])));
        VectorT<CORE::BigRat> b(
            CORE::BigRat(static_cast<long>(right[0])),
            CORE::BigRat(static_cast<long>(right[1])),
            CORE::BigRat(static_cast<long>(right[2])));
        const VectorT<CORE::BigRat> result = a.cross(b);
        for (int index = 0; index < 3; ++index) out[index] = numeric(result[index]);
        return 0;
    } catch (...) {
        return -1;
    }
}

extern "C" double ec_vector_norm(std::uint32_t dimension, const std::int64_t *values) {
    try {
        CORE::Expr sum(0);
        for (std::uint32_t index = 0; index < dimension; ++index) {
            const CORE::Expr value = integer_expression(values[index]);
            sum += value * value;
        }
        return sqrt(sum).doubleValue();
    } catch (...) {
        return std::numeric_limits<double>::quiet_NaN();
    }
}

extern "C" double ec_matrix_determinant(
    std::uint32_t dimension,
    const std::int64_t *row_major) {
    try {
        const std::size_t count = dimension * dimension;
        std::vector<CORE::BigRat> values(count);
        for (std::size_t index = 0; index < count; ++index) {
            values[index] = CORE::BigRat(static_cast<long>(row_major[index]));
        }
        const MatrixT<CORE::BigRat> matrix(
            static_cast<int>(dimension), static_cast<int>(dimension), &values[0]);
        return numeric(matrix.determinant());
    } catch (...) {
        return std::numeric_limits<double>::quiet_NaN();
    }
}

extern "C" int ec_matrix_binary(
    int operation,
    std::uint32_t dimension,
    const std::int64_t *left,
    const std::int64_t *right,
    double *out) {
    try {
        const std::size_t count = dimension * dimension;
        std::vector<CORE::BigRat> a(count), b(count);
        for (std::size_t index = 0; index < count; ++index) {
            a[index] = CORE::BigRat(static_cast<long>(left[index]));
            b[index] = CORE::BigRat(static_cast<long>(right[index]));
        }
        const MatrixT<CORE::BigRat> ma(
            static_cast<int>(dimension), static_cast<int>(dimension), &a[0]);
        const MatrixT<CORE::BigRat> mb(
            static_cast<int>(dimension), static_cast<int>(dimension), &b[0]);
        MatrixT<CORE::BigRat> result = operation == 0 ? ma + mb :
            (operation == 1 ? ma - mb : ma * mb);
        for (std::uint32_t row = 0; row < dimension; ++row) {
            for (std::uint32_t column = 0; column < dimension; ++column) {
                out[row * dimension + column] = numeric(result(row, column));
            }
        }
        return 0;
    } catch (...) {
        return -1;
    }
}

extern "C" int ec_matrix_transpose(
    std::uint32_t dimension,
    const std::int64_t *row_major,
    double *out) {
    try {
        const std::size_t count = dimension * dimension;
        std::vector<CORE::BigRat> values(count);
        for (std::size_t index = 0; index < count; ++index) {
            values[index] = CORE::BigRat(static_cast<long>(row_major[index]));
        }
        const MatrixT<CORE::BigRat> matrix(
            static_cast<int>(dimension), static_cast<int>(dimension), &values[0]);
        const MatrixT<CORE::BigRat> result = transpose(matrix);
        for (std::uint32_t row = 0; row < dimension; ++row) {
            for (std::uint32_t column = 0; column < dimension; ++column) {
                out[row * dimension + column] = numeric(result(row, column));
            }
        }
        return 0;
    } catch (...) {
        return -1;
    }
}

extern "C" int ec_matrix_adjugate(
    std::uint32_t dimension,
    const std::int64_t *row_major,
    double *determinant,
    double *out) {
    try {
        const std::size_t count = dimension * dimension;
        std::vector<CORE::BigRat> values(count);
        for (std::size_t index = 0; index < count; ++index) {
            values[index] = CORE::BigRat(static_cast<long>(row_major[index]));
        }
        const MatrixT<CORE::BigRat> matrix(
            static_cast<int>(dimension), static_cast<int>(dimension), &values[0]);
        MatrixT<CORE::BigRat> adjugate(static_cast<int>(dimension));
        const CORE::BigRat det = matrix.bareissInverse(&adjugate);
        *determinant = numeric(det);
        for (std::uint32_t row = 0; row < dimension; ++row) {
            for (std::uint32_t column = 0; column < dimension; ++column) {
                out[row * dimension + column] = numeric(adjugate(row, column));
            }
        }
        return 0;
    } catch (...) {
        return -1;
    }
}

extern "C" int ec_orientation2(const std::int64_t *coordinates) {
    try {
        return ::orientation2d(point2(coordinates), point2(coordinates + 2), point2(coordinates + 4));
    } catch (...) {
        return -2;
    }
}

extern "C" double ec_area2(const std::int64_t *coordinates) {
    try {
        return numeric(::area(point2(coordinates), point2(coordinates + 2), point2(coordinates + 4)));
    } catch (...) {
        return std::numeric_limits<double>::quiet_NaN();
    }
}

extern "C" int ec_between2(const std::int64_t *coordinates) {
    try {
        return ::between(point2(coordinates), point2(coordinates + 2), point2(coordinates + 4)) ? 1 : 0;
    } catch (...) {
        return -2;
    }
}

extern "C" int ec_line2_relation(int operation, const std::int64_t *coordinates) {
    try {
        const ::Line2d first(point2(coordinates), point2(coordinates + 2));
        const ::Point2d third = point2(coordinates + 4);
        if (operation == 0) return first.orientation(third);
        if (operation == 1) return first.contains(third) ? 1 : 0;
        if (operation == 5) return first.isVertical() ? 1 : 0;
        if (operation == 6) return first.isHorizontal() ? 1 : 0;
        const ::Line2d second(third, point2(coordinates + 6));
        switch (operation) {
            case 2: return first.isParallel(second) ? 1 : 0;
            case 3: return first.intersects(second);
            case 4: return first.isCoincident(second) ? 1 : 0;
            default: return -3;
        }
    } catch (...) {
        return -2;
    }
}

extern "C" int ec_line2_intersection(const std::int64_t *coordinates, double *output) {
    try {
        const ::Line2d first(point2(coordinates), point2(coordinates + 2));
        const ::Line2d second(point2(coordinates + 4), point2(coordinates + 6));
        ::GeomObj *object = first.intersection(second);
        if (object == 0) return 1;
        ::Point2d *point = dynamic_cast< ::Point2d *>(object);
        if (point == 0) {
            delete object;
            return 1;
        }
        output[0] = numeric(point->X());
        output[1] = numeric(point->Y());
        delete object;
        return 0;
    } catch (...) {
        return -2;
    }
}

extern "C" int ec_segment2_relation(int operation, const std::int64_t *coordinates) {
    try {
        ::Segment2d first(point2(coordinates), point2(coordinates + 2));
        const ::Point2d third = point2(coordinates + 4);
        if (operation == 0) return first.contains(third) ? 1 : 0;
        const ::Segment2d second(third, point2(coordinates + 6));
        switch (operation) {
            case 1: return first.intersects(second);
            case 2: return first.isCoincident(second) ? 1 : 0;
            case 3: return first.isParallel(second) ? 1 : 0;
            default: return -3;
        }
    } catch (...) {
        return -2;
    }
}

extern "C" int ec_incircle2(const std::int64_t *coordinates) {
    try {
        const ::Point2d a = point2(coordinates);
        const ::Point2d b = point2(coordinates + 2);
        const ::Point2d c = point2(coordinates + 4);
        const ::Point2d query = point2(coordinates + 6);
        const ::Circle2d circle(a, b, c);
        return circle.side_of(query);
    } catch (...) {
        return -2;
    }
}

extern "C" int ec_circle2_relation(
    int operation,
    std::int64_t radius,
    const std::int64_t *coordinates) {
    try {
        if (radius < 0) return -3;
        ::Circle2d circle(point2(coordinates), integer_expression(radius));
        const ::Point2d a = point2(coordinates + 2);
        const ::Point2d b = point2(coordinates + 4);
        CORE::Expr distance;
        switch (operation) {
            case 0:
                distance = circle.distance(::Line2d(a, b));
                break;
            case 1:
                distance = ::Segment2d(a, b).distance(point2(coordinates))
                    - integer_expression(radius);
                break;
            default: return -1;
        }
        return CORE::sign(distance);
    } catch (...) {
        return -2;
    }
}

extern "C" double ec_circle2_distance(
    int operation,
    std::int64_t first_radius,
    std::int64_t second_radius,
    const std::int64_t *coordinates) {
    try {
        if (first_radius < 0 || second_radius < 0) {
            return std::numeric_limits<double>::quiet_NaN();
        }
        ::Circle2d first(point2(coordinates), static_cast<double>(first_radius));
        switch (operation) {
            case 0:
                return numeric(first.distance(point2(coordinates + 2)));
            case 1: {
                ::Circle2d second(
                    point2(coordinates + 2), static_cast<double>(second_radius));
                return numeric(first.distance(second));
            }
            default:
                return std::numeric_limits<double>::quiet_NaN();
        }
    } catch (...) {
        return std::numeric_limits<double>::quiet_NaN();
    }
}

extern "C" double ec_point2_distance(const std::int64_t *coordinates) {
    try {
        return numeric(point2(coordinates).distance(point2(coordinates + 2)));
    } catch (...) {
        return std::numeric_limits<double>::quiet_NaN();
    }
}

extern "C" double ec_line2_point_distance(const std::int64_t *coordinates) {
    try {
        return numeric(::Line2d(point2(coordinates), point2(coordinates + 2)).distance(point2(coordinates + 4)));
    } catch (...) {
        return std::numeric_limits<double>::quiet_NaN();
    }
}

extern "C" double ec_segment2_point_distance(const std::int64_t *coordinates) {
    try {
        return numeric(::Segment2d(point2(coordinates), point2(coordinates + 2)).distance(point2(coordinates + 4)));
    } catch (...) {
        return std::numeric_limits<double>::quiet_NaN();
    }
}

extern "C" int ec_orientation3(const std::int64_t *coordinates) {
    try {
        return ::orientation3d(
            point3(coordinates), point3(coordinates + 3), point3(coordinates + 6), point3(coordinates + 9));
    } catch (...) {
        return -2;
    }
}

extern "C" double ec_volume3(const std::int64_t *coordinates) {
    try {
        const ::Point3d a = point3(coordinates);
        const ::Point3d b = point3(coordinates + 3);
        const ::Point3d c = point3(coordinates + 6);
        const ::Point3d d = point3(coordinates + 9);
        return numeric(CORE::Expr(::orientation3d(a, b, c, d)) * ::volume(a, b, c, d));
    } catch (...) {
        return std::numeric_limits<double>::quiet_NaN();
    }
}

extern "C" int ec_line3_relation(int operation, const std::int64_t *coordinates) {
    try {
        const ::Line3d first(point3(coordinates), point3(coordinates + 3));
        const ::Point3d third = point3(coordinates + 6);
        if (operation == 0) return first.contains(third) ? 1 : 0;
        const ::Line3d second(third, point3(coordinates + 9));
        switch (operation) {
            case 1: return first.isParallel(second) ? 1 : 0;
            case 2: return first.isSkew(second) ? 1 : 0;
            case 3: return first.intersects(second);
            case 4: return first.isCoincident(second) ? 1 : 0;
            default: return -3;
        }
    } catch (...) {
        return -2;
    }
}

extern "C" int ec_segment3_relation(int operation, const std::int64_t *coordinates) {
    try {
        const ::Segment3d first(point3(coordinates), point3(coordinates + 3));
        const ::Point3d third = point3(coordinates + 6);
        if (operation == 0) return first.contains(third) ? 1 : 0;
        const ::Segment3d second(third, point3(coordinates + 9));
        switch (operation) {
            case 1: return first.intersects(second);
            case 2: return first.isCoincident(second) ? 1 : 0;
            case 3: return first.isCoplanar(second) ? 1 : 0;
            default: return -3;
        }
    } catch (...) {
        return -2;
    }
}

extern "C" int ec_plane3_relation(int operation, const std::int64_t *coordinates) {
    try {
        const ::Plane3d first(point3(coordinates), point3(coordinates + 3), point3(coordinates + 6));
        const ::Point3d fourth = point3(coordinates + 9);
        if (operation == 0) return first.contains(fourth) ? 1 : 0;
        if (operation == 1) {
            const CORE::Expr value = first.apply(fourth);
            return CORE::sign(value);
        }
        const ::Point3d fifth = point3(coordinates + 12);
        if (operation == 2) return first.contains(::Line3d(fourth, fifth)) ? 1 : 0;
        if (operation == 3) return first.intersects(::Line3d(fourth, fifth));
        if (operation == 4) return first.contains(::Segment3d(fourth, fifth)) ? 1 : 0;
        if (operation == 5) return first.intersects(::Segment3d(fourth, fifth));
        const ::Plane3d second(fourth, fifth, point3(coordinates + 15));
        if (operation == 6) return first.isParallel(second) ? 1 : 0;
        if (operation == 7) return first.intersects(second);
        return -3;
    } catch (...) {
        return -2;
    }
}

extern "C" double ec_point3_distance(const std::int64_t *coordinates) {
    try {
        return numeric(point3(coordinates).distance(point3(coordinates + 3)));
    } catch (...) {
        return std::numeric_limits<double>::quiet_NaN();
    }
}

extern "C" double ec_line3_point_distance(const std::int64_t *coordinates) {
    try {
        return numeric(::Line3d(point3(coordinates), point3(coordinates + 3)).distance(point3(coordinates + 6)));
    } catch (...) {
        return std::numeric_limits<double>::quiet_NaN();
    }
}

extern "C" double ec_segment3_point_distance(const std::int64_t *coordinates) {
    try {
        return numeric(::Segment3d(point3(coordinates), point3(coordinates + 3))
                           .distance(point3(coordinates + 6)));
    } catch (...) {
        return std::numeric_limits<double>::quiet_NaN();
    }
}

extern "C" double ec_plane3_point_distance(const std::int64_t *coordinates) {
    try {
        return numeric(::Plane3d(point3(coordinates), point3(coordinates + 3), point3(coordinates + 6))
                           .distance(point3(coordinates + 9)));
    } catch (...) {
        return std::numeric_limits<double>::quiet_NaN();
    }
}

extern "C" int ec_triangle3_relation(int operation, const std::int64_t *coordinates) {
    try {
        const ::Triangle3d triangle(point3(coordinates), point3(coordinates + 3), point3(coordinates + 6));
        const ::Point3d fourth = point3(coordinates + 9);
        switch (operation) {
            case 0: return triangle.isCoplanar(fourth) ? 1 : 0;
            case 1: return triangle.contains(fourth) ? 1 : 0;
            case 2: return triangle.isOnEdge(fourth) ? 1 : 0;
            case 3: return triangle.inside(fourth) ? 1 : 0;
            case 4: return triangle.do_intersect(::Segment3d(fourth, point3(coordinates + 12))) ? 1 : 0;
            case 5: return triangle.do_intersect(::Line3d(fourth, point3(coordinates + 12))) ? 1 : 0;
            case 6:
                return triangle.do_intersect(::Triangle3d(
                    fourth, point3(coordinates + 12), point3(coordinates + 15))) ? 1 : 0;
            default: return -3;
        }
    } catch (...) {
        return -2;
    }
}

extern "C" int ec_polynomial_eval(
    const std::int64_t *coefficients,
    std::size_t coefficient_count,
    std::int64_t x,
    char *out,
    std::size_t out_len) {
    try {
        const CORE::Polynomial<CORE::BigInt> value =
            polynomial<CORE::BigInt>(coefficients, coefficient_count);
        return write_string(render(value.eval(CORE::BigInt(std::to_string(x)))), out, out_len);
    } catch (...) {
        return -4;
    }
}

extern "C" int ec_polynomial_binary_eval(
    int operation,
    const std::int64_t *left,
    std::size_t left_count,
    const std::int64_t *right,
    std::size_t right_count,
    std::int64_t x,
    char *out,
    std::size_t out_len) {
    try {
        CORE::Polynomial<CORE::BigInt> a = polynomial<CORE::BigInt>(left, left_count);
        const CORE::Polynomial<CORE::BigInt> b = polynomial<CORE::BigInt>(right, right_count);
        CORE::Polynomial<CORE::BigInt> result;
        switch (operation) {
            case 0: result = a + b; break;
            case 1: result = a - b; break;
            case 2: result = a * b; break;
            case 3:
                a.pseudoRemainder(b);
                result = a;
                break;
            case 4: result = composeHorner(a, b); break;
            default: return -1;
        }
        return write_string(render(result.eval(CORE::BigInt(std::to_string(x)))), out, out_len);
    } catch (...) {
        return -4;
    }
}

extern "C" int ec_polynomial_derivative_eval(
    const std::int64_t *coefficients,
    std::size_t coefficient_count,
    std::uint32_t order,
    std::int64_t x,
    char *out,
    std::size_t out_len) {
    try {
        CORE::Polynomial<CORE::BigInt> value =
            polynomial<CORE::BigInt>(coefficients, coefficient_count);
        for (std::uint32_t index = 0; index < order; ++index) value.differentiate();
        return write_string(render(value.eval(CORE::BigInt(std::to_string(x)))), out, out_len);
    } catch (...) {
        return -4;
    }
}

extern "C" int ec_polynomial_resultant(
    const std::int64_t *left,
    std::size_t left_count,
    const std::int64_t *right,
    std::size_t right_count,
    char *out,
    std::size_t out_len) {
    try {
        return write_string(
            render(CORE::res(
                polynomial<CORE::BigInt>(left, left_count),
                polynomial<CORE::BigInt>(right, right_count))),
            out,
            out_len);
    } catch (...) {
        return -4;
    }
}

extern "C" int ec_polynomial_discriminant(
    const std::int64_t *coefficients,
    std::size_t coefficient_count,
    char *out,
    std::size_t out_len) {
    try {
        return write_string(
            render(CORE::disc(polynomial<CORE::BigInt>(coefficients, coefficient_count))),
            out,
            out_len);
    } catch (...) {
        return -4;
    }
}

extern "C" int ec_polynomial_gcd_degree(
    const std::int64_t *left,
    std::size_t left_count,
    const std::int64_t *right,
    std::size_t right_count) {
    try {
        return CORE::gcd(
                   polynomial<CORE::BigInt>(left, left_count),
                   polynomial<CORE::BigInt>(right, right_count))
            .getTrueDegree();
    } catch (...) {
        return -4;
    }
}

extern "C" int ec_polynomial_square_free_degree(
    const std::int64_t *coefficients,
    std::size_t coefficient_count) {
    try {
        CORE::Polynomial<CORE::BigInt> value =
            polynomial<CORE::BigInt>(coefficients, coefficient_count);
        value.sqFreePart();
        return value.getTrueDegree();
    } catch (...) {
        return -4;
    }
}

extern "C" int ec_polynomial_root_count(
    const std::int64_t *coefficients,
    std::size_t coefficient_count) {
    try {
        const CORE::Sturm<CORE::BigInt> roots(
            polynomial<CORE::BigInt>(coefficients, coefficient_count));
        return roots.numberOfRoots();
    } catch (...) {
        return -4;
    }
}

extern "C" int ec_polynomial_root_count_interval(
    const std::int64_t *coefficients,
    std::size_t coefficient_count,
    std::int64_t lower,
    std::int64_t upper) {
    try {
        const CORE::Sturm<CORE::BigInt> roots(
            polynomial<CORE::BigInt>(coefficients, coefficient_count));
        return roots.numberOfRoots(CORE::BigFloat(static_cast<long>(lower)),
                                   CORE::BigFloat(static_cast<long>(upper)));
    } catch (...) {
        return -4;
    }
}

extern "C" int ec_polynomial_isolate_roots(
    const std::int64_t *coefficients,
    std::size_t coefficient_count,
    double *interval_pairs,
    std::size_t pair_capacity) {
    try {
        const CORE::Sturm<CORE::BigInt> roots(
            polynomial<CORE::BigInt>(coefficients, coefficient_count));
        CORE::BFVecInterval intervals;
        roots.isolateRoots(intervals);
        if (intervals.size() > pair_capacity) return -3;
        for (std::size_t index = 0; index < intervals.size(); ++index) {
            interval_pairs[2 * index] = intervals[index].first.doubleValue();
            interval_pairs[2 * index + 1] = intervals[index].second.doubleValue();
        }
        return static_cast<int>(intervals.size());
    } catch (...) {
        return -4;
    }
}

extern "C" int ec_bivariate_eval(
    const std::int64_t *coefficients,
    std::size_t x_count,
    std::size_t y_count,
    std::int64_t x,
    std::int64_t y,
    char *out,
    std::size_t out_len) {
    try {
        const CORE::BiPoly<CORE::BigInt> value = bivariate(coefficients, x_count, y_count);
        const CORE::BigInt result = value.eval(
            CORE::BigInt(std::to_string(x)), CORE::BigInt(std::to_string(y)));
        return write_string(render(result), out, out_len);
    } catch (...) {
        return -4;
    }
}

extern "C" int ec_bivariate_resultant(
    int eliminate_y,
    const std::int64_t *left,
    const std::int64_t *right,
    std::size_t x_count,
    std::size_t y_count,
    std::int64_t retained_value,
    char *out,
    std::size_t out_len) {
    try {
        CORE::BiPoly<CORE::BigInt> a = bivariate(left, x_count, y_count);
        CORE::BiPoly<CORE::BigInt> b = bivariate(right, x_count, y_count);
        const CORE::Polynomial<CORE::BigInt> resultant =
            eliminate_y ? CORE::resY(a, b) : CORE::resX(a, b);
        return write_string(
            render(resultant.eval(CORE::BigInt(std::to_string(retained_value)))), out, out_len);
    } catch (...) {
        return -4;
    }
}

extern "C" int ec_delaunay_dt4(
    const std::int64_t *xy,
    std::size_t point_count,
    std::uint32_t *triangle_indices,
    std::size_t triangle_capacity) {
    try {
        std::vector<CORE::Expr> x(point_count), y(point_count);
        for (std::size_t index = 0; index < point_count; ++index) {
            x[index] = integer_expression(xy[2 * index]);
            y[index] = integer_expression(xy[2 * index + 1]);
        }
        const std::vector<std::uint32_t> cells =
            exactcore_hyper_comparison::exhaustive_empty_circle_complex(x, y);
        const std::size_t face_count = cells.size() / 3;
        if (face_count > triangle_capacity) return -3;
        for (std::size_t index = 0; index < cells.size(); ++index)
            triangle_indices[index] = cells[index];
        return static_cast<int>(face_count);
    } catch (...) {
        return -4;
    }
}
