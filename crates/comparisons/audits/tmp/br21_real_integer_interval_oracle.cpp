#include <real/real.hpp>
#include <boost/multiprecision/cpp_int.hpp>
#include <cstdint>
#include <iostream>
#include <random>
#include <string>
#include <vector>

using Big = boost::multiprecision::cpp_int;
using Real = boost::real::real<int>;
using Exact = boost::real::exact_number<int>;

struct Rational {
    Big numerator;
    Big denominator;
};

static const Big radix = (std::numeric_limits<int>::max() / 4) * 2;

static Big pow_radix(std::size_t exponent) {
    Big result = 1;
    for (std::size_t i = 0; i < exponent; ++i) result *= radix;
    return result;
}

static Rational decode(const Exact& value) {
    Big mantissa = 0;
    for (int digit : value.digits) {
        mantissa *= radix;
        mantissa += digit;
    }
    if (!value.positive) mantissa = -mantissa;
    const long scale = static_cast<long>(value.exponent) -
                       static_cast<long>(value.digits.size());
    if (scale >= 0) return {mantissa * pow_radix(scale), 1};
    return {mantissa, pow_radix(static_cast<std::size_t>(-scale))};
}

static Rational canonical(Big numerator, Big denominator = 1) {
    if (denominator < 0) {
        numerator = -numerator;
        denominator = -denominator;
    }
    return {std::move(numerator), std::move(denominator)};
}

static bool less(const Rational& lhs, const Rational& rhs) {
    return lhs.numerator * rhs.denominator < rhs.numerator * lhs.denominator;
}

static std::string brief(const Rational& value) {
    return value.numerator.convert_to<std::string>() + "/" +
           value.denominator.convert_to<std::string>();
}

struct Counts {
    std::size_t intervals = 0;
    std::size_t exclusions = 0;
    std::size_t reversed = 0;
    std::size_t non_nested = 0;
    std::size_t exceptions = 0;
    std::map<std::string, std::size_t> exclusions_by_op;
    std::map<std::string, std::size_t> exceptions_by_op;
};

template <typename Make>
static void probe(const char* operation, std::size_t case_index,
                  const Rational& expected, Make make, Counts& counts) {
    int active_precision = 0;
    try {
        Real result = make();
        auto iterator = result.get_real_itr();
        Rational previous_lower{};
        Rational previous_upper{};
        bool have_previous = false;
        for (int precision = 1; precision <= 9; ++precision) {
            active_precision = precision;
            const auto interval = iterator.get_interval();
            const Rational lower = decode(interval.lower_bound);
            const Rational upper = decode(interval.upper_bound);
            ++counts.intervals;
            if (less(upper, lower)) {
                if (counts.reversed < 6) {
                    std::cout << "reversed op=" << operation << " case=" << case_index
                              << " p=" << precision << " lower=" << brief(lower)
                              << " upper=" << brief(upper) << '\n';
                }
                ++counts.reversed;
            }
            if (less(expected, lower) || less(upper, expected)) {
                if (counts.exclusions < 12) {
                    std::cout << "exclusion op=" << operation << " case=" << case_index
                              << " p=" << precision << " lower=" << brief(lower)
                              << " expected=" << brief(expected)
                              << " upper=" << brief(upper) << '\n';
                }
                ++counts.exclusions;
                ++counts.exclusions_by_op[operation];
            }
            if (have_previous &&
                (less(lower, previous_lower) || less(previous_upper, upper))) {
                if (counts.non_nested < 6) {
                    std::cout << "non_nested op=" << operation << " case=" << case_index
                              << " p=" << precision << '\n';
                }
                ++counts.non_nested;
            }
            previous_lower = lower;
            previous_upper = upper;
            have_previous = true;
            ++iterator;
            active_precision = -precision;
        }
    } catch (const std::exception& error) {
        if (counts.exceptions < 12) {
            std::cout << "exception op=" << operation << " case=" << case_index
                      << " stage=" << active_precision
                      << " what='" << error.what() << "'\n";
        }
        ++counts.exceptions;
        ++counts.exceptions_by_op[operation];
    }
}

static Big random_big(std::mt19937_64& rng, unsigned bits) {
    Big result = 0;
    for (unsigned produced = 0; produced < bits; produced += 64) {
        result <<= 64;
        result += rng();
    }
    if (bits % 64) result &= (Big(1) << bits) - 1;
    if (rng() & 1) result = -result;
    return result;
}

int main() {
    std::vector<std::pair<Big, Big>> cases;
    const Big r = radix;
    const std::vector<Big> special = {
        0, 1, -1, r - 2, r - 1, r, r + 1, -(r - 1), -r,
        2 * r - 1, r * r - 1, r * r, -(r * r - 1)
    };
    for (const Big& x : special) {
        for (const Big& y : special) {
            if (cases.size() >= 90) break;
            cases.emplace_back(x, y);
        }
        if (cases.size() >= 90) break;
    }
    std::mt19937_64 rng(0x494e54455256414cULL);
    while (cases.size() < 250) {
        Big x = random_big(rng, 1 + rng() % 256);
        Big y = random_big(rng, 1 + rng() % 256);
        if (y == 0) y = 1;
        cases.emplace_back(std::move(x), std::move(y));
    }

    Counts counts;
    for (std::size_t i = 0; i < cases.size(); ++i) {
        const Big x = cases[i].first;
        const Big y = cases[i].second;
        const std::string xs = x.convert_to<std::string>();
        const std::string ys = y.convert_to<std::string>();
        probe("add", i, canonical(x + y), [=] {
            Real a(xs), b(ys); return a + b;
        }, counts);
        probe("sub", i, canonical(x - y), [=] {
            Real a(xs), b(ys); return a - b;
        }, counts);
        probe("mul", i, canonical(x * y), [=] {
            Real a(xs), b(ys); return a * b;
        }, counts);
        if (y != 0) {
            probe("div", i, canonical(x, y), [=] {
                Real a(xs), b(ys); return a / b;
            }, counts);
        }
    }
    std::cout << "summary cases=" << cases.size()
              << " intervals=" << counts.intervals
              << " exclusions=" << counts.exclusions
              << " reversed=" << counts.reversed
              << " non_nested=" << counts.non_nested
              << " exceptions=" << counts.exceptions << '\n';
    for (const char* op : {"add", "sub", "mul", "div"}) {
        std::cout << op << " exclusions=" << counts.exclusions_by_op[op]
                  << " exceptions=" << counts.exceptions_by_op[op] << '\n';
    }
    return counts.exclusions == 0 && counts.reversed == 0 &&
                   counts.non_nested == 0 && counts.exceptions == 0
               ? 0 : 1;
}
