#include <real/real.hpp>
#include <boost/multiprecision/cpp_int.hpp>
#include <iostream>
#include <limits>
#include <map>
#include <random>
#include <string>
#include <utility>
#include <vector>

using Big = boost::multiprecision::cpp_int;
using Real = boost::real::real<int>;
using Exact = boost::real::exact_number<int>;

struct Q { Big n; Big d; };
struct Decimal { std::string text; Q value; };
static const Big radix = (std::numeric_limits<int>::max() / 4) * 2;

static Big power(Big base, unsigned exponent) {
    Big result = 1;
    while (exponent) {
        if (exponent & 1) result *= base;
        exponent >>= 1;
        if (exponent) base *= base;
    }
    return result;
}

static Q canon(Big n, Big d = 1) {
    if (d < 0) { n = -n; d = -d; }
    return {std::move(n), std::move(d)};
}
static Q add(const Q& a, const Q& b) { return canon(a.n*b.d + b.n*a.d, a.d*b.d); }
static Q sub(const Q& a, const Q& b) { return canon(a.n*b.d - b.n*a.d, a.d*b.d); }
static Q mul(const Q& a, const Q& b) { return canon(a.n*b.n, a.d*b.d); }
static Q divq(const Q& a, const Q& b) { return canon(a.n*b.d, a.d*b.n); }
static bool less(const Q& a, const Q& b) { return a.n*b.d < b.n*a.d; }
static std::string brief(const Q& value) {
    return value.n.convert_to<std::string>() + "/" + value.d.convert_to<std::string>();
}

static Q decode(const Exact& value) {
    Big mantissa = 0;
    for (int digit : value.digits) {
        mantissa *= radix;
        mantissa += digit;
    }
    if (!value.positive) mantissa = -mantissa;
    const long scale = static_cast<long>(value.exponent) -
                       static_cast<long>(value.digits.size());
    return scale >= 0
        ? canon(mantissa * power(radix, static_cast<unsigned>(scale)))
        : canon(mantissa, power(radix, static_cast<unsigned>(-scale)));
}

static std::string decimal_text(const Big& numerator, unsigned scale) {
    const bool negative = numerator < 0;
    Big magnitude = negative ? -numerator : numerator;
    std::string digits = magnitude.convert_to<std::string>();
    std::string result = negative ? "-" : "";
    if (scale == 0) return result + digits;
    if (digits.size() <= scale) {
        result += "0.";
        result.append(scale - digits.size(), '0');
        result += digits;
    } else {
        result += digits.substr(0, digits.size() - scale);
        result += '.';
        result += digits.substr(digits.size() - scale);
    }
    return result;
}

struct Totals {
    std::size_t intervals = 0;
    std::size_t exclusions = 0;
    std::size_t reversed = 0;
    std::size_t non_nested = 0;
    std::size_t exceptions = 0;
    std::map<std::string, std::size_t> exclusions_by_kind;
    std::map<std::string, std::size_t> exceptions_by_kind;
};

template<class Factory>
static void probe(const std::string& kind, std::size_t index, const Q& expected,
                  Factory factory, Totals& totals) {
    int stage = 0;
    try {
        Real result = factory();
        auto it = result.get_real_itr();
        Q old_lo{}, old_hi{};
        bool have_old = false;
        for (int p = 1; p <= 8; ++p) {
            stage = p;
            const auto interval = it.get_interval();
            const Q lo = decode(interval.lower_bound);
            const Q hi = decode(interval.upper_bound);
            ++totals.intervals;
            if (less(hi, lo)) ++totals.reversed;
            if (less(expected, lo) || less(hi, expected)) {
                if (totals.exclusions < 15) {
                    std::cout << "exclusion kind=" << kind << " case=" << index
                              << " p=" << p << " lower=" << brief(lo)
                              << " expected=" << brief(expected)
                              << " upper=" << brief(hi) << '\n';
                }
                ++totals.exclusions;
                ++totals.exclusions_by_kind[kind];
            }
            if (have_old && (less(lo, old_lo) || less(old_hi, hi))) {
                if (totals.non_nested < 8) {
                    std::cout << "non_nested kind=" << kind << " case=" << index
                              << " p=" << p << '\n';
                }
                ++totals.non_nested;
            }
            old_lo = lo;
            old_hi = hi;
            have_old = true;
            ++it;
        }
    } catch (const std::exception& error) {
        if (totals.exceptions < 15) {
            std::cout << "exception kind=" << kind << " case=" << index
                      << " stage=" << stage << " what='" << error.what() << "'\n";
        }
        ++totals.exceptions;
        ++totals.exceptions_by_kind[kind];
    }
}

static Big random_big(std::mt19937_64& rng, unsigned bits) {
    Big value = 0;
    for (unsigned done = 0; done < bits; done += 64) {
        value <<= 64;
        value += rng();
    }
    if (bits % 64) value &= (Big(1) << bits) - 1;
    if (rng() & 1) value = -value;
    return value;
}

int main() {
    std::vector<Decimal> values;
    const std::vector<std::pair<Big, unsigned>> special = {
        {1,1}, {-1,1}, {1,20}, {-1,20}, {5,1}, {-5,1},
        {1073741821,1}, {-1073741821,1}, {10737418215LL,1},
        {999999999999999999LL,18}, {-999999999999999999LL,18}
    };
    for (const auto& [n, scale] : special) {
        values.push_back({decimal_text(n, scale), canon(n, power(Big(10), scale))});
    }
    std::mt19937_64 rng(0x444543494d414c53ULL);
    while (values.size() < 90) {
        Big n = random_big(rng, 1 + rng() % 220);
        if (n == 0) n = 1;
        unsigned scale = 1 + rng() % 55;
        values.push_back({decimal_text(n, scale), canon(n, power(Big(10), scale))});
    }

    Totals totals;
    for (std::size_t i = 0; i < values.size(); ++i) {
        const Decimal a = values[i];
        const Decimal b = values[(i * 37 + 11) % values.size()];
        probe("value", i, a.value, [=] { return Real(a.text); }, totals);
        probe("add", i, add(a.value, b.value), [=] {
            Real x(a.text), y(b.text); return x + y;
        }, totals);
        probe("sub", i, sub(a.value, b.value), [=] {
            Real x(a.text), y(b.text); return x - y;
        }, totals);
        probe("mul", i, mul(a.value, b.value), [=] {
            Real x(a.text), y(b.text); return x * y;
        }, totals);
        if (b.value.n != 0) {
            probe("div", i, divq(a.value, b.value), [=] {
                Real x(a.text), y(b.text); return x / y;
            }, totals);
        }
    }
    for (std::size_t i = 0; i < 24; ++i) {
        const Decimal a = values[i];
        probe("alias_add", i, add(a.value,a.value), [=] {
            Real x(a.text); return x+x;
        }, totals);
        probe("copy_add", i, add(a.value,a.value), [=] {
            Real x(a.text), y=x; return x+y;
        }, totals);
        probe("distinct_add", i, add(a.value,a.value), [=] {
            Real x(a.text), y(a.text); return x+y;
        }, totals);
        probe("alias_sub", i, canon(0), [=] {
            Real x(a.text); return x-x;
        }, totals);
        probe("alias_mul", i, mul(a.value,a.value), [=] {
            Real x(a.text); return x*x;
        }, totals);
        probe("distinct_mul", i, mul(a.value,a.value), [=] {
            Real x(a.text), y(a.text); return x*y;
        }, totals);
        if (a.value.n != 0) {
            probe("alias_div", i, canon(1), [=] {
                Real x(a.text); return x/x;
            }, totals);
        }
        probe("shared_tree_cancel", i, canon(0), [=] {
            Real x(a.text); Real square=x*x; return square-square;
        }, totals);
    }
    std::cout << "summary values=" << values.size()
              << " intervals=" << totals.intervals
              << " exclusions=" << totals.exclusions
              << " reversed=" << totals.reversed
              << " non_nested=" << totals.non_nested
              << " exceptions=" << totals.exceptions << '\n';
    for (const char* kind : {"value", "add", "sub", "mul", "div",
                             "alias_add", "copy_add", "alias_sub", "alias_mul",
                             "distinct_add", "distinct_mul", "alias_div",
                             "shared_tree_cancel"}) {
        std::cout << kind << " exclusions=" << totals.exclusions_by_kind[kind]
                  << " exceptions=" << totals.exceptions_by_kind[kind] << '\n';
    }
    return totals.exclusions || totals.reversed || totals.non_nested || totals.exceptions;
}
