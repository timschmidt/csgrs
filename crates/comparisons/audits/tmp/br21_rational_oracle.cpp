#include <real/real.hpp>
#include <boost/multiprecision/cpp_int.hpp>
#include <iostream>
#include <limits>
#include <string>
#include <utility>
#include <vector>

using Big = boost::multiprecision::cpp_int;
using Integer = boost::real::integer_number<int>;
using RationalRep = boost::real::real_rational<int>;
using Real = boost::real::real<int>;
using Exact = boost::real::exact_number<int>;
struct Q { Big n; Big d; };
static const Big radix = (std::numeric_limits<int>::max() / 4) * 2;

static Big power(Big base, unsigned exponent) {
    Big out = 1;
    while (exponent--) out *= base;
    return out;
}
static Q canon(Big n, Big d = 1) {
    if (d < 0) { n = -n; d = -d; }
    return {std::move(n), std::move(d)};
}
static Q add(const Q& a, const Q& b) { return canon(a.n*b.d+b.n*a.d,a.d*b.d); }
static Q sub(const Q& a, const Q& b) { return canon(a.n*b.d-b.n*a.d,a.d*b.d); }
static Q mul(const Q& a, const Q& b) { return canon(a.n*b.n,a.d*b.d); }
static Q divq(const Q& a, const Q& b) { return canon(a.n*b.d,a.d*b.n); }
static bool less(const Q& a, const Q& b) { return a.n*b.d < b.n*a.d; }
static bool equal(const Q& a, const Q& b) {
    return a.n*b.d == b.n*a.d;
}
static Big decode_integer(const Integer& value) {
    Big out = 0;
    for (int digit : value.digits) { out *= Integer::BASE; out += digit; }
    return value.positive ? out : -out;
}
static Q decode(const RationalRep& value) {
    Big n = decode_integer(value.a);
    if (!value.positive) n = -n;
    return canon(n, decode_integer(value.b));
}
static Q decode(const Exact& value) {
    Big mantissa = 0;
    for (int digit : value.digits) { mantissa *= radix; mantissa += digit; }
    if (!value.positive) mantissa = -mantissa;
    const long scale = static_cast<long>(value.exponent)-
                       static_cast<long>(value.digits.size());
    return scale >= 0 ? canon(mantissa*power(radix, scale))
                      : canon(mantissa, power(radix, -scale));
}

static void check_rep(const char* op, int x, int xd, int y, int yd,
                      const RationalRep& got, const Q& expected,
                      std::size_t& failures) {
    if (!equal(decode(got), expected)) {
        if (failures < 12) {
            std::cout << "direct " << op << " " << x << '/' << xd << " and "
                      << y << '/' << yd << '\n';
        }
        ++failures;
    }
}

template<class Factory>
static void check_real(const std::string& op, std::size_t index,
                       const Q& expected, Factory factory,
                       std::size_t& intervals, std::size_t& exclusions,
                       std::size_t& exceptions) {
    try {
        Real result = factory();
        auto it = result.get_real_itr();
        for (int p = 1; p <= 6; ++p) {
            const auto interval = it.get_interval();
            const Q lo = decode(interval.lower_bound);
            const Q hi = decode(interval.upper_bound);
            ++intervals;
            if (less(expected, lo) || less(hi, expected)) {
                if (exclusions < 16) {
                    std::cout << "mixed exclusion op=" << op << " case=" << index
                              << " p=" << p << '\n';
                }
                ++exclusions;
            }
            ++it;
        }
    } catch (const std::exception& error) {
        if (exceptions < 10) {
            std::cout << "mixed exception op=" << op << " case=" << index
                      << " what='" << error.what() << "'\n";
        }
        ++exceptions;
    }
}

int main() {
    std::size_t direct_checks = 0;
    std::size_t direct_failures = 0;
    for (int x = -12; x <= 12; ++x) {
        for (int xd = 1; xd <= 9; ++xd) {
            for (int y = -12; y <= 12; ++y) {
                for (int yd = 1; yd <= 9; ++yd) {
                    RationalRep a(Integer(std::to_string(x)), Integer(std::to_string(xd)));
                    RationalRep b(Integer(std::to_string(y)), Integer(std::to_string(yd)));
                    const Q aq = canon(x, xd), bq = canon(y, yd);
                    check_rep("add",x,xd,y,yd,a+b,add(aq,bq),direct_failures); ++direct_checks;
                    check_rep("sub",x,xd,y,yd,a-b,sub(aq,bq),direct_failures); ++direct_checks;
                    check_rep("mul",x,xd,y,yd,a*b,mul(aq,bq),direct_failures); ++direct_checks;
                    if (y != 0) {
                        check_rep("div",x,xd,y,yd,a/b,divq(aq,bq),direct_failures);
                        ++direct_checks;
                    }
                }
            }
        }
    }

    const std::vector<std::pair<std::string,Q>> rationals = {
        {"-1/2", canon(-1,2)}, {"-7/3", canon(-7,3)},
        {"-1073741823/5", canon(-1073741823,5)},
        {"1/2", canon(1,2)}, {"7/3", canon(7,3)}
    };
    const std::vector<std::pair<std::string,Q>> explicits = {
        {"2",canon(2)}, {"-2",canon(-2)}, {"0.25",canon(1,4)}
    };
    std::size_t intervals = 0, exclusions = 0, exceptions = 0, index = 0;
    std::size_t comparison_checks = 0, comparison_failures = 0;
    for (const auto& [rs, rq] : rationals) {
        for (const auto& [es, eq] : explicits) {
            check_real("r+e",index,add(rq,eq),[=]{Real r(rs,"rational"),e(es);return r+e;},intervals,exclusions,exceptions);
            check_real("e+r",index,add(eq,rq),[=]{Real r(rs,"rational"),e(es);return e+r;},intervals,exclusions,exceptions);
            check_real("r-e",index,sub(rq,eq),[=]{Real r(rs,"rational"),e(es);return r-e;},intervals,exclusions,exceptions);
            check_real("e-r",index,sub(eq,rq),[=]{Real r(rs,"rational"),e(es);return e-r;},intervals,exclusions,exceptions);
            check_real("r*e",index,mul(rq,eq),[=]{Real r(rs,"rational"),e(es);return r*e;},intervals,exclusions,exceptions);
            check_real("e*r",index,mul(eq,rq),[=]{Real r(rs,"rational"),e(es);return e*r;},intervals,exclusions,exceptions);
            if (eq.n != 0) check_real("r/e",index,divq(rq,eq),[=]{Real r(rs,"rational"),e(es);return r/e;},intervals,exclusions,exceptions);
            if (rq.n != 0) check_real("e/r",index,divq(eq,rq),[=]{Real r(rs,"rational"),e(es);return e/r;},intervals,exclusions,exceptions);
            try {
                Real r(rs,"rational"), e(es);
                const bool expected_lt = less(rq, eq);
                const bool got_lt = r < e;
                ++comparison_checks;
                if (got_lt != expected_lt) ++comparison_failures;
            } catch (...) { ++comparison_failures; ++comparison_checks; }
            ++index;
        }
    }
    std::cout << "summary direct_checks=" << direct_checks
              << " direct_failures=" << direct_failures
              << " mixed_intervals=" << intervals
              << " mixed_exclusions=" << exclusions
              << " mixed_exceptions=" << exceptions
              << " comparison_checks=" << comparison_checks
              << " comparison_failures=" << comparison_failures << '\n';
    return direct_failures || exclusions || exceptions || comparison_failures;
}
