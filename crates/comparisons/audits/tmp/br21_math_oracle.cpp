#include <real/real.hpp>
#include <boost/multiprecision/cpp_int.hpp>
#include <gmp.h>
#include <mpfr.h>
#include <iostream>
#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>

using Big = boost::multiprecision::cpp_int;
using Real = boost::real::real<int>;
using Exact = boost::real::exact_number<int>;
struct Q { Big n; Big d; };
static const Big radix = (std::numeric_limits<int>::max() / 4) * 2;

static Big ipow(Big base, unsigned exponent) {
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
static bool less(const Q& a, const Q& b) { return a.n*b.d < b.n*a.d; }
static Q decode(const Exact& value) {
    Big mantissa = 0;
    for (int digit : value.digits) { mantissa *= radix; mantissa += digit; }
    if (!value.positive) mantissa = -mantissa;
    const long scale = static_cast<long>(value.exponent)-
                       static_cast<long>(value.digits.size());
    return scale >= 0 ? canon(mantissa*ipow(radix, scale))
                      : canon(mantissa, ipow(radix, -scale));
}
static Q parse_decimal(std::string input) {
    bool negative = false;
    if (!input.empty() && input[0] == '-') { negative = true; input.erase(0,1); }
    const auto dot = input.find('.');
    unsigned scale = 0;
    if (dot != std::string::npos) {
        scale = static_cast<unsigned>(input.size()-dot-1);
        input.erase(dot,1);
    }
    Big numerator(input.empty() ? "0" : input);
    if (negative) numerator = -numerator;
    return canon(numerator, ipow(Big(10), scale));
}
static void set_q(mpfr_t out, const Q& value, mpfr_rnd_t rounding) {
    mpq_t rational;
    mpq_init(rational);
    const std::string encoded = value.n.convert_to<std::string>() + "/" +
                                value.d.convert_to<std::string>();
    if (mpq_set_str(rational, encoded.c_str(), 10) != 0) std::abort();
    mpq_canonicalize(rational);
    mpfr_set_q(out, rational, rounding);
    mpq_clear(rational);
}

using MpfrFn = int (*)(mpfr_ptr, mpfr_srcptr, mpfr_rnd_t);
struct Totals {
    std::size_t intervals = 0;
    std::size_t definite_exclusions = 0;
    std::size_t reversed = 0;
    std::size_t non_nested = 0;
    std::size_t exceptions = 0;
    std::map<std::string,std::size_t> exclusions_by_fn;
    std::map<std::string,std::size_t> exceptions_by_fn;
};

template<class Factory>
static void probe_mpfr(const std::string& name, std::size_t index, const Q& input,
                       MpfrFn function, bool increasing, Factory factory,
                       Totals& totals) {
    mpfr_t xlo, xhi, reflo, refhi, boostlo, boosthi;
    mpfr_inits2(2048, xlo, xhi, reflo, refhi, boostlo, boosthi, (mpfr_ptr)0);
    set_q(xlo, input, MPFR_RNDD);
    set_q(xhi, input, MPFR_RNDU);
    if (increasing) {
        function(reflo, xlo, MPFR_RNDD);
        function(refhi, xhi, MPFR_RNDU);
    } else {
        function(reflo, xhi, MPFR_RNDD);
        function(refhi, xlo, MPFR_RNDU);
    }
    try {
        Real result = factory();
        auto it = result.get_real_itr();
        Q oldlo{}, oldhi{};
        bool have_old = false;
        for (int p = 1; p <= 6; ++p) {
            const auto interval = it.get_interval();
            const Q lo = decode(interval.lower_bound);
            const Q hi = decode(interval.upper_bound);
            ++totals.intervals;
            if (less(hi, lo)) ++totals.reversed;
            set_q(boostlo, lo, MPFR_RNDD);
            set_q(boosthi, hi, MPFR_RNDU);
            if (mpfr_less_p(boosthi, reflo) || mpfr_greater_p(boostlo, refhi)) {
                if (totals.definite_exclusions < 18) {
                    std::cout << "definite exclusion fn=" << name << " case=" << index
                              << " p=" << p << '\n';
                }
                ++totals.definite_exclusions;
                ++totals.exclusions_by_fn[name];
            }
            if (have_old && (less(lo, oldlo) || less(oldhi, hi))) {
                ++totals.non_nested;
            }
            oldlo = lo; oldhi = hi; have_old = true;
            ++it;
        }
    } catch (const std::exception& error) {
        if (totals.exceptions < 12) {
            std::cout << "exception fn=" << name << " case=" << index
                      << " what='" << error.what() << "'\n";
        }
        ++totals.exceptions;
        ++totals.exceptions_by_fn[name];
    }
    mpfr_clears(xlo, xhi, reflo, refhi, boostlo, boosthi, (mpfr_ptr)0);
}

template<class Factory>
static void probe_exact(const std::string& name, std::size_t index,
                        const Q& expected, Factory factory, Totals& totals) {
    try {
        Real result = factory();
        auto it = result.get_real_itr();
        Q oldlo{}, oldhi{}; bool have_old = false;
        for (int p=1; p<=6; ++p) {
            const auto interval = it.get_interval();
            const Q lo=decode(interval.lower_bound), hi=decode(interval.upper_bound);
            ++totals.intervals;
            if (less(hi,lo)) ++totals.reversed;
            if (less(expected,lo) || less(hi,expected)) {
                if (totals.definite_exclusions < 18)
                    std::cout << "exact exclusion fn=" << name << " case=" << index << " p=" << p << '\n';
                ++totals.definite_exclusions; ++totals.exclusions_by_fn[name];
            }
            if (have_old && (less(lo,oldlo) || less(oldhi,hi))) ++totals.non_nested;
            oldlo=lo; oldhi=hi; have_old=true; ++it;
        }
    } catch (const std::exception& error) {
        if (totals.exceptions < 12)
            std::cout << "exception fn=" << name << " case=" << index << " what='" << error.what() << "'\n";
        ++totals.exceptions; ++totals.exceptions_by_fn[name];
    }
}

int main(int argc, char** argv) {
    Totals totals;
    const std::string only = argc > 1 ? argv[1] : "all";
    const long only_case = argc > 2 ? std::stol(argv[2]) : -1;
    const auto run = [&](const char* name) { return only == "all" || only == name; };
    const auto run_case = [&](std::size_t i) { return only_case < 0 || static_cast<long>(i) == only_case; };
    const std::vector<std::string> exp_inputs={"-2","-0.5","0","0.1","1","2"};
    const std::vector<std::string> log_inputs={"0.1","0.5","1","2","10"};
    const std::vector<std::string> sqrt_inputs={"0","0.1","1","2","4","10"};
    const std::vector<std::string> symmetric={"-0.5","-0.1","0","0.1","0.5"};
    const std::vector<std::string> cos_inputs={"0","0.1","0.5"};
    const std::vector<std::string> inverse_inputs={"-1","-0.5","-0.1","0","0.1","0.5","1"};
    const std::vector<std::string> atan_inputs={"-2","-0.5","0","0.5","2"};
    std::size_t index=0;
    if (run("exp")) for (const auto& s:exp_inputs) probe_mpfr("exp",index++,parse_decimal(s),mpfr_exp,true,[=]{return Real::exp(Real(s));},totals);
    index=0; if (run("log")) for (const auto& s:log_inputs) probe_mpfr("log",index++,parse_decimal(s),mpfr_log,true,[=]{return Real::log(Real(s));},totals);
    index=0; if (run("sqrt")) for (const auto& s:sqrt_inputs) probe_mpfr("sqrt",index++,parse_decimal(s),mpfr_sqrt,true,[=]{return Real::sqrt(Real(s));},totals);
    index=0; if (run("sin")) for (const auto& s:symmetric) probe_mpfr("sin",index++,parse_decimal(s),mpfr_sin,true,[=]{return Real::sin(Real(s));},totals);
    index=0; if (run("cos")) for (const auto& s:cos_inputs) probe_mpfr("cos",index++,parse_decimal(s),mpfr_cos,false,[=]{return Real::cos(Real(s));},totals);
    index=0; if (run("tan")) for (const auto& s:symmetric) probe_mpfr("tan",index++,parse_decimal(s),mpfr_tan,true,[=]{return Real::tan(Real(s));},totals);
    index=0; if (run("asin")) for (const auto& s:inverse_inputs) { if (run_case(index)) probe_mpfr("asin",index,parse_decimal(s),mpfr_asin,true,[=]{return Real::asin(Real(s));},totals); ++index; }
    index=0; if (run("acos")) for (const auto& s:inverse_inputs) { if (run_case(index)) probe_mpfr("acos",index,parse_decimal(s),mpfr_acos,false,[=]{return Real::acos(Real(s));},totals); ++index; }
    index=0; if (run("atan")) for (const auto& s:atan_inputs) { if (run_case(index)) probe_mpfr("atan",index,parse_decimal(s),mpfr_atan,true,[=]{return Real::atan(Real(s));},totals); ++index; }

    const std::vector<std::string> bases={"-2","-2.1","-0.5","0","2.1"};
    if (run("power")) for (std::size_t bi=0; bi<bases.size(); ++bi) {
        const Q base=parse_decimal(bases[bi]);
        for (unsigned exponent=0; exponent<=4; ++exponent) {
            Q expected=canon(ipow(base.n,exponent),ipow(base.d,exponent));
            probe_exact("power",bi*5+exponent,expected,[=]{return Real::power(Real(bases[bi]),Real(std::to_string(exponent)));},totals);
        }
    }

    bool floor_minus_two_wrong = false;
    Exact minus_two("-2");
    if (run("floor")) {
        minus_two.floor();
        floor_minus_two_wrong = !(minus_two.digits == std::vector<int>{2} &&
                                  minus_two.exponent == 1 && !minus_two.positive);
    }
    std::size_t rational_floor_failures=0, rational_floor_checks=0;
    if (run("floor")) for(int n=-30;n<=30;++n) for(int d=1;d<=12;++d) {
        boost::real::real_rational<> q(boost::real::integer_number<>(std::to_string(n)),
                                      boost::real::integer_number<>(std::to_string(d)));
        const auto got=boost::real::floor(q);
        Big decoded=0; for(int digit:got.digits){decoded*=decltype(got)::BASE;decoded+=digit;}
        if(!got.positive) decoded=-decoded;
        int expected=n/d; if(n<0 && n%d) --expected;
        ++rational_floor_checks; if(decoded!=expected) ++rational_floor_failures;
    }

    std::cout << "summary intervals=" << totals.intervals
              << " definite_exclusions=" << totals.definite_exclusions
              << " reversed=" << totals.reversed
              << " non_nested=" << totals.non_nested
              << " exceptions=" << totals.exceptions
              << " exact_floor_minus_two_wrong=" << floor_minus_two_wrong
              << " rational_floor_checks=" << rational_floor_checks
              << " rational_floor_failures=" << rational_floor_failures << '\n';
    for (const char* name:{"exp","log","sqrt","sin","cos","tan","asin","acos","atan","power"})
        std::cout << name << " exclusions=" << totals.exclusions_by_fn[name]
                  << " exceptions=" << totals.exceptions_by_fn[name] << '\n';
    return totals.definite_exclusions || totals.reversed || totals.non_nested ||
           totals.exceptions || floor_minus_two_wrong || rational_floor_failures;
}
