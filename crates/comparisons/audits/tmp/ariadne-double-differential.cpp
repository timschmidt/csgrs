#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <string_view>
#include <vector>

#include <mpfr.h>

#include "numeric/builtin.hpp"
#include "numeric/double.hpp"
#include "numeric/rounding.hpp"

using namespace Ariadne;

using AriadneFn = double (*)(double);
using MpfrFn = int (*)(mpfr_ptr, mpfr_srcptr, mpfr_rnd_t);

struct KernelCase {
    std::string_view name;
    AriadneFn ariadne;
    MpfrFn mpfr;
    double lo;
    double hi;
};

static double reference(MpfrFn fn, double x, mpfr_rnd_t rnd) {
    mpfr_t in, out;
    mpfr_init2(in, 53);
    mpfr_init2(out, 53);
    mpfr_set_d(in, x, MPFR_RNDN);
    fn(out, in, rnd);
    double result = mpfr_get_d(out, rnd);
    mpfr_clear(out);
    mpfr_clear(in);
    return result;
}

static void report(std::string_view name, char const* direction, double x,
                   double got, double expected, std::uint64_t& shown) {
    if (shown++ < 5) {
        std::cout << name << ' ' << direction << " x=" << std::setprecision(17)
                  << x << " got=" << got << " reference=" << expected << '\n';
    }
}

int main() {
    std::cout << std::unitbuf;
    std::vector<KernelCase> cases{
        {"sqrt", sqrt_rnd, mpfr_sqrt, 0.0, 1.0e300},
        {"exp", exp_rnd, mpfr_exp, -700.0, 700.0},
        {"log", log_rnd, mpfr_log, std::numeric_limits<double>::denorm_min(), 1.0e300},
        {"sin", sin_rnd, mpfr_sin, -1.0e6, 1.0e6},
        {"cos", cos_rnd, mpfr_cos, -1.0e6, 1.0e6},
        {"tan", tan_rnd, mpfr_tan, -1.0e6, 1.0e6},
        {"asin", asin_rnd, mpfr_asin, -1.0, 1.0},
        {"acos", acos_rnd, mpfr_acos, -1.0, 1.0},
        {"atan", atan_rnd, mpfr_atan, -1.0e6, 1.0e6},
    };

    std::mt19937_64 rng(0xA71AD9EULL);
    for (auto const& c : cases) {
        std::uniform_real_distribution<double> dist(c.lo, c.hi);
        std::uint64_t down_bad = 0, up_bad = 0, down_shown = 0, up_shown = 0;
        for (std::uint64_t i = 0; i < 200000; ++i) {
            double x = dist(rng);
            if (c.name == "sqrt" || c.name == "log") {
                int exponent = static_cast<int>(rng() % 2098) - 1074;
                x = std::ldexp(0.5 + std::generate_canonical<double, 53>(rng), exponent);
                if (!std::isfinite(x) || x <= 0.0) x = std::numeric_limits<double>::denorm_min();
            }

            set_builtin_rounding_downward();
            double ref_down = reference(c.mpfr, x, MPFR_RNDD);
            double got_down = std::numeric_limits<double>::quiet_NaN();
            bool down_threw = false;
            try { got_down = c.ariadne(x); }
            catch (std::exception const& e) {
                down_threw = true;
                if (down_shown++ < 5) std::cout << c.name << " down x=" << x << " threw=" << e.what() << '\n';
            }
            if (down_threw || (std::isnan(ref_down) ? !std::isnan(got_down) : !(got_down <= ref_down))) {
                ++down_bad;
                if (!down_threw) report(c.name, "down", x, got_down, ref_down, down_shown);
            }

            set_builtin_rounding_upward();
            double ref_up = reference(c.mpfr, x, MPFR_RNDU);
            double got_up = std::numeric_limits<double>::quiet_NaN();
            bool up_threw = false;
            try { got_up = c.ariadne(x); }
            catch (std::exception const& e) {
                up_threw = true;
                if (up_shown++ < 5) std::cout << c.name << " up x=" << x << " threw=" << e.what() << '\n';
            }
            if (up_threw || (std::isnan(ref_up) ? !std::isnan(got_up) : !(got_up >= ref_up))) {
                ++up_bad;
                if (!up_threw) report(c.name, "up", x, got_up, ref_up, up_shown);
            }
        }
        set_builtin_rounding_to_nearest();
        std::cout << c.name << " totals down_bad=" << down_bad
                  << " up_bad=" << up_bad << '\n';
    }

    for (double x : {0.0, 1.0e20, -1.0e20, 1.0e100, -1.0e100}) {
        try {
            set_builtin_rounding_downward();
            double sd = sin_rnd(x);
            double cd = cos_rnd(x);
            set_builtin_rounding_upward();
            double su = sin_rnd(x);
            double cu = cos_rnd(x);
            set_builtin_rounding_to_nearest();
            std::cout << "wide x=" << std::setprecision(17) << x
                      << " sin=[" << sd << ',' << su << "]"
                      << " ref=[" << reference(mpfr_sin, x, MPFR_RNDD) << ','
                      << reference(mpfr_sin, x, MPFR_RNDU) << "]"
                      << " cos=[" << cd << ',' << cu << "]"
                      << " ref=[" << reference(mpfr_cos, x, MPFR_RNDD) << ','
                      << reference(mpfr_cos, x, MPFR_RNDU) << "]\n";
        } catch (std::exception const& e) {
            std::cout << "wide x=" << x << " threw=" << e.what() << '\n';
        }
    }

    set_builtin_rounding_downward();
    double ld = log_rnd(0.0);
    set_builtin_rounding_upward();
    double lu = log_rnd(0.0);
    set_builtin_rounding_to_nearest();
    std::cout << "log_zero=[" << ld << ',' << lu << "]\n";
}
