#include <boost/multiprecision/cpp_int.hpp>
#include <cstdint>
#include <iostream>
#include <limits>
#include <random>
#include <string>

#include "/home/tim/Documents/GitHub/workspace/exact-real-references/spigot/bi_internal.h"

using boost::multiprecision::cpp_int;

static bigint to_spigot(cpp_int value) {
    bool negative = value < 0;
    if (negative)
        value = -value;
    bigint out(0U);
    const cpp_int mask = (cpp_int(1) << 32) - 1;
    std::vector<uint32_t> words;
    while (value != 0) {
        words.push_back(static_cast<uint32_t>((value & mask).convert_to<uint64_t>()));
        value >>= 32;
    }
    for (auto it = words.rbegin(); it != words.rend(); ++it) {
        out <<= 32;
        out += bigint(static_cast<unsigned>(*it));
    }
    return negative ? -out : out;
}

static std::string dec(const cpp_int &value) {
    return value.convert_to<std::string>();
}

[[noreturn]] static void fail(const char *op, unsigned case_no,
                              const cpp_int &a, const cpp_int &b,
                              const std::string &got,
                              const std::string &expected) {
    std::cerr << "case=" << case_no << " op=" << op
              << " a=" << a << " b=" << b
              << " got=" << got << " expected=" << expected << '\n';
    std::exit(1);
}

static void check(const char *op, unsigned case_no,
                  const cpp_int &a, const cpp_int &b,
                  const bigint &got, const cpp_int &expected) {
    const auto got_s = bigint_decstring(got);
    const auto expected_s = dec(expected);
    if (got_s != expected_s)
        fail(op, case_no, a, b, got_s, expected_s);
}

static cpp_int random_value(std::mt19937_64 &rng, unsigned words) {
    cpp_int value = 0;
    for (unsigned i = 0; i < words; ++i) {
        value <<= 64;
        value += rng();
    }
    if (rng() & 1)
        value = -value;
    return value;
}

static cpp_int floor_div(const cpp_int &a, const cpp_int &b) {
    cpp_int q = a / b;
    cpp_int r = a % b;
    if (r != 0 && ((a < 0) != (b < 0)))
        --q;
    return q;
}

static cpp_int integer_sqrt(cpp_int n) {
    if (n == 0)
        return 0;
    cpp_int lo = 0, hi = cpp_int(1) << ((boost::multiprecision::msb(n) + 2) / 2);
    while (lo + 1 < hi) {
        cpp_int mid = (lo + hi) / 2;
        if (mid * mid <= n)
            lo = mid;
        else
            hi = mid;
    }
    return lo;
}

int main() {
    std::mt19937_64 rng(0x535049474f54ULL);
    unsigned checks = 0;
    constexpr unsigned cases = 12000;
    for (unsigned i = 0; i < cases; ++i) {
        unsigned words = i < 11000 ? 1 + rng() % 8 : 16 + rng() % 49;
        cpp_int a = random_value(rng, words);
        cpp_int b = random_value(rng, 1 + rng() % words);
        if (b == 0)
            b = 1;
        bigint sa = to_spigot(a), sb = to_spigot(b);

        check("roundtrip-a", i, a, b, sa, a); ++checks;
        check("roundtrip-b", i, a, b, sb, b); ++checks;
        check("add", i, a, b, sa + sb, a + b); ++checks;
        check("sub", i, a, b, sa - sb, a - b); ++checks;
        check("mul", i, a, b, sa * sb, a * b); ++checks;
        check("div", i, a, b, sa / sb, a / b); ++checks;
        check("mod", i, a, b, sa % sb, a % b); ++checks;
        check("fdiv", i, a, b, fdiv(sa, sb), floor_div(a, b)); ++checks;

        unsigned shift = rng() % 257;
        check("shl", i, a, b, sa << shift, a << shift); ++checks;
        cpp_int divisor = cpp_int(1) << shift;
        cpp_int expected_shr = floor_div(a, divisor);
        if (!(a < 0 && expected_shr == -1)) {
            check("shr", i, a, b, sa >> shift, expected_shr); ++checks;
        }

        cpp_int magnitude = a < 0 ? -a : a;
        check("sqrt", i, a, b, bigint_sqrt(to_spigot(magnitude)),
              integer_sqrt(magnitude)); ++checks;

        unsigned exponent = rng() % 9;
        cpp_int small = static_cast<int>(rng() % 2001) - 1000;
        check("power", i, small, exponent,
              bigint_power(to_spigot(small), exponent),
              boost::multiprecision::pow(small, exponent)); ++checks;

        int expected_cmp = a < b ? -1 : a > b ? 1 : 0;
        int got_cmp = sa < sb ? -1 : sa > sb ? 1 : 0;
        if (got_cmp != expected_cmp)
            fail("compare", i, a, b, std::to_string(got_cmp),
                 std::to_string(expected_cmp));
        ++checks;

        if (magnitude != 0) {
            unsigned expected_log = boost::multiprecision::msb(magnitude);
            unsigned got_log = bigint_approxlog2(to_spigot(magnitude));
            if (got_log != expected_log)
                fail("approxlog2", i, a, b, std::to_string(got_log),
                     std::to_string(expected_log));
            ++checks;
        }
        unsigned bit = rng() % (words * 64 + 65);
        int expected_bit = static_cast<int>((magnitude >> bit) & 1);
        int got_bit = bigint_bit(to_spigot(magnitude), bit);
        if (got_bit != expected_bit)
            fail("bit", i, a, b, std::to_string(got_bit),
                 std::to_string(expected_bit));
        ++checks;
    }
    std::cout << "cases=" << cases << " checks=" << checks << "\n";
}
