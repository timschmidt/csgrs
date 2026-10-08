#include <real/integer_number.hpp>
#include <boost/multiprecision/cpp_int.hpp>
#include <cstdint>
#include <iostream>
#include <map>
#include <random>
#include <string>

using Big = boost::multiprecision::cpp_int;
using Integer = boost::real::integer_number<int>;

static Big decode(const Integer& value) {
    Big result = 0;
    for (int digit : value.digits) {
        if (digit < 0 || digit >= Integer::BASE) {
            throw std::runtime_error("invalid limb");
        }
        result *= Integer::BASE;
        result += digit;
    }
    return value.positive ? result : -result;
}

static Big random_big(std::mt19937_64& rng, unsigned bits) {
    Big value = 0;
    for (unsigned produced = 0; produced < bits; produced += 64) {
        value <<= 64;
        value += rng();
    }
    if (bits % 64) value &= (Big(1) << bits) - 1;
    return value;
}

static std::string text(const Big& value) {
    return value.convert_to<std::string>();
}

static void check(const char* op, std::size_t index, const Integer& got,
                  const Big& expected, std::size_t& mismatches,
                  std::size_t& negative_zero,
                  std::map<std::string, std::size_t>& mismatches_by_op,
                  std::map<std::string, std::size_t>& negative_zero_by_op) {
    const Big decoded = decode(got);
    if (decoded == 0 && !got.positive) {
        ++negative_zero;
        ++negative_zero_by_op[op];
    }
    if (decoded != expected) {
        if (mismatches < 8) {
            std::cout << op << " case=" << index << " got=" << decoded
                      << " expected=" << expected << '\n';
        }
        ++mismatches;
        ++mismatches_by_op[op];
    }
}

int main() {
    std::mt19937_64 rng(0x42524f5354524541ULL);
    std::size_t mismatches = 0;
    std::size_t negative_zero = 0;
    std::map<std::string, std::size_t> mismatches_by_op;
    std::map<std::string, std::size_t> negative_zero_by_op;
    constexpr std::size_t cases = 500;
    for (std::size_t i = 0; i < cases; ++i) {
        const unsigned xbits = 1 + rng() % 512;
        const unsigned ybits = 1 + rng() % 512;
        Big x = random_big(rng, xbits);
        Big y = random_big(rng, ybits);
        if (rng() & 1) x = -x;
        if (rng() & 1) y = -y;
        if (y == 0) y = 1;

        Integer a(text(x));
        Integer b(text(y));
        check("add", i, a + b, x + y, mismatches, negative_zero,
              mismatches_by_op, negative_zero_by_op);
        check("sub", i, a - b, x - y, mismatches, negative_zero,
              mismatches_by_op, negative_zero_by_op);
        check("mul", i, a * b, x * y, mismatches, negative_zero,
              mismatches_by_op, negative_zero_by_op);

        Integer positive_b(text(y < 0 ? -y : y));
        const Big divisor = y < 0 ? -y : y;
        Integer dividend_for_div = a;
        check("div", i, dividend_for_div.divide(positive_b), x / divisor,
              mismatches, negative_zero, mismatches_by_op, negative_zero_by_op);

        Integer dividend_for_mod = a;
        Integer divisor_for_mod = positive_b;
        Big expected_mod = x % divisor;
        if (expected_mod < 0) expected_mod += divisor;
        check("mod", i, dividend_for_mod % divisor_for_mod, expected_mod,
              mismatches, negative_zero, mismatches_by_op, negative_zero_by_op);
    }
    std::cout << "summary cases=" << cases << " operation_checks=" << cases * 5
              << " mismatches=" << mismatches
              << " negative_zero=" << negative_zero << '\n';
    for (const char* op : {"add", "sub", "mul", "div", "mod"}) {
        std::cout << op << " mismatches=" << mismatches_by_op[op]
                  << " negative_zero=" << negative_zero_by_op[op] << '\n';
    }
    return mismatches == 0 && negative_zero == 0 ? 0 : 1;
}
