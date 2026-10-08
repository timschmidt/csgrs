#include <real/integer_number.hpp>
#include <cstdint>
#include <iostream>
#include <string>

using Integer = boost::real::integer_number<int>;

static std::int64_t decode(const Integer& value) {
    std::int64_t result = 0;
    for (int digit : value.digits) {
        if (digit < 0 || digit >= Integer::BASE) {
            throw std::runtime_error("invalid limb");
        }
        result = result * Integer::BASE + digit;
    }
    return value.positive ? result : -result;
}

static void report(const char* op, int x, int y, std::int64_t got,
                   std::int64_t expected, std::size_t& mismatches) {
    if (got != expected) {
        if (mismatches < 12) {
            std::cout << op << " x=" << x << " y=" << y
                      << " got=" << got << " expected=" << expected << '\n';
        }
        ++mismatches;
    }
}

int main(int argc, char** argv) {
    if (argc != 2) return 64;
    const std::string op = argv[1];
    std::size_t cases = 0;
    std::size_t mismatches = 0;
    std::size_t negative_zero = 0;
    for (int x = -50; x <= 50; ++x) {
        for (int y = -50; y <= 50; ++y) {
            if ((op == "mod" || op == "div") && y <= 0) continue;
            Integer a(std::to_string(x));
            Integer b(std::to_string(y));
            Integer got;
            std::int64_t expected = 0;
            if (op == "add") {
                got = a + b;
                expected = x + y;
            } else if (op == "sub") {
                got = a - b;
                expected = x - y;
            } else if (op == "mul") {
                got = a * b;
                expected = x * y;
            } else if (op == "mod") {
                got = a % b;
                expected = ((x % y) + y) % y;
            } else if (op == "div") {
                got = a.divide(b);
                expected = x / y;
            } else {
                return 65;
            }
            ++cases;
            const auto decoded = decode(got);
            report(op.c_str(), x, y, decoded, expected, mismatches);
            if (decoded == 0 && !got.positive) ++negative_zero;
        }
    }
    std::cout << "summary op=" << op << " cases=" << cases
              << " mismatches=" << mismatches
              << " negative_zero=" << negative_zero << '\n';
    return mismatches == 0 && negative_zero == 0 ? 0 : 1;
}
