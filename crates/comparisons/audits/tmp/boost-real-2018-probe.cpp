#include <real/real.hpp>
#include <real/real_algorithm.hpp>
#include <real/real_explicit.hpp>

#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <typeinfo>

unsigned int boost::real::real::maximum_precision = 10;
unsigned int boost::real::real_algorithm::maximum_precision = 10;

namespace {

int threes(unsigned int) { return 3; }

unsigned int calls_a = 0;
unsigned int calls_b = 0;
int counted_a(unsigned int) { ++calls_a; return 3; }
int counted_b(unsigned int) { ++calls_b; return 3; }

std::string decimal_hundredths(int n) {
    if (n == 0) return "0.00";
    std::ostringstream out;
    if (n < 0) {
        out << '-';
        n = -n;
    }
    out << (n / 100) << '.' << std::setw(2) << std::setfill('0') << (n % 100);
    return out.str();
}

long long scaled_10000(const boost::real::boundary& b) {
    long long value = 0;
    for (int digit : b.digits) value = value * 10 + digit;
    int shift = b.exponent - static_cast<int>(b.digits.size()) + 4;
    if (shift < 0) throw std::runtime_error("unexpected precision beyond 1e-4");
    while (shift-- > 0) value *= 10;
    return b.positive ? value : -value;
}

long long scaled_100000000(const boost::real::boundary& b) {
    long long value = 0;
    for (int digit : b.digits) value = value * 10 + digit;
    int shift = b.exponent - static_cast<int>(b.digits.size()) + 8;
    if (shift < 0) throw std::runtime_error("unexpected precision beyond 1e-8");
    while (shift-- > 0) value *= 10;
    return b.positive ? value : -value;
}

boost::real::boundary exact_boundary(const boost::real::real& value) {
    boost::real::interval i = value.cend().approximation_interval;
    if (!i.is_a_number()) throw std::runtime_error("finite expression did not collapse");
    return i.lower_bound;
}

void print_explicit(const std::string& input) {
    try {
        boost::real::real_explicit x(input);
        std::cout << std::quoted(input) << " accepted exponent=" << x.exponent()
                  << " positive=" << x.positive() << " digits=";
        for (int d : x.digits()) std::cout << d << ',';
        std::cout << '\n';
    } catch (const std::exception& e) {
        std::cout << std::quoted(input) << " threw " << typeid(e).name()
                  << ": " << e.what() << '\n';
    }
}

} // namespace

int main(int argc, char** argv) {
    const std::string mode = argc > 1 ? argv[1] : "";

    if (mode == "batch") {
        boost::real::real_explicit source("1.23456");
        auto it = source.cbegin();
        it.iterate_n_times(2);
        boost::real::boundary truth = source.cend().approximation_interval.lower_bound;
        bool contains = !(truth < it.approximation_interval.lower_bound)
                     && !(it.approximation_interval.upper_bound < truth);
        std::cout << "jump2=" << it.approximation_interval
                  << " truth=" << truth.as_string()
                  << " contains=" << contains << '\n';
        return contains ? 0 : 2;
    }

    if (mode == "invalid") {
        for (const std::string& s : {"1a2", "1e2", "+", "-", ".", "-.", "--1", " 1", "-0"}) {
            print_explicit(s);
        }
        boost::real::real_explicit raw({1, 12, -4}, 1, true);
        std::cout << "raw digits=";
        for (int d : raw.digits()) std::cout << d << ',';
        std::cout << " interval=" << raw.cend().approximation_interval << '\n';
        return 0;
    }

    if (mode == "negative-zero") {
        boost::real::real minus_zero("-0");
        boost::real::real plus_zero("0");
        std::cout << "minus=" << minus_zero.cend().approximation_interval
                  << " plus=" << plus_zero.cend().approximation_interval
                  << " minus_lt_plus=" << (minus_zero < plus_zero)
                  << " equal=" << (minus_zero == plus_zero) << '\n';
        return 0;
    }

    if (mode == "terminal-nine") {
        boost::real::real_explicit explicit_nine("0.9");
        boost::real::interval explicit_end = explicit_nine.cend().approximation_interval;
        boost::real::real nine("0.9");
        std::cout << "explicit_end=" << explicit_end
                  << " exact=" << explicit_end.is_a_number();
        try {
            std::cout << " self_equal=" << (nine == nine);
        } catch (const boost::real::precision_exception&) {
            std::cout << " self_equal=precision_exception";
        }
        std::cout << '\n';
        return 0;
    }

    if (mode == "max") {
        boost::real::real_algorithm direct(threes, 0);
        direct.set_maximum_precision(2);
        std::cout << "algorithm max=" << direct.max_precision()
                  << " end_digits=" << direct.cend().approximation_interval.lower_bound.digits.size() << '\n';

        boost::real::real a(counted_a, 0);
        boost::real::real b(counted_b, 0);
        a.set_maximum_precision(1);
        b.set_maximum_precision(4);
        try {
            std::cout << (a == b) << '\n';
        } catch (const boost::real::precision_exception&) {
            std::cout << "comparison precision_exception" << '\n';
        }
        std::cout << "caps=" << a.max_precision() << ',' << b.max_precision()
                  << " calls=" << calls_a << ',' << calls_b << '\n';
        return 0;
    }

    if (mode == "copy-precision") {
        boost::real::real source("1.2345");
        source.set_maximum_precision(2);
        boost::real::real copied(source);
        boost::real::real assigned("9");
        assigned = source;
        std::cout << "source=" << source.max_precision()
                  << " copied=" << copied.max_precision()
                  << " assigned=" << assigned.max_precision() << '\n';
        return 0;
    }

    if (mode == "self-compound") {
        boost::real::real x = boost::real::real("1") + boost::real::real("2");
        x += x;
        std::cout << "observed=" << x.cend().approximation_interval << " expected=6\n";
        return 0;
    }

    if (mode == "assign-algorithm") {
        boost::real::real target("7");
        boost::real::real source(threes, 0);
        target = source;
        std::cout << target.cbegin().approximation_interval << '\n';
        return 0;
    }

    if (mode == "default-explicit") {
        boost::real::real_explicit x;
        std::cout << x.cbegin().approximation_interval << '\n';
        return 0;
    }

    if (mode == "empty-list") {
        boost::real::real x(std::initializer_list<int>{});
        std::cout << x.cbegin().approximation_interval << '\n';
        return 0;
    }

    if (mode == "iterator-leak") {
        volatile int sink = 0;
        for (int i = 0; i < 100; ++i) {
            boost::real::real x = boost::real::real("1.2") + boost::real::real("3.4");
            auto it = x.cbegin();
            sink += static_cast<int>(it.approximation_interval.lower_bound.digits.size());
        }
        std::cout << "sink=" << sink << '\n';
        return 0;
    }

    if (mode == "assignment-leak") {
        boost::real::real one("1");
        boost::real::real two("2");
        boost::real::real x = one + two;
        boost::real::real y = two + one;
        for (int i = 0; i < 100; ++i) x = y;
        std::cout << x.cend().approximation_interval << '\n';
        return 0;
    }

    if (mode == "exhaustive") {
        unsigned long long checked = 0;
        unsigned long long noncollapsed = 0;
        unsigned long long not_contained = 0;
        unsigned long long wrong_exact = 0;
        unsigned long long noncollapsed_by_op[3] = {};
        unsigned long long not_contained_by_op[3] = {};
        unsigned long long wrong_exact_by_op[3] = {};
        for (int lhs = -120; lhs <= 120; ++lhs) {
            for (int rhs = -120; rhs <= 120; ++rhs) {
                boost::real::real a(decimal_hundredths(lhs));
                boost::real::real b(decimal_hundredths(rhs));
                boost::real::real sum_value = a + b;
                boost::real::real difference_value = a - b;
                boost::real::real product_value = a * b;
                boost::real::interval sum_interval = sum_value.cend().approximation_interval;
                boost::real::interval difference_interval = difference_value.cend().approximation_interval;
                boost::real::interval product_interval = product_value.cend().approximation_interval;
                const long long expected[3] = {
                    static_cast<long long>(lhs + rhs) * 100,
                    static_cast<long long>(lhs - rhs) * 100,
                    static_cast<long long>(lhs) * rhs
                };
                const boost::real::interval intervals[3] = {
                    sum_interval, difference_interval, product_interval
                };
                for (int op = 0; op < 3; ++op) {
                    long long lower = scaled_10000(intervals[op].lower_bound);
                    long long upper = scaled_10000(intervals[op].upper_bound);
                    if (expected[op] < lower || expected[op] > upper) {
                        if (not_contained < 12) {
                            std::cerr << "not-contained op=" << op << ' ' << lhs << ' ' << rhs
                                      << " expected=" << expected[op]
                                      << " interval=" << intervals[op] << '\n';
                        }
                        ++not_contained;
                        ++not_contained_by_op[op];
                    }
                    if (!intervals[op].is_a_number()) {
                        ++noncollapsed;
                        ++noncollapsed_by_op[op];
                    } else if (lower != expected[op]) {
                        if (wrong_exact < 12) {
                            std::cerr << "wrong exact op=" << op << ' ' << lhs << ' ' << rhs
                                      << " expected=" << expected[op]
                                      << " observed=" << lower << '\n';
                        }
                        ++wrong_exact;
                        ++wrong_exact_by_op[op];
                    }
                }
                checked += 3;
            }
        }
        std::cout << "arithmetic enclosure checks=" << checked
                  << " noncollapsed=" << noncollapsed
                  << " not_contained=" << not_contained
                  << " wrong_exact=" << wrong_exact
                  << " by_op_noncollapsed=" << noncollapsed_by_op[0] << ','
                  << noncollapsed_by_op[1] << ',' << noncollapsed_by_op[2]
                  << " by_op_not_contained=" << not_contained_by_op[0] << ','
                  << not_contained_by_op[1] << ',' << not_contained_by_op[2]
                  << " by_op_wrong_exact=" << wrong_exact_by_op[0] << ','
                  << wrong_exact_by_op[1] << ',' << wrong_exact_by_op[2] << '\n';
        return not_contained == 0 && wrong_exact == 0 ? 0 : 3;
    }


    if (mode == "prefix-exhaustive") {
        unsigned long long checked = 0;
        unsigned long long not_contained = 0;
        unsigned long long not_contained_by_op[3] = {};
        unsigned long long not_contained_by_step[5] = {};
        for (int lhs = -120; lhs <= 120; ++lhs) {
            for (int rhs = -120; rhs <= 120; ++rhs) {
                boost::real::real a(decimal_hundredths(lhs));
                boost::real::real b(decimal_hundredths(rhs));
                boost::real::real values[3] = {a + b, a - b, a * b};
                const long long expected[3] = {
                    static_cast<long long>(lhs + rhs) * 1000000,
                    static_cast<long long>(lhs - rhs) * 1000000,
                    static_cast<long long>(lhs) * rhs * 10000
                };
                for (int op = 0; op < 3; ++op) {
                    auto it = values[op].cbegin();
                    for (int step = 0; step < 5; ++step) {
                        const long long lower = scaled_100000000(it.approximation_interval.lower_bound);
                        const long long upper = scaled_100000000(it.approximation_interval.upper_bound);
                        if (expected[op] < lower || expected[op] > upper) {
                            if (not_contained < 16) {
                                std::cerr << "prefix not-contained op=" << op
                                          << " step=" << step << ' ' << lhs << ' ' << rhs
                                          << " expected=" << expected[op]
                                          << " interval=" << it.approximation_interval << '\n';
                            }
                            ++not_contained;
                            ++not_contained_by_op[op];
                            ++not_contained_by_step[step];
                        }
                        ++checked;
                        if (step != 4) ++it;
                    }
                }
            }
        }
        std::cout << "prefix enclosure checks=" << checked
                  << " not_contained=" << not_contained
                  << " by_op=" << not_contained_by_op[0] << ','
                  << not_contained_by_op[1] << ',' << not_contained_by_op[2]
                  << " by_step=" << not_contained_by_step[0] << ','
                  << not_contained_by_step[1] << ',' << not_contained_by_step[2] << ','
                  << not_contained_by_step[3] << ',' << not_contained_by_step[4] << '\n';
        return not_contained == 0 ? 0 : 4;
    }

    std::cerr << "unknown mode\n";
    return 64;
}
