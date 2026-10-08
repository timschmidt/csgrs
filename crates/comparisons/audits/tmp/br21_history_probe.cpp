#include <real/real.hpp>
#include <iostream>
#include <vector>

using Real = boost::real::real<int>;
using Interval = boost::real::interval<int>;

static Real make_expression() {
    Real a("0.123456789012345678901234567890123456789");
    Real b("0.987654321098765432109876543210987654321");
    return (a * b) + (a / b);
}

static std::vector<Interval> trace(Real value, int count) {
    std::vector<Interval> result;
    auto it = value.get_real_itr();
    for (int i = 0; i < count; ++i) {
        result.push_back(it.get_interval());
        ++it;
    }
    return result;
}

int main() {
    constexpr int count = 7;
    const auto baseline = trace(make_expression(), count);

    Real warmed = make_expression();
    Real alias = warmed;
    auto first = warmed.get_real_itr();
    for (int i = 0; i < count; ++i) ++first;
    const auto replay = trace(alias, count);

    int mismatches = 0;
    for (int i = 0; i < count; ++i) {
        if (!(baseline[i] == replay[i])) {
            ++mismatches;
            std::cout << "history_mismatch precision=" << (i + 1)
                      << " baseline=" << baseline[i]
                      << " replay=" << replay[i] << '\n';
        }
    }

    Real cap_a = make_expression();
    Real cap_b = cap_a;
    cap_a.set_maximum_precision(73);
    std::cout << "history_mismatches=" << mismatches
              << " aliased_cap=" << cap_b.maximum_precision() << '\n';
}
