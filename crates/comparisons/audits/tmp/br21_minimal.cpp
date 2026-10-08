#include <real/real.hpp>

int main() {
    boost::real::real<int> a("1.25"), b("2.5");
    auto c = a * b + a;
    auto it = c.get_real_itr();
    ++it;
    return it.get_interval().lower_bound.digits.empty();
}
