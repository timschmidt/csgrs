#include <real/real.hpp>

boost::real::real<> make_a();
boost::real::real<> make_b();

int main() {
    auto value = make_a() + make_b();
    return value > boost::real::real<>("3") ? 0 : 1;
}
