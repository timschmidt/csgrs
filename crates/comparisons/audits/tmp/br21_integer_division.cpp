#include <real/real_rational.hpp>

int main() {
    boost::real::integer_number<> a("7");
    boost::real::integer_number<> b("2");
    auto result = a / b;
    return result == boost::real::integer_number<>("3") ? 0 : 1;
}
