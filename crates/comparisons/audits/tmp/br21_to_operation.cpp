#include <real/real.hpp>

int main() {
    boost::real::real<> value("-1/2", "rational");
    value.to_operation();
}
