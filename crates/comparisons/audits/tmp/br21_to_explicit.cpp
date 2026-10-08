#include <real/real.hpp>

int main() {
    boost::real::real<> value("2", "integer");
    value.to_explicit();
}
