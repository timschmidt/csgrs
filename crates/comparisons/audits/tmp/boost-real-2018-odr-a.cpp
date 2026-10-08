#include <real/real.hpp>

unsigned int boost::real::real::maximum_precision = 10;
unsigned int boost::real::real_algorithm::maximum_precision = 10;

int odr_other();
int main() { return odr_other(); }
