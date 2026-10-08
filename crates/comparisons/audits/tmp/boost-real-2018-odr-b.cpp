#include <real/real.hpp>

int odr_other() {
    boost::real::real x("1");
    return x[0] == 1 ? 0 : 1;
}
