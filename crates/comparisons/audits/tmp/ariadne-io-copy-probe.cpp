#include "ariadne.hpp"

using namespace Ariadne;

int main() {
    Figure original(ApproximateBoxType({{-1, 1}, {-1, 1}}), Projection2d(2, 0, 1));
    Figure copy = original;
    return copy.get_bounding_box().dimension() == 2 ? 0 : 1;
}
