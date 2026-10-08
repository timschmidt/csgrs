#include <climits>
#include <iostream>

#include "/home/tim/Documents/GitHub/workspace/exact-real-references/spigot/bi_internal.h"

int main() {
    const int edge = INT_MIN;
    bigint constructed(edge);
    std::cout << bigint_decstring(constructed) << '\n';

    bigint added(1);
    added += edge;
    std::cout << bigint_decstring(added) << '\n';

    bigint multiplied(2);
    multiplied *= edge;
    std::cout << bigint_decstring(multiplied) << '\n';

    std::cout << (constructed == edge) << '\n';
}
