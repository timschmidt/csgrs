#include <cstdlib>
#include <iostream>
#include <malloc.h>
#include <vector>

#include "numeric/dyadic.hpp"

using namespace Ariadne;

int main(int argc, char** argv) {
    std::size_t n = argc > 1 ? std::strtoull(argv[1], nullptr, 10) : 0;
    auto before = mallinfo2();
    std::vector<Dyadic> values;
    values.reserve(n);
    for (std::size_t i = 0; i < n; ++i) values.emplace_back(static_cast<unsigned>(i));
    auto after = mallinfo2();
    std::cout << values.size() << " heap_delta="
              << (after.uordblks - before.uordblks) << '\n';
}
