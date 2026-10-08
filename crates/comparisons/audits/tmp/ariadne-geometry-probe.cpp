#include <cstdlib>
#include <iostream>
#include <string>

#include "geometry/binary_tree.hpp"
#include "geometry/grid_paving.hpp"
#include "geometry/union_of_intervals.hpp"
#include "function/projection.hpp"

using namespace Ariadne;

int main(int argc, char** argv) {
    if (argc != 2) {
        return 2;
    }
    const std::string mode(argv[1]);

    if (mode == "set") {
        BinaryTreeNode node(false);
        node.split();
        std::cout << "before leaf=" << node.is_leaf() << '\n';
        node.set(true);
        std::cout << "after leaf=" << node.is_leaf()
                  << " left=" << static_cast<const void*>(node.left_node())
                  << " right=" << static_cast<const void*>(node.right_node()) << '\n';
        std::cout.flush();
        std::_Exit(0);
    }

    if (mode == "set_destruct") {
        BinaryTreeNode node(false);
        node.split();
        node.set(true);
        return 0;
    }

    if (mode == "self_assign") {
        BinaryTreeNode node(false);
        node.split();
        node.left_node()->set_enabled();
        node = node;
        std::cout << node << '\n';
        return 0;
    }

    if (mode == "clone") {
        Grid grid(1);
        GridTreePaving paving(grid, 0u, BinaryWord("100"), BinaryWord("01"));
        GridTreeSubpaving disabled_branch = paving.branch(false);
        GridTreeSubpaving* copy = disabled_branch.clone();
        std::cout << "source_size=" << disabled_branch.size()
                  << " clone_size=" << copy->size()
                  << " source_cell=" << disabled_branch.root_cell()
                  << " clone_cell=" << copy->root_cell() << '\n';
        delete copy;
        return 0;
    }

    if (mode == "union") {
        UnionOfIntervals<Dyadic> lhs({Interval<Dyadic>(Dyadic(2), Dyadic(3))});
        UnionOfIntervals<Dyadic> rhs({Interval<Dyadic>(Dyadic(0), Dyadic(1)),
                                      Interval<Dyadic>(Dyadic(2), Dyadic(4))});
        std::cout << "intersection=" << intersection(lhs, rhs)
                  << " intersects=" << intersect(lhs, rhs) << '\n';
        UnionOfIntervals<Dyadic> empty(List<Interval<Dyadic>>{});
        std::cout << "empty_size=" << empty.size() << '\n';
        return 0;
    }

    if (mode == "empty_measure") {
        UnionOfIntervals<Dyadic> empty(List<Interval<Dyadic>>{});
        std::cout << measure(empty) << '\n';
        return 0;
    }

    if (mode == "projection") {
        Grid grid(2);
        GridTreePaving source(grid);
        source.adjoin(GridCell(grid, 0u, BinaryWord("0110")));
        Array<SizeType> indices(1u);
        indices[0] = 1u;
        GridTreePaving projected = image(source, Projection(2u, indices));
        std::cout << "source=";
        for (const GridCell& cell : source) {
            std::cout << cell.word();
        }
        std::cout << " projected=";
        for (const GridCell& cell : projected) {
            std::cout << cell.word();
        }
        std::cout << '\n';
        return 0;
    }

    if (mode == "layout") {
        std::cout << "BinaryTreeNode=" << sizeof(BinaryTreeNode)
                  << " GridCell=" << sizeof(GridCell)
                  << " GridTreePaving=" << sizeof(GridTreePaving) << '\n';
        return 0;
    }

    return 3;
}
