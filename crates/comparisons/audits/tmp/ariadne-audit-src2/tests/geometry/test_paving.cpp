/***************************************************************************
 *            test_paving.cpp
 *
 *  Copyright  2012-20  Pieter Collins
 *
 ****************************************************************************/

/*
 *  This file is part of Ariadne.
 *
 *  Ariadne is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Ariadne is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with Ariadne.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <iostream>
#include <fstream>

#include "config.hpp"
#include "geometry/paving_interface.hpp"
#include "geometry/grid_paving.hpp"
#include "function/function.hpp"
#include "geometry/function_set.hpp"
#include "io/figure.hpp"
#include "conclog/logging.hpp"

#include "../test.hpp"

using namespace ConcLog;

using namespace Ariadne;
using namespace std;

class TestPaving
{
  public:
    TestPaving() = default;
  public:
    Void test() const;
    Void test_constructors() const;
    Void test_iterator() const;
    Void test_branch() const;
    Void test_geometry() const;
    Void test_approximation() const;
};

Void TestPaving::test() const {
    ARIADNE_TEST_CALL(test_constructors());
    ARIADNE_TEST_CALL(test_iterator());
    ARIADNE_TEST_CALL(test_branch());
    ARIADNE_TEST_CALL(test_geometry());
    ARIADNE_TEST_CALL(test_approximation());
}

Void TestPaving::test_constructors() const {
    Grid grid(2);
    GridTreePaving grid_set(grid);
    PavingInterface& paving = grid_set;

    ARIADNE_TEST_PRINT(paving);
    ARIADNE_TEST_EQUAL(paving.grid(),grid);
    BinaryWord path("10010");
    GridCell cell(paving.grid(),1u,path);
    ARIADNE_TEST_PRINT(cell);
    paving.adjoin(cell);
    ARIADNE_TEST_PRINT(paving);
    path=BinaryWord("110110");
    cell=GridCell(paving.grid(),1u,path);
    paving.adjoin(GridCell(grid,Nat(1),BinaryWord("110110")));
    ARIADNE_TEST_PRINT(paving);
    ARIADNE_TEST_EQUALS(paving.size(),2);

    PavingHandle paving_copy = paving;
    ARIADNE_TEST_EQUAL(paving_copy.grid(),grid);
    ARIADNE_TEST_EQUALS(paving_copy.size(),2);

    paving.adjoin(GridCell(grid,Nat(1),BinaryWord("0101")));
    ARIADNE_TEST_EQUALS(paving.size(),3);
    ARIADNE_TEST_EQUALS(paving_copy.size(),2);

}

Void TestPaving::test_iterator() const {
    Grid grid(2);
    GridTreePaving grid_set(grid);
    PavingInterface& paving = grid_set;
    Nat height = 1;

    GridCell cell1(grid,height,BinaryWord("10010"));
    GridCell cell2(grid,height,BinaryWord("110110"));
    paving.adjoin(cell2);
    paving.adjoin(cell1);

    PavingInterface::ConstIterator iter=paving.begin();
    ARIADNE_TEST_EQUAL(*iter,cell1);
    ++iter;
    ARIADNE_TEST_EQUAL(*iter,cell2);
    ++iter;
    ARIADNE_TEST_EQUALS(iter,paving.end());
}

Void TestPaving::test_branch() const {
    Grid grid(2);
    GridTreePaving grid_set(grid);
    PavingInterface& paving = grid_set;
    GridCell cell(grid,0,BinaryWord(""));

    Nat height = 0;

    BinaryWord word1("0001");
    BinaryWord word2("0010");
    BinaryWord word3("1100");
    BinaryWord word4("1110");

    ExactBoxType box1{{0.0_x,0.25_x},{0.25_x,0.5_x}};
    ExactBoxType box2{{0.25_x,0.5_x},{0.0_x,0.25_x}};
    ExactBoxType box3{{0.5_x,0.75_x},{0.5_x,0.75_x}};
    ExactBoxType box4{{0.75_x,1.0_x},{0.5_x,0.75_x}};

    paving.adjoin(GridCell(grid,height,word1));
    paving.adjoin(GridCell(grid,height,word2));
    paving.adjoin(GridCell(grid,height,word3));
    paving.adjoin(GridCell(grid,height,word4));

    ARIADNE_TEST_PRINT(paving);
    ARIADNE_TEST_ASSERT(paving.size()==4);

    SubPavingHandle branch0 = paving.branch(0);
    ARIADNE_TEST_PRINT(branch0);
    ARIADNE_TEST_ASSERT(branch0.size()==2);
    ARIADNE_TEST_ASSERT(branch0.begin()->word()==word1);
    ARIADNE_TEST_ASSERT((++branch0.begin())->word()==word2);

    SubPavingHandle branch1 = paving.branch(1);
    ARIADNE_TEST_PRINT(branch1);
    ARIADNE_TEST_ASSERT(branch1.size()==2);
    ARIADNE_TEST_ASSERT(branch1.begin()->word()==word3);
    ARIADNE_TEST_ASSERT((++branch1.begin())->word()==word4);
}

Void TestPaving::test_geometry() const {
    Grid grid(2);
    GridTreePaving grid_tree_set(grid);
    Nat height(0);

    PavingHandle paving_handle1(grid_tree_set);
    PavingInterface& paving_interface1 = paving_handle1;
    paving_handle1.adjoin(GridCell(grid,height,BinaryWord("10")));
    PavingHandle paving_handle2(grid_tree_set);
    PavingInterface& paving_interface2 = paving_handle2;
    paving_handle2.adjoin(GridCell(grid,height,BinaryWord("1")));
    PavingHandle paving_handle3(grid_tree_set);
    PavingInterface& paving_interface3 = paving_handle3;
    paving_handle3.adjoin(GridCell(grid,height,BinaryWord("0110")));

    ARIADNE_TEST_PRINT(paving_handle1);
    ARIADNE_TEST_PRINT(paving_handle2);
    ARIADNE_TEST_PRINT(paving_handle3);

    paving_handle1.subset(paving_handle2);
    paving_handle1.subset(paving_interface2);
    paving_interface1.subset(paving_handle2);
    paving_interface1.subset(paving_interface2);
    subset(paving_handle1,paving_handle2);
    subset(paving_handle1,paving_interface2);
    subset(paving_interface1,paving_handle2);
    subset(paving_interface1,paving_interface2);

    ARIADNE_TEST_ASSERT(subset(paving_interface1,paving_interface1));
    ARIADNE_TEST_ASSERT(subset(paving_interface1,paving_interface2));
    ARIADNE_TEST_ASSERT(!subset(paving_interface2,paving_interface1));
    ARIADNE_TEST_ASSERT(!subset(paving_interface1,paving_interface3));
    ARIADNE_TEST_ASSERT(!subset(paving_interface3,paving_interface1));
    ARIADNE_TEST_ASSERT(!subset(paving_interface2,paving_interface3));
    ARIADNE_TEST_ASSERT(!subset(paving_interface3,paving_interface2));

    ARIADNE_TEST_ASSERT(paving_handle1.intersects(paving_handle2));
    ARIADNE_TEST_ASSERT(intersect(paving_handle1,paving_handle2));

    ARIADNE_TEST_ASSERT(intersect(paving_interface1,paving_interface1));
    ARIADNE_TEST_ASSERT(intersect(paving_interface1,paving_interface2));
    ARIADNE_TEST_ASSERT(intersect(paving_interface2,paving_interface1));
    ARIADNE_TEST_ASSERT(!intersect(paving_interface1,paving_interface3));
    ARIADNE_TEST_ASSERT(!intersect(paving_interface3,paving_interface1));
    ARIADNE_TEST_ASSERT(!intersect(paving_interface2,paving_interface3));
    ARIADNE_TEST_ASSERT(!intersect(paving_interface3,paving_interface2));

    ARIADNE_TEST_EQUALS(geometric_union(paving_handle1,paving_handle2).size(),1);
    ARIADNE_TEST_EQUALS(geometric_union(paving_handle1,paving_handle3).size(),2);
    ARIADNE_TEST_EQUALS(geometric_union(paving_handle2,paving_handle3).size(),2);
    ARIADNE_TEST_EQUALS(intersection(paving_handle1,paving_handle2).size(),1);
    ARIADNE_TEST_EQUALS(intersection(paving_handle2,paving_handle3).size(),0);
    ARIADNE_TEST_EQUALS(difference(paving_handle1,paving_handle2).size(),0);
    ARIADNE_TEST_EQUALS(difference(paving_handle2,paving_handle1).size(),1);

    PavingHandle paving_handle4(grid_tree_set);
    paving_handle4.adjoin(GridCell(grid,height,BinaryWord("11")));
    ARIADNE_TEST_EQUAL(intersection(paving_handle1,paving_handle2),paving_handle1);
    ARIADNE_TEST_EQUAL(difference(paving_handle2,paving_handle1),paving_handle4);

    ARIADNE_TEST_SAME(geometric_union(paving_handle1,paving_handle3).bounding_box(),UpperBoxType({{0.25_x,1.0_x},{0.0_x,0.75_x}}));


}

Void TestPaving::test_approximation() const {
    Grid grid(2);
    GridTreePaving grid_set(grid);
    PavingInterface& paving = grid_set;

    EffectiveVectorMultivariateFunction x=EffectiveVectorMultivariateFunction::identity(2);
    EffectiveConstrainedImageSet set(RealBox({{-1.0_x,1.0_x},{-1.0_x,1.0_x}}));
    set.apply( {2*x[0]+x[1]+x[0]*x[0]/4,x[0]+x[1]} );
    Nat depth = 2;
    paving.adjoin_outer_approximation(set,depth);
    paving.recombine();

    Nat height = 6;
    BinaryWord tree("111010111110101111011010100101100001111101000101010001111"
                    "100100010101001110100010101001011110110000101101000101111"
                    "000010101001011011111101000101010001011101000101010000111"
                    "011011011011000000011111110110100001111100001010100011111"
                    "010001110000011101010000111110110000101101000101111001000"
                    "100110111100100010010110110010001011011011010000011111011"
                    "011001001011000010111110000101100001011011110000101100001"
                    "111111100100010001110110010000011101111001000100111110000"
                    "101100011010100101100000111011011110111100001011000011111"
                    "01000111000000000000");
    BinaryWord leaves("0000000010010001001010001011010101001010000100110110010"
                      "1010100010010100101001010000000010000011011101010101101"
                      "0010100101010001001101100101101110100101110101000010000"
                      "0001010010010101001010001010010010100100110101001101001"
                      "00101001010101101000000101001000100100000000000");
    GridTreePaving expected_grid_set(grid,height,tree,leaves);
    PavingInterface& expected_paving = expected_grid_set;

    ARIADNE_TEST_BINARY_PREDICATE(subset,expected_paving,paving);

    PavingHandle error_paving = expected_paving;
    error_paving.remove(paving);

    Figure fig(widen(set.bounding_box(),0.5_x),Projection2d(2,0,1));
    fig << fill_colour(1.0,0.0,1.0) << paving;
    fig << fill_colour(0.5,0.0,0.5) << expected_paving;
    fig << fill_colour(0.0,0.0,1.0) << set;
    fig << fill_colour(1.0,0.0,0.0) << error_paving;
    fig.write("test_paving-geometry");
}


Int main(Int argc, const char* argv[])
{
    TestPaving().test();
    return ARIADNE_TEST_FAILURES;
}

