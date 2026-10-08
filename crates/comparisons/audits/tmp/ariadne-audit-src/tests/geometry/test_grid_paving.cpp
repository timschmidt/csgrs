/***************************************************************************
 *            test_grid_paving.cpp
 *
 *  Copyright  2008-20  Ivan S. Zapreev, Pieter Collins
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
#include <sstream>
#include <string>

#include "config.hpp"
#include "helper/stlio.hpp"

#include "utility/macros.hpp"
#include "utility/tribool.hpp"
#include "numeric/numeric.hpp"

#include "geometry/binary_tree.hpp"
#include "geometry/function_set.hpp"
#include "geometry/grid_paving.hpp"
#include "geometry/set_interface.hpp"

#include "../test.hpp"

using namespace Ariadne;
using namespace std;

static const Nat extentZero = 0;
static const Nat extentOne = 1;
static const Nat extentTwo = 2;
static const Nat extentThree = 3;
static const Nat extentFour = 4;

namespace {

Void test_grid_paving_cursor(){

    //Allocate the Grid
    Grid theGrid( {-0.25, 0.25, 1.5}, {0.25, 0.25, 0.25} );

    //Define the higth of the primary root cell.
    const Nat theExtent = 2;

    //Create the binary tree;
    BinaryTreeNode * pRootTreeNode = new BinaryTreeNode(true);

    //Mince the tree to the certain higth
    pRootTreeNode->mince(3);

    //Mark some nodes as disabled
    pRootTreeNode->left_node()->right_node()->left_node()->set_disabled();
    pRootTreeNode->left_node()->left_node()->right_node()->set_disabled();
    pRootTreeNode->right_node()->right_node()->right_node()->set_disabled();

    //Create the path to the root node of the subpaving, which
    //corresponds to pRootTreeNode->left_node()->right_node()
    BinaryWord thePathToSubPavingRoot;
    thePathToSubPavingRoot.push_back(false);
    thePathToSubPavingRoot.push_back(true);

    //Create the GridTreeSubpaving
    GridTreeSubpaving theGridSubPavingSmall( theGrid, theExtent, thePathToSubPavingRoot, pRootTreeNode->left_node()->right_node() );
    //Create the Cursor for the GridTreeSubpaving
    GridTreeCursor theGSPCursorSmall( &theGridSubPavingSmall );

    //Create the GridTreeSubpaving
    GridTreeSubpaving theGridSubPavingLarge( theGrid, theExtent, BinaryWord(), pRootTreeNode );
    //Create the Cursor for the GridTreeSubpaving
    GridTreeCursor theGSPCursorLarge( &theGridSubPavingLarge );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Check the created Cursor data on a small subpaving");
    ExactBoxType expected_box = make_box("[-0.5,0]x[0.5,1]x[1.25,2.25]");
    ARIADNE_TEST_EQUAL( expected_box, theGSPCursorSmall.cell().box() );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Check the created Cursor data on a large subpaving");
    expected_box = make_box("[-0.5,0.5]x[0,1]x[1.25,2.25]");
    ARIADNE_TEST_EQUAL( expected_box, theGSPCursorLarge.cell().box() );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Moving up from the Cursor's root node should cause the NotAllowedMoveException exception");
    ARIADNE_TEST_THROWS( theGSPCursorSmall.move_up(), NotAllowedMoveException);

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Let's move the cursor of theGSPCursorLarge: left, right, we should be in the root of the smaller subpaving then");
    theGSPCursorLarge.move_left();
    theGSPCursorLarge.move_right();
    GridTreeSubpaving tmpSubPaving = *theGSPCursorLarge;
    GridTreeSubpaving expected_tree_subset( theGrid, theExtent, make_binary_word("01"), new BinaryTreeNode( *theGridSubPavingSmall.binary_tree()) );
    ARIADNE_PRINT_TEST_COMMENT("The cursor has moved and we have the following subpaving: ");
    ARIADNE_TEST_EQUAL( expected_tree_subset, tmpSubPaving);

    expected_box = make_box("[-0.5,0]x[0.5,1]x[1.25,2.25]");
    ARIADNE_PRINT_TEST_COMMENT("The box pointed by the cursor is: ");
    ARIADNE_TEST_EQUAL( expected_box, theGSPCursorLarge.cell().box() );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Check the is_enabled/disabled/leaf/root functions, while moving the cursor in the large and small subpavings");
    ARIADNE_TEST_EQUAL(theGSPCursorSmall.is_root(), true);
    ARIADNE_TEST_EQUAL(theGSPCursorSmall.is_leaf(), false);
    ARIADNE_TEST_EQUAL(theGSPCursorLarge.is_root(), false);
    ARIADNE_TEST_EQUAL(theGSPCursorLarge.is_leaf(), false);
    ARIADNE_TEST_EQUAL(theGSPCursorLarge.is_enabled(), false);
    ARIADNE_TEST_EQUAL(theGSPCursorLarge.is_disabled(), false);
    ARIADNE_PRINT_TEST_COMMENT("Moving to the left, to a disabled node");
    theGSPCursorLarge.move_left();
    ARIADNE_TEST_EQUAL(theGSPCursorLarge.is_leaf(), true);
    ARIADNE_TEST_EQUAL(theGSPCursorLarge.is_root(), false);
    ARIADNE_TEST_EQUAL(theGSPCursorLarge.is_enabled(), false);
    ARIADNE_TEST_EQUAL(theGSPCursorLarge.is_disabled(), true);
    ARIADNE_PRINT_TEST_COMMENT("Moving up and to the right, to an enabled node");
    theGSPCursorLarge.move_up();
    theGSPCursorLarge.move_right();
    ARIADNE_TEST_EQUAL(theGSPCursorLarge.is_leaf(), true);
    ARIADNE_TEST_EQUAL(theGSPCursorLarge.is_root(), false);
    ARIADNE_TEST_EQUAL(theGSPCursorLarge.is_enabled(), true);
    ARIADNE_TEST_EQUAL(theGSPCursorLarge.is_disabled(), false);

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Test that moving to the left and right with the leaf nodes causes the NotAllowedMoveException exception");
    ARIADNE_PRINT_TEST_COMMENT("Get to the leaf node on the left");
    theGSPCursorSmall.move_left();

    ARIADNE_PRINT_TEST_COMMENT("Test the present state of theGSPCursorSmall");
    expected_box = make_box("[-0.5,0]x[0.5,1]x[1.25,1.75]");
    ARIADNE_PRINT_TEST_COMMENT("The box pointed by the cursor is: ");
    ARIADNE_TEST_EQUAL( expected_box, theGSPCursorSmall.cell().box() );

    ARIADNE_PRINT_TEST_COMMENT("Try to get to the node on the left");
    ARIADNE_TEST_THROWS(theGSPCursorSmall.move_left(), NotAllowedMoveException );

    ARIADNE_PRINT_TEST_COMMENT("Try to get to the node on the right");
    ARIADNE_TEST_THROWS(theGSPCursorSmall.move_right(), NotAllowedMoveException );

    ARIADNE_PRINT_TEST_COMMENT("The state of theGSPCursorSmall is supposed to remain unchanged");
    ARIADNE_PRINT_TEST_COMMENT("The box pointed by the cursor is: ");
    ARIADNE_TEST_EQUAL( expected_box, theGSPCursorSmall.cell().box() );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Test the set_enabled/set_disabled methods of the cursor");
    delete pRootTreeNode;
    pRootTreeNode = new BinaryTreeNode(true);
    ARIADNE_PRINT_TEST_COMMENT("Mince the enabled binary tree node to level 2");
    pRootTreeNode->mince(2);
    GridTreePaving theGridTreePaving( theGrid, theExtent, pRootTreeNode );
    //Create the Cursor for the GridTreeSubpaving
    GridTreeCursor theGPCursor( &theGridTreePaving );
    //Move to the leaf node
    ARIADNE_PRINT_TEST_COMMENT("Move the cursor to the right most leaf and check it");
    theGPCursor.move_right();
    theGPCursor.move_right();
    ARIADNE_TEST_EQUAL(theGPCursor.is_leaf(), true);
    ARIADNE_TEST_EQUAL(theGPCursor.is_root(), false);
    ARIADNE_TEST_EQUAL(theGPCursor.is_enabled(), true);
    ARIADNE_TEST_EQUAL(theGPCursor.is_disabled(), false);
    ARIADNE_PRINT_TEST_COMMENT("Set the leaf as disabled");
    theGPCursor.set_disabled();
    ARIADNE_TEST_EQUAL(theGPCursor.is_disabled(), true);
    ARIADNE_PRINT_TEST_COMMENT("Set the leaf (back) as enabled");
    theGPCursor.set_enabled();
    ARIADNE_TEST_EQUAL(theGPCursor.is_enabled(), true);
    ARIADNE_PRINT_TEST_COMMENT("Move one node up, to a non-leaf node and try to enable it, the NotALeafNodeException should be thrown");
    theGPCursor.move_up();
    ARIADNE_TEST_THROWS( theGPCursor.set_disabled(), NotALeafNodeException);
}

Void test_grid_paving_const_iterator(){
    std::vector< GridCell *> expected_result( 8 );

    //Allocate the Grid
    Grid theGrid( {-0.25, 0.25}, {0.25, 0.25} );

    //Define the higth of the primary root cell.
    const Nat theExtent = 2;

    //Create the binary tree;
    BinaryTreeNode * pRootTreeNode = new BinaryTreeNode(true);

    //Create the GridTreeSubpaving
    GridTreeSubpaving theGridSubPavingLarge( theGrid, theExtent, BinaryWord(), pRootTreeNode );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Test the sequence in which GridPavingIterator goes through the tree leafs ");
    ARIADNE_PRINT_TEST_COMMENT("The tree depth is 0 and all leaf nodes are enabled");

    expected_result[0] = new GridCell( theGrid, 2, make_binary_word("") );

    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result, theGridSubPavingLarge, 1 );
    ARIADNE_CLEAN_TEST_VECTOR( expected_result );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Test the sequence in which GridPavingIterator goes through the tree leafs ");
    ARIADNE_PRINT_TEST_COMMENT("The tree depth is 1 and all leaf nodes are enabled");

    //Mince the tree to the certain higth
    pRootTreeNode->mince(1);

    expected_result[0] =  new GridCell( theGrid, 2, make_binary_word("0") );
    expected_result[1] =  new GridCell( theGrid, 2, make_binary_word("1") );

    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result, theGridSubPavingLarge, 2 );
    ARIADNE_CLEAN_TEST_VECTOR( expected_result );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Test the sequence in which GridPavingIterator goes through the tree leafs ");
    ARIADNE_PRINT_TEST_COMMENT("The tree depth is 2 and all leaf nodes are enabled");

    //Mince the tree to the certain higth
    pRootTreeNode->mince(2);

    expected_result[0] =  new GridCell( theGrid, 2, make_binary_word("00") );
    expected_result[1] =  new GridCell( theGrid, 2, make_binary_word("01") );
    expected_result[2] =  new GridCell( theGrid, 2, make_binary_word("10") );
    expected_result[3] =  new GridCell( theGrid, 2, make_binary_word("11") );

    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result, theGridSubPavingLarge, 4 );
    ARIADNE_CLEAN_TEST_VECTOR( expected_result );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Test the sequence in which GridPavingIterator goes through the tree leafs ");
    ARIADNE_PRINT_TEST_COMMENT("The tree depth is 3 and all leaf nodes are enabled");

    //Mince the tree to the certain higth
    pRootTreeNode->mince(3);

    expected_result[0] = new GridCell( theGrid, 2, make_binary_word("000") );
    expected_result[1] = new GridCell( theGrid, 2, make_binary_word("001") );
    expected_result[2] = new GridCell( theGrid, 2, make_binary_word("010") );
    expected_result[3] = new GridCell( theGrid, 2, make_binary_word("011") );
    expected_result[4] = new GridCell( theGrid, 2, make_binary_word("100") );
    expected_result[5] = new GridCell( theGrid, 2, make_binary_word("101") );
    expected_result[6] = new GridCell( theGrid, 2, make_binary_word("110") );
    expected_result[7] = new GridCell( theGrid, 2, make_binary_word("111") );

    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result, theGridSubPavingLarge, 8 );
    ARIADNE_CLEAN_TEST_VECTOR( expected_result );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Disable some of the leaf nodes and test GridPavingIterator");
    ARIADNE_PRINT_TEST_COMMENT("The tree depth is 3 and some tree nodes are disabled");

    //Mark some nodes as disabled
    pRootTreeNode->left_node()->left_node()->left_node()->set_disabled();
    pRootTreeNode->left_node()->right_node()->right_node()->set_disabled();
    pRootTreeNode->right_node()->left_node()->left_node()->set_disabled();
    pRootTreeNode->right_node()->left_node()->right_node()->set_disabled();

    //Reuse the previous results
    std::vector< GridCell *>  expected_result_tmp(4);
    expected_result_tmp[0] = new GridCell( theGrid, 2, make_binary_word("001") );
    expected_result_tmp[1] = new GridCell( theGrid, 2, make_binary_word("010") );
    expected_result_tmp[2] = new GridCell( theGrid, 2, make_binary_word("110") );
    expected_result_tmp[3] = new GridCell( theGrid, 2, make_binary_word("111") );

    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result_tmp, theGridSubPavingLarge, 4 );
    ARIADNE_CLEAN_TEST_VECTOR( expected_result_tmp );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Recombine the tree and test GridPavingIterator");
    ARIADNE_PRINT_TEST_COMMENT("The tree depth is 3 but some nodes are at depth 2");

    //Recombine the paving
    theGridSubPavingLarge.recombine();

    //Reuse some of the previous results
    expected_result_tmp[0] = new GridCell( theGrid, 2, make_binary_word("001") );
    expected_result_tmp[1] = new GridCell( theGrid, 2, make_binary_word("010") );
    expected_result_tmp[2] = new GridCell( theGrid, 2, make_binary_word("11") );

    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result_tmp, theGridSubPavingLarge, 3 );
    ARIADNE_CLEAN_TEST_VECTOR( expected_result_tmp );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Mince the tree back to level 3, enable/disable some nodes, recombine and and test GridPavingIterator");
    ARIADNE_PRINT_TEST_COMMENT("The tree depth is 3 but some nodes are at depth 2");

    //Mince the tree to the certain higth, (only for enabled nodes)
    pRootTreeNode->mince(3);
    //Split the disabled node
    pRootTreeNode->right_node()->left_node()->split();

    //Mark some nodes as disabled/enabled
    pRootTreeNode->right_node()->right_node()->left_node()->set_disabled();
    pRootTreeNode->right_node()->left_node()->left_node()->set_enabled();
    pRootTreeNode->right_node()->left_node()->right_node()->set_enabled();

    //Recombine the paving
    theGridSubPavingLarge.recombine();

    //Reuse some of the previous result strings
    expected_result_tmp[0] = new GridCell( theGrid, 2, make_binary_word("001") );
    expected_result_tmp[1] = new GridCell( theGrid, 2, make_binary_word("010") );
    expected_result_tmp[2] = new GridCell( theGrid, 2, make_binary_word("10") );
    expected_result_tmp[3] = new GridCell( theGrid, 2, make_binary_word("111") );

    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result_tmp, theGridSubPavingLarge, 4 );
    ARIADNE_CLEAN_TEST_VECTOR( expected_result_tmp );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Test how the constant Cursor can be retrieved from the Constant Iterator");
    GridTreeSubpaving::ConstIterator it = theGridSubPavingLarge.begin();
    const GridTreeCursor theGPCursor = it.cursor();
    ARIADNE_TEST_EQUAL(theGPCursor.is_leaf(), true);
    ARIADNE_TEST_EQUAL(theGPCursor.is_root(), false);
    ARIADNE_TEST_EQUAL(theGPCursor.is_enabled(), true);
    ARIADNE_TEST_EQUAL(theGPCursor.is_disabled(), false);
    ARIADNE_PRINT_TEST_COMMENT("Set the leaf as disabled");
    theGPCursor.set_disabled();
    ARIADNE_TEST_EQUAL(theGPCursor.is_disabled(), true);
    ARIADNE_PRINT_TEST_COMMENT("Set the leaf (back) as enabled");
    theGPCursor.set_enabled();
    ARIADNE_TEST_EQUAL(theGPCursor.is_enabled(), true);
}

Void test_grid_sub_paving_one(){

    //Allocate the Grid, one Dimension
    Vector<ApproximateDouble> originOne(1, 0.0_x);
    Vector<ApproximateDouble> lengthsOne(1, 0.5_x );
    const Grid theOneDimGrid( originOne, lengthsOne );

    //Define the higth of the primary root cell.
    const Nat theRootExtent = 2;

    //Create the binary tree;
    BinaryTreeNode * pRootTreeNode = new BinaryTreeNode(true);

    //Create the path to the root node of the subpaving, which
    //corresponds to pRootTreeNode->left_node()->right_node()
    BinaryWord thePathToSubPavingRoot;
    thePathToSubPavingRoot.push_back(false);
    thePathToSubPavingRoot.push_back(true);

    //Create the GridTreeSubpaving
    GridTreeSubpaving theGridSPOneDim( theOneDimGrid, theRootExtent, thePathToSubPavingRoot, pRootTreeNode );

    //
    ARIADNE_PRINT_TEST_CASE_TITLE("Test Mincing operations of GridTreeSubpaving on the one dimensional Grid");
    ARIADNE_PRINT_TEST_COMMENT("The initial paving: ");
    ARIADNE_TEST_EQUAL( theGridSPOneDim.grid() , theOneDimGrid );
    ARIADNE_TEST_EQUAL( theGridSPOneDim.root_cell().root_extent() , theRootExtent );
    ARIADNE_TEST_EQUAL( theGridSPOneDim.root_cell().word() , thePathToSubPavingRoot );
    ARIADNE_TEST_EQUAL( (*theGridSPOneDim.binary_tree()) , (*pRootTreeNode) );
    ARIADNE_TEST_EQUAL( theGridSPOneDim.root_cell().box() , make_box("[0,0.5]") );

    //Mince to the level two from the root of the paving, this should give
    //us 4 leaf nodes with intervals of width 0.125 (in the original space)
    theGridSPOneDim.mince_to_tree_depth(2);

    ARIADNE_PRINT_TEST_COMMENT("Minced the sub-paving to depth 2");
    std::vector< GridCell* > expected_result_arr(4);

    expected_result_arr[0] = new GridCell( theOneDimGrid, theRootExtent, make_binary_word("0100") );
    expected_result_arr[1] = new GridCell( theOneDimGrid, theRootExtent, make_binary_word("0101") );
    expected_result_arr[2] = new GridCell( theOneDimGrid, theRootExtent, make_binary_word("0110") );
    expected_result_arr[3] = new GridCell( theOneDimGrid, theRootExtent, make_binary_word("0111") );

    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result_arr, theGridSPOneDim, 4 );
    ARIADNE_CLEAN_TEST_VECTOR( expected_result_arr );

    ARIADNE_PRINT_TEST_COMMENT("Recombine and subdivide the sub-paving to cell width 1.1");
    theGridSPOneDim.recombine();
    theGridSPOneDim.subdivide(1.1);
    ARIADNE_PRINT_TEST_COMMENT( "We should have a single node as in the initial sub paving: ");
    ARIADNE_TEST_EQUAL( theGridSPOneDim.binary_tree()->is_leaf(), true );
    ARIADNE_TEST_EQUAL( theGridSPOneDim.binary_tree()->is_enabled(), true );

    ARIADNE_PRINT_TEST_COMMENT("Subdivide the sub-paving to cell width 0.4, this should give us two sub cells");
    theGridSPOneDim.subdivide(0.4);

    expected_result_arr[0] = new GridCell( theOneDimGrid, theRootExtent, make_binary_word("010") );
    expected_result_arr[1] = new GridCell( theOneDimGrid, theRootExtent, make_binary_word("011") );

    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result_arr, theGridSPOneDim, 2 );
    ARIADNE_CLEAN_TEST_VECTOR( expected_result_arr );

    ARIADNE_PRINT_TEST_COMMENT("Subdivide the sub-paving to cell width 0.126, this should give us four sub cells");
    theGridSPOneDim.subdivide(0.126);

    expected_result_arr[0] = new GridCell( theOneDimGrid, theRootExtent, make_binary_word("0100") );
    expected_result_arr[1] = new GridCell( theOneDimGrid, theRootExtent, make_binary_word("0101") );
    expected_result_arr[2] = new GridCell( theOneDimGrid, theRootExtent, make_binary_word("0110") );
    expected_result_arr[3] = new GridCell( theOneDimGrid, theRootExtent, make_binary_word("0111") );

    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result_arr, theGridSPOneDim, 4 );
    //NOTE: To not clear the vector, the expected result is reused in the next test;

    ARIADNE_PRINT_TEST_COMMENT("Recombine and Subdivide the sub-paving to cell width 0.126, this should give us four sub cells");
    theGridSPOneDim.recombine();
    theGridSPOneDim.subdivide(0.126);

    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result_arr, theGridSPOneDim, 4 );
    ARIADNE_CLEAN_TEST_VECTOR( expected_result_arr );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Test Mincing operations of GridTreeSubpaving on the two dimensional Grid");
    //Allocate the Grid, one Dimension
    const Grid theTwoDimGrid( {-0.25, 0.5}, {0.25, 0.5} );

    //Create the GridTreeSubpaving
    GridTreeSubpaving theGridSPTwoDim( theTwoDimGrid, theRootExtent, thePathToSubPavingRoot, pRootTreeNode );

    ARIADNE_PRINT_TEST_COMMENT("Recombine and Mince the sub-paving to depth 2, this should give us four sub cells");
    theGridSPTwoDim.recombine();
    theGridSPTwoDim.mince_to_tree_depth(2);

    expected_result_arr[0] =  new GridCell( theTwoDimGrid, theRootExtent, make_binary_word("0100") );
    expected_result_arr[1] =  new GridCell( theTwoDimGrid, theRootExtent, make_binary_word("0101") );
    expected_result_arr[2] =  new GridCell( theTwoDimGrid, theRootExtent, make_binary_word("0110") );
    expected_result_arr[3] =  new GridCell( theTwoDimGrid, theRootExtent, make_binary_word("0111") );

    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result_arr, theGridSPTwoDim, 4 );
    //Do not clean the Array yet, the result well be reused

    ARIADNE_PRINT_TEST_COMMENT("Recombine and Subdivide the sub-paving to cell width 0.5, this should give us four sub cells");
    //At this moment the coordinate cell widths are: for x -- 0.25 and for y -- 0.5
    theGridSPTwoDim.recombine();
    theGridSPTwoDim.subdivide(0.51);

    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result_arr, theGridSPTwoDim, 4 );
    ARIADNE_CLEAN_TEST_VECTOR( expected_result_arr );

    ARIADNE_PRINT_TEST_COMMENT("Subdivide the sub-paving to cell width 0.4, this should give us sixteen sub cells");
    //At this moment the coordinate cell widths are: for x -- 0.25 and for y -- 0.5
    theGridSPTwoDim.subdivide(0.4);

    BinaryTreeNode binaryTree( make_binary_word("1111001001100100111001001100100"), make_binary_word("1111111111111111") );
    GridTreeSubpaving expected_result( theTwoDimGrid, theRootExtent, make_binary_word("01"), &binaryTree );
    ARIADNE_PRINT_TEST_COMMENT("The sub paving with 16 leaf nodes: ");
    ARIADNE_TEST_EQUAL( expected_result, theGridSPTwoDim );

    ARIADNE_PRINT_TEST_COMMENT("The depth of the sub-paving's tree should be 4");
    ARIADNE_TEST_EQUAL( theGridSPTwoDim.tree_depth(), 4u );
}

Void test_grid_sub_paving_two() {
    //Allocate a trivial Grid two dimensional grid
    Grid theTwoDimGrid(2, 1.0);

    //Define the higth of the primary root cell.
    const Nat theRootExtent = 2;

    //Create the binary tree;
    BinaryTreeNode * pRootTreeNode = new BinaryTreeNode(true);

    //Create the path to the root node of the subpaving, which
    //corresponds to pRootTreeNode->left_node()->left_node()
    BinaryWord thePathToSubPavingRoot;
    thePathToSubPavingRoot.push_back(false);
    thePathToSubPavingRoot.push_back(false);

    //Create the GridTreeSubpaving
    GridTreeSubpaving theGridSPOneDim( theTwoDimGrid, theRootExtent, thePathToSubPavingRoot, pRootTreeNode );
    std::vector< GridCell* > expected_result_arr( 16 );

    ARIADNE_PRINT_TEST_COMMENT("Take the [-1,1]x[-1,1] box and do mince(0), i.e. down to the lovel of zero cells");
    expected_result_arr[0]  = new GridCell( theTwoDimGrid, theRootExtent, make_binary_word("0000") );
    expected_result_arr[1]  = new GridCell( theTwoDimGrid, theRootExtent, make_binary_word("0001") );
    expected_result_arr[2]  = new GridCell( theTwoDimGrid, theRootExtent, make_binary_word("0010") );
    expected_result_arr[3]  = new GridCell( theTwoDimGrid, theRootExtent, make_binary_word("0011") );
    theGridSPOneDim.mince(0);
    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result_arr, theGridSPOneDim, 4 );
    ARIADNE_CLEAN_TEST_VECTOR( expected_result_arr );

    ARIADNE_PRINT_TEST_COMMENT("Take the [-1,1]x[-1,1] box and do mince(1), i.e. one time in each dimension from the zero cell level");
    expected_result_arr[0]  = new GridCell( theTwoDimGrid, theRootExtent, make_binary_word("000000") );
    expected_result_arr[1]  = new GridCell( theTwoDimGrid, theRootExtent, make_binary_word("000001") );
    expected_result_arr[2]  = new GridCell( theTwoDimGrid, theRootExtent, make_binary_word("000010") );
    expected_result_arr[3]  = new GridCell( theTwoDimGrid, theRootExtent, make_binary_word("000011") );
    expected_result_arr[4]  = new GridCell( theTwoDimGrid, theRootExtent, make_binary_word("000100") );
    expected_result_arr[5]  = new GridCell( theTwoDimGrid, theRootExtent, make_binary_word("000101") );
    expected_result_arr[6]  = new GridCell( theTwoDimGrid, theRootExtent, make_binary_word("000110") );
    expected_result_arr[7]  = new GridCell( theTwoDimGrid, theRootExtent, make_binary_word("000111") );
    expected_result_arr[8]  = new GridCell( theTwoDimGrid, theRootExtent, make_binary_word("001000") );
    expected_result_arr[9]  = new GridCell( theTwoDimGrid, theRootExtent, make_binary_word("001001") );
    expected_result_arr[10] = new GridCell( theTwoDimGrid, theRootExtent, make_binary_word("001010") );
    expected_result_arr[11] = new GridCell( theTwoDimGrid, theRootExtent, make_binary_word("001011") );
    expected_result_arr[12] = new GridCell( theTwoDimGrid, theRootExtent, make_binary_word("001100") );
    expected_result_arr[13] = new GridCell( theTwoDimGrid, theRootExtent, make_binary_word("001101") );
    expected_result_arr[14] = new GridCell( theTwoDimGrid, theRootExtent, make_binary_word("001110") );
    expected_result_arr[15] = new GridCell( theTwoDimGrid, theRootExtent, make_binary_word("001111") );
    theGridSPOneDim.mince(1);
    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result_arr, theGridSPOneDim, 16 );
    ARIADNE_CLEAN_TEST_VECTOR( expected_result_arr );
}

Void test_grid_sub_paving_three() {
    //Allocate a trivial Grid two dimensional grid
    Grid theTwoDimGrid(2, 1.0);

    //Define the higth of the primary root cell.
    const Nat theRootExtent = 1;

    //Create the binary tree;
    BinaryTreeNode * pRootTreeNode = new BinaryTreeNode(true);

    //Create the path to the root node of the subpaving, which corresponds
    //to pRootTreeNode->left_node()->left_node()->left_node()->left_node()
    BinaryWord thePathToSubPavingRoot = make_binary_word("0000");

    //Create the GridTreeSubpaving
    GridTreeSubpaving theGridSPOneDim( theTwoDimGrid, theRootExtent, thePathToSubPavingRoot, pRootTreeNode );
    std::vector< GridCell* > expected_result_arr( 4 );

    ARIADNE_PRINT_TEST_COMMENT("Take the [-1,-0.5]x[-1,-0.5] box and do mince(0), i.e. nothing should get minced");
    theGridSPOneDim.mince(0);
    expected_result_arr[0]  = new GridCell( theTwoDimGrid, theRootExtent, make_binary_word("0000") );
    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result_arr, theGridSPOneDim, 1 );
    ARIADNE_CLEAN_TEST_VECTOR( expected_result_arr );

    ARIADNE_PRINT_TEST_COMMENT("Take the [-1,-0.5]x[-1,-0.5] box and do mince(1), i.e. nothing should get minced");
    theGridSPOneDim.mince(1);
    expected_result_arr[0]  = new GridCell( theTwoDimGrid, theRootExtent, make_binary_word("0000") );
    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result_arr, theGridSPOneDim, 1 );
    ARIADNE_CLEAN_TEST_VECTOR( expected_result_arr );

    ARIADNE_PRINT_TEST_COMMENT("Take the [-1,-0.5]x[-1,-0.5] box and do mince(2), i.e. we should get four subcells");
    expected_result_arr[0]  = new GridCell( theTwoDimGrid, theRootExtent, make_binary_word("000000") );
    expected_result_arr[1]  = new GridCell( theTwoDimGrid, theRootExtent, make_binary_word("000001") );
    expected_result_arr[2]  = new GridCell( theTwoDimGrid, theRootExtent, make_binary_word("000010") );
    expected_result_arr[3]  = new GridCell( theTwoDimGrid, theRootExtent, make_binary_word("000011") );
    theGridSPOneDim.mince(2);
    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result_arr, theGridSPOneDim, 4 );
    ARIADNE_CLEAN_TEST_VECTOR( expected_result_arr );
}

Void test_grid_paving(){
    //Create a trivial 4-dimensional Grid
    Grid trivialFourDimGrid( {0.0,0.0,0.0,0.0}, {1.0,1.0,1.0,1.0} );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Test allocation of a trivial GridTreePaving");
    GridTreePaving * pTrivialPaving = new GridTreePaving(4, true);
    GridTreePaving expected_result1( trivialFourDimGrid , 0, make_binary_word("0"), make_binary_word("1") );
    ARIADNE_PRINT_TEST_COMMENT("A trivial paving for [0,1]x[0,1]x[0,1]x[0,1], enabled cell: ");
    ARIADNE_TEST_EQUAL( expected_result1, (*pTrivialPaving) );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Test GridTreePaving copy constructor");
    GridTreePaving theTrivialPaving( ( *pTrivialPaving ) );
    pTrivialPaving->mince_to_tree_depth(1);

    ARIADNE_PRINT_TEST_COMMENT("Minced trivial paving for [0,1]x[0,1]x[0,1]x[0,1], enabled cell: ");
    GridTreePaving expected_result_minced(trivialFourDimGrid , 0, make_binary_word("100"), make_binary_word("11") );
    ARIADNE_TEST_EQUAL( expected_result_minced, (*pTrivialPaving) );

    ARIADNE_PRINT_TEST_COMMENT("A copy of the original paving, should stay unchanged: ");
    ARIADNE_TEST_EQUAL( expected_result1, theTrivialPaving );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Test GridTreePaving (Grid, ExactBoxType) constructor");
    //Allocate the Grid, one Dimension
    const Grid theTwoDimGrid( {-0.25,0.5}, {0.25,0.5} );
    //Note: the box is related to the grid, but not to the original space
    GridTreePaving theTwoDimPaving( theTwoDimGrid, make_box("[0,1.5]x[-1.5,3.5]") );
    ARIADNE_PRINT_TEST_COMMENT("The resulting GridTreePaving: ");
    GridTreePaving expected_result2( theTwoDimGrid, 4, make_binary_word("0"), make_binary_word("0") );
    ARIADNE_TEST_EQUAL( expected_result2, theTwoDimPaving );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Test GridTreePaving (Grid, Height, BooleanArray, BooleanArray ) constructor");
    BooleanArray theTree(9), theEnabledLeafs(5);
    theTree[0] = true;
    theTree[1] = true;
    theTree[2] = false;  theEnabledLeafs[0] = true;
    theTree[3] = false;  theEnabledLeafs[1] = false;
    theTree[4] = true;
    theTree[5] = true;
    theTree[6] = false;  theEnabledLeafs[2] = true;
    theTree[7] = false;  theEnabledLeafs[3] = false;
    theTree[8] = false;  theEnabledLeafs[4] = true;
    GridTreePaving theTwoDimTreePaving( theTwoDimGrid, 2, theTree, theEnabledLeafs );
    ARIADNE_PRINT_TEST_COMMENT("The resulting GridTreePaving: ");
    BinaryTreeNode * pRootTreeNode = new BinaryTreeNode(false);
    pRootTreeNode->split();
    pRootTreeNode->left_node()->split();
    pRootTreeNode->left_node()->left_node()->set_enabled();
    pRootTreeNode->right_node()->split();
    pRootTreeNode->right_node()->right_node()->set_enabled();
    pRootTreeNode->right_node()->left_node()->split();
    pRootTreeNode->right_node()->left_node()->left_node()->set_enabled();
    GridTreePaving expected_result3( theTwoDimGrid, 2, pRootTreeNode );
    ARIADNE_TEST_EQUAL( expected_result3, theTwoDimTreePaving );

    // !!!
    //Create a trivial 2-dimensional Grid
    Grid trivialTwoDimGrid( {0.0,0.0}, {1.0,1.0} );
    ARIADNE_PRINT_TEST_CASE_TITLE("Test GridTreePaving::restrict_to_extent( const Nat theExtent )");
    GridTreePaving initialTreeSetOne( trivialTwoDimGrid, 2, make_binary_word("1111001000100"), make_binary_word("10010001") );
    GridTreePaving initialTreeSetOneCopyOne( initialTreeSetOne );
    GridTreePaving initialTreeSetOneCopyTwo( initialTreeSetOne );

    ARIADNE_PRINT_TEST_COMMENT("The initialTreeSetOne.root_cell().root_extent() == 2, theExtent == 3, nothing should change");
    initialTreeSetOne.restrict_to_extent( 3 );
    ARIADNE_TEST_EQUAL( initialTreeSetOneCopyOne, initialTreeSetOne );

    ARIADNE_PRINT_TEST_COMMENT("The initialTreeSetOneCopyOne.root_cell().root_extent() == 2, theExtent == 1");
    initialTreeSetOneCopyOne.restrict_to_extent( 1 );
    GridTreePaving expectedTreeSetOne( trivialTwoDimGrid, 2, make_binary_word("11110010000"), make_binary_word("100100") );
    ARIADNE_TEST_EQUAL( expectedTreeSetOne, initialTreeSetOneCopyOne );

    ARIADNE_PRINT_TEST_COMMENT("The initialTreeSetOneCopyTwo.root_cell().root_extent() == 2, theExtent == 0");
    initialTreeSetOneCopyTwo.restrict_to_extent( 0 );
    GridTreePaving expectedTreeSetTwo( trivialTwoDimGrid, 2, make_binary_word("111010000"), make_binary_word("00100") );
    ARIADNE_TEST_EQUAL( expectedTreeSetTwo, initialTreeSetOneCopyTwo );

    ARIADNE_PRINT_TEST_COMMENT("The initialTreeSetTwo.root_cell().root_extent() == 1, theExtent == 0");
    GridTreePaving initialTreeSetTwo( trivialTwoDimGrid, 1, make_binary_word("11000"), make_binary_word("100") );
    initialTreeSetTwo.restrict_to_extent( 0 );
    GridTreePaving expectedTreeSetThree( trivialTwoDimGrid, 1, make_binary_word("100"), make_binary_word("00") );
    ARIADNE_TEST_EQUAL( expectedTreeSetThree, initialTreeSetTwo );
}

Void test_grid_cell(){
    BinaryWord expected_result;

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Test the static methods of the GridCell");
    BinaryWord theBinaryPath;

    theBinaryPath = GridCell::primary_cell_path( 1, 0, 0 );
    ARIADNE_PRINT_TEST_COMMENT( "Dimension: 1, topCellExtent: 0, bottomCellExtent: 0"  );
    ARIADNE_TEST_EQUAL( expected_result , theBinaryPath );

    expected_result = make_binary_word( "1" ) ;
    theBinaryPath = GridCell::primary_cell_path( 1, 1, 0 );
    ARIADNE_PRINT_TEST_COMMENT( "Dimension: 1, topCellExtent: 1, bottomCellExtent: 0" );
    ARIADNE_TEST_EQUAL( expected_result , theBinaryPath );

    expected_result = make_binary_word( "01" );
    theBinaryPath = GridCell::primary_cell_path( 1, 2, 0 );
    ARIADNE_PRINT_TEST_COMMENT( "Dimension: 1, topCellExtent: 2, bottomCellExtent: 0" );
    ARIADNE_TEST_EQUAL( expected_result , theBinaryPath );

    expected_result = make_binary_word( "0" );
    theBinaryPath = GridCell::primary_cell_path( 1, 2, 1 );
    ARIADNE_PRINT_TEST_COMMENT( "Dimension: 1, topCellExtent: 2, bottomCellExtent: 1" );
    ARIADNE_TEST_EQUAL( expected_result , theBinaryPath );

    expected_result.clear();
    theBinaryPath = GridCell::primary_cell_path( 1, 2, 2 );
    ARIADNE_PRINT_TEST_COMMENT( "Dimension: 1, topCellExtent: 2, bottomCellExtent: 2" );
    ARIADNE_TEST_EQUAL( expected_result , theBinaryPath );

    Grid theGrid( {0.0,0.0}, {1.0,1.0} );
    //pFirstCell_01 == pSecondCell_01
    GridCell * pFirstCell_01 = new GridCell( theGrid, 0, make_binary_word("01") );
    GridCell * pSecondCell_01 = new GridCell( theGrid, 1, make_binary_word("1101") );
    //pFirstCell_01 != pThirdCell_01 (pFirstCell_01 is left to pThirdCell_01)
    GridCell * pThirdCell_01 = new GridCell( theGrid, 2, make_binary_word("001111") );
    //pFirstCell_01 < pFourthCell_01
    GridCell * pFourthCell_01 = new GridCell( theGrid, 1, make_binary_word("11011") );

    //!!!
    ARIADNE_PRINT_TEST_COMMENT( "pFirstCell_01 == pFirstCell_01, check for operators < and ==" );
    ARIADNE_TEST_EQUAL( false , ( (*pFirstCell_01) < (*pFirstCell_01) )  );
    ARIADNE_TEST_EQUAL( true , ( (*pFirstCell_01) == (*pFirstCell_01) )  );

    //!!!
    ARIADNE_PRINT_TEST_COMMENT( "pFirstCell_01 == pSecondCell_01, check for operators < and ==" );
    ARIADNE_TEST_EQUAL( false , ( (*pFirstCell_01) < (*pSecondCell_01) )  );
    ARIADNE_TEST_EQUAL( true , ( (*pFirstCell_01) == (*pSecondCell_01) )  );
    //!!!
    ARIADNE_PRINT_TEST_COMMENT( "pFirstCell_01 == pSecondCell_01, check for operators < and ==" );
    ARIADNE_TEST_EQUAL( false , ( (*pSecondCell_01) < (*pFirstCell_01) )  );
    ARIADNE_TEST_EQUAL( true , ( (*pSecondCell_01) == (*pFirstCell_01) )  );
    //!!!
    ARIADNE_PRINT_TEST_COMMENT( "pFirstCell_01 != pThirdCell_01 (pFirstCell_01 is left to pThirdCell_01), check for operators < and ==" );
    ARIADNE_TEST_EQUAL( true , ( (*pFirstCell_01) < (*pThirdCell_01) )  );
    ARIADNE_TEST_EQUAL( false , ( (*pFirstCell_01) == (*pThirdCell_01) )  );
    //!!!
    ARIADNE_PRINT_TEST_COMMENT( "pFirstCell_01 != pThirdCell_01 (pFirstCell_01 is left to pThirdCell_01), check for operators < and ==" );
    ARIADNE_TEST_EQUAL( false , ( (*pThirdCell_01) < (*pFirstCell_01) )  );
    ARIADNE_TEST_EQUAL( false , ( (*pThirdCell_01) == (*pFirstCell_01) )  );
    //!!!
    ARIADNE_PRINT_TEST_COMMENT( "pFirstCell_01 < pFourthCell_01, check for operators < and ==" );
    ARIADNE_TEST_EQUAL( true , ( (*pFirstCell_01) < (*pFourthCell_01) )  );
    ARIADNE_TEST_EQUAL( false , ( (*pFirstCell_01) == (*pFourthCell_01) )  );
    //!!!
    ARIADNE_PRINT_TEST_COMMENT( "pFirstCell_01 < pFourthCell_01, check for operators < and ==" );
    ARIADNE_TEST_EQUAL( false , ( (*pFourthCell_01) < (*pFirstCell_01) )  );
    ARIADNE_TEST_EQUAL( false , ( (*pFourthCell_01) == (*pFirstCell_01) )  );
    //!!!
    ARIADNE_PRINT_TEST_COMMENT( "pFirstCell_01 < pFourthCell_01, check for an exception in the compare_grid_cells method" );
    ARIADNE_TEST_THROWS( GridCell::compare_grid_cells( pFourthCell_01, (*pFirstCell_01), 1000), InvalidInput );

    //!!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Test Cell splitting, dimension: 2");
    string word = "001111";
    GridCell * pThirdCell_To_Split = new GridCell( theGrid, 2, make_binary_word( word ) );
    ExactBoxType expected_cell_box = make_box("[0.5,1.0]x[0.5,1.0]");
    ARIADNE_PRINT_TEST_COMMENT("The initial GridCell, as given by it's box: ");
    ARIADNE_TEST_EQUAL( expected_cell_box, pThirdCell_To_Split->box() );
    ARIADNE_PRINT_TEST_COMMENT("The left sub-box of the initial GridCell, as given by it's box: ");
    ExactBoxType expected_left_sub_cell_box = make_box("[0.5,0.75]x[0.5,1.0]");
    ARIADNE_TEST_EQUAL( expected_left_sub_cell_box, pThirdCell_To_Split->split(false).box() );
    ARIADNE_PRINT_TEST_COMMENT("The right sub-box of the initial GridCell, as given by it's box: ");
    ExactBoxType expected_right_sub_cell_box = make_box("[0.75,1.0]x[0.5,1.0]");
    ARIADNE_TEST_EQUAL( expected_right_sub_cell_box, pThirdCell_To_Split->split(true).box() );
    ARIADNE_PRINT_TEST_COMMENT("The word of the initial GridCell: ");
    BinaryWord expected_cell_word = make_binary_word( word );
    ARIADNE_TEST_EQUAL( expected_cell_word, pThirdCell_To_Split->word() );

    //!!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Test taking the cell's interior, dimension: 2");
    ARIADNE_PRINT_TEST_COMMENT("The interior of the grid cell given by the box of the corresponding open grid cell: ");
    ARIADNE_TEST_EQUAL( expected_cell_box, pThirdCell_To_Split->interior().box() );
}

Void test_grid_open_cell_two(){
    //Allocate a trivial Grid two dimensional grid
    Grid theTrivialGrid(2, 1.0);

    //!!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Construct a trivial open cell and split it, dimension: 2");
    GridOpenCell trivialOpenCell = GridOpenCell( theTrivialGrid, 0, BinaryWord() );
    ExactBoxType expected_trivial_open_cell_box = make_box("[0.0,2.0]x[0.0,2.0]");
    ARIADNE_PRINT_TEST_COMMENT("The trivial open cell, as given by its box:");
    ARIADNE_TEST_EQUAL( expected_trivial_open_cell_box, trivialOpenCell.box() );
    ARIADNE_PRINT_TEST_COMMENT("The left open sub cell, as given by its box:");
    ExactBoxType expected_left_open_cell_box = make_box("[0.0,1.0]x[0.0,2.0]");
    ARIADNE_TEST_EQUAL( expected_left_open_cell_box, trivialOpenCell.split( TernaryChild::LEFT ).box() );
    ARIADNE_PRINT_TEST_COMMENT("The middle open sub cell, as given by its box:");
    ExactBoxType expected_middle_open_cell_box = make_box("[0.5,1.5]x[0.0,2.0]");
    ARIADNE_TEST_EQUAL( expected_middle_open_cell_box, trivialOpenCell.split( TernaryChild::MIDDLE ).box() );
    ARIADNE_PRINT_TEST_COMMENT("The right open sub cell, as given by its box:");
    ExactBoxType expected_right_open_cell_box = make_box("[1.0,2.0]x[0.0,2.0]");
    ARIADNE_TEST_EQUAL( expected_right_open_cell_box, trivialOpenCell.split( TernaryChild::RIGHT ).box() );

    //!!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Construct an open cell 01 extent=2 and split it, dimension: 2");
    GridOpenCell openCell01 = GridOpenCell( theTrivialGrid, 2, make_binary_word( "0011111" ) );
    ExactBoxType expected_open_cell_box01 = make_box("[0.75,1.25]x[0.5,1.5]");
    ARIADNE_PRINT_TEST_COMMENT("The open cell, as given by its box:");
    ARIADNE_TEST_EQUAL( expected_open_cell_box01, openCell01.box() );
    ARIADNE_PRINT_TEST_COMMENT("The left open sub cell, as given by its box:");
    ExactBoxType expected_left_open_cell_box01 = make_box("[0.75,1.25]x[0.5,1.0]");
    ARIADNE_TEST_EQUAL( expected_left_open_cell_box01, openCell01.split( TernaryChild::LEFT ).box() );
    ARIADNE_PRINT_TEST_COMMENT("The middle open sub cell, as given by its box:");
    ExactBoxType expected_middle_open_cell_box01 = make_box("[0.75,1.25]x[0.75,1.25]");
    ARIADNE_TEST_EQUAL( expected_middle_open_cell_box01, openCell01.split( TernaryChild::MIDDLE ).box() );
    ARIADNE_PRINT_TEST_COMMENT("The right open sub cell, as given by its box:");
    ExactBoxType expected_right_open_cell_box01 = make_box("[0.75,1.25]x[1.0,1.5]");
    ARIADNE_TEST_EQUAL( expected_right_open_cell_box01, openCell01.split( TernaryChild::RIGHT ).box() );
}

Void test_grid_open_cell_three(){
    //Allocate a trivial Grid two dimensional grid
    Grid theGrid(2, 1.0);

    //!!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Construct a [0.0,1.0]x[0.0,1.0] box and make and open over approximation, dimension: 2");
    ExactBoxType theBox = make_box("[0.0,1.0]x[0.0,1.0]");
    ExactBoxType expectedOpenCellBoxType = make_box("[-1.0,3.0]x[-1.0,3.0]");
    BinaryWord expectedOpenCellWord = make_binary_word("00");
    Nat expectedOpenCellRootExtent = 2;
    GridOpenCell actualOpenCell = GridOpenCell::outer_approximation( theBox, theGrid );
    ARIADNE_TEST_EQUAL( expectedOpenCellBoxType, actualOpenCell.box() );
    ARIADNE_TEST_EQUAL( expectedOpenCellWord, actualOpenCell.word() );
    ARIADNE_TEST_EQUAL( expectedOpenCellRootExtent, actualOpenCell.root_extent() );

    //!!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Construct a [-0.5,0.5]x[-0.5,0.5] box and make and open over approximation, dimension: 2");
    theBox = make_box("[-0.5,0.5]x[-0.5,0.5]");
    expectedOpenCellBoxType = make_box("[-1.0,1.0]x[-1.0,1.0]");
    expectedOpenCellWord = make_binary_word("00");
    expectedOpenCellRootExtent = 1;
    actualOpenCell = GridOpenCell::outer_approximation( theBox, theGrid );
    ARIADNE_TEST_EQUAL( expectedOpenCellBoxType, actualOpenCell.box() );
    ARIADNE_TEST_EQUAL( expectedOpenCellWord, actualOpenCell.word() );
    ARIADNE_TEST_EQUAL( expectedOpenCellRootExtent, actualOpenCell.root_extent() );

    //!!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Construct a [0.1,0.3]x[0.1,0.3] box and make and open over approximation, dimension: 2");
    theBox = make_box("[0.1,0.3]x[0.1,0.3]");
    expectedOpenCellBoxType = make_box("[0.0,0.5]x[0.0,0.5]");
    expectedOpenCellWord = make_binary_word("0000");
    expectedOpenCellRootExtent = 0;
    actualOpenCell = GridOpenCell::outer_approximation( theBox, theGrid );
    ARIADNE_TEST_EQUAL( expectedOpenCellBoxType, actualOpenCell.box() );
    ARIADNE_TEST_EQUAL( expectedOpenCellWord, actualOpenCell.word() );
    ARIADNE_TEST_EQUAL( expectedOpenCellRootExtent, actualOpenCell.root_extent() );

    //!!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Construct a [0.1,0.3]x[0.1,0.6] box and make and open over approximation, dimension: 2");
    theBox = make_box("[0.1,0.3]x[0.1,0.6]");
    expectedOpenCellBoxType = make_box("[0.0,0.5]x[0.0,1.0]");
    expectedOpenCellWord = make_binary_word("000");
    expectedOpenCellRootExtent = 0;
    actualOpenCell = GridOpenCell::outer_approximation( theBox, theGrid );
    ARIADNE_TEST_EQUAL( expectedOpenCellBoxType, actualOpenCell.box() );
    ARIADNE_TEST_EQUAL( expectedOpenCellWord, actualOpenCell.word() );
    ARIADNE_TEST_EQUAL( expectedOpenCellRootExtent, actualOpenCell.root_extent() );

    //!!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Construct a [0.5,0.6]x[0.1,0.6] box and make and open over approximation, dimension: 2");
    theBox = make_box("[0.5,0.6]x[0.1,0.6]");
    expectedOpenCellBoxType = make_box("[0.25,0.75]x[0.0,1.0]");
    expectedOpenCellWord = make_binary_word("001");
    expectedOpenCellRootExtent = 0;
    actualOpenCell = GridOpenCell::outer_approximation( theBox, theGrid );
    ARIADNE_TEST_EQUAL( expectedOpenCellBoxType, actualOpenCell.box() );
    ARIADNE_TEST_EQUAL( expectedOpenCellWord, actualOpenCell.word() );
    ARIADNE_TEST_EQUAL( expectedOpenCellRootExtent, actualOpenCell.root_extent() );

    //!!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Construct a [0.6,0.7]x[0.1,0.6] box and make and open over approximation, dimension: 2");
    theBox = make_box("[0.6,0.7]x[0.1,0.6]");
    expectedOpenCellBoxType = make_box("[0.25,0.75]x[0.0,1.0]");
    expectedOpenCellWord = make_binary_word("001");
    expectedOpenCellRootExtent = 0;
    actualOpenCell = GridOpenCell::outer_approximation( theBox, theGrid );
    ARIADNE_TEST_EQUAL( expectedOpenCellBoxType, actualOpenCell.box() );
    ARIADNE_TEST_EQUAL( expectedOpenCellWord, actualOpenCell.word() );
    ARIADNE_TEST_EQUAL( expectedOpenCellRootExtent, actualOpenCell.root_extent() );

    //!!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Construct a [0.9,1.1]x[0.9,1.1] box and make and open over approximation, dimension: 2");
    theBox = make_box("[0.9,1.1]x[0.9,1.1]");
    expectedOpenCellBoxType = make_box("[0.875,1.125]x[0.875,1.125]");
    expectedOpenCellWord = make_binary_word("0011111111");
    expectedOpenCellRootExtent = 2;
    actualOpenCell = GridOpenCell::outer_approximation( theBox, theGrid );
    ARIADNE_TEST_EQUAL( expectedOpenCellBoxType, actualOpenCell.box() );
    ARIADNE_TEST_EQUAL( expectedOpenCellWord, actualOpenCell.word() );
    ARIADNE_TEST_EQUAL( expectedOpenCellRootExtent, actualOpenCell.root_extent() );

    //!!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Construct a [0.2,0.3]x[0.7,0.8] box and make and open over approximation, dimension: 2");
    theBox = make_box("[0.2,0.3]x[0.7,0.8]");
    expectedOpenCellBoxType = make_box("[0.1875,0.3125]x[0.6875,0.8125]");
    expectedOpenCellWord = make_binary_word("01001111");
    expectedOpenCellRootExtent = 0;
    actualOpenCell = GridOpenCell::outer_approximation( theBox, theGrid );
    ARIADNE_TEST_EQUAL( expectedOpenCellBoxType, actualOpenCell.box() );
    ARIADNE_TEST_EQUAL( expectedOpenCellWord, actualOpenCell.word() );
    ARIADNE_TEST_EQUAL( expectedOpenCellRootExtent, actualOpenCell.root_extent() );
}

Void test_grid_open_cell_four(){
    //Allocate a trivial Grid two dimensional grid
    Grid theGrid(2, 1.0);

    //!!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Test that the open cell intersects with itself, dimension: 2");
    GridOpenCell leftOpenCell = GridOpenCell( theGrid, 2, make_binary_word( "0011111" ) );
    GridOpenCell rightOpenCell = GridOpenCell( theGrid, 2, make_binary_word( "0011111" ) );
    ARIADNE_TEST_EQUAL( GridOpenCell::intersect( leftOpenCell, rightOpenCell ), true );

    //!!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Test that the open cells [0.0,1.0]x[0.0,1.0] and [-0.25,0.25]x[0.0,1.0] intersect, dimension: 2");
    ExactBoxType expectedLeftOpenCellBoxType = make_box("[0.0,1.0]x[0.0,1.0]");
    leftOpenCell = GridOpenCell( theGrid, 0, make_binary_word( "00" ) );
    ExactBoxType expectedRightOpenCellBoxType = make_box("[-0.25,0.25]x[0.0,1.0]");
    rightOpenCell = GridOpenCell( theGrid, 1, make_binary_word( "01101" ) );
    //First check that the cells result in correct boxes
    ARIADNE_TEST_EQUAL( expectedLeftOpenCellBoxType, leftOpenCell.box() );
    ARIADNE_TEST_EQUAL( expectedRightOpenCellBoxType, rightOpenCell.box() );
    //Test the intersect
    ARIADNE_TEST_EQUAL( GridOpenCell::intersect( leftOpenCell, rightOpenCell ), true );

    //!!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Test that the open cells [0.0,0.5]x[0.5,1.5] and [0.5,1.0]x[0.5,1.5] intersect, dimension: 2");
    expectedLeftOpenCellBoxType = make_box("[0.0,0.5]x[0.5,1.5]");
    leftOpenCell = GridOpenCell( theGrid, 0, make_binary_word( "010" ) );
    expectedRightOpenCellBoxType = make_box("[0.5,1.0]x[0.5,1.5]");
    rightOpenCell = GridOpenCell( theGrid, 0, make_binary_word( "110" ) );
    //First check that the cells result in correct boxes
    ARIADNE_TEST_EQUAL( expectedLeftOpenCellBoxType, leftOpenCell.box() );
    ARIADNE_TEST_EQUAL( expectedRightOpenCellBoxType, rightOpenCell.box() );
    //Test the intersect
    ARIADNE_TEST_EQUAL( GridOpenCell::intersect( leftOpenCell, rightOpenCell ), false );
}

Void test_grid_open_cell_five() {
    //The vector for storing the results
    std::vector< GridCell* > expected_result_arr(4);

    //Allocate a trivial Grid two dimensional grid
    Grid theTrivialGrid(2, 1.0);

    //!!! 00
    ARIADNE_PRINT_TEST_CASE_TITLE("Construct closure of an open cell 00 extent=0, dimension: 2");
    GridOpenCell openCell = GridOpenCell( theTrivialGrid, 0, BinaryWord() );

    Nat theRootExtent = 2;
    expected_result_arr[0] = new GridCell( theTrivialGrid, theRootExtent, make_binary_word("0011") );
    expected_result_arr[1] = new GridCell( theTrivialGrid, theRootExtent, make_binary_word("0110") );
    expected_result_arr[2] = new GridCell( theTrivialGrid, theRootExtent, make_binary_word("1001") );
    expected_result_arr[3] = new GridCell( theTrivialGrid, theRootExtent, make_binary_word("1100") );

    GridTreePaving closureSet = openCell.closure();
    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result_arr, closureSet, 4 );
    ARIADNE_CLEAN_TEST_VECTOR( expected_result_arr );

    //!!! 01
    ARIADNE_PRINT_TEST_CASE_TITLE("Construct closure of an open cell 01 extent=2, dimension: 2");
    openCell = GridOpenCell( theTrivialGrid, 2, make_binary_word( "0011111" ) );

    theRootExtent = 2;
    expected_result_arr[0] = new GridCell( theTrivialGrid, theRootExtent, make_binary_word("0011111") );
    expected_result_arr[1] = new GridCell( theTrivialGrid, theRootExtent, make_binary_word("0110101") );
    expected_result_arr[2] = new GridCell( theTrivialGrid, theRootExtent, make_binary_word("1001010") );
    expected_result_arr[3] = new GridCell( theTrivialGrid, theRootExtent, make_binary_word("1100000") );

    closureSet = openCell.closure();
    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result_arr, closureSet, 4 );

    //!!! 02
    ARIADNE_PRINT_TEST_CASE_TITLE("Construct closure of an open cell 01 extent=1, dimension: 2");
    openCell = GridOpenCell( theTrivialGrid, 1, make_binary_word( "11111" ) );

    closureSet = openCell.closure();
    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result_arr, closureSet, 4 );

    //!!! 03
    ARIADNE_PRINT_TEST_CASE_TITLE("Construct closure of an open cell 01 extent=1, dimension: 2");
    openCell = GridOpenCell( theTrivialGrid, 0, make_binary_word( "111" ) );

    closureSet = openCell.closure();
    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result_arr, closureSet, 4 );

    ARIADNE_CLEAN_TEST_VECTOR( expected_result_arr );

    //!!! 04
    ARIADNE_PRINT_TEST_CASE_TITLE("Construct closure of an open cell 02 extent=2, dimension: 2");
    openCell = GridOpenCell( theTrivialGrid, 2, make_binary_word( "00110101" ) );

    theRootExtent = 2;
    expected_result_arr[0] = new GridCell( theTrivialGrid, theRootExtent, make_binary_word("00110101") );
    expected_result_arr[1] = new GridCell( theTrivialGrid, theRootExtent, make_binary_word("00110111") );
    expected_result_arr[2] = new GridCell( theTrivialGrid, theRootExtent, make_binary_word("01100000") );
    expected_result_arr[3] = new GridCell( theTrivialGrid, theRootExtent, make_binary_word("01100010") );

    closureSet = openCell.closure();
    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result_arr, closureSet, 4 );

    //!!! 05
    ARIADNE_PRINT_TEST_CASE_TITLE("Construct closure of an open cell 02 extent=1, dimension: 2");
    openCell = GridOpenCell( theTrivialGrid, 1, make_binary_word( "110101" ) );

    closureSet = openCell.closure();
    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result_arr, closureSet, 4 );

    //!!! 06
    ARIADNE_PRINT_TEST_CASE_TITLE("Construct closure of an open cell 02 extent=0, dimension: 2");
    openCell = GridOpenCell( theTrivialGrid, 0, make_binary_word( "0101" ) );

    closureSet = openCell.closure();
    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result_arr, closureSet, 4 );

    ARIADNE_CLEAN_TEST_VECTOR( expected_result_arr );

    //!!! 07
    ARIADNE_PRINT_TEST_CASE_TITLE("Construct closure of an open cell 03 extent=2, dimension: 2");
    openCell = GridOpenCell( theTrivialGrid, 2, make_binary_word( "000111111" ) );

    theRootExtent = 2;
    expected_result_arr[0] = new GridCell( theTrivialGrid, theRootExtent, make_binary_word("000111111") );
    expected_result_arr[1] = new GridCell( theTrivialGrid, theRootExtent, make_binary_word("001101010") );
    expected_result_arr[2] = new GridCell( theTrivialGrid, theRootExtent, make_binary_word("010010101") );
    expected_result_arr[3] = new GridCell( theTrivialGrid, theRootExtent, make_binary_word("011000000") );

    closureSet = openCell.closure();
    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result_arr, closureSet, 4 );

    //!!! 08
    ARIADNE_PRINT_TEST_CASE_TITLE("Construct closure of an open cell 03 extent=1, dimension: 2");
    openCell = GridOpenCell( theTrivialGrid, 1, make_binary_word( "0111111" ) );

    closureSet = openCell.closure();
    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result_arr, closureSet, 4 );

    ARIADNE_CLEAN_TEST_VECTOR( expected_result_arr );

    //!!! 09
    ARIADNE_PRINT_TEST_CASE_TITLE("Construct closure of an open cell 04 extent=2, dimension: 2");
    openCell = GridOpenCell( theTrivialGrid, 2, make_binary_word( "00000001" ) );

    theRootExtent = 2;
    expected_result_arr[0] = new GridCell( theTrivialGrid, theRootExtent, make_binary_word("00000001") );
    expected_result_arr[1] = new GridCell( theTrivialGrid, theRootExtent, make_binary_word("00000011") );
    expected_result_arr[2] = new GridCell( theTrivialGrid, theRootExtent, make_binary_word("00000100") );
    expected_result_arr[3] = new GridCell( theTrivialGrid, theRootExtent, make_binary_word("00000110") );

    closureSet = openCell.closure();
    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result_arr, closureSet, 4 );

    //!!! 10
    ARIADNE_PRINT_TEST_CASE_TITLE("Construct closure of an open cell 04 extent=1, dimension: 2");
    openCell = GridOpenCell( theTrivialGrid, 1, make_binary_word( "000001" ) );

    closureSet = openCell.closure();
    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result_arr, closureSet, 4 );

    ARIADNE_CLEAN_TEST_VECTOR( expected_result_arr );

    //!!! 11
    ARIADNE_PRINT_TEST_CASE_TITLE("Construct closure of an open cell 05 extent=2, dimension: 2");
    openCell = GridOpenCell( theTrivialGrid, 2, make_binary_word( "00000011" ) );

    theRootExtent = 2;
    expected_result_arr[0] = new GridCell( theTrivialGrid, theRootExtent, make_binary_word("00000011") );
    expected_result_arr[1] = new GridCell( theTrivialGrid, theRootExtent, make_binary_word("00000110") );
    expected_result_arr[2] = new GridCell( theTrivialGrid, theRootExtent, make_binary_word("00001001") );
    expected_result_arr[3] = new GridCell( theTrivialGrid, theRootExtent, make_binary_word("00001100") );

    closureSet = openCell.closure();
    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result_arr, closureSet, 4 );

    //!!! 12
    ARIADNE_PRINT_TEST_CASE_TITLE("Construct closure of an open cell 05 extent=1, dimension: 2");
    openCell = GridOpenCell( theTrivialGrid, 1, make_binary_word( "000011" ) );

    closureSet = openCell.closure();
    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result_arr, closureSet, 4 );

    ARIADNE_CLEAN_TEST_VECTOR( expected_result_arr );

    //!!! 13
    ARIADNE_PRINT_TEST_CASE_TITLE("Construct closure of an open cell 06 extent=2, dimension: 2");
    openCell = GridOpenCell( theTrivialGrid, 2, make_binary_word( "00000010" ) );

    theRootExtent = 2;
    expected_result_arr[0] = new GridCell( theTrivialGrid, theRootExtent, make_binary_word("00000010") );
    expected_result_arr[1] = new GridCell( theTrivialGrid, theRootExtent, make_binary_word("00000011") );
    expected_result_arr[2] = new GridCell( theTrivialGrid, theRootExtent, make_binary_word("00001000") );
    expected_result_arr[3] = new GridCell( theTrivialGrid, theRootExtent, make_binary_word("00001001") );

    closureSet = openCell.closure();
    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result_arr, closureSet, 4 );

    //!!! 14
    ARIADNE_PRINT_TEST_CASE_TITLE("Construct closure of an open cell 06 extent=1, dimension: 2");
    openCell = GridOpenCell( theTrivialGrid, 1, make_binary_word( "000010" ) );

    closureSet = openCell.closure();
    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result_arr, closureSet, 4 );

    ARIADNE_CLEAN_TEST_VECTOR( expected_result_arr );

    //!!! 15
    ARIADNE_PRINT_TEST_CASE_TITLE("Construct closure of an open cell 07 extent=2, dimension: 2");
    openCell = GridOpenCell( theTrivialGrid, 2, make_binary_word( "00000000" ) );

    theRootExtent = 2;
    expected_result_arr[0] = new GridCell( theTrivialGrid, theRootExtent, make_binary_word("00000000") );
    expected_result_arr[1] = new GridCell( theTrivialGrid, theRootExtent, make_binary_word("00000001") );
    expected_result_arr[2] = new GridCell( theTrivialGrid, theRootExtent, make_binary_word("00000010") );
    expected_result_arr[3] = new GridCell( theTrivialGrid, theRootExtent, make_binary_word("00000011") );

    closureSet = openCell.closure();
    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result_arr, closureSet, 4 );

    //!!! 16
    ARIADNE_PRINT_TEST_CASE_TITLE("Construct closure of an open cell 07 extent=1, dimension: 2");
    openCell = GridOpenCell( theTrivialGrid, 1, make_binary_word( "000000" ) );

    closureSet = openCell.closure();
    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result_arr, closureSet, 4 );

    ARIADNE_CLEAN_TEST_VECTOR( expected_result_arr );

    //!!! 17
    ARIADNE_PRINT_TEST_CASE_TITLE("Construct closure of an open cell 08 extent=2, dimension: 2");
    openCell = GridOpenCell( theTrivialGrid, 2, make_binary_word( "0000001" ) );

    theRootExtent = 2;
    expected_result_arr[0] = new GridCell( theTrivialGrid, theRootExtent, make_binary_word("0000001") );
    expected_result_arr[1] = new GridCell( theTrivialGrid, theRootExtent, make_binary_word("0000011") );
    expected_result_arr[2] = new GridCell( theTrivialGrid, theRootExtent, make_binary_word("0000100") );
    expected_result_arr[3] = new GridCell( theTrivialGrid, theRootExtent, make_binary_word("0000110") );

    closureSet = openCell.closure();
    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result_arr, closureSet, 4 );

    //!!! 18
    ARIADNE_PRINT_TEST_CASE_TITLE("Construct closure of an open cell 08 extent=1, dimension: 2");
    openCell = GridOpenCell( theTrivialGrid, 1, make_binary_word( "00001" ) );

    closureSet = openCell.closure();
    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result_arr, closureSet, 4 );

    ARIADNE_CLEAN_TEST_VECTOR( expected_result_arr );

    //!!! 19
    ARIADNE_PRINT_TEST_CASE_TITLE("Construct closure of an open cell 09 extent=2, dimension: 2");
    openCell = GridOpenCell( theTrivialGrid, 2, make_binary_word( "0000" ) );

    theRootExtent = 2;
    expected_result_arr[0] = new GridCell( theTrivialGrid, theRootExtent, make_binary_word("0000") );
    expected_result_arr[1] = new GridCell( theTrivialGrid, theRootExtent, make_binary_word("0001") );
    expected_result_arr[2] = new GridCell( theTrivialGrid, theRootExtent, make_binary_word("0010") );
    expected_result_arr[3] = new GridCell( theTrivialGrid, theRootExtent, make_binary_word("0011") );

    closureSet = openCell.closure();
    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result_arr, closureSet, 4 );

    //!!! 20
    ARIADNE_PRINT_TEST_CASE_TITLE("Construct closure of an open cell 09 extent=1, dimension: 2");
    openCell = GridOpenCell( theTrivialGrid, 1, make_binary_word( "00" ) );

    closureSet = openCell.closure();
    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result_arr, closureSet, 4 );

    ARIADNE_CLEAN_TEST_VECTOR( expected_result_arr );
}

Void test_grid_open_cell_six() {
    std::vector<GridOpenCell> expectedResult;
    std::vector<GridOpenCell> actualResult;
    Nat theRootExtent;

    //Allocate a trivial Grid two dimensional grid
    Grid theTrivialGrid(2, 1.0);

    //!!! 00
    ARIADNE_PRINT_TEST_CASE_TITLE("Intersect the open cell with itself, dimension: 2");
    GridOpenCell openCellOne = GridOpenCell( theTrivialGrid, 0, BinaryWord() );
    GridOpenCell openCellTwo = GridOpenCell( theTrivialGrid, 0, BinaryWord() );

    theRootExtent = 2;
    expectedResult.push_back( GridOpenCell( theTrivialGrid, theRootExtent, make_binary_word( "001100" ) ) );
    expectedResult.push_back( GridOpenCell( theTrivialGrid, theRootExtent, make_binary_word( "001101" ) ) );
    expectedResult.push_back( GridOpenCell( theTrivialGrid, theRootExtent, make_binary_word( "001110" ) ) );
    expectedResult.push_back( GridOpenCell( theTrivialGrid, theRootExtent, make_binary_word( "001111" ) ) );
    expectedResult.push_back( GridOpenCell( theTrivialGrid, theRootExtent, make_binary_word( "011000" ) ) );
    expectedResult.push_back( GridOpenCell( theTrivialGrid, theRootExtent, make_binary_word( "011010" ) ) );
    expectedResult.push_back( GridOpenCell( theTrivialGrid, theRootExtent, make_binary_word( "100100" ) ) );
    expectedResult.push_back( GridOpenCell( theTrivialGrid, theRootExtent, make_binary_word( "100101" ) ) );
    expectedResult.push_back( GridOpenCell( theTrivialGrid, theRootExtent, make_binary_word( "110000" ) ) );

    actualResult = GridOpenCell::intersection( openCellOne, openCellTwo );

    ARIADNE_TEST_EQUAL( expectedResult, actualResult );
    expectedResult.erase( expectedResult.begin(), expectedResult.end() );

    //!!! 01
    ARIADNE_PRINT_TEST_CASE_TITLE("Intersect an open cell with an open cell that is a robust subset, dimension: 2");
    openCellOne = GridOpenCell( theTrivialGrid, 0, make_binary_word( "00" ) );
    openCellTwo = GridOpenCell( theTrivialGrid, 0, make_binary_word( "0011" ) );

    theRootExtent = 0;
    expectedResult.push_back( GridOpenCell( theTrivialGrid, theRootExtent, make_binary_word( "0011" ) ) );

    actualResult = GridOpenCell::intersection( openCellOne, openCellTwo );

    ARIADNE_TEST_EQUAL( expectedResult, actualResult );
    expectedResult.erase( expectedResult.begin(), expectedResult.end() );

    //!!! 02
    ARIADNE_PRINT_TEST_CASE_TITLE("Intersect two intersectping open cells, dimension: 2");
    openCellOne = GridOpenCell( theTrivialGrid, 0, make_binary_word( "00" ) );
    openCellTwo = GridOpenCell( theTrivialGrid, 0, make_binary_word( "1011" ) );

    theRootExtent = 0;
    expectedResult.push_back( GridOpenCell( theTrivialGrid, theRootExtent, make_binary_word( "101100" ) ) );
    expectedResult.push_back( GridOpenCell( theTrivialGrid, theRootExtent, make_binary_word( "101101" ) ) );
    expectedResult.push_back( GridOpenCell( theTrivialGrid, theRootExtent, make_binary_word( "111000" ) ) );

    actualResult = GridOpenCell::intersection( openCellOne, openCellTwo );

    ARIADNE_TEST_EQUAL( expectedResult, actualResult );
    expectedResult.erase( expectedResult.begin(), expectedResult.end() );

    //!!! 03
    ARIADNE_PRINT_TEST_CASE_TITLE("Intersect two non-intersectping open cells, dimension: 2");
    openCellOne = GridOpenCell( theTrivialGrid, 0, make_binary_word( "0000" ) );
    openCellTwo = GridOpenCell( theTrivialGrid, 0, make_binary_word( "1000" ) );

    actualResult = GridOpenCell::intersection( openCellOne, openCellTwo );

    ARIADNE_TEST_EQUAL( expectedResult, actualResult );
    expectedResult.erase( expectedResult.begin(), expectedResult.end() );

    //!!! 04
    ARIADNE_PRINT_TEST_CASE_TITLE("Intersect two intersectping open cells whoes intersection is just one GridCell, dimension: 2");
    openCellOne = GridOpenCell( theTrivialGrid, 0, make_binary_word( "0000" ) );
    openCellTwo = GridOpenCell( theTrivialGrid, 2, make_binary_word( "0001101111" ) );

    theRootExtent = 0;
    expectedResult.push_back( GridOpenCell( theTrivialGrid, theRootExtent, make_binary_word( "00010100" ) ) );

    actualResult = GridOpenCell::intersection( openCellOne, openCellTwo );

    ARIADNE_TEST_EQUAL( expectedResult, actualResult );
    expectedResult.erase( expectedResult.begin(), expectedResult.end() );

    //!!! 05
    ARIADNE_PRINT_TEST_CASE_TITLE("Intersect two intersectping open cells based on x-splitted primary cells, dimension: 2");
    openCellOne = GridOpenCell( theTrivialGrid, 0, make_binary_word( "0" ) );
    openCellTwo = GridOpenCell( theTrivialGrid, 0, make_binary_word( "1" ) );

    theRootExtent = 2;
    expectedResult.push_back( GridOpenCell( theTrivialGrid, theRootExtent, make_binary_word( "0011100" ) ) );
    expectedResult.push_back( GridOpenCell( theTrivialGrid, theRootExtent, make_binary_word( "0011110" ) ) );
    expectedResult.push_back( GridOpenCell( theTrivialGrid, theRootExtent, make_binary_word( "0110100" ) ) );

    actualResult = GridOpenCell::intersection( openCellOne, openCellTwo );

    ARIADNE_TEST_EQUAL( expectedResult, actualResult );
    expectedResult.erase( expectedResult.begin(), expectedResult.end() );
}

Void test_grid_open_cell_one(){
    //Allocate a trivial Grid two dimensional grid
    Grid theTrivialGrid(2, 1.0);

    //!!! 00
    ARIADNE_PRINT_TEST_CASE_TITLE("Constructing a trivial open cell, dimension: 2");
    GridOpenCell trivialOpenCell = GridOpenCell( theTrivialGrid, 0, BinaryWord() );
    ExactBoxType expected_trivial_open_cell_box = make_box("[0.0,2.0]x[0.0,2.0]");
    ARIADNE_PRINT_TEST_COMMENT("The trivial open cell, as given by its box:");
    ARIADNE_TEST_EQUAL( expected_trivial_open_cell_box, trivialOpenCell.box() );

    //!!! 01
    ARIADNE_PRINT_TEST_CASE_TITLE("Constructing an open cell 01 extent=2, dimension: 2");
    GridOpenCell openCell01 = GridOpenCell( theTrivialGrid, 2, make_binary_word( "0011111" ) );
    ExactBoxType expected_open_cell_box01 = make_box("[0.75,1.25]x[0.5,1.5]");
    ARIADNE_PRINT_TEST_COMMENT("The open cell, as given by its box:");
    ARIADNE_TEST_EQUAL( expected_open_cell_box01, openCell01.box() );

    //!!! 02
    ARIADNE_PRINT_TEST_CASE_TITLE("Constructing an open cell 01 extent=1, dimension: 2");
    GridOpenCell openCell02 = GridOpenCell( theTrivialGrid, 1, make_binary_word( "11111" ) );
    ExactBoxType expected_open_cell_box02 = make_box("[0.75,1.25]x[0.5,1.5]");
    ARIADNE_PRINT_TEST_COMMENT("The open cell, as given by its box:");
    ARIADNE_TEST_EQUAL( expected_open_cell_box02, openCell02.box() );

    //!!! 03
    ARIADNE_PRINT_TEST_CASE_TITLE("Constructing an open cell 01 extent=0, dimension: 2");
    GridOpenCell openCell03 = GridOpenCell( theTrivialGrid, 0, make_binary_word( "111" ) );
    ExactBoxType expected_open_cell_box03 = make_box("[0.75,1.25]x[0.5,1.5]");
    ARIADNE_PRINT_TEST_COMMENT("The open cell, as given by its box:");
    ARIADNE_TEST_EQUAL( expected_open_cell_box03, openCell03.box() );

    //!!! 04
    ARIADNE_PRINT_TEST_CASE_TITLE("Constructing an open cell 02 extent=2, dimension: 2");
    GridOpenCell openCell04 = GridOpenCell( theTrivialGrid, 2, make_binary_word( "00110101" ) );
    ExactBoxType expected_open_cell_box04 = make_box("[0.0,0.5]x[0.75,1.25]");
    ARIADNE_PRINT_TEST_COMMENT("The open cell, as given by its box:");
    ARIADNE_TEST_EQUAL( expected_open_cell_box04, openCell04.box() );

    //!!! 05
    ARIADNE_PRINT_TEST_CASE_TITLE("Constructing an open cell 02 extent=1, dimension: 2");
    GridOpenCell openCell05 = GridOpenCell( theTrivialGrid, 1, make_binary_word( "110101" ) );
    ExactBoxType expected_open_cell_box05 = make_box("[0.0,0.5]x[0.75,1.25]");
    ARIADNE_PRINT_TEST_COMMENT("The open cell, as given by its box:");
    ARIADNE_TEST_EQUAL( expected_open_cell_box05, openCell05.box() );

    //!!! 06
    ARIADNE_PRINT_TEST_CASE_TITLE("Constructing an open cell 02 extent=0, dimension: 2");
    GridOpenCell openCell06 = GridOpenCell( theTrivialGrid, 0, make_binary_word( "0101" ) );
    ExactBoxType expected_open_cell_box06 = make_box("[0.0,0.5]x[0.75,1.25]");
    ARIADNE_PRINT_TEST_COMMENT("The open cell, as given by its box:");
    ARIADNE_TEST_EQUAL( expected_open_cell_box06, openCell06.box() );

    //!!! 07
    ARIADNE_PRINT_TEST_CASE_TITLE("Constructing an open cell 03 extent=2, dimension: 2");
    GridOpenCell openCell07 = GridOpenCell( theTrivialGrid, 2, make_binary_word( "000111111" ) );
    ExactBoxType expected_open_cell_box07 = make_box("[-0.125,0.125]x[0.75,1.25]");
    ARIADNE_PRINT_TEST_COMMENT("The open cell, as given by its box:");
    ARIADNE_TEST_EQUAL( expected_open_cell_box07, openCell07.box() );

    //!!! 08
    ARIADNE_PRINT_TEST_CASE_TITLE("Constructing an open cell 03 extent=1, dimension: 2");
    GridOpenCell openCell08 = GridOpenCell( theTrivialGrid, 1, make_binary_word( "0111111" ) );
    ExactBoxType expected_open_cell_box08 = make_box("[-0.125,0.125]x[0.75,1.25]");
    ARIADNE_PRINT_TEST_COMMENT("The open cell, as given by its box:");
    ARIADNE_TEST_EQUAL( expected_open_cell_box08, openCell08.box() );

    //!!! 09
    ARIADNE_PRINT_TEST_CASE_TITLE("Constructing an open cell 04 extent=2, dimension: 2");
    GridOpenCell openCell09 = GridOpenCell( theTrivialGrid, 2, make_binary_word( "00000001" ) );
    ExactBoxType expected_open_cell_box09 = make_box("[-1.0,-0.5]x[-0.75,-0.25]");
    ARIADNE_PRINT_TEST_COMMENT("The open cell, as given by its box:");
    ARIADNE_TEST_EQUAL( expected_open_cell_box09, openCell09.box() );

    //!!! 10
    ARIADNE_PRINT_TEST_CASE_TITLE("Constructing an open cell 04 extent=1, dimension: 2");
    GridOpenCell openCell10 = GridOpenCell( theTrivialGrid, 1, make_binary_word( "000001" ) );
    ExactBoxType expected_open_cell_box10 = make_box("[-1.0,-0.5]x[-0.75,-0.25]");
    ARIADNE_PRINT_TEST_COMMENT("The open cell, as given by its box:");
    ARIADNE_TEST_EQUAL( expected_open_cell_box10, openCell10.box() );

    //!!! 11
    ARIADNE_PRINT_TEST_CASE_TITLE("Constructing an open cell 05 extent=2, dimension: 2");
    GridOpenCell openCell11 = GridOpenCell( theTrivialGrid, 2, make_binary_word( "00000011" ) );
    ExactBoxType expected_open_cell_box11 = make_box("[-0.75,-0.25]x[-0.75,-0.25]");
    ARIADNE_PRINT_TEST_COMMENT("The open cell, as given by its box:");
    ARIADNE_TEST_EQUAL( expected_open_cell_box11, openCell11.box() );

    //!!! 12
    ARIADNE_PRINT_TEST_CASE_TITLE("Constructing an open cell 05 extent=1, dimension: 2");
    GridOpenCell openCell12 = GridOpenCell( theTrivialGrid, 1, make_binary_word( "000011" ) );
    ExactBoxType expected_open_cell_box12 = make_box("[-0.75,-0.25]x[-0.75,-0.25]");
    ARIADNE_PRINT_TEST_COMMENT("The open cell, as given by its box:");
    ARIADNE_TEST_EQUAL( expected_open_cell_box12, openCell12.box() );

    //!!! 13
    ARIADNE_PRINT_TEST_CASE_TITLE("Constructing an open cell 06 extent=2, dimension: 2");
    GridOpenCell openCell13 = GridOpenCell( theTrivialGrid, 2, make_binary_word( "00000010" ) );
    ExactBoxType expected_open_cell_box13 = make_box("[-0.75,-0.25]x[-1.0,-0.5]");
    ARIADNE_PRINT_TEST_COMMENT("The open cell, as given by its box:");
    ARIADNE_TEST_EQUAL( expected_open_cell_box13, openCell13.box() );

    //!!! 14
    ARIADNE_PRINT_TEST_CASE_TITLE("Constructing an open cell 06 extent=1, dimension: 2");
    GridOpenCell openCell14 = GridOpenCell( theTrivialGrid, 1, make_binary_word( "000010" ) );
    ExactBoxType expected_open_cell_box14 = make_box("[-0.75,-0.25]x[-1.0,-0.5]");
    ARIADNE_PRINT_TEST_COMMENT("The open cell, as given by its box:");
    ARIADNE_TEST_EQUAL( expected_open_cell_box14, openCell14.box() );

    //!!! 15
    ARIADNE_PRINT_TEST_CASE_TITLE("Constructing an open cell 07 extent=2, dimension: 2");
    GridOpenCell openCell15 = GridOpenCell( theTrivialGrid, 2, make_binary_word( "00000000" ) );
    ExactBoxType expected_open_cell_box15 = make_box("[-1.0,-0.5]x[-1.0,-0.5]");
    ARIADNE_PRINT_TEST_COMMENT("The open cell, as given by its box:");
    ARIADNE_TEST_EQUAL( expected_open_cell_box15, openCell15.box() );

    //!!! 16
    ARIADNE_PRINT_TEST_CASE_TITLE("Constructing an open cell 07 extent=1, dimension: 2");
    GridOpenCell openCell16 = GridOpenCell( theTrivialGrid, 1, make_binary_word( "000000" ) );
    ExactBoxType expected_open_cell_box16 = make_box("[-1.0,-0.5]x[-1.0,-0.5]");
    ARIADNE_PRINT_TEST_COMMENT("The open cell, as given by its box:");
    ARIADNE_TEST_EQUAL( expected_open_cell_box16, openCell16.box() );

    //!!! 17
    ARIADNE_PRINT_TEST_CASE_TITLE("Constructing an open cell 08 extent=2, dimension: 2");
    GridOpenCell openCell17 = GridOpenCell( theTrivialGrid, 2, make_binary_word( "0000001" ) );
    ExactBoxType expected_open_cell_box17 = make_box("[-0.75,-0.25]x[-1.0,0.0]");
    ARIADNE_PRINT_TEST_COMMENT("The open cell, as given by its box:");
    ARIADNE_TEST_EQUAL( expected_open_cell_box17, openCell17.box() );

    //!!! 18
    ARIADNE_PRINT_TEST_CASE_TITLE("Constructing an open cell 08 extent=1, dimension: 2");
    GridOpenCell openCell18 = GridOpenCell( theTrivialGrid, 1, make_binary_word( "00001" ) );
    ExactBoxType expected_open_cell_box18 = make_box("[-0.75,-0.25]x[-1.0,0.0]");
    ARIADNE_PRINT_TEST_COMMENT("The open cell, as given by its box:");
    ARIADNE_TEST_EQUAL( expected_open_cell_box18, openCell18.box() );

    //!!! 19
    ARIADNE_PRINT_TEST_CASE_TITLE("Constructing an open cell 09 extent=2, dimension: 2");
    GridOpenCell openCell19 = GridOpenCell( theTrivialGrid, 2, make_binary_word( "0000" ) );
    ExactBoxType expected_open_cell_box19 = make_box("[-1.0,1.0]x[-1.0,1.0]");
    ARIADNE_PRINT_TEST_COMMENT("The open cell, as given by its box:");
    ARIADNE_TEST_EQUAL( expected_open_cell_box19, openCell19.box() );

    //!!! 20
    ARIADNE_PRINT_TEST_CASE_TITLE("Constructing an open cell 09 extent=1, dimension: 2");
    GridOpenCell openCell20 = GridOpenCell( theTrivialGrid, 1, make_binary_word( "00" ) );
    ExactBoxType expected_open_cell_box20 = make_box("[-1.0,1.0]x[-1.0,1.0]");
    ARIADNE_PRINT_TEST_COMMENT("The open cell, as given by its box:");
    ARIADNE_TEST_EQUAL( expected_open_cell_box20, openCell20.box() );
}

Void test_adjoin_operation_one(){
    //Allocate a trivial Grid
    Grid theGrid(2, 1.0);

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Test adjoining a GridCell to the GridTreePaving");
    //Define the GridCell that is rooted to a high primary cell
    const Int theHigherCellExtent = 2;
    BinaryWord theHigherCellPath;
    theHigherCellPath.push_back(true);
    theHigherCellPath.push_back(false);
    theHigherCellPath.push_back(true);
    theHigherCellPath.push_back(false);
    GridCell theHigherLevelCell( theGrid, theHigherCellExtent, theHigherCellPath );

    ARIADNE_PRINT_TEST_COMMENT("The GridCell with the primary root cell extent = 2");
    ExactBoxType expected_box = make_box("[2,3]x[-1,0]");
    ARIADNE_PRINT_TEST_COMMENT("The initial GridCell, as given by it's box: ");
    ARIADNE_TEST_EQUAL( expected_box, theHigherLevelCell.box() );

    //Define the higth of the primary root cell.
    //Create the GridTreePaving with the box related to the grid, but not to the original space
    GridTreePaving theOneCellPaving( theGrid, true );
    ARIADNE_PRINT_TEST_COMMENT(theOneCellPaving);
    ARIADNE_PRINT_TEST_COMMENT("The GridTreePaving with the primary root cell extent = 0");
    GridTreePaving expected_one_cell_paving( theGrid, 0, make_binary_word("0"), make_binary_word("1") );
    ARIADNE_TEST_EQUAL( expected_one_cell_paving, theOneCellPaving );

    ARIADNE_PRINT_TEST_COMMENT("The GridTreePaving after adding the cell: ");
    theOneCellPaving.adjoin( theHigherLevelCell );
    GridTreePaving expected_two_cell_paving( theGrid, 2, make_binary_word("111010001101000"), make_binary_word("00100100") );
    ARIADNE_TEST_EQUAL( expected_two_cell_paving, theOneCellPaving );
}

Void test_adjoin_operation_two(){

    //Allocate a trivial Grid
    Grid theGrid(2, 1.0);

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Test adjoining a GridCell to the GridTreePaving");
    //Define the GridCell that is rooted to the lower primary cell
    const Int theLowerCellExtent = 1;
    BinaryWord theLowerCellPath;
    theLowerCellPath.push_back(true);
    theLowerCellPath.push_back(true);
    GridCell theLowerLevelCell( theGrid, theLowerCellExtent, theLowerCellPath );

    ARIADNE_PRINT_TEST_COMMENT("The GridCell with the primary root cell extent = 1");
    ExactBoxType expected_box = make_box("[0,1]x[0,1]");
    ARIADNE_PRINT_TEST_COMMENT("The box of the initial GridCell: ");
    ARIADNE_TEST_EQUAL( expected_box, theLowerLevelCell.box() );

    //Define the higth of the primary root cell.
    const Nat theExtent = 2;
    //Create the binary tree;
    BinaryTreeNode * pRootTreeNode = new BinaryTreeNode(false);
    pRootTreeNode->split();
    pRootTreeNode->right_node()->split();
    pRootTreeNode->right_node()->left_node()->split();
    pRootTreeNode->right_node()->left_node()->right_node()->split();
    pRootTreeNode->right_node()->left_node()->right_node()->left_node()->set_enabled();

    //Create the GridTreePaving with the box is related to the grid, but not to the original space
    GridTreePaving theOneCellPaving( theGrid, theExtent, pRootTreeNode );
    ARIADNE_PRINT_TEST_COMMENT("The GridTreePaving with the primary root cell extent = 2");
    GridTreePaving expected_tree_set( theGrid, theExtent, make_binary_word("101101000"), make_binary_word("00100") );
    ARIADNE_TEST_EQUAL( expected_tree_set, theOneCellPaving );

    ARIADNE_PRINT_TEST_COMMENT("The GridTreePaving after adding the cell: ");
    theOneCellPaving.adjoin( theLowerLevelCell );
    GridTreePaving expected_tree_set_result( theGrid, theExtent, make_binary_word("111010001101000"), make_binary_word("00100100") );
    ARIADNE_TEST_EQUAL( expected_tree_set_result, theOneCellPaving );
}

Void test_adjoin_operation_three(){
    string expected_result;

    //Allocate a trivial Grid
    Grid theGrid(2, 1.0);

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Test adjoining a GridCell to the GridTreePaving");
    //Define the GridCell that is rooted to the same primary cell
    const Int theLowerCellExtent = 2;
    BinaryWord theLowerCellPath;
    theLowerCellPath.push_back(false);
    theLowerCellPath.push_back(false);
    theLowerCellPath.push_back(true);
    theLowerCellPath.push_back(true);
    GridCell theLowerLevelCell( theGrid, theLowerCellExtent, theLowerCellPath );

    ARIADNE_PRINT_TEST_COMMENT("The GridCell with the primary root cell extent = 2");
    ExactBoxType expected_box = make_box("[0,1]x[0,1]");
    ARIADNE_PRINT_TEST_COMMENT("The initial GridCell: ");
    ARIADNE_TEST_EQUAL( expected_box, theLowerLevelCell.box() );

    //Define the higth of the primary root cell.
    const Nat theExtent = 2;
    //Create the binary tree;
    BinaryTreeNode * pRootTreeNode = new BinaryTreeNode(false);
    pRootTreeNode->split();
    pRootTreeNode->right_node()->split();
    pRootTreeNode->right_node()->left_node()->split();
    pRootTreeNode->right_node()->left_node()->right_node()->split();
    pRootTreeNode->right_node()->left_node()->right_node()->left_node()->set_enabled();

    //Create the GridTreePaving with the box is related to the grid, but not to the original space
    ARIADNE_PRINT_TEST_COMMENT("The GridTreePaving with the primary root cell extent = 2");
    GridTreePaving theOneCellPaving( theGrid, theExtent, pRootTreeNode );
    GridTreePaving expected_tree_set( theGrid, theExtent, make_binary_word("101101000"), make_binary_word("00100") );
    ARIADNE_TEST_EQUAL( expected_tree_set, theOneCellPaving );

    ARIADNE_PRINT_TEST_COMMENT("The GridTreePaving after adding the cell: ");
    theOneCellPaving.adjoin( theLowerLevelCell );
    GridTreePaving expected_tree_set_result( theGrid, theExtent, make_binary_word("111010001101000"), make_binary_word("00100100") );
    ARIADNE_TEST_EQUAL( expected_tree_set_result, theOneCellPaving );
}

Void test_adjoin_outer_approximation_operation(){
    //Allocate a trivial Grid
    Grid theTrivialGrid(2, 1.0);

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Test adjoining_outer_approximation a SetInterface to the GridTreePaving");
    ExactBoxType initialRectangle( make_box("[-0.5,1.5]x[-0.3,1.0]") );

    //Define the higth of the primary root cell.
    const Nat theExtent = 2;
    //Create the binary tree;
    BinaryTreeNode * pRootTreeNode = new BinaryTreeNode(false);
    pRootTreeNode->split();
    pRootTreeNode->right_node()->split();
    pRootTreeNode->right_node()->left_node()->split();
    pRootTreeNode->right_node()->left_node()->right_node()->split();
    pRootTreeNode->right_node()->left_node()->right_node()->left_node()->set_enabled();

    //Create the GridTreePaving with the box related to the grid, but not to the original space
    ARIADNE_PRINT_TEST_COMMENT("The initial GridTreePaving with the primary root cell extent = 2");
    GridTreePaving theOneCellPaving( theTrivialGrid, theExtent, pRootTreeNode );
    BinaryWord tree = make_binary_word("101101000");
    BinaryWord leaves = make_binary_word("00100");
    GridTreePaving expected_grid_tree_set1( theTrivialGrid, theExtent, tree, leaves );
    ARIADNE_TEST_EQUAL( expected_grid_tree_set1, theOneCellPaving );

    ARIADNE_PRINT_TEST_COMMENT("The GridTreePaving after adding the cell: ");
    theOneCellPaving.adjoin_outer_approximation( initialRectangle, 1 );
    tree = make_binary_word("1110101111110010011001001110010011001001111001000111001000111110010011001001001111001000000");
    leaves = make_binary_word("0001011111010111111010010100010111111010100000");
    GridTreePaving expected_grid_tree_set2( theTrivialGrid, 4, tree, leaves );
    ARIADNE_TEST_EQUAL( expected_grid_tree_set2, theOneCellPaving );

    ARIADNE_PRINT_TEST_COMMENT("Recombined GridTreePaving after adding the cell: ");
    std::vector< GridCell* > expected_result_arr(16);
    expected_result_arr[0] = new GridCell( theTrivialGrid, 4, make_binary_word("[0,0,1,1,0,0,0,0,0,1]") );
    expected_result_arr[1] = new GridCell( theTrivialGrid, 4, make_binary_word("[0,0,1,1,0,0,0,0,1,1]") );
    expected_result_arr[2] = new GridCell( theTrivialGrid, 4, make_binary_word("[0,0,1,1,0,0,0,1]") );
    expected_result_arr[3] = new GridCell( theTrivialGrid, 4, make_binary_word("[0,0,1,1,0,0,1,0,0,1]") );
    expected_result_arr[4] = new GridCell( theTrivialGrid, 4, make_binary_word("[0,0,1,1,0,0,1,0,1,1]") );
    expected_result_arr[5] = new GridCell( theTrivialGrid, 4, make_binary_word("[0,0,1,1,0,0,1,1]") );
    expected_result_arr[6] = new GridCell( theTrivialGrid, 4, make_binary_word("[0,0,1,1,0,1,0,0,0,0]") );
    expected_result_arr[7] = new GridCell( theTrivialGrid, 4, make_binary_word("[0,0,1,1,0,1,0,0,1,0]") );
    expected_result_arr[8] = new GridCell( theTrivialGrid, 4, make_binary_word("[0,0,1,1,0,1,1,0,0,0]") );
    expected_result_arr[9] = new GridCell( theTrivialGrid, 4, make_binary_word("[0,0,1,1,0,1,1,0,1,0]") );
    expected_result_arr[10] = new GridCell( theTrivialGrid, 4, make_binary_word("[0,0,1,1,1,0,0,0,0,1]") );
    expected_result_arr[11] = new GridCell( theTrivialGrid, 4, make_binary_word("[0,0,1,1,1,0,0,0,1,1]") );
    expected_result_arr[12] = new GridCell( theTrivialGrid, 4, make_binary_word("[0,0,1,1,1,0,0,1]") );
    expected_result_arr[13] = new GridCell( theTrivialGrid, 4, make_binary_word("[0,0,1,1,1,0,1,0]") );
    expected_result_arr[14] = new GridCell( theTrivialGrid, 4, make_binary_word("[0,0,1,1,1,1,0,0,0,0]") );
    expected_result_arr[15] = new GridCell( theTrivialGrid, 4, make_binary_word("[0,0,1,1,1,1,0,0,1,0]") );
    theOneCellPaving.recombine();
    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result_arr, theOneCellPaving, 16 );
    ARIADNE_CLEAN_TEST_VECTOR( expected_result_arr );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Create an outer_approximation of the rectangle on the scaling grid and get the GridTreePaving");
    Grid theScalingGrid(2, 2.0);
    GridTreePaving theOuterApproxGridTreePaving = outer_approximation( initialRectangle, theScalingGrid, 1 );
    //IVAN S. ZAPREEV
    //NOTE: The recombination is needed because in the scaling Grid doing
    //    outer_approximation( theScalingGrid, initialRectangle, 2 )
    //will result in subdivisions that are equal to unit cell in the original
    //space in this case we will get much more elements, e.g. the cells
    //  [-1,0]x[0,2], [0,2]x[0,2] will be subdivided as well
    theOuterApproxGridTreePaving.recombine();
    expected_result_arr[0] = new GridCell( theScalingGrid, 3, make_binary_word("[1,1,0,0,0,0,1,1]") );
    expected_result_arr[1] = new GridCell( theScalingGrid, 3, make_binary_word("[1,1,0,0,0,1,1]") );
    expected_result_arr[2] = new GridCell( theScalingGrid, 3, make_binary_word("[1,1,0,0,1,0,0,1]") );
    expected_result_arr[3] = new GridCell( theScalingGrid, 3, make_binary_word("[1,1,0,0,1,0,1,1]") );
    expected_result_arr[4] = new GridCell( theScalingGrid, 3, make_binary_word("[1,1,0,0,1,1]") );
    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result_arr, theOuterApproxGridTreePaving, 5 );
    ARIADNE_CLEAN_TEST_VECTOR( expected_result_arr );
}

Void test_adjoin_inner_approximation_operation_one(){
    //Allocate a trivial Grid
    Grid theTrivialGrid(2, 1.0);

    //Create an empty set to which we will be adding inner approximations
    //theSetTwo = empty, with the bounding box [0,1]x[0,1]
    GridTreePaving theSetZero( theTrivialGrid, extentZero, new BinaryTreeNode( make_binary_word("0"), make_binary_word("0") ) );
    GridTreePaving theSetZeroCopy( theSetZero );
    ExactBoxSetType theBoxZeroOne = make_box("[-0.9,-0.1]x[0.1,0.9]");
    ExactBoxType theBoundingBoxZeroOne = make_box("[0.01,0.99]x[0.01,0.99]");

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE(" theSetZero.adjoin_inner_approximation( theBoxZeroOne, 0, 4) ");
    ARIADNE_PRINT_TEST_COMMENT("theSetZero");
    cout << theSetZero << endl;
    ARIADNE_PRINT_TEST_COMMENT("theBoxZeroOne");
    cout << theBoxZeroOne << endl;
    ARIADNE_PRINT_TEST_COMMENT("Nothing should be added since nothing of theBoxZeroOne intersects with the primary cell of height zero.");
    theSetZero.adjoin_inner_approximation( theBoxZeroOne, extentZero, extentFour);
    ARIADNE_TEST_EQUAL( theSetZero , theSetZeroCopy );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE(" theSetZero.adjoin_inner_approximation( theBoxZeroOne, theBoundingBoxZeroOne, 4) ");
    ARIADNE_PRINT_TEST_COMMENT("theBoundingBoxZeroOne");
    cout << theBoundingBoxZeroOne << endl;
    ARIADNE_PRINT_TEST_COMMENT("Nothing should be added since theBoundingBoxZeroOne is rooted to the primary cell of height zero.");
    theSetZero.adjoin_inner_approximation( theBoxZeroOne, theBoundingBoxZeroOne, 4);
    ARIADNE_TEST_EQUAL( theSetZero, theSetZeroCopy );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE(" theSetZero.adjoin_inner_approximation( theBoxZeroOne, 1, 2) ");
    ARIADNE_PRINT_TEST_COMMENT("The complete inner approximation of theBoxZeroOne should be added sine it is enclosed in the primary cell of height one.");
    theSetZero.adjoin_inner_approximation( theBoxZeroOne, extentOne, 2);
    std::vector< GridCell* > expected_result_arr( 4 );
    expected_result_arr[0] = new GridCell( theTrivialGrid, extentOne, make_binary_word("010011") );
    expected_result_arr[1] = new GridCell( theTrivialGrid, extentOne, make_binary_word("010110") );
    expected_result_arr[2] = new GridCell( theTrivialGrid, extentOne, make_binary_word("011001") );
    expected_result_arr[3] = new GridCell( theTrivialGrid, extentOne, make_binary_word("011100") );
    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result_arr, theSetZero, 4 );
    ARIADNE_CLEAN_TEST_VECTOR( expected_result_arr );
}

Void test_adjoin_inner_approximation_operation_two(){
    //Allocate a trivial Grid
    Grid theTrivialGrid(2, 1.0);

    //Create an empty set to which we will be adding inner approximations
    //theSetTwo = empty, with the bounding box [-1,1]x[-1,1]
    GridTreePaving theSetOne( theTrivialGrid, extentOne, new BinaryTreeNode( make_binary_word("0"), make_binary_word("0") ) );
    GridTreePaving theSetOneCopy( theSetOne );
    ExactBoxSetType theBoxOneOne = make_box("[-1.9,-0.1]x[0.1,1.75]");
    ExactBoxType theBoundingBoxOneOne = make_box("[-0.99,0.99]x[-0.99,1.99]");

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE(" theSetOne.adjoin_inner_approximation( theBoxOneOne, 1, 2) ");
    ARIADNE_PRINT_TEST_COMMENT("theSetOne");
    cout << theSetOne << endl;
    ARIADNE_PRINT_TEST_COMMENT("theBoxOneOne");
    cout << theBoxOneOne << endl;
    theSetOne.adjoin_inner_approximation( theBoxOneOne, extentOne, 2);
    std::vector< GridCell* > expected_result_arr( 7 );
    expected_result_arr[0] = new GridCell( theTrivialGrid, extentOne, make_binary_word("010001") );
    expected_result_arr[1] = new GridCell( theTrivialGrid, extentOne, make_binary_word("010011") );
    expected_result_arr[2] = new GridCell( theTrivialGrid, extentOne, make_binary_word("0101") );
    expected_result_arr[3] = new GridCell( theTrivialGrid, extentOne, make_binary_word("011001") );
    expected_result_arr[4] = new GridCell( theTrivialGrid, extentOne, make_binary_word("01110") );
    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result_arr, theSetOne, 5 );
    ARIADNE_CLEAN_TEST_VECTOR( expected_result_arr );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE(" theSetOneCopy.adjoin_inner_approximation( theBoxOneOne, theBoundingBoxOneOne, 2) ");
    ARIADNE_PRINT_TEST_COMMENT("theBoundingBoxZeroOne");
    cout << theBoundingBoxOneOne << endl;
    ARIADNE_PRINT_TEST_COMMENT("Nothing should be added since theBoundingBoxOneOne is rooted to the primary cell of height two.");
    theSetOneCopy.adjoin_inner_approximation( theBoxOneOne, theBoundingBoxOneOne, 2);
    expected_result_arr[0]  = new GridCell( theTrivialGrid, extentTwo, make_binary_word("00010001") );
    expected_result_arr[1]  = new GridCell( theTrivialGrid, extentTwo, make_binary_word("00010011") );
    expected_result_arr[2]  = new GridCell( theTrivialGrid, extentTwo, make_binary_word("000101") );
    expected_result_arr[3]  = new GridCell( theTrivialGrid, extentTwo, make_binary_word("00011001") );
    expected_result_arr[4]  = new GridCell( theTrivialGrid, extentTwo, make_binary_word("0001110") );
    expected_result_arr[5]  = new GridCell( theTrivialGrid, extentTwo, make_binary_word("010000") );
    expected_result_arr[6] = new GridCell( theTrivialGrid, extentTwo, make_binary_word("0100100") );
    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result_arr, theSetOneCopy, 7 );
    ARIADNE_CLEAN_TEST_VECTOR( expected_result_arr );
}

Void test_adjoin_inner_approximation_operation_three(){
    //Allocate a trivial Grid
    Grid theTrivialGrid(2, 1.0);

    //Create a more complex set set to which we will be adding inner approximations
    //theSetTwo = [-1,0]x[-1,0] U [0,1]x[0,1] U [1,3]x[1,3]
    //The set's bounding box is [-1,3]x[-1,3]
    GridTreePaving theSetTwo( theTrivialGrid, extentTwo, new BinaryTreeNode( make_binary_word("1111001000100"), make_binary_word("1001001") ) );
    GridTreePaving theSetTwoCopy( theSetTwo );
    ExactBoxSetType theBoxTwoOne = make_box("[0.49,1.51]x[0.49,1.51]");

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE(" theSetTwo.adjoin_inner_approximation( theBoxTwoOne, 1, 2) ");
    ARIADNE_PRINT_TEST_COMMENT("theSetTwo");
    cout << theSetTwo << endl;
    ARIADNE_PRINT_TEST_COMMENT("theBoxTwoOne");
    cout << theBoxTwoOne << endl;
    theSetTwo.adjoin_inner_approximation( theBoxTwoOne, extentOne, 2);
    std::vector< GridCell* > expected_result_arr( 5 );
    expected_result_arr[0] = new GridCell( theTrivialGrid, extentTwo, make_binary_word("0000") );
    expected_result_arr[1] = new GridCell( theTrivialGrid, extentTwo, make_binary_word("0011") );
    expected_result_arr[2] = new GridCell( theTrivialGrid, extentTwo, make_binary_word("11") );
    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result_arr, theSetTwo, 3 );
    ARIADNE_CLEAN_TEST_VECTOR( expected_result_arr );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE(" theSetTwoCopy.adjoin_inner_approximation( theBoxTwoOne, 2, 2) ");
    ARIADNE_PRINT_TEST_COMMENT("theSetTwoCopy");
    cout << theSetTwoCopy << endl;
    theSetTwoCopy.adjoin_inner_approximation( theBoxTwoOne, extentTwo, 2);
    expected_result_arr[0] = new GridCell( theTrivialGrid, extentTwo, make_binary_word("0000") );
    expected_result_arr[1] = new GridCell( theTrivialGrid, extentTwo, make_binary_word("0011") );
    expected_result_arr[2] = new GridCell( theTrivialGrid, extentTwo, make_binary_word("011010") );
    expected_result_arr[3] = new GridCell( theTrivialGrid, extentTwo, make_binary_word("100101") );
    expected_result_arr[4] = new GridCell( theTrivialGrid, extentTwo, make_binary_word("11") );
    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result_arr, theSetTwoCopy, 5 );
    ARIADNE_CLEAN_TEST_VECTOR( expected_result_arr );
}

Void test_restrict() {
    std::vector< GridCell* > expected_result_arr(3);

    //Allocate a trivial Grid
    Grid theTrivialGrid(2, 1.0);

    //Create the binary tree with two enabled nodes;
    BinaryTreeNode * pTwoEnabledNodeTreeH2 = new BinaryTreeNode(false);
    pTwoEnabledNodeTreeH2->split();
    pTwoEnabledNodeTreeH2->right_node()->split();
    pTwoEnabledNodeTreeH2->right_node()->left_node()->split();
    pTwoEnabledNodeTreeH2->right_node()->left_node()->left_node()->set_enabled();
    pTwoEnabledNodeTreeH2->right_node()->left_node()->right_node()->split();
    pTwoEnabledNodeTreeH2->right_node()->left_node()->right_node()->left_node()->set_enabled();
    //Create the GridTreePaving
    GridTreePaving theTwoCellPavingH2( theTrivialGrid, extentTwo, pTwoEnabledNodeTreeH2 );

    //Create another binary tree
    BinaryTreeNode * pThreeEnabledNodeTreeH2 = new BinaryTreeNode( *pTwoEnabledNodeTreeH2 );
    pThreeEnabledNodeTreeH2->left_node()->split();
    pThreeEnabledNodeTreeH2->left_node()->left_node()->set_enabled();
    pThreeEnabledNodeTreeH2->right_node()->left_node()->right_node()->right_node()->set_enabled();
    pThreeEnabledNodeTreeH2->right_node()->left_node()->right_node()->left_node()->set_disabled();
    pThreeEnabledNodeTreeH2->right_node()->left_node()->left_node()->split();
    pThreeEnabledNodeTreeH2->right_node()->left_node()->left_node()->right_node()->set_disabled();

    //Create another GridTreePaving
    GridTreePaving theThreeCellPavingH2( theTrivialGrid, extentTwo, pThreeEnabledNodeTreeH2 );

    //Create another binary tree
    BinaryTreeNode * pThreeEnabledNodeTreeH3 = new BinaryTreeNode(false);
    pThreeEnabledNodeTreeH3->split();
    pThreeEnabledNodeTreeH3->right_node()->split();
    pThreeEnabledNodeTreeH3->right_node()->right_node()->copy_from( pThreeEnabledNodeTreeH2 );
    BinaryWord thePathToSubPavingRoot;
    thePathToSubPavingRoot.push_back(true);
    thePathToSubPavingRoot.push_back(true);
    //Create the GridTreeSubpaving
    GridTreeSubpaving theThreeCellSubPavingH3( theTrivialGrid, extentThree , thePathToSubPavingRoot, pThreeEnabledNodeTreeH3->right_node()->right_node() );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Test restrict operation: GridTreePaving1.restrict( GridTreePaving2 )");
    ARIADNE_PRINT_TEST_COMMENT("The initial GridTreePaving1: ");
    expected_result_arr[0] = new GridCell( theTrivialGrid, 2, make_binary_word("[0,0]") );
    expected_result_arr[1] = new GridCell( theTrivialGrid, 2, make_binary_word("[1,0,0,0]") );
    expected_result_arr[2] = new GridCell( theTrivialGrid, 2, make_binary_word("[1,0,1,1]") );
    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result_arr, theThreeCellPavingH2, 3 );
    ARIADNE_CLEAN_TEST_VECTOR( expected_result_arr );

    ARIADNE_PRINT_TEST_COMMENT("The initial GridTreePaving2: ");
    expected_result_arr[0] = new GridCell( theTrivialGrid, 2, make_binary_word("[1,0,0]") );
    expected_result_arr[1] = new GridCell( theTrivialGrid, 2, make_binary_word("[1,0,1,0]") );
    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result_arr, theTwoCellPavingH2, 2 );
    ARIADNE_CLEAN_TEST_VECTOR( expected_result_arr );

    ARIADNE_PRINT_TEST_COMMENT("The result after restrict: ");
    theThreeCellPavingH2.restrict( theTwoCellPavingH2 );
    expected_result_arr[0] = new GridCell( theTrivialGrid, 2, make_binary_word("[1,0,0,0]") );
    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result_arr, theThreeCellPavingH2, 1 );
    ARIADNE_CLEAN_TEST_VECTOR( expected_result_arr );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Test restrict operation: GridTreePaving.restrict( GridTreeSubpaving )");
    ARIADNE_PRINT_TEST_COMMENT("The initial GridTreePaving: ");
    expected_result_arr[0] = new GridCell( theTrivialGrid, 2, make_binary_word("[1,0,0]") );
    expected_result_arr[1] = new GridCell( theTrivialGrid, 2, make_binary_word("[1,0,1,0]") );
    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result_arr, theTwoCellPavingH2, 2 );
    ARIADNE_CLEAN_TEST_VECTOR( expected_result_arr );

    ARIADNE_PRINT_TEST_COMMENT("The initial GridTreeSubpaving: ");
    expected_result_arr[0] = new GridCell( theTrivialGrid, 3, make_binary_word("[1,1,0,0]") );
    expected_result_arr[1] = new GridCell( theTrivialGrid, 3, make_binary_word("[1,1,1,0,0,0]") );
    expected_result_arr[2] = new GridCell( theTrivialGrid, 3, make_binary_word("[1,1,1,0,1,1]") );
    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result_arr, theThreeCellSubPavingH3, 3 );
    ARIADNE_CLEAN_TEST_VECTOR( expected_result_arr );

    ARIADNE_PRINT_TEST_COMMENT("The result after restrict: ");
    theTwoCellPavingH2.restrict( theThreeCellSubPavingH3 );
    expected_result_arr[0] = new GridCell( theTrivialGrid, 3, make_binary_word("[1,1,1,0,0,0]") );
    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result_arr, theTwoCellPavingH2, 1 );
    ARIADNE_CLEAN_TEST_VECTOR( expected_result_arr );

    //TODO: Test the case when the GridTreePaving has primary cell of the level 3
    //    The GridTreeSubpaving is at level 1 and it's primary cell is at level 2
}

Void test_remove_one() {
    std::vector< GridCell* > expected_result_arr(3);

    //Allocate a trivial Grid
    Grid theTrivialGrid(2, 1.0);

    //Create the binary tree with two enabled nodes;
    BinaryTreeNode * pTwoEnabledNodeTreeH2 = new BinaryTreeNode(false);
    pTwoEnabledNodeTreeH2->split();
    pTwoEnabledNodeTreeH2->right_node()->split();
    pTwoEnabledNodeTreeH2->right_node()->left_node()->split();
    pTwoEnabledNodeTreeH2->right_node()->left_node()->left_node()->set_enabled();
    pTwoEnabledNodeTreeH2->right_node()->left_node()->right_node()->split();
    pTwoEnabledNodeTreeH2->right_node()->left_node()->right_node()->left_node()->set_enabled();
    //Create the GridTreePaving
    GridTreePaving theTwoCellPavingH2( theTrivialGrid, extentTwo, pTwoEnabledNodeTreeH2 );

    //Create another binary tree
    BinaryTreeNode * pThreeEnabledNodeTreeH2 = new BinaryTreeNode( *pTwoEnabledNodeTreeH2 );
    pThreeEnabledNodeTreeH2->left_node()->split();
    pThreeEnabledNodeTreeH2->left_node()->left_node()->set_enabled();
    pThreeEnabledNodeTreeH2->right_node()->left_node()->right_node()->right_node()->set_enabled();
    pThreeEnabledNodeTreeH2->right_node()->left_node()->right_node()->left_node()->set_disabled();
    pThreeEnabledNodeTreeH2->right_node()->left_node()->left_node()->split();
    pThreeEnabledNodeTreeH2->right_node()->left_node()->left_node()->right_node()->set_disabled();

    //Create another GridTreePaving
    GridTreePaving theThreeCellPavingH2( theTrivialGrid, extentTwo, pThreeEnabledNodeTreeH2 );

    //Create another binary tree
    BinaryTreeNode * pThreeEnabledNodeTreeH3 = new BinaryTreeNode(false);
    pThreeEnabledNodeTreeH3->split();
    pThreeEnabledNodeTreeH3->right_node()->split();
    pThreeEnabledNodeTreeH3->right_node()->right_node()->copy_from( pThreeEnabledNodeTreeH2 );
    BinaryWord thePathToSubPavingRoot;
    thePathToSubPavingRoot.push_back(true);
    thePathToSubPavingRoot.push_back(true);
    //Create the GridTreeSubpaving
    GridTreeSubpaving theThreeCellSubPavingH3( theTrivialGrid, extentThree , thePathToSubPavingRoot, pThreeEnabledNodeTreeH3->right_node()->right_node() );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Test remove operation: GridTreePaving1.remove( GridTreePaving2 )");
    ARIADNE_PRINT_TEST_COMMENT("The initial GridTreePaving1: ");
    expected_result_arr[0] = new GridCell( theTrivialGrid, 2, make_binary_word("[0,0]") );
    expected_result_arr[1] = new GridCell( theTrivialGrid, 2, make_binary_word("[1,0,0,0]") );
    expected_result_arr[2] = new GridCell( theTrivialGrid, 2, make_binary_word("[1,0,1,1]") );
    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result_arr, theThreeCellPavingH2, 3 );
    ARIADNE_CLEAN_TEST_VECTOR( expected_result_arr );

    ARIADNE_PRINT_TEST_COMMENT("The initial GridTreePaving2: ");
    expected_result_arr[0] = new GridCell( theTrivialGrid, 2, make_binary_word("[1,0,0]") );
    expected_result_arr[1] = new GridCell( theTrivialGrid, 2, make_binary_word("[1,0,1,0]") );
    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result_arr, theTwoCellPavingH2, 2 );
    ARIADNE_CLEAN_TEST_VECTOR( expected_result_arr );

    ARIADNE_PRINT_TEST_COMMENT("The result after removal: ");
    theThreeCellPavingH2.remove( theTwoCellPavingH2 );
    expected_result_arr[0] = new GridCell( theTrivialGrid, 2, make_binary_word("[0,0]") );
    expected_result_arr[1] = new GridCell( theTrivialGrid, 2, make_binary_word("[1,0,1,1]") );
    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result_arr, theThreeCellPavingH2, 2 );
    ARIADNE_CLEAN_TEST_VECTOR( expected_result_arr );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Test remove operation: GridTreePaving.remove( GridTreeSubpaving )");
    ARIADNE_PRINT_TEST_COMMENT("The initial GridTreePaving: ");
    expected_result_arr[0] = new GridCell( theTrivialGrid, 2, make_binary_word("[1,0,0]") );
    expected_result_arr[1] = new GridCell( theTrivialGrid, 2, make_binary_word("[1,0,1,0]") );
    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result_arr, theTwoCellPavingH2, 2 );
    ARIADNE_CLEAN_TEST_VECTOR( expected_result_arr );

    ARIADNE_PRINT_TEST_COMMENT("The initial GridTreeSubpaving: ");
    expected_result_arr[0] = new GridCell( theTrivialGrid, 3, make_binary_word("[1,1,0,0]") );
    expected_result_arr[1] = new GridCell( theTrivialGrid, 3, make_binary_word("[1,1,1,0,0,0]") );
    expected_result_arr[2] = new GridCell( theTrivialGrid, 3, make_binary_word("[1,1,1,0,1,1]") );
    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result_arr, theThreeCellSubPavingH3, 3 );
    ARIADNE_CLEAN_TEST_VECTOR( expected_result_arr );

    ARIADNE_PRINT_TEST_COMMENT("The result after remove: ");
    theTwoCellPavingH2.remove( theThreeCellSubPavingH3 );
    expected_result_arr[0] = new GridCell( theTrivialGrid, 3, make_binary_word("[1,1,1,0,0,1]") );
    expected_result_arr[1] = new GridCell( theTrivialGrid, 3, make_binary_word("[1,1,1,0,1,0]") );
    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result_arr, theTwoCellPavingH2, 2 );
    ARIADNE_CLEAN_TEST_VECTOR( expected_result_arr );

    //TODO: Test the case when the GridTreePaving has primary cell of the level 3
    //    The GridTreeSubpaving is at level 1 and it's primary cell is at level 2
}

Void test_remove_two() {
    std::vector< GridCell* > expected_result_arr(4);

    //Allocate a trivial Grid
    Grid theTrivialGrid(2, 1.0);

    //Create a GridTreePaving and its copies
    GridTreePaving theSet01( theTrivialGrid, extentOne, make_binary_word("110010100"), make_binary_word("10001") );
    GridTreePaving theSet02( theSet01 );
    GridTreePaving theSet03( theSet01 );
    GridTreePaving theSet04( theSet01 );
    GridTreePaving theSet05( theSet01 );
    GridTreePaving theSet06( theSet01 );

    //Create the cells we will be removing and test removals on the copies of the set

    ARIADNE_PRINT_TEST_CASE_TITLE("Remove a GridCell (p.c. extent=0) form a GridTreePaving(p.c. extent=1): The cell is a subset.");
    GridCell lowerPrimaryCellSubset( theTrivialGrid, extentZero, make_binary_word("11") );
    theSet01.remove( lowerPrimaryCellSubset );
    expected_result_arr[0] = new GridCell( theTrivialGrid, extentOne, make_binary_word("00") );
    expected_result_arr[1] = new GridCell( theTrivialGrid, extentOne, make_binary_word("1110") );
    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result_arr, theSet01, 2 );
    ARIADNE_CLEAN_TEST_VECTOR( expected_result_arr );

    ARIADNE_PRINT_TEST_CASE_TITLE("Remove a GridCell (p.c. extent=0) form a GridTreePaving(p.c. extent=1): The cell does not intersect the set.");
    GridCell lowerPrimaryCellNoIntersection( theTrivialGrid, extentZero, make_binary_word("00") );
    theSet02.remove( lowerPrimaryCellNoIntersection );
    expected_result_arr[0] = new GridCell( theTrivialGrid, extentOne, make_binary_word("00") );
    expected_result_arr[1] = new GridCell( theTrivialGrid, extentOne, make_binary_word("111") );
    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result_arr, theSet02, 2 );
    ARIADNE_CLEAN_TEST_VECTOR( expected_result_arr );

    ARIADNE_PRINT_TEST_CASE_TITLE("Remove a GridCell (p.c. extent=0) form a GridTreePaving(p.c. extent=1): The cell intersects the set.");
    GridCell lowerPrimaryCellIntersection( theTrivialGrid, extentZero, BinaryWord() );
    theSet03.remove( lowerPrimaryCellIntersection );
    expected_result_arr[0] = new GridCell( theTrivialGrid, extentOne, make_binary_word("00") );
    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result_arr, theSet03, 1 );
    ARIADNE_CLEAN_TEST_VECTOR( expected_result_arr );

    ARIADNE_PRINT_TEST_CASE_TITLE("Remove a GridCell (p.c. extent=2) form a GridTreePaving(p.c. extent=1): The cell is a subset.");
    GridCell higherPrimaryCellSubset( theTrivialGrid, extentTwo, make_binary_word("000011") );
    theSet04.remove( higherPrimaryCellSubset );
    expected_result_arr[0] = new GridCell( theTrivialGrid, extentTwo, make_binary_word("00000") );
    expected_result_arr[1] = new GridCell( theTrivialGrid, extentTwo, make_binary_word("000010") );
    expected_result_arr[2] = new GridCell( theTrivialGrid, extentTwo, make_binary_word("00111") );
    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result_arr, theSet04, 3 );
    ARIADNE_CLEAN_TEST_VECTOR( expected_result_arr );

    ARIADNE_PRINT_TEST_CASE_TITLE("Remove a GridCell (p.c. extent=2) form a GridTreePaving(p.c. extent=1): The cell does not intersect the set.");
    GridCell higherPrimaryCellNoIntersection( theTrivialGrid, extentTwo, make_binary_word("010") );
    theSet05.remove( higherPrimaryCellNoIntersection );
    expected_result_arr[0] = new GridCell( theTrivialGrid, extentTwo, make_binary_word("0000") );
    expected_result_arr[1] = new GridCell( theTrivialGrid, extentTwo, make_binary_word("00111") );
    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result_arr, theSet05, 2 );
    ARIADNE_CLEAN_TEST_VECTOR( expected_result_arr );

    ARIADNE_PRINT_TEST_CASE_TITLE("Remove a GridCell (p.c. extent=2) form a GridTreePaving(p.c. extent=1): The cell intersects the set.");
    GridCell higherPrimaryCellIntersection( theTrivialGrid, extentTwo, make_binary_word("0011") );
    theSet06.remove( higherPrimaryCellIntersection );
    expected_result_arr[0] = new GridCell( theTrivialGrid, extentTwo, make_binary_word("0000") );
    ARIADNE_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result_arr, theSet06, 1 );
    ARIADNE_CLEAN_TEST_VECTOR( expected_result_arr );
}

Void test_cell_subset_subset() {

    //Allocate a trivial Grid two dimensional grid
    Grid theTrivialGrid(2, 1.0);

    const Nat smallExtent = 0;
    const Nat mediumExtent = 1;
    const Nat bigExtent = 2;

    //Create the cell, will be rooted to the primary cell mediumExtent
    GridCell theCell( theTrivialGrid, mediumExtent, make_binary_word("110") );

    //Create the binary tree, will be rooted to the primary cell of height bigExtent
    BinaryTreeNode * pBinaryTreeRoot = new BinaryTreeNode(true);
    pBinaryTreeRoot->split();
    pBinaryTreeRoot->left_node()->split();
    pBinaryTreeRoot->right_node()->split();
    pBinaryTreeRoot->left_node()->right_node()->set_disabled();
    pBinaryTreeRoot->right_node()->left_node()->set_disabled();

    pBinaryTreeRoot->left_node()->left_node()->split();
    pBinaryTreeRoot->left_node()->left_node()->right_node()->split();
    pBinaryTreeRoot->left_node()->left_node()->left_node()->split();
    pBinaryTreeRoot->left_node()->left_node()->right_node()->left_node()->set_disabled();
    pBinaryTreeRoot->left_node()->left_node()->left_node()->right_node()->set_disabled();

    //Create the GridTreeSubpaving, will be rooted to the primary cell of height smallExtent
    GridTreeSubpaving theSmallSubPaving( theTrivialGrid, smallExtent, make_binary_word("0011"),
                                      pBinaryTreeRoot->left_node()->left_node()->right_node()->right_node() );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Test subset operation GridCell (extent=1), GridTreeSubpaving.mince_to_tree_depth(2) (extent=0)");
    theSmallSubPaving.mince_to_tree_depth(2);
    ARIADNE_PRINT_TEST_COMMENT("theCell");
    cout << theCell << endl;
    ARIADNE_PRINT_TEST_COMMENT("theSmallSubPaving");
    cout << theSmallSubPaving << endl;
    ARIADNE_TEST_EQUAL( subset( theCell, theSmallSubPaving), false );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Test subset operation GridCell (extent=1), GridTreeSubpaving.recombine() (extent=0)");
    theSmallSubPaving.recombine();
    ARIADNE_PRINT_TEST_COMMENT("theCell");
    cout << theCell << endl;
    ARIADNE_PRINT_TEST_COMMENT("theSmallSubPaving");
    cout << theSmallSubPaving << endl;
    ARIADNE_TEST_EQUAL( subset( theCell, theSmallSubPaving), false );

    //Create the GridTreeSubpaving, will be rooted to the primary cell of height bigExtent
    GridTreeSubpaving theBigSubPaving( theTrivialGrid, bigExtent, make_binary_word("0011"),
                                    pBinaryTreeRoot->left_node()->left_node()->right_node()->right_node() );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Test subset operation GridCell (extent=1), GridTreeSubpaving.mince_to_tree_depth(2) (extent=2)");
    theSmallSubPaving.mince_to_tree_depth(2);
    ARIADNE_PRINT_TEST_COMMENT("theCell");
    cout << theCell << endl;
    ARIADNE_PRINT_TEST_COMMENT("theBigSubPaving");
    cout << theBigSubPaving << endl;
    ARIADNE_TEST_EQUAL( subset( theCell, theBigSubPaving), true );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Test subset operation GridCell (extent=1), GridTreeSubpaving.recombine() (extent=2)");
    theSmallSubPaving.recombine();
    ARIADNE_PRINT_TEST_COMMENT("theCell");
    cout << theCell << endl;
    ARIADNE_PRINT_TEST_COMMENT("theBigSubPaving");
    cout << theBigSubPaving << endl;
    ARIADNE_TEST_EQUAL( subset( theCell, theBigSubPaving), true );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Test subset operation GridCell (extent=1), GridTreeSubpaving.mince_to_tree_depth(2), left_node()->left_node()->set_disabled() (extent=2)");
    theSmallSubPaving.mince_to_tree_depth(2);
    theSmallSubPaving.binary_tree()->left_node()->left_node()->set_disabled();
    ARIADNE_PRINT_TEST_COMMENT("theCell");
    cout << theCell << endl;
    ARIADNE_PRINT_TEST_COMMENT("theBigSubPaving");
    cout << theBigSubPaving << endl;
    ARIADNE_TEST_EQUAL( subset( theCell, theBigSubPaving), false );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Test subset operation GridCell (extent=1), GridTreeSubpaving.mince_to_tree_depth(2), left_node()->left_node()->set_enabled(), right_node()->make_leaf(false) (extent=2)");
    theSmallSubPaving.mince_to_tree_depth(2);
    theSmallSubPaving.binary_tree()->left_node()->left_node()->set_enabled();
    theSmallSubPaving.binary_tree()->right_node()->make_leaf(false);
    ARIADNE_PRINT_TEST_COMMENT("theCell");
    cout << theCell << endl;
    ARIADNE_PRINT_TEST_COMMENT("theBigSubPaving");
    cout << theBigSubPaving << endl;
    ARIADNE_TEST_EQUAL( subset( theCell, theBigSubPaving), true );


    //Create the GridTreeSub, will be rooted to the primary cell of height smallExtent
    GridTreePaving theSmallPaving( theTrivialGrid, smallExtent, pBinaryTreeRoot->left_node()->left_node()->right_node()->right_node() );
    //Restore the binary tree
    theSmallPaving.binary_tree()->right_node()->set_enabled();
    theSmallPaving.binary_tree()->right_node()->split();

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Test subset operation GridCell (extent=1), GridTreePaving.mince_to_tree_depth(2) (after restoring the binary tree) (extent=2)");
    theSmallPaving.mince_to_tree_depth(2);
    ARIADNE_PRINT_TEST_COMMENT("theCell");
    cout << theCell << endl;
    ARIADNE_PRINT_TEST_COMMENT("theSmallPaving");
    cout << theSmallPaving << endl;
    ARIADNE_TEST_EQUAL( subset( theCell, theSmallPaving), true );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Test subset operation GridCell (extent=1), GridTreePaving.recombine() (extent=2)");
    theSmallPaving.recombine();
    ARIADNE_PRINT_TEST_COMMENT("theCell");
    cout << theCell << endl;
    ARIADNE_PRINT_TEST_COMMENT("theSmallPaving");
    cout << theSmallPaving << endl;
    ARIADNE_TEST_EQUAL( subset( theCell, theSmallPaving), true );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Test subset operation GridCell (extent=1), GridTreePaving.mince_to_tree_depth(2), left_node()->left_node()->set_disabled() (extent=2)");
    theSmallPaving.mince_to_tree_depth(2);
    theSmallPaving.binary_tree()->left_node()->left_node()->set_disabled();
    ARIADNE_PRINT_TEST_COMMENT("theCell");
    cout << theCell << endl;
    ARIADNE_PRINT_TEST_COMMENT("theSmallPaving");
    cout << theSmallPaving << endl;
    ARIADNE_TEST_EQUAL( subset( theCell, theSmallPaving), false );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Test subset operation GridCell (extent=1), GridTreePaving.mince_to_tree_depth(2), left_node()->left_node()->set_enabled(), ...right_node()->make_leaf(false) (extent=2)");
    theSmallPaving.mince_to_tree_depth(2);
    theSmallPaving.binary_tree()->left_node()->left_node()->set_enabled();
    theSmallPaving.binary_tree()->right_node()->make_leaf(false);
    ARIADNE_PRINT_TEST_COMMENT("theCell");
    cout << theCell << endl;
    ARIADNE_PRINT_TEST_COMMENT("theSmallPaving");
    cout << theSmallPaving << endl;
    ARIADNE_TEST_EQUAL( subset( theCell, theSmallPaving), true );


    //Create the GridTreeSub, will be rooted to the primary cell of height bigExtent
    BinaryTreeNode * pBinaryTreeRootCopy = new BinaryTreeNode( * pBinaryTreeRoot );
    GridTreePaving theBigPaving( theTrivialGrid, bigExtent, pBinaryTreeRootCopy );
    //Restore the binary tree
    pBinaryTreeRootCopy->left_node()->left_node()->right_node()->right_node()->right_node()->set_enabled();
    pBinaryTreeRootCopy->left_node()->left_node()->right_node()->right_node()->right_node()->split();

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Test subset operation GridCell (extent=1), GridTreePaving.mince_to_tree_depth(6) (after restoring the binary tree) (extent=2)");
    theBigPaving.mince_to_tree_depth(6);
    ARIADNE_PRINT_TEST_COMMENT("theCell");
    cout << theCell << endl;
    ARIADNE_PRINT_TEST_COMMENT("theBigPaving");
    cout << theBigPaving << endl;
    ARIADNE_TEST_EQUAL( subset( theCell, theBigPaving), true );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Test subset operation GridCell (extent=1), GridTreePaving.recombine() (extent=2)");
    theBigPaving.recombine();
    ARIADNE_PRINT_TEST_COMMENT("theCell");
    cout << theCell << endl;
    ARIADNE_PRINT_TEST_COMMENT("theBigPaving");
    cout << theBigPaving << endl;
    ARIADNE_TEST_EQUAL( subset( theCell, theBigPaving), true );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Test subset operation GridCell (extent=1), GridTreePaving.mince_to_tree_depth(6), ...left_node()->left_node()->set_disabled() (extent=2)");
    theBigPaving.mince_to_tree_depth(6);
    theBigPaving.binary_tree()->left_node()->left_node()->right_node()->right_node()->left_node()->left_node()->set_disabled();
    ARIADNE_PRINT_TEST_COMMENT("theCell");
    cout << theCell << endl;
    ARIADNE_PRINT_TEST_COMMENT("theBigPaving");
    cout << theBigPaving << endl;
    ARIADNE_TEST_EQUAL( subset( theCell, theBigPaving), false );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Test subset operation GridCell (extent=1), GridTreePaving.mince_to_tree_depth(6), ...left_node()->left_node()->set_enabled(), ...right_node()->make_leaf(false) (extent=2)");
    theBigPaving.mince_to_tree_depth(6);
    theBigPaving.binary_tree()->left_node()->left_node()->right_node()->right_node()->left_node()->left_node()->set_enabled();
    theBigPaving.binary_tree()->left_node()->left_node()->right_node()->right_node()->right_node()->make_leaf(false);
    ARIADNE_PRINT_TEST_COMMENT("theCell");
    cout << theCell << endl;
    ARIADNE_PRINT_TEST_COMMENT("theBigPaving");
    cout << theBigPaving << endl;
    ARIADNE_TEST_EQUAL( subset( theCell, theBigPaving), true );
}

Void test_subsets_join() {

    //Allocate a trivial Grid two dimensional grid
    Grid theTrivialGrid(2, 1.0);

    const Nat smallExtent = 1;
    const Nat bigExtent = 2;

    //Make set one
    BinaryWord tree = make_binary_word("1100100");
    BinaryWord leaves = make_binary_word("1010");
    BinaryTreeNode binaryTreeRootOne( tree, leaves );
    GridTreeSubpaving theSet1( theTrivialGrid, smallExtent, BinaryWord(), &binaryTreeRootOne );

    //Make set two
    tree = make_binary_word("1100100");
    leaves = make_binary_word("0101");
    BinaryTreeNode binaryTreeRootTwo( tree, leaves );
    GridTreeSubpaving theSet2( theTrivialGrid, bigExtent, make_binary_word("00"), &binaryTreeRootTwo );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Join theSet1 and theSet2, when recombined they should give us the primary cell of height 1 rooted to the primary cell of height 2");
    ARIADNE_PRINT_TEST_COMMENT("theSet1");
    cout << theSet1 << endl;
    ARIADNE_PRINT_TEST_COMMENT("theSet2");
    cout << theSet2 << endl;
    GridTreePaving resultSet = join( theSet1, theSet2 );
    resultSet.recombine();
    GridTreePaving expectedResultSet( theTrivialGrid, bigExtent, make_binary_word("11000"), make_binary_word("1000") );
    ARIADNE_TEST_EQUAL( expectedResultSet, resultSet);
}

Void test_subsets_intersection() {

    //Allocate a trivial Grid two dimensional grid
    Grid theTrivialGrid(2, 1.0);

    const Nat smallExtent = 1;
    const Nat bigExtent = 2;

    //Make set one
    BinaryWord tree = make_binary_word("1100100");
    BinaryWord leaves = make_binary_word("1010");
    BinaryTreeNode binaryTreeRootOne( tree, leaves );
    GridTreeSubpaving theSet1( theTrivialGrid, smallExtent, BinaryWord(), &binaryTreeRootOne );

    //Make set two
    tree = make_binary_word("1100100");
    leaves = make_binary_word("0111");
    BinaryTreeNode binaryTreeRootTwo( tree, leaves );
    GridTreeSubpaving theSet2( theTrivialGrid, bigExtent, make_binary_word("00"), &binaryTreeRootTwo );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Intersect theSet1 and theSet2, should give us a set rooted to the primary cell of height 2");
    ARIADNE_PRINT_TEST_COMMENT("theSet1");
    cout << theSet1 << endl;
    ARIADNE_PRINT_TEST_COMMENT("theSet2");
    cout << theSet2 << endl;
    GridTreePaving resultSet = intersection( theSet1, theSet2 );
    GridTreePaving expectedResultSet( theTrivialGrid, bigExtent, make_binary_word("11110010000"), make_binary_word("001000") );
    ARIADNE_TEST_EQUAL( expectedResultSet, resultSet);
}

Void test_subsets_difference() {

    //Allocate a trivial Grid two dimensional grid
    Grid theTrivialGrid(2, 1.0);

    const Nat smallExtent = 1;
    const Nat bigExtent = 2;

    //Make set one
    BinaryWord tree = make_binary_word("1100100");
    BinaryWord leaves = make_binary_word("1010");
    BinaryTreeNode binaryTreeRootOne( tree, leaves );
    GridTreeSubpaving theSet1( theTrivialGrid, smallExtent, BinaryWord(), &binaryTreeRootOne );

    //Make set two
    tree = make_binary_word("1100100");
    leaves = make_binary_word("0111");
    BinaryTreeNode binaryTreeRootTwo( tree, leaves );
    GridTreeSubpaving theSet2( theTrivialGrid, bigExtent, make_binary_word("00"), &binaryTreeRootTwo );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Removing theSet1 from theSet2, should give us a set rooted to the primary cell of height 2");
    ARIADNE_PRINT_TEST_COMMENT("theSet1");
    cout << theSet1 << endl;
    ARIADNE_PRINT_TEST_COMMENT("theSet2");
    cout << theSet2 << endl;
    GridTreePaving resultSet = difference( theSet1, theSet2 );
    GridTreePaving expectedResultSet( theTrivialGrid, bigExtent, make_binary_word("11110010000"), make_binary_word("100000") );
    ARIADNE_TEST_EQUAL( expectedResultSet, resultSet);
}

Void test_cell_intersect_subset() {

    //Allocate a trivial Grid two dimensional grid
    Grid theTrivialGrid(2, 1.0);

    BinaryWord tree = make_binary_word("1111001000100");
    BinaryWord leaves = make_binary_word("1001001");
    //Create the set and the subset of this set, they are both rooted to the same primary node of extentTwo
    GridTreePaving theSet( theTrivialGrid, extentTwo, new BinaryTreeNode( tree, leaves ) );
    //The subset is basically the zero level primary cell
    BinaryWord path = make_binary_word("11");
    GridTreeSubpaving theSubset( theTrivialGrid, extentOne, path, theSet.binary_tree()->left_node()->left_node()->right_node()->right_node() );

    GridCell theLowCell( theTrivialGrid, extentZero, make_binary_word("111") );        // does intersect with the set and the subset
    GridCell theMediumCellOne( theTrivialGrid, extentOne, make_binary_word("1000") );    // does not intersect with the set and the subset
    GridCell theMediumCellTwo( theTrivialGrid, extentOne, make_binary_word("0000") );    // does intersect with the set but not the subset
    GridCell theHighCellOne( theTrivialGrid, extentTwo, make_binary_word("11") );        // does intersect with the set but not the subset
    GridCell theHighCellTwo( theTrivialGrid, extentTwo, make_binary_word("01") );        // does not intersect with the set and the subset
    GridCell theHighCellThree( theTrivialGrid, extentTwo, make_binary_word("00") );        // does intersect with the set and the subset

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Testing Bool intersect( const GridCell& , const GridTreeSubpaving& )");
    ARIADNE_PRINT_TEST_COMMENT("theSet");
    cout << theSet << endl;
    ARIADNE_PRINT_TEST_COMMENT("theLowCell");
    cout << theLowCell << endl;
    ARIADNE_TEST_EQUAL( intersect( theLowCell, theSet ), true );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Testing Bool intersect( const GridCell& , const GridTreeSubpaving& )");
    ARIADNE_PRINT_TEST_COMMENT("theSet");
    cout << theSet << endl;
    ARIADNE_PRINT_TEST_COMMENT("theMediumCellOne");
    cout << theMediumCellOne << endl;
    ARIADNE_TEST_EQUAL( intersect( theMediumCellOne, theSet ), false );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Testing Bool intersect( const GridCell& , const GridTreeSubpaving& )");
    ARIADNE_PRINT_TEST_COMMENT("theSet");
    cout << theSet << endl;
    ARIADNE_PRINT_TEST_COMMENT("theMediumCellTwo");
    cout << theMediumCellTwo << endl;
    ARIADNE_TEST_EQUAL( intersect( theMediumCellTwo, theSet ), true );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Testing Bool intersect( const GridCell& , const GridTreeSubpaving& )");
    ARIADNE_PRINT_TEST_COMMENT("theSet");
    cout << theSet << endl;
    ARIADNE_PRINT_TEST_COMMENT("theHighCellOne");
    cout << theHighCellOne << endl;
    ARIADNE_TEST_EQUAL( intersect( theHighCellOne, theSet ), true );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Testing Bool intersect( const GridCell& , const GridTreeSubpaving& )");
    ARIADNE_PRINT_TEST_COMMENT("theSet");
    cout << theSet << endl;
    ARIADNE_PRINT_TEST_COMMENT("");
    cout << theHighCellTwo << endl;
    ARIADNE_TEST_EQUAL( intersect( theHighCellTwo, theSet ), false );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Testing Bool intersect( const GridCell& , const GridTreeSubpaving& )");
    ARIADNE_PRINT_TEST_COMMENT("theSet");
    cout << theSet << endl;
    ARIADNE_PRINT_TEST_COMMENT("theHighCellThree");
    cout << theHighCellThree << endl;
    ARIADNE_TEST_EQUAL( intersect( theHighCellThree, theSet ), true );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Testing Bool intersect( const GridCell& , const GridTreePaving& )");
    ARIADNE_PRINT_TEST_COMMENT("theSubset");
    cout << theSubset << endl;
    ARIADNE_PRINT_TEST_COMMENT("theLowCell");
    cout << theLowCell << endl;
    ARIADNE_TEST_EQUAL( intersect( theLowCell, theSubset ), true );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Testing Bool intersect( const GridCell& , const GridTreePaving& )");
    ARIADNE_PRINT_TEST_COMMENT("theSubset");
    cout << theSubset << endl;
    ARIADNE_PRINT_TEST_COMMENT("theMediumCellOne");
    cout << theMediumCellOne << endl;
    ARIADNE_TEST_EQUAL( intersect( theMediumCellOne, theSubset ), false );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Testing Bool intersect( const GridCell& , const GridTreePaving& )");
    ARIADNE_PRINT_TEST_COMMENT("theSubset");
    cout << theSubset << endl;
    ARIADNE_PRINT_TEST_COMMENT("theMediumCellTwo");
    cout << theMediumCellTwo << endl;
    ARIADNE_TEST_EQUAL( intersect( theMediumCellTwo, theSubset ), false );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Testing Bool intersect( const GridCell& , const GridTreePaving& )");
    ARIADNE_PRINT_TEST_COMMENT("theSubset");
    cout << theSubset << endl;
    ARIADNE_PRINT_TEST_COMMENT("theHighCellOne");
    cout << theHighCellOne << endl;
    ARIADNE_TEST_EQUAL( intersect( theHighCellOne, theSubset ), false );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Testing Bool intersect( const GridCell& , const GridTreePaving& )");
    ARIADNE_PRINT_TEST_COMMENT("theSubset");
    cout << theSubset << endl;
    ARIADNE_PRINT_TEST_COMMENT("theHighCellTwo");
    cout << theHighCellTwo << endl;
    ARIADNE_TEST_EQUAL( intersect( theHighCellTwo, theSubset ), false );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Testing Bool intersect( const GridCell& , const GridTreePaving& )");
    ARIADNE_PRINT_TEST_COMMENT("theSubset");
    cout << theSubset << endl;
    ARIADNE_PRINT_TEST_COMMENT("theHighCellThree");
    cout << theHighCellThree << endl;
    ARIADNE_TEST_EQUAL( intersect( theHighCellThree, theSubset ), true );

}


Void test_subset_intersect_subset() {

    //Allocate a trivial Grid two dimensional grid
    Grid theTrivialGrid(2, 1.0);

    BinaryWord tree = make_binary_word("1111001000100");
    BinaryWord leaves = make_binary_word("1001001");
    //Create the set and the subset of this set, they are both rooted to the same primary node of extentTwo
    GridTreePaving theSet( theTrivialGrid, extentTwo, new BinaryTreeNode( tree, leaves ) );
    //The subset is basically the zero level primary cell
    BinaryWord path = make_binary_word("11");
    GridTreeSubpaving theSubset( theTrivialGrid, extentOne, path, theSet.binary_tree()->left_node()->left_node()->right_node()->right_node() );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Testing Bool intersect( const GridTreeSubpaving& theSet, const GridTreeSubpaving& theSubset )");
    ARIADNE_PRINT_TEST_COMMENT("theSet");
    cout << theSet << endl;
    ARIADNE_PRINT_TEST_COMMENT("theSubset");
    cout << theSubset << endl;
    ARIADNE_TEST_EQUAL( intersect( theSet, theSubset ), true );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Testing Bool intersect( const GridTreeSubpaving& theSubset, const GridTreeSubpaving& theSet )");
    ARIADNE_PRINT_TEST_COMMENT("theSet");
    cout << theSet << endl;
    ARIADNE_PRINT_TEST_COMMENT("theSubset");
    cout << theSubset << endl;
    ARIADNE_TEST_EQUAL( intersect( theSubset, theSet ), true );

    //Make the subset one
    BinaryWord treeOne = make_binary_word("11110100001010100");
    BinaryWord leavesOne = make_binary_word("010010001");
    BinaryTreeNode binaryTreeRootOne( treeOne, leavesOne );
    //Create a set that intersects with theSet but not the theSubset
    GridTreeSubpaving theSubSetOne( theTrivialGrid, extentThree, make_binary_word("11"), &binaryTreeRootOne );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Testing Bool intersect( const GridTreeSubpaving& theSet, const GridTreeSubpaving& theSubSetOne )");
    ARIADNE_PRINT_TEST_COMMENT("theSet");
    cout << theSet << endl;
    ARIADNE_PRINT_TEST_COMMENT("theSubSetOne");
    cout << theSubSetOne << endl;
    ARIADNE_TEST_EQUAL( intersect( theSet, theSubSetOne ), true );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Testing Bool intersect( const GridTreeSubpaving& theSubset, const GridTreeSubpaving& theSubSetOne )");
    ARIADNE_PRINT_TEST_COMMENT("theSubset");
    cout << theSubset << endl;
    ARIADNE_PRINT_TEST_COMMENT("theSubSetOne");
    cout << theSubSetOne << endl;
    ARIADNE_TEST_EQUAL( intersect( theSubset, theSubSetOne ), false );

    //Make the subset two: a subset that does not intersect with theSubSetOne
    //This is done simply changing the prefix in such a way that they do not intersect
    GridTreeSubpaving theSubSetTwo( theTrivialGrid, extentThree, make_binary_word("10"), &binaryTreeRootOne );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Testing Bool intersect( const GridTreeSubpaving& theSubSetOne, const GridTreeSubpaving& theSubSetTwo )");
    ARIADNE_PRINT_TEST_COMMENT("theSubSetOne");
    cout << theSubSetOne << endl;
    ARIADNE_PRINT_TEST_COMMENT("theSubSetTwo");
    cout << theSubSetTwo << endl;
    ARIADNE_TEST_EQUAL( intersect( theSubSetOne, theSubSetTwo ), false );

    //Make a set that intersects with theSet: Just root it to height zero and
    //reuse the tree make a copy of the tree to avoid double deallocations.
    GridTreePaving theSetOne( theTrivialGrid, extentZero, new BinaryTreeNode( binaryTreeRootOne ) );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Testing Bool intersect( const GridTreeSubpaving& theSet, const GridTreeSubpaving& theSetOne )");
    ARIADNE_PRINT_TEST_COMMENT("theSet");
    cout << theSet << endl;
    ARIADNE_PRINT_TEST_COMMENT("theSetOne");
    cout << theSetOne << endl;
    ARIADNE_PRINT_TEST_COMMENT("Mince theSet and theSetOne to depth 10, just to make things more complex");
    theSet.mince_to_tree_depth(10);
    theSetOne.mince_to_tree_depth(10);
    ARIADNE_TEST_EQUAL( intersect( theSet, theSetOne ), true );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Testing Bool intersect( const GridTreeSubpaving& theSetOne, const GridTreeSubpaving& theSet )");
    ARIADNE_PRINT_TEST_COMMENT("Mince theSet to depth 20, just to make things more complex");
    theSet.mince_to_tree_depth(20);
    ARIADNE_TEST_EQUAL( intersect( theSetOne, theSet ), true );

}

Void test_subset_subset_subset() {

    //Allocate a trivial Grid two dimensional grid
    Grid theTrivialGrid(2, 1.0);

    //Create the set and the subset of this set, they are both rooted to the same primary node of extentTwo
    //theSetOne = [-1,0]x[-1,0] U [0,1]x[0,1] U [1,3]x[1,3]
    GridTreePaving theSetOne( theTrivialGrid, extentTwo, new BinaryTreeNode( make_binary_word("1111001000100"), make_binary_word("1001001") ) );
    //theSubsetOne = [-1,0]x[-1,0] U [0,1]x[0,1]
    GridTreeSubpaving theSubsetOne( theTrivialGrid, extentTwo, make_binary_word("0"), theSetOne.binary_tree()->left_node() );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Testing Bool subset( const GridTreeSubpaving& theSetOne, const GridTreeSubpaving& theSubsetOne )");
    ARIADNE_PRINT_TEST_COMMENT("theSetOne");
    cout << theSetOne << endl;
    ARIADNE_PRINT_TEST_COMMENT("theSubsetOne");
    cout << theSubsetOne << endl;
    ARIADNE_TEST_EQUAL( subset( theSetOne, theSubsetOne ), false );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Testing Bool subset( const GridTreeSubpaving& theSubsetOne, const GridTreeSubpaving& theSetOne )");
    ARIADNE_TEST_EQUAL( subset( theSubsetOne, theSetOne ), true );

    //Make two subsets such that the tree of the first one is a super tree of the second one,
    //but the second one contains the first one as a set
    BinaryTreeNode binaryTreeRootTwo( make_binary_word("1111001000100"), make_binary_word("1001000") );
    //theSubSetTwo = [-1,0]x[-1,0] U [0,1]x[0,1]
    GridTreeSubpaving theSubSetTwo( theTrivialGrid, extentTwo, BinaryWord(), &binaryTreeRootTwo );
    BinaryTreeNode binaryTreeRootThree( make_binary_word("100"), make_binary_word("10") );
    //theSubSetThree = [-1,1]x[-1,3]
    GridTreeSubpaving theSubSetThree( theTrivialGrid, extentThree, make_binary_word("110"), &binaryTreeRootThree );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Testing Bool subset( const GridTreeSubpaving& theSubSetTwo, const GridTreeSubpaving& theSubSetThree )");
    ARIADNE_PRINT_TEST_COMMENT("theSubSetTwo");
    cout << theSubSetTwo << endl;
    ARIADNE_PRINT_TEST_COMMENT("theSubSetThree");
    cout << theSubSetThree << endl;
    ARIADNE_TEST_EQUAL( subset( theSubSetTwo, theSubSetThree ), true );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Testing Bool subset( const GridTreeSubpaving& theSubSetThree, const GridTreeSubpaving& theSubSetTwo )");
    ARIADNE_TEST_EQUAL( subset( theSubSetThree, theSubSetTwo ), false );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Testing Bool subset( const GridTreeSubpaving& theSubSetTwo, const GridTreeSubpaving& theSubSetThree )");
    ARIADNE_PRINT_TEST_COMMENT("Mince theSubSetThree to depth 10, just to make things more complex");
    theSubSetThree.mince_to_tree_depth(10);
    ARIADNE_TEST_EQUAL( subset( theSubSetTwo, theSubSetThree ), true );

    //Create a new set which is a subset of theSubSetThree but is not a superset of theSubSetTwo
    BinaryTreeNode binaryTreeRootFour( binaryTreeRootThree );
    binaryTreeRootFour.recombine();
    binaryTreeRootFour.mince(5);
    binaryTreeRootFour.left_node()->left_node()->left_node()->left_node()->left_node()->set_disabled();
    //theSubSetThree = [-1,1]x[-1,3] \ [-1,-1]x[-0.5,-0.5]
    GridTreeSubpaving theSubSetFour( theTrivialGrid, extentThree, make_binary_word("110"), &binaryTreeRootFour );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Testing Bool subset( const GridTreeSubpaving& theSubSetFour, const GridTreeSubpaving& theSubSetThree )");
    ARIADNE_PRINT_TEST_COMMENT("theSubSetFour");
    cout << theSubSetFour << endl;
    ARIADNE_TEST_EQUAL( subset( theSubSetFour, theSubSetThree ), true );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Testing Bool subset( const GridTreeSubpaving& theSubSetTwo, const GridTreeSubpaving& theSubSetFour )");
    ARIADNE_TEST_EQUAL( subset( theSubSetTwo, theSubSetFour ), false );

}

Void test_subset_intersects_box() {

    //Allocate a trivial Grid two dimensional grid
    Grid theTrivialGrid(2, 1.0);

    //Create the set and the subset of this set, they are both rooted to the same primary node of extentTwo
    //theSetOne = [-1,0]x[-1,0] U [0,1]x[0,1] U [1,3]x[1,3]
    GridTreePaving theSetOne( theTrivialGrid, extentTwo, new BinaryTreeNode( make_binary_word("1111001000100"), make_binary_word("1001001") ) );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Testing ValidatedKleenean GridTreeSubpaving::intersects( const ExactBoxType& box ) ");
    ARIADNE_PRINT_TEST_COMMENT("A box that does not intersects with theSetOne");
    ExactBoxType box = make_box("[-2.0,-1.5]x[10,20]");
    cout << "theSetOne: " << theSetOne << endl;
    cout << "ExactBoxType: " << box << endl;
    ARIADNE_TEST_EQUAL( theSetOne.intersects( box ), false );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Testing ValidatedKleenean GridTreeSubpaving::intersects( const ExactBoxType& box ) ");
    ARIADNE_PRINT_TEST_COMMENT("A box that encloses theSetOne as a strict subset");
    box = make_box("[-2.0,4.0]x[-2.0,4.0]");
    cout << "theSetOne: " << theSetOne << endl;
    cout << "ExactBoxType: " << box << endl;
    ARIADNE_TEST_EQUAL( theSetOne.intersects( box ), true );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Testing ValidatedKleenean GridTreeSubpaving::intersects( const ExactBoxType& box ) ");
    ARIADNE_PRINT_TEST_COMMENT("A box that coincides with one cell of theSetOne");
    box = make_box("[-1.0,0.0]x[-1.0,0.0]");
    cout << "theSetOne: " << theSetOne << endl;
    cout << "ExactBoxType: " << box << endl;
    ARIADNE_TEST_EQUAL( theSetOne.intersects( box ), true );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Testing ValidatedKleenean GridTreeSubpaving::intersects( const ExactBoxType& box ) ");
    ARIADNE_PRINT_TEST_COMMENT("A box that is a subset of one cell of theSetOne");
    box = make_box("[1.5,2.5]x[1.5,2.5]");
    cout << "theSetOne: " << theSetOne << endl;
    cout << "ExactBoxType: " << box << endl;
    ARIADNE_TEST_EQUAL( theSetOne.intersects( box ), true );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Testing ValidatedKleenean GridTreeSubpaving::intersects( const ExactBoxType& box ) ");
    ARIADNE_PRINT_TEST_COMMENT("A box that intersects two out of three enabled cells of theSetOne");
    box = make_box("[0.3,1.7]x[0.6,1.2]");
    cout << "theSetOne: " << theSetOne << endl;
    cout << "ExactBoxType: " << box << endl;
    ARIADNE_TEST_EQUAL( theSetOne.intersects( box ), true );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Testing ValidatedKleenean GridTreeSubpaving::intersects( const ExactBoxType& box ) ");
    ARIADNE_PRINT_TEST_COMMENT("A box that is located within the bounding box of theSetOne but does not intersect any enabled cells");
    box = make_box("[-0.6,-0.3]x[1.5,3.0]");
    cout << "theSetOne: " << theSetOne << endl;
    cout << "ExactBoxType: " << box << endl;
    ARIADNE_TEST_EQUAL( theSetOne.intersects( box ), false );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Testing ValidatedKleenean GridTreeSubpaving::intersects( const ExactBoxType& box ) ");
    ARIADNE_PRINT_TEST_COMMENT("A box that shares a border with some of the enabled cells of theSetOne");
    box = make_box("[-1.0,1.0]x[1.0,3.0]");
    cout << "theSetOne: " << theSetOne << endl;
    cout << "ExactBoxType: " << box << endl;
    //NOTE: The common border makes ExactBoxType:intersects(ExactBoxType) return false so we have no intersection here
    ARIADNE_TEST_EQUAL( theSetOne.intersects( box ), false );

    //TODO: I do not know how to test indeterminate result of the intersection here
    //Somehow I need two boxes for which we can not determine if they intersect or not.
}

Void test_subset_subset_box(){

    //Allocate a trivial Grid two dimensional grid
    Grid theTrivialGrid(2, 1.0);

    //Create the set rooted to the primary node of extentTwo
    //theSetOne = [-1,0]x[-1,0] U [0,1]x[0,1] U [1,3]x[1,3]
    GridTreePaving theSetOne( theTrivialGrid, extentTwo, new BinaryTreeNode( make_binary_word("1111001000100"), make_binary_word("1001001") ) );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Testing ValidatedKleenean GridTreeSubpaving::subset( const ExactBoxType& box ) ");
    ARIADNE_PRINT_TEST_COMMENT("A box that does not intersect with theSetOne");
    ExactBoxType box = make_box("[-2.0,-1.5]x[10,20]");
    cout << "theSetOne: " << theSetOne << endl;
    cout << "ExactBoxType: " << box << endl;
    ARIADNE_TEST_EQUAL( theSetOne.subset( box ), false );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Testing ValidatedKleenean GridTreeSubpaving::subset( const ExactBoxType& box ) ");
    ARIADNE_PRINT_TEST_COMMENT("A box that encloses theSetOne as a strict subset");
    box = make_box("[-2.0,4.0]x[-2.0,4.0]");
    cout << "theSetOne: " << theSetOne << endl;
    cout << "ExactBoxType: " << box << endl;
    ARIADNE_TEST_EQUAL( theSetOne.subset( box ), true );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Testing ValidatedKleenean GridTreeSubpaving::subset( const ExactBoxType& box ) ");
    ARIADNE_PRINT_TEST_COMMENT("A box that coincides with one cell of theSetOne");
    box = make_box("[-1.0,0.0]x[-1.0,0.0]");
    cout << "theSetOne: " << theSetOne << endl;
    cout << "ExactBoxType: " << box << endl;
    ARIADNE_TEST_EQUAL( theSetOne.subset( box ), false );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Testing ValidatedKleenean GridTreeSubpaving::subset( const ExactBoxType& box ) ");
    ARIADNE_PRINT_TEST_COMMENT("A box that is a subset of one cell of theSetOne");
    box = make_box("[1.5,2.5]x[1.5,2.5]");
    cout << "theSetOne: " << theSetOne << endl;
    cout << "ExactBoxType: " << box << endl;
    ARIADNE_TEST_EQUAL( theSetOne.subset( box ), false );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Testing ValidatedKleenean GridTreeSubpaving::subset( const ExactBoxType& box ) ");
    ARIADNE_PRINT_TEST_COMMENT("A box that intersects two out of three enabled cells of theSetOne");
    box = make_box("[0.3,1.7]x[0.6,1.2]");
    cout << "theSetOne: " << theSetOne << endl;
    cout << "ExactBoxType: " << box << endl;
    ARIADNE_TEST_EQUAL( theSetOne.subset( box ), false );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Testing ValidatedKleenean GridTreeSubpaving::subset( const ExactBoxType& box ) ");
    ARIADNE_PRINT_TEST_COMMENT("A box that is located within the bounding box of theSetOne but does not intersect any enabled cells");
    box = make_box("[-0.6,-0.3]x[1.5,3.0]");
    cout << "theSetOne: " << theSetOne << endl;
    cout << "ExactBoxType: " << box << endl;
    ARIADNE_TEST_EQUAL( theSetOne.subset( box ), false );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Testing ValidatedKleenean GridTreeSubpaving::subset( const ExactBoxType& box ) ");
    ARIADNE_PRINT_TEST_COMMENT("A box that shares a border with some of the enabled cells of theSetOne");
    box = make_box("[-1.0,1.0]x[1.0,3.0]");
    cout << "theSetOne: " << theSetOne << endl;
    cout << "ExactBoxType: " << box << endl;
    ARIADNE_TEST_EQUAL( theSetOne.subset( box ), false );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Testing ValidatedKleenean GridTreeSubpaving::subset( const ExactBoxType& box ) ");
    ARIADNE_PRINT_TEST_COMMENT("A box that is contains the set but is located strictly subset the bounding cell of the set");
    //theSetTwo = [0,1]x[0,1]
    GridTreePaving theSetTwo( theTrivialGrid, extentTwo, new BinaryTreeNode( make_binary_word("1111001000100"), make_binary_word("0001000") ) );
    box = make_box("[-0.5,1.5]x[-0.5, 1.5]");
    cout << "theSetTwo: " << theSetTwo << endl;
    cout << "ExactBoxType: " << box << endl;
    ARIADNE_TEST_EQUAL( theSetTwo.subset( box ), true );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Testing ValidatedKleenean GridTreeSubpaving::subset( const ExactBoxType& box ) ");
    ARIADNE_PRINT_TEST_COMMENT("A box that contains the set but is located subset the bounding cell of the set, sharing a border with it");
    //theSetThree = [-1,0]x[-1,0] U [0,1]x[0,1]
    GridTreePaving theSetThree( theTrivialGrid, extentTwo, new BinaryTreeNode( make_binary_word("1111001000100"), make_binary_word("1001000") ) );
    box = make_box("[-1.0,1.0]x[-1.0, 1.0]");
    cout << "theSetThree: " << theSetThree << endl;
    cout << "ExactBoxType: " << box << endl;
    ARIADNE_TEST_EQUAL( theSetThree.subset( box ), true );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Testing ValidatedKleenean GridTreeSubpaving::subset( const ExactBoxType& box ) ");
    ARIADNE_PRINT_TEST_COMMENT("A box that contains the set and is partially located subset the bounding cell of the set, sharing a border with it");
    box = make_box("[-1.0,1.5]x[-1.0, 4.0]");
    cout << "theSetThree: " << theSetThree << endl;
    cout << "ExactBoxType: " << box << endl;
    ARIADNE_TEST_EQUAL( theSetThree.subset( box ), true );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Testing ValidatedKleenean GridTreeSubpaving::subset( const ExactBoxType& box ) ");
    ARIADNE_PRINT_TEST_COMMENT("A box that contains the set and is partially located subset the bounding cell of the set");
    //theSetFour = [0,1]x[0,1]
    GridTreePaving theSetFour( theTrivialGrid, extentTwo, new BinaryTreeNode( make_binary_word("1111001000100"), make_binary_word("0001000") ) );
    box = make_box("[-0.5,1.5]x[-0.5, 4.0]");
    cout << "theSetFour: " << theSetFour << endl;
    cout << "ExactBoxType: " << box << endl;
    ARIADNE_TEST_EQUAL( theSetFour.subset( box ), true );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Testing ValidatedKleenean GridTreeSubpaving::subset( const ExactBoxType& box ) ");
    ARIADNE_PRINT_TEST_COMMENT("A box that coincides with the set");
    box = make_box("[0.0,1.0]x[0.0,1.0]");
    cout << "theSetFour: " << theSetFour << endl;
    cout << "ExactBoxType: " << box << endl;
    ARIADNE_TEST_EQUAL( theSetFour.subset( box ), true );

    //TODO: I do not know how to test indeterminate result of the subset relation here.
    //I need two boxes for which we can not determine if they one is a subset of another.

}

Void test_subset_superset_box(){
    //Allocate a trivial Grid two dimensional grid
    Grid theTrivialGrid(2, 1.0);

    //Create the set and the subset of this set, they are both rooted to the same primary node of extentTwo
    //theSetOne = [-1,0]x[-1,0] U [0,1]x[0,1] U [1,2]x[1,2] U [1,2]x[2,3] U [2,3]x[1,2] U [2,3]x[2,3]
    GridTreePaving theSetOne( theTrivialGrid, extentTwo, new BinaryTreeNode( make_binary_word("1111001000101100100"), make_binary_word("1001001111") ) );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Testing ValidatedKleenean GridTreeSubpaving::superset( const ExactBoxType& box ) ");
    ARIADNE_PRINT_TEST_COMMENT("A box that is disjoint from the set and is placed outside the set's box");
    ExactBoxType box = make_box("[5.0,6.0]x[7.0,8.0]");
    cout << "theSetOne: " << theSetOne << endl;
    cout << "ExactBoxType: " << box << endl;
    ARIADNE_TEST_EQUAL( theSetOne.superset( box ), false );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Testing ValidatedKleenean GridTreeSubpaving::superset( const ExactBoxType& box ) ");
    ARIADNE_PRINT_TEST_COMMENT("A box that is disjoint from the set and is placed within the set's box");
    box = make_box("[-0.5,0.5]x[1.5,2.5]");
    cout << "theSetOne: " << theSetOne << endl;
    cout << "ExactBoxType: " << box << endl;
    ARIADNE_TEST_EQUAL( theSetOne.superset( box ), false );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Testing ValidatedKleenean GridTreeSubpaving::superset( const ExactBoxType& box ) ");
    ARIADNE_PRINT_TEST_COMMENT("A box that is disjoint from the set but shares a borders with the set's box");
    box = make_box("[-2.0,-1.0]x[-1.0,2.0]");
    cout << "theSetOne: " << theSetOne << endl;
    cout << "ExactBoxType: " << box << endl;
    ARIADNE_TEST_EQUAL( theSetOne.superset( box ), false );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Testing ValidatedKleenean GridTreeSubpaving::superset( const ExactBoxType& box ) ");
    ARIADNE_PRINT_TEST_COMMENT("A box that only intersects the set and is partially covered by it's cells");
    box = make_box("[1.5,2.5]x[1.5,3.5]");
    cout << "theSetOne: " << theSetOne << endl;
    cout << "ExactBoxType: " << box << endl;
    ARIADNE_TEST_EQUAL( theSetOne.superset( box ), false );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Testing ValidatedKleenean GridTreeSubpaving::superset( const ExactBoxType& box ) ");
    ARIADNE_PRINT_TEST_COMMENT("A box that coincides with a cell of the set");
    box = make_box("[-1.0,-1.0]x[0.0,0.0]");
    cout << "theSetOne: " << theSetOne << endl;
    cout << "ExactBoxType: " << box << endl;
    ARIADNE_TEST_EQUAL( theSetOne.superset( box ), true );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Testing ValidatedKleenean GridTreeSubpaving::superset( const ExactBoxType& box ) ");
    ARIADNE_PRINT_TEST_COMMENT("A box that is within the set and shares a border with an enabled cell");
    box = make_box("[-1.0,1.0]x[1.0,2.0]");
    cout << "theSetOne: " << theSetOne << endl;
    cout << "ExactBoxType: " << box << endl;
    ARIADNE_TEST_EQUAL( theSetOne.superset( box ), false );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Testing ValidatedKleenean GridTreeSubpaving::superset( const ExactBoxType& box ) ");
    ARIADNE_PRINT_TEST_COMMENT("A box that is partially covered by two enabled cells of the set");
    box = make_box("[-0.5,0.5]x[-0.5,0.5]");
    cout << "theSetOne: " << theSetOne << endl;
    cout << "ExactBoxType: " << box << endl;
    ARIADNE_TEST_EQUAL( theSetOne.superset( box ), false );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Testing ValidatedKleenean GridTreeSubpaving::superset( const ExactBoxType& box ) ");
    ARIADNE_PRINT_TEST_COMMENT("A box that is a subset of an enabled cell");
    box = make_box("[1.5,1.8]x[1.3,1.9]");
    cout << "theSetOne: " << theSetOne << endl;
    cout << "ExactBoxType: " << box << endl;
    ARIADNE_TEST_EQUAL( theSetOne.superset( box ), true );

    // !!!
    ARIADNE_PRINT_TEST_CASE_TITLE("Testing ValidatedKleenean GridTreeSubpaving::superset( const ExactBoxType& box ) ");
    ARIADNE_PRINT_TEST_COMMENT("A box that is covered by several enabled cells of the set");
    box = make_box("[1.5,2.5]x[1.5,2.5]");
    cout << "theSetOne: " << theSetOne << endl;
    cout << "ExactBoxType: " << box << endl;
    ARIADNE_TEST_EQUAL( theSetOne.superset( box ), true );

    //TODO: I do not know how to test indeterminate result of the superset relation here.
    //I need two boxes for which we can not determine if they one is a superset of another.

}

void test_products() {
    Grid g1(2);
    Grid g2(1);
    GridCell gc1(g1,3,BinaryWord("0100110001"));
    GridCell gc2(g2,2,BinaryWord("1001"));

    Grid g3(3);
    GridCell gc3(g3,3,BinaryWord("011001110000011"));
    ARIADNE_TEST_EQUALS(product(gc1,gc2),gc3);


    GridTreePaving gtp1(g1);
    gtp1.adjoin(GridCell(g1,3,BinaryWord("0100110001")));
    gtp1.adjoin(GridCell(g1,3,BinaryWord("0100110011")));

    GridTreePaving gtp2(g2);
    gtp2.adjoin(GridCell(g2,2,BinaryWord("1001")));

    GridTreePaving gtp3(g3);
    gtp3.adjoin(GridCell(g3,3,BinaryWord("011001110000011")));
    gtp3.adjoin(GridCell(g3,3,BinaryWord("011001110000111")));

    ARIADNE_TEST_EQUALS(product(gtp1,gtp2),gtp3);

    EffectiveVectorMultivariateFunction x = EffectiveVectorMultivariateFunction::identity(2);
    EffectiveVectorMultivariateFunction f({x[0]*x[0]-exp(x[1])-11});

    ARIADNE_TEST_EQUALS(outer_skew_product(gtp1,g2,f).size(),11);


}

} // namespace Ariadne

Int main() {
    test_grid_paving_cursor();

    test_grid_paving_const_iterator();

    test_grid_cell();

    test_grid_open_cell_one();
    test_grid_open_cell_two();
    test_grid_open_cell_three();
    test_grid_open_cell_four();
    test_grid_open_cell_five();
    test_grid_open_cell_six();

    test_grid_sub_paving_one();
    test_grid_sub_paving_two();
    test_grid_sub_paving_three();

    test_grid_paving();

    test_adjoin_operation_one();
    test_adjoin_operation_two();
    test_adjoin_operation_three();

    test_adjoin_outer_approximation_operation();
    test_adjoin_inner_approximation_operation_one();
    test_adjoin_inner_approximation_operation_two();
    test_adjoin_inner_approximation_operation_three();

    test_restrict();

    test_remove_one();
    test_remove_two();

    test_cell_subset_subset();

    test_subsets_join();
    test_subsets_intersection();
    test_subsets_difference();
    test_cell_intersect_subset();
    test_subset_intersect_subset();
    test_subset_subset_subset();
    test_subset_intersects_box();
    test_subset_subset_box();
    test_subset_superset_box();

    test_products();

    return ARIADNE_TEST_FAILURES;
}

