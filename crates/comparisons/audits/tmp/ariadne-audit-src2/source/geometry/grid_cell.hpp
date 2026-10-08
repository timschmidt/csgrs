/***************************************************************************
 *            geometry/grid_cell.hpp
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

/*! \file geometry/grid_cell.hpp
 *  \brief Cells on a grid defined by a binary word.
 */

#ifndef ARIADNE_GRID_CELL_HPP
#define ARIADNE_GRID_CELL_HPP

#include <iostream>
#include <iomanip>
#include <string>
#include <memory>

#include "utility/tribool.hpp"
#include "utility/array.hpp"

#include "utility/binary_word.hpp"

#include "utility/exceptions.hpp"
#include "geometry/box.hpp"
#include "geometry/point.hpp"
#include "geometry/list_set.hpp"

#include "numeric/numeric.hpp"

#include "geometry/set_interface.hpp"
#include "geometry/paving_interface.hpp"
#include "algebra/vector.hpp"
#include "geometry/grid.hpp"


namespace Ariadne {

// Some type definitions
typedef std::vector<Bool> BooleanArray;
typedef Array<Int> IndexArray;
typedef Array<SizeType> SizeArray;

typedef unsigned short dimension_type;

// Some pre-declarations
class BinaryTreeNode;
class Grid;
class GridAbstractCell;
class GridCell;
class GridOpenCell;

typedef Box<ExactIntervalType> LatticeBoxType;

enum class BinaryTreeDirection : ComparableEnumerationType { LEFT=-1, UP=0, RIGHT=+1 };
enum class BinaryChild : ComparableEnumerationType { LEFT=-1, RIGHT=+1 };
enum class TernaryChild : ComparableEnumerationType { LEFT=-1, MIDDLE=0, RIGHT=+1 };

//! \brief An abstract cell of a grid paving. This class is the base of the GridCell - a regular cell on the Grid
//! and the GridOpenCell - an open cell on a Grid. Here we only store common data and operations
//!
//!This class contains the geometric data of a cell, as well as the combinatoric data needed to easily adjoin it to a tree.
//!It does not contain the tree structure, so cannot be used in cursors/iterators.
//!NOTE: The "Primary root cell" is the cell that the \a GridTreeSet can be rooted to
//!(the \a GridTreeSubset can be rooted to a non primary cell).
class GridAbstractCell {
  protected:
    //! \brief The intrinsic grid of the cell. Note that grids are internally passed by reference.
    Grid _theGrid;

    //! \brief The level of the primary root cell relative to the zero level.
    Nat _theExtent;

    //! \brief The word describing the path in a binary tree. This path defines the cell in the Grid.
    BinaryWord _theWord;

    //! \brief The geometric box represented by the cell.
    ExactBoxType _theBoxType;

    //! \brief The box is given as an explicit parameter. This should only be used
    //! by friend classes which have already computed the box.
    GridAbstractCell(const Grid& theGrid, const Nat theExtent, const BinaryWord& theWord, const ExactBoxType& theBoxType);

    //! \brief The copy constructor for the \a GridAbstractCell
    GridAbstractCell(const GridAbstractCell& theGridCell);

    friend class GridTreeCursor;

    //! \brief having \a theExtent the height of the primary cell, with \a leftBottomCorner and \a rightTopCorner
    //! defining the primary cell corners for \a theExtent-1, we recompute \a leftBottomCorner and \a rightTopCorner
    //! for the current level \a theExtent.
    static inline Void primary_cell_at_extent( const Nat theExtent, Int & leftBottomCorner, Int & rightTopCorner );

    //! \brief This function allows to compare to cells it is used by the operator== and operator< methods of this class
    //! The value of \a comparator should be either \a COMPARE_EQUAL or \a COMPARE_LESS
    //! The function checks that both cells are on the same grid and then alignes their primary cells.
    //! The latter is done by extending the binary word of the cell with the lowest primary cell with
    //! the corresponding prefix. When the words are alligned, wi simply use the == and < operators of
    //! the \a BinaryWord class.
    //! NOTE: Since the function is only based on comparison of the Grids, the words definings the cells and the
    //! primary cell heights we can use it in sub-classes for comparing cells of the same types
    static Bool compare_abstract_grid_cells(const GridAbstractCell * pCellLeft, const GridAbstractCell &cellRight, const Nat comparator );

  public:

    //! \brief The constant for the grid cells comparison
    static const Nat COMPARE_EQUAL = 0;
    static const Nat COMPARE_LESS = 1;

    //! \brief The underlying grid.
    const Grid& grid() const;

    //! \brief The extent (height in each coordinate) of the primary cell to which this cell is rooted.
    Nat root_extent() const;

    //! \brief The height in the tree of the primary cell to which this cell is rooted.
    Nat root_height() const;

    //! \brief The depth in the lattice at which the cell lies.
    Int depth() const;

    //! \brief The word describing the path in a binary tree from the primary cell of height (this.height()) to this cell.
    const BinaryWord& word() const;

    //! \brief The geometric box represented by the cell.
    const ExactBoxType& box() const;

    //! \brief The dimension of the cell. Needed by the BoxExpression interface.
    dimension_type dimension() const;

    //! \brief The upper and lower bound in the \a i<sup>th</sup> coordinate.
    ExactIntervalType operator[](dimension_type i) const;

    //! \brief Allows to assign one GridCell to another
    GridAbstractCell& operator=( const GridAbstractCell & otherCell );

    //! \brief The primary-cell box on some grid is defined by the height above the zero level cells.
    //!    The coordinates of the primary cells change uniformly. For example (3-D):
    //!      |Height | (left bottom) | (right top)   |
    //!      |-------|---------------|---------------|
    //!      |   0   | (0, 0, 0)     | (1, 1, 1)     |
    //!      |   1   | (-1, -1, -1)  | (1, 1, 1)     |
    //!      |   2   | (-1, -1, -1)  | (3, 3, 3)     |
    //!      |   3   | (-5, -5, -5)  | (3, 3, 3)     |
    //!      |   4   | (-5, -5, -5)  | (11, 11, 11)  |
    //!    The cell size is always double the size of the previous-level cell and one
    //!    of the corners stays the same as well. If Li, Ri are the left-, right-corned
    //!    coordinates at livel i then we have the following recursive definition:
    //!      L0 = 0, R0 = 0
    //!      Li = 2*L(i-1) - R(i-1) (if i is odd)
    //!      Li = L(i-1)            (if i is even)
    //!      Ri = R(i-1)            (if i is odd)
    //!      Ri = 2*R(i-1) - L(i-1) (if i is even)
    //!
    //! WARNING: The returned box must be interptered relative to some grid.
    //! In other words, we assume some lattice and the relation to the original
    //! space is not taken into account.
    //!
    //! Here
    //! \a theExtent defines the height of the primary cell above the zero level and
    //! \a dimensions is the number of dimension in the considered space
    static LatticeBoxType primary_cell_lattice_box( const Nat theExtent, const dimension_type dimensions );

    //! \brief Takes the lattice box \a theLatticeBoxType related to some (unknown) grid and computes the
    //! smallest primary cell on that grid that will contain this box.
    //! The method returns the extent (height in each coordinate) of that primary cell.
    static Nat smallest_enclosing_primary_cell_extent( const LatticeBoxType& theLatticeBoxType );

    //! /brief Computes the extent (height in each coordinate) of the primary cell that encloses
    //! (may be exactly) the given box on the given grid.
    static Nat smallest_enclosing_primary_cell_extent( const UpperBoxType & theBoxType, const Grid& theGrid );

    //! \brief This method returns the path from the \a topPCellHeight to the \a bottomPCellHeight
    //! in the \a dimensions dimensional space. We assume that \a topPCellHeight >= \a bottomPCellHeight
    //! if not, then we return an empty binary word.
    static BinaryWord primary_cell_path( const DimensionType dimensions, const Nat topPCellHeight, const Nat bottomPCellHeight);

    //! \brief Apply grid data \a theGrid to \a theLatticeBoxType in order to compute
    //! the box dimensions in the original space
    static ExactBoxType lattice_box_to_space(const LatticeBoxType & theLatticeBoxType, const Grid& theGrid );
};

//! \ingroup BasicSetSubModule
//! \ingroup DiscretisedSetSubModule
//! \brief A cell of a grid paving. Note that this class represents the closed cell.
//! It is uniquely defined by the path from the primary cell and is exactly on cell in a grid.
//! This is different from the GridOpenCell which is open and the path from the root cell defines
//! its lower left sub cell wheres the complete open cell is a union of 4 cells.
//!
//! This class does not contain the tree structure, so cannot be used in cursors/iterators.
//! \sa GridTreeSet
class GridCell : public GridAbstractCell {
  protected:

    friend class GridTreeCursor;

    //! \brief Tests if theCellOne is a subset of theCellTwo, the paths to the cells,
    //! aligned to some common primary cell, which height will be referenced by
    //! \a pPrimaryCellHeight, are returned as \a pathPrefixOne and \a pathPrefixTwo
    friend Bool subset( const GridCell& theCellOne, const GridCell& theCellTwo, BinaryWord * pPathPrefixOne,
                            BinaryWord * pPathPrefixTwo, Nat * pPrimaryCellHeight);

  public:
    //! \brief Default constructor. Needed for some containers and iterators.
    GridCell();

    //! \brief Construct a cell based on \a theGrid, the primary cell of level \a theExtent,
    //! and the path to the SubPaving's root cell which is accessible from the primary cell
    //! via the path \a _theWord. Note that, \a theExtent is the height relative to the Grid,
    //! but not to the original space!
    GridCell(const Grid& theGrid, const Nat theExtent, const BinaryWord& theWord);

    //! \brief Copy constructor.
    GridCell( const GridCell & otherCell );

    //! \brief Allows to split the given cell into two sub-cells. When isRight == true
    //! then we return the right sub-cell, otherwise the left one */
    GridCell split(Bool isRight) const;

    //! \brief Splits the given cell into two sub-cells, returning both.
    Pair<GridCell,GridCell> split() const;

    //! \brief The equality operator.
    Bool operator==(const GridCell& otherCell) const;

    //! \brief Allows to assign one GridCell to another
    GridCell& operator=( const GridCell & otherCell );

    //! \brief A total order on cells on the same grid, by height and word prefix.
    Bool operator<(const GridCell& otherCell) const;

    //! \brief Stream insertion operator for the GridCell.
    friend OutputStream& operator<<(OutputStream& os, const GridCell& theCell);

    //! \brief The cartesian prodcut of the cells, on the product grid.
    friend GridCell product(GridCell const& gc1, GridCell const& gc2);

    //! \brief The dimension of the cell.
    DimensionType dimension() const;

    //! \brief Allows to convert the given GridCell into an open grid cell (GridOpenCell)*/
    GridOpenCell interior() const;

    //! \brief this method computes the box corresponding to this cell in the grid lattice.*/
    static LatticeBoxType compute_lattice_box( const DimensionType dimensions, const Nat theExtent, const BinaryWord& theWord );

    //! \brief this method computes the box in the original space based on the \a theGrid,
    //! and a cell which is obtained by traversing the path given by \a _theWord from the
    //! primary cell located at the heigth \a theExtent above the zero level cells
    //! (relative to the grid).
    static ExactBoxType compute_box(const Grid& theGrid, const Nat theExtent, const BinaryWord& theWord);

    //! /brief Creates an over approximation for the \a theBoxType on \a theGrid. \a theBoxType
    //! is in the original space coordinates. We compute the over approximation as the
    //! smallest primary cell on the Grid, such that it contains \a theBoxType (after it's
    //! mapping on \a theGrid )
    static GridCell smallest_enclosing_primary_cell( const UpperBoxType & theBoxType, const Grid& theGrid );

    //! \brief This function allows to compare to cells it is used by the operator== and operator< methods of this class
    //! The value of \a comparator should be either \a COMPARE_EQUAL or \a COMPARE_LESS
    //! The function checks that both cells are on the same grid and then alignes their primary cells.
    //! The latter is done by extending the binary word of the cell with the lowest primary cell with
    //! the corresponding prefix. When the words are alligned, wi simply use the == and < operators of
    //! the \a BinaryWord class.
    static Bool compare_grid_cells(const GridCell * pCellLeft, const GridCell &cellRight, const Nat comparator );

    //! \brief Computes the neighboring cell to the given cell (defined by \a theGrid, \a theExtent, \a theWord)
    //! in the given dimension \a dim. The resulting cell will the of the same size as the given one.
    static GridCell neighboringCell( const Grid& theGrid, const Nat theExtent, const BinaryWord& theWord, const DimensionType dim );
};

//! \brief An open cell of a grid paving. This cell is open and the path from the primary cell
//! defines its lower left sub cell wheres the complete open cell is a union of 4 cells.
//!
//!This class does not contain the tree structure, so cannot be used in cursors/iterators.
//!NOTE: The open cell is defined by its base cell which is a simple GridCell corresponding
//!to the left-most quadrant cell of the open cell. In this respect, height() and word() return
//!values defining the base cell and box() returns the entire box related to the open cell.
//!This box should be treated as an open set, i.e. its borders should be excluded.
class GridOpenCell: public GridAbstractCell {
  protected:

    friend class GridCell;

    //! \brief The box is given as an explicit parameter. This should only be used
    //! by friend classes which have already computed the box.
    GridOpenCell(const Grid& theGrid, const Nat theExtent, const BinaryWord& theWord, const ExactBoxType& theBoxType);

    //! \brief This method allows to enumerate the GridCells that are located in the positive
    //! axis direction from the base cell. The base cell is defined by \a theExtent and
    //! \a theBaseCellWord. When called \a cellPosition should be an empty word. This is a
    //! technical parameter used in the method's recursivce calls. \a theResultSet is the vector
    //! to which all the neighboring cell will be added, i.e. it is a return parameter.
    //! NOTE: The cell defined by \a theExtent, \a theBaseCellWord will also be in \a theResultSet.
    Void neighboring_cells( const Nat theExtent, const BinaryWord& theBaseCellWord,
                            BinaryWord& cellPosition, GridTreePaving& theResultSet ) const;

    //! \brief This method allows to compute the neighboring (to the right) cell of
    //! the base cell given by \a theExtent and \a theBaseCellWord. Here \a cellPosition
    //! is the position of the neighboring cell with resect to the base cell in the
    //! theGrid.dimensions() dimensional space. Note that if \a cellPosition consists
    //! of theGrid.dimensions() zeroes then we adjoin the base cell itself.
    static GridCell neighboring_cell( const Grid& theGrid, const Nat theExtent,
                                      const BinaryWord& theBaseCellWord, BinaryWord& cellPosition );

    //! \brief This method allows to find the smallest open cell that contains \a theBoxType.
    //! The search is started from \a theOpenCell that is an open cell covering \a theBoxType.
    //! Note: This method is recursive and it assumes that \a theOpenCell
    //! and \a theBoxType are on the same Grid! The latter is not checked, but only assumed.
    //! In the open cell based on \a theOpenCell does not cover \a theBoxType this method returns nullptr.
    static GridOpenCell * smallest_open_subcell( const GridOpenCell &theOpenCell, const ExactBoxType & theBoxType );

    //! \brief Takes the cell \a theCell of the set where it belongs, \a theSet, and see if it has
    //! neighbors in this set. If yes then it creates the open cell lying inside theSet such that
    //! it covers the borders. All such open cells are added to the vector \a result. \a cellPosition
    //! is a technical parameter that has to be set to an empty word. Also, this method add the interior
    //! of theCell to the vector \a result.
    static Void cover_cell_and_borders( const GridCell& theCell, const GridTreePaving& theSet,
                                        BinaryWord& cellPosition, std::vector<GridOpenCell>& result );

  public:
    //! \brief Default constructor. Needed for some containers and iterators.
    GridOpenCell();

    //! \brief Construct a cell based on \a theGrid, the primary cell of level \a theExtent,
    //! and the path to the SubPaving's root cell which is accessible from the primary cell
    //! via the path \a _theWord. Note that, \a theExtent is the height level relative to the Grid,
    //! but not to the original space!
    GridOpenCell(const Grid& theGrid, const Nat theExtent, const BinaryWord& theWord);

    //! \brief Copy constructor.
    GridOpenCell( const GridOpenCell & otherCell );

    //! \brief Allows to split the given cell into two sub-cells. When isRight == true
    //! then we return the right sub-cell, if false then the left one, otherwise the middle one */
    GridOpenCell split(TernaryChild isRight) const;

    //! \brief The equality operator.
    Bool operator==(const GridOpenCell& otherCell) const;

    //! \brief Stream insertion operator.
    friend OutputStream& operator<<(OutputStream& os, const GridOpenCell& theOpenCell);

    //! \brief Allows to assign one GridCell to another
    GridOpenCell& operator=( const GridOpenCell & otherCell );

    //! \brief A total order on cells on the same grid, by height and word prefix.
    Bool operator<(const GridOpenCell& otherCell) const;

    //! \brief Computes all the cells that constitute the GridOpenCell in the form of the GridTreeSet.*/
    GridTreePaving closure() const;

    //! \brief Computes the intersection of two GridOpenCell as a list of open cells whoes union
    //! gives the intersection. This is done becase the intersection can no always be represented
    //! as one open cell. If the intersection is empty then it returns an empty vector.
    //! WARNING: This operation is expensive because we do not always get a single cell as a result.
    //! Moreover, currently intersecting an open cell with itself or a subcell sharing a common border
    //! with the cell will result in more that one open cell. Still the operation is robust, it always
    //! results in a set of open cells whoes union is the exact intersection of the given open cells.
    static List<GridOpenCell> intersection( const GridOpenCell & theLeftOpenCell, const GridOpenCell & theRightOpenCell );

    //! \brief Tests if the two open cells intersect
    static Bool intersect( const GridOpenCell & theLeftOpenCell, const GridOpenCell & theRightOpenCell );

    //! \brief this method computes the box in the original space based on the \a theGrid.
    //! This box should be treated as an open set. I.e. the borders of the box must be excluded.
    //! Note tha, \a theExtent and \a theWord define the left bottom cell (base cell) of the open cell.
    static ExactBoxType compute_box(const Grid& theGrid, const Nat theExtent, const BinaryWord& theWord);

    //! \brief This function allows to compare to cells it is used by the operator== and operator< methods of this class
    //! The value of \a comparator should be either \a COMPARE_EQUAL or \a COMPARE_LESS
    //! The function checks that both cells are on the same grid and then alignes their primary cells.
    //! The latter is done by extending the binary word of the cell with the lowest primary cell with
    //! the corresponding prefix. When the words are alligned, wi simply use the == and < operators of
    //! the \a BinaryWord class.
    static Bool compare_grid_cells(const GridOpenCell * pCellLeft, const GridOpenCell &cellRight, const Nat comparator );

    //! \brief This method computes the smallest open cell that contains the given box \a theBoxType
    //! (in the original space), taking into account the given grid \a theGrid.
    static GridOpenCell outer_approximation( const ExactBoxType & theBoxType, const Grid& theGrid );
};


/****************************************************************************************************/
/***************************************Inline functions*********************************************/
/****************************************************************************************************/

/*****************************************GridAbstractCell*******************************************/

inline GridAbstractCell::GridAbstractCell(const GridAbstractCell& theGridCell):
    _theGrid(theGridCell._theGrid), _theExtent(theGridCell._theExtent),
    _theWord(theGridCell._theWord), _theBoxType(theGridCell._theBoxType) {
}

inline GridAbstractCell::GridAbstractCell(const Grid& theGrid, const Nat theExtent,
                                            const BinaryWord& theWord, const ExactBoxType& theBoxType):
    _theGrid(theGrid), _theExtent(theExtent),
    _theWord(theWord), _theBoxType(theBoxType) {
}

inline Void GridAbstractCell::primary_cell_at_extent( const Nat theExtent, Int & leftBottomCorner, Int & rightTopCorner ) {
    if ( theExtent % 2 == 1 ) {
        leftBottomCorner = 2*leftBottomCorner - rightTopCorner;
    } else {
        rightTopCorner   = 2*rightTopCorner - leftBottomCorner;
    }
}

inline const Grid& GridAbstractCell::grid() const {
    return _theGrid;
}

inline Nat GridAbstractCell::root_extent() const {
    return _theExtent;
}

inline Nat GridAbstractCell::root_height() const {
    return _theExtent * _theGrid.dimension();
}

inline Int GridAbstractCell::depth() const {
    return _theWord.size() - _theExtent * _theGrid.dimension();
}

inline const BinaryWord& GridAbstractCell::word() const {
    return _theWord;
}

inline const ExactBoxType& GridAbstractCell::box() const {
    return _theBoxType;
}

inline dimension_type GridAbstractCell::dimension() const {
    return _theGrid.dimension();
}

inline ExactIntervalType GridAbstractCell::operator[](dimension_type i) const {
    return _theBoxType[i];
}


inline GridAbstractCell& GridAbstractCell::operator=(const GridAbstractCell & otherCell ) {
    _theGrid = otherCell._theGrid;
    _theExtent = otherCell._theExtent;
    _theWord = otherCell._theWord;
    _theBoxType = otherCell._theBoxType;

    return *this;
}

/*********************************************GridCell***********************************************/

inline GridCell::GridCell() : GridAbstractCell( Grid(), 0, BinaryWord(), ExactBoxType() ) {
}

inline GridCell::GridCell(const Grid& theGrid, const Nat theExtent, const BinaryWord& theWord) :
                    GridAbstractCell( theGrid, theExtent, theWord, compute_box(theGrid, theExtent, theWord) ) {
}

inline GridCell::GridCell(const GridCell & otherCell ) : GridAbstractCell(otherCell) {
}

inline GridCell& GridCell::operator=(const GridCell & otherCell ) {
    GridAbstractCell::operator=(otherCell);
    return *this;
}

inline Bool GridCell::operator==( const GridCell& other ) const {
    return compare_grid_cells( this, other, COMPARE_EQUAL );
}

inline Bool GridCell::operator<(const GridCell& other) const {
    return compare_grid_cells( this, other, COMPARE_LESS );
}

inline Bool GridCell::compare_grid_cells(const GridCell * pCellLeft, const GridCell &cellRight, const Nat comparator ) {
    return GridAbstractCell::compare_abstract_grid_cells( pCellLeft, cellRight, comparator );
}

//The box is not related to the Grid, i.e. it is in the original space, whereas the binary tree is related to the Lattice of the grid:
inline ExactBoxType GridCell::compute_box(const Grid& theGrid, const Nat theExtent, const BinaryWord& theWord) {
    //Compute the lattice box and then map it to the original space using the grid data
    return lattice_box_to_space( GridCell::compute_lattice_box( theGrid.dimension(), theExtent, theWord ), theGrid );
}

/*********************************************GridOpenCell***********************************************/

inline GridOpenCell::GridOpenCell(const Grid& theGrid, const Nat theExtent, const BinaryWord& theWord, const ExactBoxType& theBoxType) :
                            GridAbstractCell(theGrid, theExtent, theWord, theBoxType) {
}

inline GridOpenCell::GridOpenCell() : GridAbstractCell( Grid(), 0, BinaryWord(), compute_box(Grid(), 0, BinaryWord()) ) {
}

inline GridOpenCell::GridOpenCell(const Grid& theGrid, const Nat theExtent, const BinaryWord& theWord) :
                    GridAbstractCell( theGrid, theExtent, theWord, compute_box(theGrid, theExtent, theWord) ) {
}

inline GridOpenCell::GridOpenCell(const GridOpenCell & otherCell ) : GridAbstractCell(otherCell) {
}

inline GridOpenCell& GridOpenCell::operator=(const GridOpenCell & otherCell ) {
    GridAbstractCell::operator=(otherCell);
    return *this;
}

inline Bool GridOpenCell::operator==( const GridOpenCell& other ) const {
    return compare_grid_cells( this, other, COMPARE_EQUAL );
}

inline Bool GridOpenCell::operator<(const GridOpenCell& other) const {
    return compare_grid_cells( this, other, COMPARE_LESS );
}

inline Bool GridOpenCell::compare_grid_cells(const GridOpenCell * pCellLeft, const GridOpenCell &cellRight, const Nat comparator ) {
    return GridAbstractCell::compare_abstract_grid_cells( pCellLeft, cellRight, comparator );
}

inline Bool GridOpenCell::intersect( const GridOpenCell & theLeftOpenCell, const GridOpenCell & theRightOpenCell ) {
    return definitely( theLeftOpenCell.box().overlaps(  theRightOpenCell.box() ) );
}


} // namespace Ariadne

#endif /* ARIADNE_GRID_CELL_HPP */

