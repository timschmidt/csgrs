/***************************************************************************
 *            geometry/grid_paving.hpp
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

/*! \file geometry/grid_paving.hpp
 *  \brief Grid paving is used to represent sets, based on integer and dyadic coordinate cells, of a grid.
 */

#ifndef ARIADNE_GRID_SET_HPP
#define ARIADNE_GRID_SET_HPP

#include <iostream>
#include <iomanip>
#include <string>

#include <memory>

#include "utility/tribool.hpp"
#include "utility/array.hpp"
#include "utility/iterator.hpp"

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
#include "geometry/grid_cell.hpp"

#include "io/graphics_interface.hpp"

namespace Ariadne {

// Some type definitions
typedef Array<Int> IndexArray;
typedef Array<SizeType> SizeArray;

class Projection;

// Some pre-declarations
class BinaryTreeNode;
class Grid;
class GridAbstractCell;
class GridCell;
class GridOpenCell;
class GridTreeSubpaving;
class GridTreePaving;

class GridTreeCursor;
class GridTreeConstIterator;

// Declarations of classes in other files
template<class BS> class ListSet;

//! \brief The GridTreeCursor throws this exception if we try to go beyond the binary tree.
class NotAllowedMoveException : public std::logic_error {
  public:
    NotAllowedMoveException(const StringType& str) : std::logic_error(str) { }
};


class BinaryTreeNode;

Bool subset( const GridCell& theCell, const GridTreeSubpaving& theSet );
Bool intersect( const GridCell& theCell, const GridTreeSubpaving& theSet );
Bool subset( const GridTreeSubpaving& theSet1, const GridTreeSubpaving& theSet2 );
GridTreePaving join( const GridTreeSubpaving& theSet1, const GridTreeSubpaving& theSet2 );
GridTreePaving intersection( const GridTreeSubpaving& theSet1, const GridTreeSubpaving& theSet2 );
GridTreePaving difference( const GridTreeSubpaving& theSet1, const GridTreeSubpaving& theSet2 );
Bool intersect( const GridTreeSubpaving& theSet1, const GridTreeSubpaving& theSet2 );

GridTreePaving product( const GridTreeSubpaving& theSet1, const GridTreeSubpaving& theSet2 );

GridTreePaving image(const GridTreePaving& theSet, const Projection& theProjection);

//! \brief This class represents a subpaving of a paving. Note that, the subtree enclosed into
//! this class is just a pointer to the node in the tree of some paving. This class is not
//! responsible for deallocation of that original tree.
class GridTreeSubpaving
    : public virtual SubPavingInterface
    , public virtual Drawable2dInterface
{
  protected:

    friend class GridTreeCursor;

    friend GridTreePaving outer_approximation( const EffectiveEuclideanCompactSetInterface& theSet, const Grid& theGrid, const Nat numSubdivInDim );
    friend GridTreePaving inner_approximation( const EffectiveEuclideanOpenSetInterface& theSet, const Grid& theGrid, const Nat numSubdivInDim );

    //! \brief The pointer to the root node of the subpaving tree.
    //! Note that, this is not necessarily the root node of the corresponding paving tree.
    BinaryTreeNode * _pRootTreeNode;

    //! \brief The paving cell corresponding to the root node of the SubPaving.*/
    GridCell _theGridCell;

    //! \brief this function takes the interval width and computes how many binary subdivisions
    //! one has to make in order to have sub-intervals of the width <= \a theMaxWidth
    Nat compute_number_subdiv( FloatDP theWidth, const FloatDP theMaxWidth) const;

    //! \brief This method checks whether the set defined by \a pCurrentNode is a superset
    //! of \a theBoxType, in case when it is known that the cell corresponding to the root of
    //! pCurrentNode [and defined by \a theGrid, the primary cell (\a theExtent) to which
    //! this tree is (virtually) rooted via the path theWord] encloses \a theBoxType.
    //! This is a recursive procedure and it returns true only if there are no disabled
    //! cells in \a pCurrentNode that intersect with theBoxType.
    static ValidatedKleenean _superset( const BinaryTreeNode* pCurrentNode, const Grid& theGrid,
                                        const Nat theExtent, BinaryWord &theWord, const ExactBoxType& theBoxType );

    //! \brief This method checks whether the set defined by \a pCurrentNode is a subset
    //! of \a theBoxType. The set of \a pCurrentNode is defined by \a theGrid, the primary
    //! cell (\a theExtent) to which this tree is (virtually) rooted via the path theWord.
    //! This is a recursive procedure and it returns true only if all enabled sub-cells of
    //! \a pCurrentNode are sub-sets of \a theBoxType.
    static ValidatedKleenean _subset( const BinaryTreeNode* pCurrentNode, const Grid& theGrid,
                                      const Nat theExtent, BinaryWord &theWord, const ExactBoxType& theBoxType );

    //! \brief This method checks whether \a theBoxType is disjoint from the set defined by
    //! \a pCurrentNode, \a theGrid, the primary cell (\a theExtent) to which this
    //! tree is (virtually) rooted via the path theWord. This is done using the recursive
    //! procedure by checking the cells of the tree that intersect with the box and going
    //! down to the leaves. When reaching a leaf node that is enabled we conclude that we
    //! have an intersection. If there are no such nodes then there is no intersection,
    //! and the sets are disjoint.
    static ValidatedKleenean _disjoint( const BinaryTreeNode* pCurrentNode, const Grid& theGrid,
                                        const Nat theExtent, BinaryWord &theWord, const ExactBoxType& theBoxType );

    //! \brief This method checks whether \a theBoxType overlaps the set defined by
    //! \a pCurrentNode, \a theGrid, the primary cell (\a theExtent) to which this
    //! tree is (virtually) rooted via the path theWord. This is done using the recursive
    //! procedure by checking the cells of the tree that overlap the box and going
    //! down to the leaves. When reaching a leaf node that is enabled we conclude that we
    //! have an overlap. If there are no such nodes then there is no overlap. Note that this
    //! method cannot be implemented using disjoint since disjoint tests if the closures
    //! are disjoint, and overlaps tests if the interiors are not disjoint.
    static ValidatedKleenean _intersects( const BinaryTreeNode* pCurrentNode, const Grid& theGrid,
                                          const Nat theExtent, BinaryWord &theWord, const ExactBoxType& theBoxType );

    //! Allow to convert the number of subdivisions in each dimension, i.e. \a numSubdivInDim,
    //! starting from the zero cell into the number of subdivisions that have to be done in a
    //! subpaving rooted to the primary cell of the extent \a primaryCellExtent and having a
    //! length of the path from the primary cell to the root cell of length \a primaryToRootCellPathLength.
    //! Returns zero if the binary tree already has a sufficient number of subdivisions due to
    //! its root cell being smaller than the cells obtained when subdividing \a numSubdivInDim
    //! times in each dimension starting from the zero-cell level.
    Nat zero_cell_subdivisions_to_tree_subdivisions( const Nat numSubdivInDim, const Nat primaryCellExtent,
                                                     const Nat primaryToRootCellPathLength ) const;

  public:

    //! \brief A short name for the constant Iterator
    typedef GridTreeConstIterator ConstIterator;

    //!@{
    //! \name Constructors

    //! \brief The new root node can only be constructed from the existing tree node.
    //! Here, \a pRootTreeNode is not copied, we simply store its pointer inside this class.
    //! Note that, \a pRootTreeNode should correspond to the sub-paving root node. Thus,
    //! \a theExtent defines the extent of the primary root cell of the GridTreeSet
    //! (Remember that every GridTreeSubset is just a reference to a subtree of a GridTreeSet).
    //! \a theWord defines the path to the \a pRootTreeNode node from the primary root cell
    //! of the corresponding GridTreeSet.
    GridTreeSubpaving( const Grid& theGrid, const Nat theExtent, const BinaryWord& theWord, BinaryTreeNode * pRootTreeNode );

    //! \brief A copy constructor that only copies the pointer to the root of the binary tree and the cell
    GridTreeSubpaving( const GridTreeSubpaving &otherSubset);

    //! \brief Make a dynamically-allocated copy as a GridTreeSet. Required for Drawable2dInterface.
    virtual GridTreeSubpaving* clone() const override;

    //!@}

    //! Virtual destructor. The destructor needs to be virtual since GridTreeSet is a subclass
    //! with different memory management.
    virtual ~GridTreeSubpaving() override;

    //!@{
    //! \name Properties

    //! \brief True if the set is empty.
    Bool is_empty() const;

    //! \brief The number of activated cells in the set.
    virtual SizeType size() const override;

    //! \brief The dimension of the set.
    virtual DimensionType dimension() const override;

    //! \brief Returns a constant reference to the underlying grid.
    virtual const Grid& grid() const override;

    //! \brief Returns the const pointer to the root BinaryTreeNode of the SubPaving*/
    const BinaryTreeNode * binary_tree() const;

    //! Recalculate the depth of the tree rooted at \a _pRootTreeNode
    Nat tree_depth() const;

    //! The measure (area, volume) of the set in Euclidean space.
    FloatDPApproximation measure() const;

    //! The a branch along the binary tree.
    GridTreeSubpaving branch(Bool left_or_right) const;

    //! \brief Returns the \a GridCell corresponding to the ROOT NODE of this \a GridTreeSubset
    //! WARNING: It is NOT the primary cell of the paving!
    virtual GridCell root_cell() const override;

    //! \brief Computes a bounding box for a grid set.
    virtual UpperBoxType bounding_box() const override;

    //! \brief Allows to test if the two subpavings are "equal". The method returns true if
    //! the grida are equal and the binary trees are equal. Note that, only in case both
    //! GridTreeSubset objects are recombines, this method is guaranteed to tell you that
    //! the two GridTreeSubset represent equal sets.
    Bool operator==(const GridTreeSubpaving& anotherGridTreeSubset) const;

    //!@}

    //!@{
    //! \name Modifying operations

    //! \brief Sets the ROOT NODE of this \a GridTreeSubset to either enabled (true) or disabled (false).
    virtual Void set_root_cell(Bool enabled_or_disabled) override;

    //!@}

    //!@{
    //! \name Subdivisions

    //! \brief Subdivides the tree in such a way that it's total depth becomes ( extent + numSubdivInDim ) * D
    //! Where extent is the height in each dimension of the primary cell to which the tree is rooted,
    //! \a numSubdivInDim is the number of subdivisions in each dimension, and D is the number of dimensions of our space.
    //! Note that, in case the subset is already subdivided to the required depth then nothing is done.
    //! The latter can happen if the root cell of the subset is below the depth ( extent + numSubdivInDim ) * D.
    virtual Void mince( Nat numSubdivInDim ) override;

    //! \brief Subdivides the set up to the depth specified by the parameter.
    //! Note that the depth is measured from the unit cell and thus the subdivision
    //! is done relative to it but not to the root of the original paving.
    Void mince_to_depth( const Nat theNewDepth );

    //! \brief Subdivides the tree up to the depth specified by the parameter.
    //! Note that, we start from the root of the sub-paving and thus the subdivision
    //! is done relative to it but not to the root of the original paving.
    Void mince_to_tree_depth( const Nat theNewTreeDepth );

    //! \brief Subdivide the paving until the smallest depth such that the leaf
    //! cells size is <= \a theMaxCellWidth. Note that, the disabled cells are
    //! not subdivided.
    Void subdivide( ApproximateDouble theMaxCellWidth );

    //! \brief Recombines the subdivisions, for instance if all subcells of a cell are
    //! enabled/disabled then they are put together.
    virtual Void recombine() override;

    //!@}

    //!@{
    //! \name Geometric Predicates

    //! \brief Tests if a cell is a subset of a set.
    friend Bool subset( const GridCell& theCell, const GridTreeSubpaving& theSet );

    //! \brief Tests if a cell intersect (as an open set) a paving set.
    friend Bool intersect( const GridCell& theCell, const GridTreeSubpaving& theSet );

    //! \brief Tests if a grid set \a theSet1 is a subset of \a theSet2.
    friend Bool subset( const GridTreeSubpaving& theSet1, const GridTreeSubpaving& theSet2 );

    //! \brief Join (make union of) two grid paving sets.
    friend GridTreePaving join( const GridTreeSubpaving& theSet1, const GridTreeSubpaving& theSet2 );

    //! \brief The intersection of two grid paving sets. Points only lying on the
    //! intersection of the boundaries of the two sets are not included in the result.
    friend GridTreePaving intersection( const GridTreeSubpaving& theSet1, const GridTreeSubpaving& theSet2 );

    //! \brief The difference of two grid paving sets. (Results in theSet1 minus theSet2)
    friend GridTreePaving difference( const GridTreeSubpaving& theSet1, const GridTreeSubpaving& theSet2 );

    //! \brief Tests if two grid paving sets intersect (as open sets)
    //! If at least one of the GridTreeSubsets represents an empty set, then the result is false.
    friend Bool intersect( const GridTreeSubpaving& theSet1, const GridTreeSubpaving& theSet2 );

    //! \brief Tests if a grid set equals another paving.
    virtual Bool equals(const SubPavingInterface&) const override;

    //! \brief Tests if a grid set is a subset of another paving.
    virtual Bool subset(const SubPavingInterface&) const override;

    //! \brief Tests if a grid set intersects another paving.
    virtual Bool intersects(const SubPavingInterface&) const override;

    //! \brief Tests if a cell is a subset of the set.
    virtual Bool superset( const GridCell& theCell) const override;

    //! \brief Tests if a grid set is a subset of a box.
    Bool subset( const ExactBoxType& theBoxType ) const;

    //! \brief Tests if a grid set is a superset of a box.
    Bool superset( const ExactBoxType& theBoxType ) const;

    //! \brief Tests if a grid set intersects (the interior) of a box.
    Bool intersects( const ExactBoxType& theBoxType ) const;

    //! \brief Tests if a grid set is disjoint from (the interior of) box.
    Bool disjoint( const ExactBoxType& theBoxType ) const;

    //! \brief Tests if the interior of a grid set is a superset of a box.
    ValidatedLowerKleenean covers( const ExactBoxType& theBoxType ) const;

    //! \brief Tests if (the closure of) a grid set is a subset of the interior of box.
    virtual ValidatedLowerKleenean inside( const ExactBoxType& theBoxType  ) const override;

    //! \brief Tests if (the closure of) a grid set is disjoint from (the closure of) a box.
    virtual ValidatedLowerKleenean separated( const ExactBoxType& theBoxType  ) const override;

    //! \brief Tests if a grid set overlaps (intersects the interior of) a box.
    virtual ValidatedLowerKleenean overlaps( const ExactBoxType& theBoxType ) const override;

    //!@}

    //!@{
    //! \name Iterators

    //! \brief A constant Iterator through the enabled leaf nodes of the subpaving.
    ConstIterator begin() const;

    //! \brief A constant Iterator to the end of the enabled leaf nodes of the subpaving.
    ConstIterator end() const;

    //!@}

    //!@{
    //! \name Conversions

    //! \brief An assignment operator that only copies the pointer to the root of the binary tree and the cell
    GridTreeSubpaving& operator=( const GridTreeSubpaving &otherSubset);

    //! \brief Convert to a list of ordinary boxes, unrelated to the grid.
    operator ListSet<ExactBoxType>() const;

    //!@}

    //!@{
    //! \name Input/Output

    //! \brief Draw on a two-dimensional canvas.
    virtual Void draw(CanvasInterface& canvas, const Projection2d& projection) const override;

    //! \brief Write to an output stream.
    virtual OutputStream& _write(OutputStream& os) const override;

    friend OutputStream& operator<<(OutputStream& os, const GridTreeSubpaving& theGridTreeSubset);
    //!@}

  private:
    virtual GridTreeSubpaving* _branch(Bool left_or_right) const override;
    virtual ForwardConstantIteratorInterface<GridCell>* _begin() const override;
    virtual ForwardConstantIteratorInterface<GridCell>* _end() const override;

};

//! \ingroup ListSetSubModule
//! \ingroup DiscretisedSetSubModule
//! \brief The %GridTreePaving class represents a set of cells with mixed integer and dyadic coordinates.
//! The cells can be enabled or disabled (on/off), indicating whether they belong to the paving or not.
//! It is possible to have cells that are neither on nor off, indicating that they have enabled and
//! disabled sub cells.
//!
//! A trivial cell (level 0) of the paving corresponds to a unit hypercube, of the n-dimensional state
//! space. Subdivision of any 0-level paving is based on dyadic numbers and thus is a binary-style
//! partitioning of the cell. All cells with purely integer coordinates are enclosed in a (virtual)
//! binary tree. We illustrate this for a two dimensional case:
//!   1. Take the unit cell [0, 0]*[1, 1],
//!   2. Cells [-1, 0]*[0, 1] and [0, 0]*[1, 1] are rooted to [-1, 0]*[1, 1],
//!   3. Cells [-1, -1]*[0, 0] and [0, -1]*[1, 0] are rooted to [-1, -1]*[1, 0],
//!   4. Cells [-1, -1]*[1, 0] and [-1, 0]*[1, 1] are rooted to [-1, -1]*[1, 1].
//!
//! \sa GridCell
class GridTreePaving
    : public virtual PavingInterface
    , public GridTreeSubpaving
{

  public:

    typedef Grid GridType;

    typedef GridCell CellType;

  protected:

    friend class GridTreeCursor;

    //! \brief This method takes the extent of the primary cell
    //! \a otherPavingPCellExtent and if it is:
    //!   (a) higher then for this paving, pre-pends \a _pRootTreeNode.
    //!   (b) lower then for this paving, locates it in this paging's tree.
    //!   (c) equal to the hight of this paving's primary cell, does nothing.
    //! This method returns the node corresponding to the primary cell of
    //! extent \a otherPavingPCellExtent in the (updated) paving.
    //! If \a stop_on_enabled is set to true then for the case (b) if we meet
    //! a leaf node on the path from the primary node of the paving to the primary
    //! node of the cell, we stop locating the node corresponding to the primary
    //! cell of extent \a otherPavingPCellExtent and set \a has_stopped to true.
    //! If \a stop_on_disabled is set to true then for the case (b) if we meet
    //! a leaf node on the path from the primary node of the paving to the primary
    //! node of the cell, we stop locating the node corresponding to the primary
    //! cell of extent \a otherPavingPCellExtent and set \a has_stopped to true.
    BinaryTreeNode* align_with_cell( const Nat otherPavingPCellExtent, const Bool stop_on_enabled, const Bool stop_on_disabled, Bool & has_stopped );

    //! \brief This method adjoins a cell with root extent \a cellExtent with path \a pCellPath to this paving.
    Void _adjoin_cell( const Nat cellExtent, BinaryWord const& pCellPath );

    //! \brief This method adjoins the outer approximation of \a theSet (computed on the fly) to this paving.
    //! We use the primary cell (enclosed in this paving) of extent \a primary_cell_hight and represented
    //! by the paving's binary node \a pBinaryTreeNode. When adding the outer approximation, we compute it
    //! up to the level of accuracy given by \a max_mince_tree_depth. This parameter defines, how many subdivisions
    //! of the binary tree we should make to get the proper cells for outer approximating \a theSet.
    //! This method is recursive, the parameter \a pPath defines the path to the current node pBinaryTreeNode
    //! from the root node in recursive calls, thus the initial evaluate for this method must be done with an empty word.
    static Void _adjoin_outer_approximation( const Grid & theGrid, BinaryTreeNode * pBinaryTreeNode, const Nat primary_cell_extent,
                                             const Nat max_mince_tree_depth, const EffectiveEuclideanCompactSetInterface& theSet, BinaryWord * pPath );
    static Void _adjoin_outer_approximation( const Grid & theGrid, BinaryTreeNode * pBinaryTreeNode, const Nat primary_cell_extent,
                                             const Nat max_mince_tree_depth, const ValidatedEuclideanCompactSetInterface& theSet, BinaryWord * pPath );

    //! \brief This method adjoins the inner approximation of \a theSet (computed on the fly) to this paving.
    //! We use the primary cell (enclosed in this paving) of extent \a primary_cell_extent and represented
    //! by the paving's binary node \a pBinaryTreeNode. When adding the inner approximation, we compute it
    //! up to the level of accuracy given by \a max_mince_tree_depth. This parameter defines, how many subdivisions
    //! of the binary tree we should make to get the proper cells for inner approximating \a theSet.
    //! This method is recursive, the parameter \a pPath defines the path to the current node pBinaryTreeNode
    //! from the root node in recursive calls, thus the initial evaluate for this method must be done with an empty word.
    static Void _adjoin_inner_approximation( const Grid & theGrid, BinaryTreeNode * pBinaryTreeNode, const Nat primary_cell_extent,
                                             const Nat max_mince_tree_depth, const EffectiveEuclideanOpenSetInterface& theSet, BinaryWord * pPath );

    //! \brief This method adjoins the lower approximation of \a theSet (computed on the fly) to this paving.
    //! We use the primary cell (enclosed in this paving) of extent \a primary_cell_hight and represented
    //! by the paving's binary node \a pBinaryTreeNode. When adding the lower approximation, we compute it
    //! up to the level of accuracy given by \a max_mince_tree_depth. This parameter defines, how many subdivisions
    //! of the binary tree we should make to get the proper cells for lower approximating \a theSet.
    //! This method is recursive, the parameter \a pPath defines the path to the current node pBinaryTreeNode
    //! from the root node in recursive calls, thus the initial evaluate for this method must be done with an empty word.
    //! The approximation method does not recombine cells, as knowing that both children intersect a set is more
    //! information than knowing that the parent does.
    static Void _adjoin_lower_approximation( const Grid & theGrid, BinaryTreeNode * pBinaryTreeNode, const Nat primary_cell_extent,
                                             const Nat max_mince_tree_depth, const EffectiveEuclideanOvertSetInterface& theSet, BinaryWord * pPath );

    //! \brief This method adjoins the lower approximation of \a theSet (computed on the fly) to this paving.
    //! It is specialised for open sets, for which we have the superset() operator. If a set is a superset of
    //! a cell, then we know it overlaps the cell and all its children.
    static Void _adjoin_lower_approximation( const Grid & theGrid, BinaryTreeNode * pBinaryTreeNode, const Nat primary_cell_extent,
                                             const Nat max_mince_tree_depth, const EffectiveEuclideanOpenSetInterface& theSet, BinaryWord * pPath );

    //! \brief This method is uset to do restriction of this set to the set given by
    //! \a theOtherSubPaving Note that, here we require that the extent of the primary
    //! root cell of this set is >= the extent of the primary root cell of \a theOtherSubPaving.
    Void restrict_to_lower( const GridTreeSubpaving& theOtherSubPaving );

    //! \brief This method is uset to remove \a theOtherSubPaving from this set.
    //! Note that, here we require that the extent of the primary root cell of
    //! this set is >= the extent of the primary root cell of \a theOtherSubPaving.
    Void remove_from_lower( const GridTreeSubpaving& theOtherSubPaving );

    //! \brief This method changes the primary cell of this GridTreeSet.
    //! We only can increase the extent of the primary cell, this is why
    //! if toPCellExtent <= this->cell().extent(), then nothing is done.
    Void up_to_primary_cell( const Nat toPCellExtent );

  public:
    //!@{
    //! \name Constructors

    //! \brief Create a %GridTreeSet based on zero dimensions.
    //! This constructor is needed to use the Boost Serialization library.
    GridTreePaving( );

    //! \brief The new root node can only be constructed from the existing tree node.
    //! Here, \a pRootTreeNode is not copied, we simply store its pointer inside this class.
    //! Note that, \a pRootTreeNode should correspond to the root node. \a theExtent defines
    //! the extent of the primary root cell corresponding to the \a pRootTreeNode node.
    GridTreePaving( const Grid& theGrid, const Nat theExtent, BinaryTreeNode * pRootTreeNode );

    //! \brief Construct a grid tree set from a single cell.
    GridTreePaving( const GridCell & theGridCell );

    //! \brief The copy constructor that actually copies all the data,
    //! including the paving tree. I.e. the new copy of the tree is created.
    GridTreePaving( const GridTreePaving & theGridTreeSet );

    //! A simple constructor that creates the [0, 1]*...*[0, 1] cell in the
    //! \a theDimension - dimensional space. Here we assume that we have a non scaling
    //! grid with no shift of the coordinates. I.e. Grid._data._origin = {0, ..., 0}
    //! and Grid._data._lengths = {1, ..., 1}. If enable == true then the cell is enabled
    explicit GridTreePaving( const Nat theDimension, const Bool enable = false );

    //! \brief Construct an empty tree. The \a theBoundingBoxType is used to define the lattice
    //! block (in theGrid) that will correspond to the root of the paving tree \a pRootTreeNode.
    explicit GridTreePaving( const Grid& theGrid, const Bool enable = false  );

    //! \brief Construct an empty tree. The \a theLatticeBoxType is used to define the lattice
    //! block (in theGrid) that will correspond to the root of the paving tree \a pRootTreeNode.
    explicit GridTreePaving( const Grid& theGrid, const ExactBoxType & theLatticeBoxType );

    //! \brief Construct the paving based on the block's coordinates, defined by: \a theLeftLowerPoint
    //! and \a theRightUpperPoint. These are the coordinates in the lattice defined by theGrid.
    //! The primary cell, enclosing the given block of cells is computed automatically and the
    //! binary tree is rooted to that cell. The \a theEnabledCells Array defines the enabled/disabled
    //! 0-level cells of the block (lexicographic order).
    explicit GridTreePaving( const Grid& theGrid, const IndexArray theLeftLowerPoint,
                          const IndexArray theRightUpperPoint, const BooleanArray& theEnabledCells );

    //! \brief Creates a new paving from the user data. \a theTree is an Array representation of the binary
    //! tree structure, \a theEnabledCells tells whether a node is or is not a leaf, \a theExtent gives the
    //! extent of the primary cell which is assumed to correspond to the root node of \a theTree.
    explicit GridTreePaving( const Grid& theGrid, Nat theExtent, const BooleanArray& theTree, const BooleanArray& theEnabledCells );

    //!@}

    //!@{
    //! \name Cloning/Copying/Assignment

    //! \brief The copy assignment operator, which copies all the data
    //! including the paving tree if necessary.
    GridTreePaving& operator=( const GridTreePaving & theGridTreeSet );

    //! \brief Return a new dynamically-allocated copy of the %GridTreeSet.
    //! In this case, all the data is copied.
    GridTreePaving* clone() const;

    //!@}

    //! \brief Destructor, removes all the dynamically allocated data, any
    //! GridTreeSubset referencing this %GridTreeSet becomes invalid.
    virtual ~GridTreePaving();



    //!@{
    //! \name Geometric Operations

    // PIETER: You may prefer to make inplace operations may return
    // a reference to *this, allowing chaining of operations.

    //! \brief Clears the set (makes empty set on same grid).
    Void clear( );

    //! \brief Adjoin (make inplace union with) a single cell.
    Void adjoin( const GridCell& theCell );

    //! \brief Remove a single cell.
    Void remove( const GridCell& theCell );

    //! \brief Adjoin (make inplace union with) another grid paving set.
    Void adjoin( const GridTreeSubpaving& theOtherSubPaving );

    //! \brief Restrict to (make inplace intersection with) another grid paving set.
    Void restrict( const GridTreeSubpaving& theOtherSubPaving );

    //! \brief Remove cells in another grid paving set.
    Void remove( const GridTreeSubpaving& theOtherSubPaving );

    //! \brief Adjoin (make inplace union with) an abstract paving set.
    Void adjoin( const SubPavingInterface& theOtherSubPaving );
    //! \brief Restrict to (make inplace intersection with) an abstract paving set.
    Void restrict( const SubPavingInterface& theOtherSubPaving );
    //! \brief Remove cells from  an abstract paving set.
    Void remove( const SubPavingInterface& theOtherSubPaving );

    //! \brief Restrict to cells rooted to the primary cell with the extent (at most) \a theExtent.
    Void restrict_to_extent( const Nat theExtent );

    //! /brief Creates an over approximation for the \a theBoxType on \a theGrid. \a theBoxType
    //! is in the original space coordinates. We compute the over approximation as the
    //! smallest primary cell on the Grid, such that it contains \a theBoxType (after it's
    //! mapping on \a theGrid )
    GridCell smallest_enclosing_primary_cell(const UpperBoxType& theBoxType) const;
    //!@}

    //!@{
    //! \name Geometric Operations

    //! \brief Join (make union of) two grid paving sets.
    friend GridTreePaving join( const GridTreeSubpaving& theSet1, const GridTreeSubpaving& theSet2 );

    //! \brief The intersection of two grid paving sets. Points only lying on the
    //! intersection of the boundaries of the two sets are not included in the result.
    friend GridTreePaving intersection( const GridTreeSubpaving& theSet1, const GridTreeSubpaving& theSet2 );

    //! \brief The difference of two grid paving sets. (Results in theSet1 minus theSet2)
    friend GridTreePaving difference( const GridTreeSubpaving& theSet1, const GridTreeSubpaving& theSet2 );

    //! \brief The Cartesian product of two grid paving sets.
    friend GridTreePaving product( const GridTreeSubpaving& theSet1, const GridTreeSubpaving& theSet2 );

    //! \brief The image of a grid paving set under a coordinate projection.
    friend GridTreePaving image( const GridTreePaving&, const Projection&);

    //! \brief Compute an outer-approximation to the set \f$\{ (x_1,f(x_1)) \mid x_1 \in S_1 \}\f$ preserving the projection onto \f$S_1\f$.
    friend GridTreePaving outer_skew_product(GridTreePaving const& theSet1, Grid const& theGrid2,
                                             ValidatedVectorMultivariateFunction const& theFunction);

    //!@}

    //!@{
    //! \name Geometric Approximation

    //! \brief Adjoin an over approximation to box, computing to the given resolution:
    //! \a numSubdivInDim -- defines, how many subdivisions in each dimension from the level of
    //! the zero cell we should make to get the proper cells for outer approximating \a theSet.
    //! \pre The box must have nonempty interior.
    Void adjoin_over_approximation( const ExactBoxType& theBoxType, const Nat numSubdivInDim );

    //! \brief Adjoin an outer approximation to a given set, computing to the given resolution.
    //! This method computes an outer approximation for the set \a theSet on the grid \a theGrid.
    //! Note that, the depth is the total number of subdivisions (in all dimensions) of the unit
    //! cell of the grid. This method does the followig:
    //!   1. Computes the smallest Primary cell enclosing \a theSet
    //!   2. Allocates the paving for this cell
    //!   3. Minces the paving to the level: depth + \<the primary cell extent\>
    //!   4. Iterates through the enabled leaf nodes of the paving (all the nodes are initially enabled)
    //!   5. Disables the cells that are disjoint with the \a theSet
    Void adjoin_outer_approximation( const ValidatedEuclideanCompactSetInterface& theSet, const Nat numSubdivInDim );
    Void adjoin_outer_approximation( const EffectiveEuclideanCompactSetInterface& theSet, const Nat numSubdivInDim );
    Void adjoin_outer_approximation( const UpperBoxType& theBoxType, const Nat numSubdivInDim );

    //! \brief Adjoin a lower approximation to a given set, computing to the given extent and resolution:
    //! \a numSubdivInDim -- defines, how many subdivisions in each dimension from the level of the
    //! zero cell we should make to get the proper cells for outer approximating \a theSet.
    //! A lower approximation comprises all cells intersecting a given set.
    Void adjoin_lower_approximation( const EffectiveEuclideanOvertSetInterface& theSet, const Nat heightInDim, const Nat numSubdivInDim );

    //! \brief Adjoin a lower approximation to a given set restricted to the given bounding box,
    //! computing to the given resolution: \a numSubdivInDim -- defines, how many subdivisions in each
    //! dimension from the level of the zero cell we should make to get the proper cells for outer
    //! approximating \a theSet. A lower approximation comprises all cells intersecting a given set.
    Void adjoin_lower_approximation( const EffectiveEuclideanOvertSetInterface& theSet, const ExactBoxType& bounding_box, const Nat numSubdivInDim );

    //! \brief Adjoin a lower approximation to a given set, computing to the given resolution
    //! \a numSubdivInDim -- defines, how many subdivisions in each dimension from the level of the
    //! zero cell we should make to get the proper cells for outer approximating \a theSet.
    //! A lower approximation comprises all cells intersecting a given set.
    Void adjoin_lower_approximation( const EffectiveEuclideanLocatedSetInterface& theSet, const Nat numSubdivInDim );

    //! \brief Adjoin an inner approximation to a given set, computing to the given extent and resolution:
    //! \a numSubdivInDim -- defines, how many subdivisions in each dimension from the level of the
    //! zero cell we should make to get the proper cells for outer approximating \a theSet.
    //! An inner approximation comprises all cells that are sub-cells of the given set.
    Void adjoin_inner_approximation( const EffectiveEuclideanOpenSetInterface& theSet, const Nat heightInDim, const Nat numSubdivInDim );

    //! \brief Adjoin an inner approximation to a given set restricted to the given bounding box,
    //! computing to the given resolution: \a numSubdivInDim -- defines, how many subdivisions in each
    //! dimension from the level of the zero cell we should make to get the proper cells for outer
    //! approximating \a theSet. An inner approximation comprises all cells that are sub-cells of
    //! the given set.
    Void adjoin_inner_approximation( const EffectiveEuclideanOpenSetInterface& theSet, const ExactBoxType& bounding_box, const Nat numSubdivInDim );

    //! \brief Adjoin an inner approximation to the given set, computing to the given resolution:
    //! \a numSubdivInDim -- defines, how many subdivisions in each
    //! dimension from the level of the zero cell we should make to get the proper cells for outer
    //! approximating \a theSet. An inner approximation comprises all cells that are sub-cells of
    //! the given set.
    Void adjoin_inner_approximation( const EffectiveEuclideanSetInterface& theSet, const Nat numSubdivInDim );

    Void adjoin_inner_approximation( const LowerBoxType& theBoxType, const Nat numSubdivInDim );
    //!@}

    //!@{
    //! \name Input/output routines.

    friend OutputStream& operator<<(OutputStream& os, const GridTreePaving& theGridTreeSet);
    //!@}

};

GridTreePaving outer_approximation(const ExactBoxType& theBoxType, const Grid& theGrid, const Nat fineness);
GridTreePaving outer_approximation(const ExactBoxType& theBoxType, const Nat fineness);
GridTreePaving outer_approximation( const EffectiveEuclideanCompactSetInterface& theSet, const Grid& theGrid, const Nat numSubdivInDim );
GridTreePaving outer_approximation( const EffectiveEuclideanCompactSetInterface& theSet, const Nat numSubdivInDim );
GridTreePaving outer_approximation( const ValidatedEuclideanCompactSetInterface& theSet, const Grid& theGrid, const Nat numSubdivInDim );
GridTreePaving outer_approximation( const ValidatedEuclideanCompactSetInterface& theSet, const Nat numSubdivInDim );
GridTreePaving inner_approximation( const EffectiveEuclideanOpenSetInterface& theSet, const Grid& theGrid, const Nat extent, const Nat numSubdivInDim );
GridTreePaving inner_approximation( const EffectiveEuclideanOpenSetInterface& theSet, const Grid& theGrid, const ExactBoxType& bounding_box, const Nat numSubdivInDim );
GridTreePaving inner_approximation( const EffectiveEuclideanSetInterface& theSet, const Grid& theGrid, const Nat numSubdivInDim );

template<class BS> GridTreePaving outer_approximation(const ListSet<BS>& theSet, const Grid& theGrid, const Nat numSubdivInDim);


//! \brief This class represents a cursor/Iterator that can be used to traverse a subtree.
class GridTreeCursor {
  private:
    //The size with which the stack size will be incremented
    static const Nat STACK_SIZE_INCREMENT = 256;

    //The index of the top stack element (within the stack Array)
    //WARNING: the initial value should be -1 to indicate that
    //there are no elements in the stack
    Int _currentStackIndex;

    // The nodes traversed to the current location.
    Array<BinaryTreeNode*> _theStack;

    // The subpaving to cursor on
    const GridTreeSubpaving * _pSubPaving;

    GridCell _theCurrentGridCell;

    //! \brief Push the node into the atack
    Void push( BinaryTreeNode* pLatestNode );

    //! \brief Pop the node from the atack, return nullptr is the stack is empty
    BinaryTreeNode* pop( );

    //! \brief Check is the stack contains just one element, i.e. we are at the root
    Bool is_at_the_root() const;

    //! \brief this method is supposed to update the _theCurrentGridCell value, when the cursos moves
    //! if left_or_right == false then we go left, if left_or_right == true then right
    //! if left_or_right == indeterminate then we are going one level up.
    Void updateTheCurrentGridCell( BinaryTreeDirection up_or_left_or_right );

    friend OutputStream& operator<<(OutputStream& os, const GridTreeCursor& theGridTreeCursor);

  public:
    //! \brief Default constructor constructs an invalid cursor.
    GridTreeCursor();

    //! \brief The constructor that accepts the subpaving to cursor on
    GridTreeCursor(const GridTreeSubpaving * pSubPaving);

    //! \brief The simple copy constructor, this constructor copies the pointer to
    //! GridTreeSubset and thus the internal stack information remains valid.
    GridTreeCursor(const GridTreeCursor & otherCursor);

    //! \brief This is an assignment operator that acts similar to the copy
    //! constructor. Here we copy the pointer to underlying GridTreeSubset.
    GridTreeCursor & operator=(const GridTreeCursor & otherCursor);

    //! This destructor does not deallocate the enclosed sub paving.
    ~GridTreeCursor();

    //! \brief Test if the current node is enabled.
    Bool is_enabled() const;

    //! \brief Test if the current node is disabled*/
    Bool is_disabled() const;

    //! \brief Test if the current node is a leaf.
    Bool is_leaf() const;

    //! \brief Test if the current node is the root of the subtree.
    //! Returns true if the stack has only one element.
    Bool is_root() const;

    //! \brief Returns true if the current node is the left child of the parent node
    Bool is_left_child() const;

    //! \brief Returns true if the current node is the right child of the parent node
    Bool is_right_child() const;

    //!@{
    //! \name Leaf Operations

    //! \brief Markes the leaf node as enabled, otherwise they through \a NotALeafNodeEsception
    Void set_enabled() const;

    //! \brief Markes the leaf node as disabled, otherwise they through \a NotALeafNodeEsception
    Void set_disabled() const;

    //!@}

    //! \brief Move to the parent node. Throws an NotAllowedMoveException if the current node is the root.
    //! Returns a reference to itself.
    GridTreeCursor& move_up();

    //! \brief Move to the left child node. Throws an NotAllowedMoveException if the current node is a leaf.
    GridTreeCursor& move_left();

    //! \brief Move to the right child node. Throws an NotAllowedMoveException if the current node is a leaf.
    GridTreeCursor& move_right();

    //! \brief Move to a neighbouring node. Throws an NotAllowedMoveException if the current node is a leaf
    //! and we try to move to a child, or if the current node is the root and we try to move up.
    GridTreeCursor& move(BinaryTreeDirection up_or_left_or_right);

    //! \brief Convert to a GridCell.
    const GridCell& cell() const;

    //! \brief Allows to test if the two cursors are equal, this is determined by
    //! the fact that they point to the same binary-tree node.
    //! NOTE: if _currentStackIndex < 0 for at least one of the cursors then the
    //! result is always false.
    Bool operator==(const GridTreeCursor& anotherGridTreeCursor) const;

    //! \brief The dereferencing operator which returns a
    //! reference to the GridTreeSubset for the current node.
    GridTreeSubpaving operator*();

    //! \brief The dereferencing operator which returns a constant
    //! reference to the GridTreeSubset for the current node.
    const GridTreeSubpaving operator*() const;
};

//! \brief This class allows to iterate through the enabled leaf nodes of GridTreeSubset.
//! The return objects for this Iterator are constant GridCells.
class GridTreeConstIterator
    : public IteratorFacade< GridTreeConstIterator, GridCell const, ForwardTraversalTag >
    , public virtual ForwardConstantIteratorInterface<GridCell>
{
  private:
    //! \brief When set to true indicates that this is the "end Iterator"
    Bool _is_in_end_state;

    //! \brief the cursor object that is created based on the given sub paving*/
    GridTreeCursor _pGridTreeCursor;

    friend class IteratorCoreAccess;

    //!@{
    //! \name Iterator Specific

    virtual Void increment();

    //! \brief Returns true if:
    //! both iterators are in the "end Iterator" state
    //! both iterators are NOT in the "end Iterator" state
    //! and they point to the same node of the same sub paving
    Bool equal( GridTreeConstIterator const & theOtherIterator) const;

    virtual GridCell const& dereference() const;

    //!@}

    //!@{
    //! \name Local

    //! \brief Allows to navigate to the first (\a firstLast==true ),
    //! last (\a firstLast==false) enabled leaf of the sub paving
    //! Returns true if the node was successfully found. If nothing is
    //! found then the cursor should be in the root node again.
    Bool navigate_to(Bool firstLast);

    //! \brief A recursive search function, that looks for the next enabled leaf in the tree.
    //! The search is performed from left to right. (Is used for forward iteration)
    Void find_next_enabled_leaf();

    //!@}

  public:
    //!@{
    //! \name Constructors

    //! \brief Default constructor constructs an invalid Iterator.
    //! \internal Note that a default constructor is required for compliance with the
    //! STL Trivial Iterator concept.
    GridTreeConstIterator();

    //! \brief The constructor that accepts the subpacing \a pSubPaving to iterate on
    //! The paramerter \a firstLastNone indicatges whether we want to position the Iterator
    //! on the first enabled leaf node (firstLastNone == true) or the last one (firstLastNone == false)
    //! or we are constructing the "end Iterator" that does not point anywhere.
    explicit GridTreeConstIterator( const GridTreeSubpaving * pSubPaving, const ValidatedKleenean firstLastNone );

    //! \brief The copy constructor that copies the current state by copying the
    //! underlying GridTreeCursor. The latter is copied by it's copy constructor.
    GridTreeConstIterator( const GridTreeConstIterator& theGridPavingIter ) = default;

    //!@}

    //! \brief An assignment operator that copies the current state by copying the
    //! underlying GridTreeCursor. The latter is copied by it's assignment operator.
    GridTreeConstIterator & operator=( const GridTreeConstIterator& theGridPavingIter ) = default;


    //! This destructor only deallocates the GridTreeCursor, note that the
    //! latter one does not deallocate the enclosed sub paving.
    ~GridTreeConstIterator() = default;

    //!@{
    //! \name Get the cursor of the Iterator

    // This cursor is only needed to get access to enable/disable node functionality
    GridTreeCursor const& cursor() const;

    //!@}
  private:
    // Return a dynamically-allocated copy.
    virtual GridTreeConstIterator* clone() const;
    // Test equality with another abstract iterator.
    virtual Bool equals( ForwardConstantIteratorInterface<GridCell> const & theOtherIterator) const;
    // Write to an output stream.
    virtual OutputStream& _write(OutputStream& os) const;

};

//**************************************Inline functions*********************************************/

//***************************************GridTreeCursor**********************************************/

inline const GridCell& GridTreeCursor::cell() const {
    return _theCurrentGridCell;
}

//***************************************GridTreeConstIterator************************************/

inline GridCell const& GridTreeConstIterator::dereference() const {
    return _pGridTreeCursor.cell();
}

inline GridTreeCursor const& GridTreeConstIterator::cursor() const {
    return _pGridTreeCursor;
}

//**************************************FRIENDS OF GridTreeSet******************************************/

template<class BS>
GridTreePaving outer_approximation(const ListSet<BS>& theSet, const Grid& theGrid, const Nat numSubdivInDim) {
    ARIADNE_ASSERT_MSG( theSet.dimension()==theGrid.dimension(),"theSet="<<theSet<<", theGrid="<<theGrid );
    GridTreePaving result( theGrid );
    for(auto basicSet : theSet) {
        result.adjoin_outer_approximation( basicSet, numSubdivInDim );
    }
    result.recombine();
    return result;
}

//**************************************Drawing*********************************************************/

Void draw(CanvasInterface& theGraphic, const Projection2d& theProjection, const GridCell& theGridCell);
Void draw(CanvasInterface& theGraphic, const Projection2d& theProjection, const GridTreePaving& theGridTreeSet);
Void draw(CanvasInterface& theGraphic, const Projection2d& theProjection, const EffectiveEuclideanCompactSetInterface& theSet);

} // namespace Ariadne

#endif /* ARIADNE_GRID_SET_HPP */

