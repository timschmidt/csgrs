/***************************************************************************
 *            geometry/binary_tree.hpp
 *
 *  Copyright  2008-20  Ivan S. Zapreev, Pieter Collins
 *
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

#ifndef ARIADNE_BINARY_TREE_HPP
#define ARIADNE_BINARY_TREE_HPP


#include <iostream>
#include <iomanip>

#include "function/functional.hpp"
#include "utility/macros.hpp"
#include "utility/exceptions.hpp"
#include "helper/stlio.hpp"
#include "utility/binary_word.hpp"
#include "geometry/list_set.hpp"


namespace Ariadne {

//! \brief The binary-tree node operation is not allowed on a non-leaf node.
class NotALeafNodeException : public std::logic_error {
  public:
    NotALeafNodeException(const StringType& str) : std::logic_error(str) { }
};

//! \brief The binary-tree node operation is not allowed on a leaf node.
class IsALeafNodeException : public std::logic_error {
  public:
    IsALeafNodeException(const StringType& str) : std::logic_error(str) { }
};

typedef std::vector<Bool> BooleanArray;

//***************************************BinaryTreeNode************************************/

//! \brief The binary tree node.
//!
//! This node is to be used in a binary tree designed for subdividing the state
//! space into enabled and disabled cells. This is required for representing
//! subsets of the state space.
//!
//! \b Storage: We only store pointers to the left and right subtrees and the ValidatedKleenean
//! value indicating whether this cell is enabled/disabled or we do not know.
class BinaryTreeNode {
  protected:
    //! \brief Defines whether the given node of the tree is on/off or we do not know*/
    ValidatedKleenean _isEnabled;

    /*! \brief The left and right subnodes of the tree. Note that,
     * \a pLeftNode == \a nullptr iff \a pRightNode == \a nullptr. The latter
     * is allowed iff \a _isEnabled == \a TRUE or \a _isEnabled == \a FALSE,
     * i.e. the node can be a leaf.
     */
    BinaryTreeNode* _pLeftNode;
    BinaryTreeNode* _pRightNode;

    //! \brief This method splits the enabled subtrees of the tree rooted to
    //! \a pCurrentNode in such a way that the tree depth becomes \a depth.
    //! If the initial tree depth is greater than \a depth then nothing is done.
    Void mince_node(BinaryTreeNode* pCurrentNode, const Nat depth);

    //! \brief This method recombined the sub tree nodes rooted to \a pCurrentNode.
    //! Note that, the two leaf nodes with the same parent are removed if they have
    //! the same value of isEnabled fields.
    Void recombine_node(BinaryTreeNode * pCurrentNode);

    //! \brief This method is used for recursive restoration of the binary tree from
    //! the used data, i.e. \a theTree and \a theEnabledCells. It is used in the
    //! constructor \a BinaryTreeNode( const BooleanArray& , const BooleanArray& )
    Void restore_node( BinaryTreeNode * pCurrentNode, Nat & arr_index, Nat & leaf_counter,
                       const BooleanArray& theTree, const BooleanArray& theEnabledCells);

    //! \brief This method is used in constructors for the node initialization
    Void init( ValidatedKleenean isEnabled, BinaryTreeNode* pLeftNode, BinaryTreeNode* pRightNode );

  public:
    //!@{
    //! \name Constructors

    //! \brief Construct a tree node.
      explicit BinaryTreeNode(const ValidatedKleenean _isEnabled = false );

    //! \brief The copy constructor.
    //! The the node and all it's sub nodes are copied.
    explicit BinaryTreeNode(const BinaryTreeNode& theTreeNode);

    //! \brief Constructs a binary tree from the boolean arrays. \a theTree defines
    //! the tree structure, \a theEnabledCells defines the tree nodes.
    //! IVAN S. ZAPREEV:
    //! WARNING: We assume that every node has either no children or both of them!
    //! NOTE: The dinary data of the tree is organized in the following way:
    //!          1      The \a theTree contains Depth first search lay out of the tree,
    //!         / \     where 1 stands for the non-leaf node and 0 for a leaf node, we
    //!        /   \    always visit the left su-node first, e.g. the tree on the left
    //!       2     5   is encodes the Array:     [1, 1, 0, 0, 0]
    //!      / \        where the corresponding    ^  ^  ^  ^  ^
    //!     /   \       tree nodes are:            1  2  3  4  5
    //!    3     4      \a theEnabledCells contains true/false values for the leaf nodes
    //! of the tree. Their order is the same as in \a theTree, e.g. here it is: 3,4,5
    explicit BinaryTreeNode( const BooleanArray& theTree, const BooleanArray& theEnabledCells );

    //!@}

    ~BinaryTreeNode();

    //!@{
    //! \name Properties

    //! \brief Returns true if the node is marked as enabled, otherwise false
    Bool is_enabled() const;

    //! \brief Returns true if some of the leaf nodes in the tree rooted to this node are enabled, otherwise false
    Bool has_enabled() const;

    //! \brief Returns true if all leaf nodes in the tree rooted to this node are enabled, otherwise false
    Bool all_enabled() const;

    //! \brief This method returns true if the given path defines a node in the tree that is either enabled
    //! or is in a "virtual" subtree of some enabled node (note that enabled nodes can only be leafs).
    //! Note that: \a path is treated as if it is rooted to this node, we assume that the path starts
    //! from position \a position of \a path. The parameter \a position is used for recursive calls only.
    Bool is_enabled( const BinaryWord & path, const Nat position = 0) const;

    //! \brief Returns true if the node is marked as disabled, otherwise false
    Bool is_disabled() const;

    //! \brief Returns true if the node is a leaf (pLeftNode == nullptr && pRightNode == nullptr) otherwise false
    Bool is_leaf() const;

    //! \brief Return the left or right sub-node
    BinaryTreeNode * child_node(Bool left_or_right) const;

    //! \brief Return the left sub-node
    BinaryTreeNode * left_node() const;

    //! \brief Return the right sub-node
    BinaryTreeNode * right_node() const;

    //! \brief Returns the depth of the sub-tree rooted to the given node, i.e. the depth of it's deepest node
    Nat depth() const;

    //! \brief Allows to compare to binaty tree nodes
    Bool operator==(const BinaryTreeNode & otherNode ) const;

    //! \brief Marks the node as enabled or disabled.
    Void set(Bool all_enabled_or_all_disabled);

    static Bool is_equal_nodes( const BinaryTreeNode * pFirstNode, const BinaryTreeNode * pSecondNode );

    //!@}

    //!@{
    //! \name Leaf Operations

    //! \brief This method makes the node to become a leaf node with the enabled value : \a is_enabled
    //! NOTE: the leat and the right sub-trees (are deallocated.
    //! WARNING: this method MUST NOT be called on a non-leaf node!!!
    Void make_leaf( ValidatedKleenean is_enabled );

    //! \brief Marks the leaf node as enabled, otherwise they through \a NotALeafNodeEsception
    Void set_enabled();

    //! \brief Marks the leaf node as disabled, otherwise they through \a NotALeafNodeEsception
    Void set_disabled();

    //! \brief Marks the node as neither enabled nor disabled, is only applicable to non-leaf nodes.
    //! When applied to a leaf node, throws IsALeafNodeException.
    Void set_unknown();

    //! \brief Splits the leaf node, i.e. adds two subnodes with the _isEnabled field value inherited from the parent node.
    //! The parent's _isEnabled is set to intermediate, because the subsequent operation on the subtree might enable/disable
    //! subnodes and we do not want to keep track of these changes. If the node is not a leaf then nothing is done.
    Void split();

    //! \brief This method splits the enabled subtrees of the tree rooted to
    //! this node in such a way that the tree depth becomes \a depth.
    //! If the initial tree depth is greater than \a depth then nothing is done.
    Void mince(const Nat depth);

    //!@}

    //!@{
    //! \name

    //! \brief Allows to assign one binary tree node to the other, this is done by copying
    //! the nodes value and the sub-trees. The sub-trees of this node are deallocated.
    BinaryTreeNode& operator=(const BinaryTreeNode & otherNode );

    //! \brief Copy all the data (including the sub-nodes) from the node pointed by \a pOtherNode into the given node.
    //! Note that here we will create copies of the sub nodes, and NOT just copy pointers to them!
    Void copy_from( const BinaryTreeNode * pOtherNode );

    //! \brief This method recombined the sub tree nodes. Note that, the two leaf nodes with
    //! the same parent are removed if they have the same value of isEnabled fields.
    Void recombine();

    //! \brief Stores the binary tree in a form of two arrays, their structure is the same as needed for
    //! the BinaryTreeNode( const BooleanArray& , const BooleanArray&  ) constructor
    Void tree_to_binary_words( BinaryWord & tree, BinaryWord & leaves ) const;

    //! \brief Stores the binary tree node as a string*/
    String node_to_string() const;

    //! \brief Finds(creates) the leaf node defined by the \a path and marks it as enabled.
    //! If some prefix of the \a path references an enabled node then nothing is done.
    Void add_enabled( const BinaryWord & path );

    //! \brief This method adjoins the enabled nodes of \a subTree to this tree.
    //! Note that, the position of the root node of \a subTree within this
    //! tree is defined by path.
    Void add_enabled( const BinaryTreeNode * pOtherSubTree, const BinaryWord & path );

    //! \brief This method merges \a pFromTreeRoot into \a pToTreeRoot.
    //! The enabled nodes of the former tree are added to the latter tree.
    //! If in the latter tree there is an enabled leaf node and in the former
    //! tree the corresponding node is not a leaf, then this node of the latter
    //! tree stays intact. If on the other hand we have a non-leaf node in
    //! \a pToTreeRoot and we are adding to it an enabled node of \a pFromTreeRoot
    //! then we just "substitute" the former one with the latter.
    //! NOTE: 1. This function is recursive. 2. No pointers are copied between
    //! \a pToTreeRoot and \a pFromTreeRoot.
    Void add_enabled( BinaryTreeNode* pToTreeRoot, const BinaryTreeNode* pFromTreeRoot );

    //! \brief Starting in the \a pNode node as at the root, this method counts
    //! the number of enabled leaf nodes in the subtree
    //! rooted at pNode.
    static SizeType count_enabled_leaf_nodes( const BinaryTreeNode* pNode );

    //! \brief Starting in the \a pRootTreeNode node as at the root, this method finds(creates)
    //! the leaf node defined by the \a path and marks it as enabled. If some prefix of the \a path
    //! references an enabled node then nothing is done.
    //! NOTE: This is a recursive method on the position (\a position) in the binary path (\a path).
    //! Therefore, the initial evaluate of this method should be done with \a position == 0;
    static Void add_enabled( BinaryTreeNode* pRootTreeNode, const BinaryWord& path, const Nat position = 0 );

    //! \brief Creates a binary tree of the height rootNodePath.size(), puts the subtree oldRootNode
    //! into the node defined by the path \a rootNodePath, returns the root node of the extended tree.
    static BinaryTreeNode * prepend_tree( const BinaryWord & rootNodePath, BinaryTreeNode * oldRootNode);

    //! \brief This method restricts \a pThisNode to \a pOtherNode.
    //! In essance we do the inplace AND on the tree node pThisNode.
    //! Note that, this method is recursive.
    static Void restrict( BinaryTreeNode * pThisNode, const BinaryTreeNode * pOtherNode );

    //! \brief This method removed enabled nodes of \a pOtherNode from \a pThisNode.
    //! Note that, this method is recursive.
    static Void remove( BinaryTreeNode * pThisNode, const BinaryTreeNode * pOtherNode );

    //! \brief checks if two trees intersect in a set-theory sence.
    //! I.e. we assume that pRootNodeOne and pRootNodeTwo correspond to the same (virtual) root
    //! and then we see if the enabled leaf node of one tree contain enabled leaf nodes of
    //! another tree as their (virtual) children.
    static Bool intersect( const BinaryTreeNode * pRootNodeOne, const BinaryTreeNode * pRootNodeTwo );

    //! \brief checks if the tree pRootNodeOne is a subset of the tree pRootNodeTwo, in a set-theory sence.
    //! I.e. we assume that pRootNodeOne and pRootNodeTwo correspond to the same (virtual) root
    //! and then we see if every enabled leaf node of pRootNodeOne is contained in the enabled leaf nodes of
    //! pRootNodeTwo, or it is covered by the enabled leaf nodes of pRootNodeTwo. When we _write, contained and
    //! covered then we mean: is a subnode in the (virtual) tree and all it's subnodes in the (virtual) tree.
    static Bool subset( const BinaryTreeNode * pRootNodeOne, const BinaryTreeNode * pRootNodeTwo );

    //!@}
};

//***************************************BinaryTreeNode**********************************************/

inline Void BinaryTreeNode::init( ValidatedKleenean isEnabled, BinaryTreeNode* pLeftNode, BinaryTreeNode* pRightNode ) {
    _isEnabled = isEnabled;
    _pLeftNode = pLeftNode;
    _pRightNode = pRightNode;
}

inline BinaryTreeNode::BinaryTreeNode(const ValidatedKleenean isEnabled) {
    init( isEnabled, nullptr, nullptr );
}

inline BinaryTreeNode::BinaryTreeNode(const BinaryTreeNode& theTreeNode) {
    if( (theTreeNode._pLeftNode) != nullptr && ( theTreeNode._pRightNode != nullptr ) ) {
        init( theTreeNode._isEnabled, new BinaryTreeNode( *theTreeNode._pLeftNode ), new BinaryTreeNode( *theTreeNode._pRightNode ) );
    } else {
        //NOTE: We do not allow for nodes where one leaf is nullptr and another is not
        init( theTreeNode._isEnabled, nullptr, nullptr );
    }
}

inline BinaryTreeNode::BinaryTreeNode( const BooleanArray& theTree, const BooleanArray& theEnabledCells ) {
    //Make default initialization
    init( false, nullptr, nullptr ) ;

    //If the tree is not empry and there are enabled leafs then do the thing
    if( ( theTree.size() ) > 0 && ( theEnabledCells.size() > 0 ) ) {
        Nat arr_index = 0, leaf_counter = 0;
        restore_node( this, arr_index, leaf_counter, theTree, theEnabledCells );
    } else {
        //Otherwise settle with one disabled node
        this->set_disabled();
    }
}

inline BinaryTreeNode::~BinaryTreeNode() {
    if( _pLeftNode != nullptr ) {
        delete _pLeftNode;
    }
    if( _pRightNode != nullptr ) {
        delete _pRightNode;
    }
}

inline Bool BinaryTreeNode::is_enabled() const {
    return definitely(_isEnabled);
}

inline Bool BinaryTreeNode::is_disabled() const{
    return ! possibly(_isEnabled) ;
}

inline Bool BinaryTreeNode::is_leaf() const{
    return (_pLeftNode == nullptr) && (_pRightNode == nullptr);
}

inline BinaryTreeNode * BinaryTreeNode::child_node(Bool left_or_right) const {
    return left_or_right ? _pRightNode : _pLeftNode;
}

inline BinaryTreeNode * BinaryTreeNode::left_node() const {
    return _pLeftNode;
}

inline BinaryTreeNode * BinaryTreeNode::right_node() const {
    return _pRightNode;
}

inline Void BinaryTreeNode::set(Bool enabled_or_disabled) {
    _isEnabled = enabled_or_disabled;
    if( _pLeftNode != nullptr ) {
        delete _pLeftNode;
    }
    if( _pRightNode != nullptr ) {
        delete _pRightNode;
    }
}

inline Void BinaryTreeNode::set_enabled() {
    if ( is_leaf() ) {
        _isEnabled = true;
    } else {
        throw NotALeafNodeException(ARIADNE_PRETTY_FUNCTION);
    }
}

inline Void BinaryTreeNode::copy_from( const BinaryTreeNode * pOtherNode ) {
    if( pOtherNode != nullptr ) {
        _isEnabled = pOtherNode->_isEnabled;
        if( _pLeftNode != nullptr) { delete _pLeftNode; _pLeftNode = nullptr; }
        if( _pRightNode != nullptr) { delete _pRightNode; _pRightNode = nullptr; }
        if( pOtherNode->_pLeftNode != nullptr ) { _pLeftNode = new BinaryTreeNode( * (pOtherNode->_pLeftNode) ); }
        if( pOtherNode->_pRightNode != nullptr ) { _pRightNode = new BinaryTreeNode( * (pOtherNode->_pRightNode) ); }
    }
}

inline Void BinaryTreeNode::set_disabled() {
    if ( is_leaf() ) {
        _isEnabled = false;
    } else {
        throw NotALeafNodeException(ARIADNE_PRETTY_FUNCTION);
    }
}

inline Void BinaryTreeNode::set_unknown() {
    if ( ! is_leaf() ) {
        _isEnabled = indeterminate;
    } else {
        throw IsALeafNodeException(ARIADNE_PRETTY_FUNCTION);
    }
}

inline Void BinaryTreeNode::make_leaf(ValidatedKleenean is_enabled ) {
    _isEnabled = is_enabled;
    if( _pLeftNode != nullptr ) { delete _pLeftNode; _pLeftNode= nullptr; }
    if( _pRightNode != nullptr ) { delete _pRightNode; _pRightNode= nullptr; }
}

inline Void BinaryTreeNode::split() {
    if ( is_leaf() ) {
        _pLeftNode  = new BinaryTreeNode(_isEnabled);
        _pRightNode = new BinaryTreeNode(_isEnabled);
        set_unknown();
    }
}

inline Void BinaryTreeNode::add_enabled( const BinaryWord& path ) {
    add_enabled( this, path, 0 );
}

inline Void BinaryTreeNode::mince(const Nat depth) {
    mince_node(this, depth);
}

inline Void BinaryTreeNode::recombine() {
    recombine_node(this);
}

inline String BinaryTreeNode::node_to_string() const {
    std::stringstream tmp_stream;
    tmp_stream << "BinaryTreeNode( isLeaf = " << is_leaf() << ", isEnabled = " << is_enabled() << ", isDisabled = " << is_disabled() << " )";
    return tmp_stream.str();
}

inline BinaryTreeNode& BinaryTreeNode::operator=( const BinaryTreeNode & otherNode ) {
    //Copy the node value
    _isEnabled = otherNode._isEnabled;

    //Deallocate memory for the children, if any
    if( _pLeftNode != nullptr ) { delete _pLeftNode; _pLeftNode = nullptr; }
    if( _pRightNode != nullptr ) { delete _pRightNode; _pRightNode = nullptr; }

    //Copy the children trees from the otherNode
    if( otherNode._pLeftNode != nullptr ) { _pLeftNode = new BinaryTreeNode( * ( otherNode._pLeftNode ) ); }
    if( otherNode._pRightNode != nullptr ) { _pRightNode = new BinaryTreeNode( * ( otherNode._pRightNode ) ); }

    return *this;
}

//*************************************FRIENDS OF BinaryTreeNode***************************************/

//! \brief Stream insertion operator, prints out two binary arrays, one is the tree structure
//! and the other is the true/false (enabled/disabled) values for the leaf nodes
inline OutputStream& operator<<(OutputStream& output_stream, const BinaryTreeNode & binary_tree ) {
    BinaryWord tree, leaves;
    binary_tree.tree_to_binary_words( tree, leaves );
    return output_stream << "BinaryTreeNode( Tree: " << tree << ", Leaves: " << leaves << ")";
}


} // namespace Ariadne


#endif /* ARIADNE_BINARY_TREE_HPP */
