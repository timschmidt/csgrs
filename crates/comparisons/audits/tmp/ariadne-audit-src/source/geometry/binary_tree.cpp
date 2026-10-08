/***************************************************************************
 *            geometry/binary_tree.cpp
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

#include <iostream>
#include <iomanip>

#include "config.hpp"

#include "function/functional.hpp"
#include "utility/macros.hpp"
#include "utility/exceptions.hpp"
#include "helper/stlio.hpp"
#include "geometry/list_set.hpp"
#include "geometry/binary_tree.hpp"


namespace Ariadne {


Bool BinaryTreeNode::has_enabled() const {
    if( is_leaf() ) {
        return is_enabled();
    } else {
        return ( left_node()->has_enabled() || right_node()->has_enabled() );
    }
}

Bool BinaryTreeNode::all_enabled() const {
    if( is_leaf() ) {
        return is_enabled();
    } else {
        return ( left_node()->all_enabled() && right_node()->all_enabled() );
    }
}

Bool BinaryTreeNode::is_enabled( const BinaryWord & path, const Nat position) const {
    Bool result = false ;

    if( this->is_leaf() ){
        //If we are in an enabled leaf node then the answer is true.
        //Since path.size() >= 0, the node defined by the path is a
        //subnode of an enabled node, thus we return true.
        result = is_enabled();
    } else {
        if( position < path.size() ) {
            //The path is not complete yet and we are in a
            //non-leaf node, so we follow the path in the tree
            if( path[ position ] ) {
                result = right_node()->is_enabled( path, position + 1 );
            } else {
                result = left_node()->is_enabled( path, position + 1 );
            }
        } else {
            //We are somewhere in the tree in a non-leaf node,
            //this node corresponds to the node given by the path
            //If both left and right sub trees are fully enabled,
            //then the cell defined by the binary path is "enabled"
            //in this tree otherwise it is not.
            result = all_enabled();
        }
    }
    return result;
}

Bool BinaryTreeNode::is_equal_nodes( const BinaryTreeNode * pFirstNode, const BinaryTreeNode * pSecondNode ) {
    Bool result = true;

    if( pFirstNode != pSecondNode){
        //If the pointers do not reference the same objects
        if( ( pFirstNode != nullptr ) && ( pSecondNode != nullptr ) ){
            //And both nodes are not null
            if( ! ( (* pFirstNode) == ( * pSecondNode ) ) ){
                //The objects referenced by the pointers do not contain equal data
                result = false;
            }
        } else {
            //One of the nodes is null and the other is not
            result = false;
        }
    }
    return result;
}

Bool BinaryTreeNode::operator==(const BinaryTreeNode & otherNode ) const {
    return ( ( definitely(this->_isEnabled == otherNode._isEnabled) ) ||
             ( is_indeterminate( this->_isEnabled     ) &&
               is_indeterminate( otherNode._isEnabled ) ) )            &&
        is_equal_nodes( this->_pLeftNode , otherNode._pLeftNode )   &&
        is_equal_nodes( this->_pRightNode , otherNode._pRightNode );
}

Void BinaryTreeNode::restrict( BinaryTreeNode * pThisNode, const BinaryTreeNode * pOtherNode ){
    if( ( pThisNode != nullptr ) && ( pOtherNode != nullptr ) ){
        if( pThisNode->is_leaf() && pOtherNode->is_leaf() ){
            //Both nodes are leaf nodes: Make a regular AND
            pThisNode->_isEnabled = (pThisNode->_isEnabled && pOtherNode->_isEnabled);
        } else {
            if( !pThisNode->is_leaf() && pOtherNode->is_leaf() ){
                if( pOtherNode->is_enabled() ){
                    //DO NOTHING: The restriction will not affect pThisNode
                } else {
                    //Turn the node a disabled leaf, since we do AND with false
                    pThisNode->make_leaf(false);
                }
            } else {
                if( pThisNode->is_leaf() && !pOtherNode->is_leaf() ){
                    if( pThisNode->is_enabled() ){
                        //If this node is enabled then copy in the other node
                        //Since it will be their intersection any ways
                        pThisNode->copy_from( pOtherNode );
                    } else {
                        //DO NOTHING: The restriction is empty in this case
                        //because this node is a disabled leaf
                    }
                } else {
                    //Both nodes are non-leaf nodes: Go recursively left and right
                    restrict( pThisNode->_pLeftNode, pOtherNode->_pLeftNode );
                    restrict( pThisNode->_pRightNode, pOtherNode->_pRightNode );
                }
            }
        }
    }
}

Void BinaryTreeNode::remove( BinaryTreeNode * pThisNode, const BinaryTreeNode * pOtherNode ) {
    if( ( pThisNode != nullptr ) && ( pOtherNode != nullptr ) ){
        if( pThisNode->is_leaf() && pOtherNode->is_leaf() ){
            if( pThisNode->is_enabled() && pOtherNode->is_enabled() ){
                //Both nodes are enabled leaf nodes: Make a regular subtraction, i.e. set the false
                pThisNode->_isEnabled = false;
            } else {
                //DO NOTHING: In all other cases there is nothing to be done
            }
        } else {
            if( !pThisNode->is_leaf() && pOtherNode->is_leaf() ){
                if( pOtherNode->is_enabled() ){
                    //Turn the node into a disabled leaf, since we subtract all below
                    pThisNode->make_leaf(false);
                } else {
                    //DO NOTHING: We are trying to remove a disabled node
                }
            } else {
                if( pThisNode->is_leaf() ) {
                    if( pThisNode->is_enabled() ){
                        //This is an enabled leaf node and so we might subtract smth from it
                        //The pOtherNode is not a leaf, due to previous checks so we split
                        //pThisNode and then do the recursion as in case on two non-leaf nodes
                        pThisNode->split();
                    } else {
                        //DO NOTHING: We are trying to remove from a disabled leaf node. Whatever
                        //we are trying to remove, will not have any effect on the result. We return
                        //because in all remaining cases we need to do recursion for sub trees.
                        return;
                    }
                } else {
                    //We will have to do the recursion to remove the leaf nodes
                }
                //Both nodes are non-leaf nodes now: Go recursively left and right
                remove( pThisNode->_pLeftNode, pOtherNode->_pLeftNode );
                remove( pThisNode->_pRightNode, pOtherNode->_pRightNode );
            }
        }
    }
}

Void BinaryTreeNode::restore_node( BinaryTreeNode * pCurrentNode, Nat & arr_index, Nat & leaf_counter,
                                   const BooleanArray& theTree, const BooleanArray& theEnabledCells) {
    //If we are not done with the the tree yet
    if( arr_index < theTree.size() ) {
        //If we are in a non-leaf node then go further
        if( theTree[arr_index] ) {
            pCurrentNode->split();
            //IVAN S. ZAPREEV:
            //NOTE: We assume a correct input, i.e. both children are present
            //NOTE: We increase the arr_index before calling recursion for each
            //      of the subnodes of the tree
            restore_node( pCurrentNode->_pLeftNode, ++arr_index, leaf_counter, theTree, theEnabledCells );
            restore_node( pCurrentNode->_pRightNode, ++arr_index, leaf_counter, theTree, theEnabledCells );
        } else {
            //If we are in a leaf node then chek if it needs to be enabled/disabled
            pCurrentNode->_isEnabled = ValidatedKleenean(theEnabledCells[leaf_counter]);

            leaf_counter++;
        }
    }
}

Void BinaryTreeNode::mince_node(BinaryTreeNode * pCurrentNode, const Nat depth) {
    //If we need to mince further.
    if( depth > 0 ){
        //If the node is present, i.e. the parent is not a disabled node.
        if( pCurrentNode != nullptr ){
            //If the current node is not disabled: enabled (leaf) or
            //indeterminate (non-leaf) then there is a work to do.
            if( ! pCurrentNode->is_disabled() ){
                pCurrentNode->split();

                const Nat remaining_depth = depth - 1;

                mince_node( pCurrentNode->_pLeftNode, remaining_depth );
                mince_node( pCurrentNode->_pRightNode, remaining_depth );
            }
        } else {
            throw std::runtime_error( ARIADNE_PRETTY_FUNCTION );
        }
    }
}

Void BinaryTreeNode::recombine_node(BinaryTreeNode * pCurrentNode) {
    //Just a safety check
    if( pCurrentNode != nullptr ){
        //If it is not a leaf node then it should have both of it's subnodes != nullptr
        if( ! pCurrentNode->is_leaf() ){
            BinaryTreeNode * pLeftNode = pCurrentNode->_pLeftNode;
            BinaryTreeNode * pRightNode = pCurrentNode->_pRightNode;

            //This recursive calls ensure that we do recombination from the bottom up
            recombine_node( pLeftNode );
            recombine_node( pRightNode );

            //Do the recombination for the leaf nodes rooted to pCurrentNode
            if( pLeftNode->is_leaf() && pRightNode->is_leaf() ){
                if( definitely(pLeftNode->_isEnabled == pRightNode->_isEnabled) ){
                    //Make it the leaf node with the derived _isEnabled value
                    pCurrentNode->make_leaf( pLeftNode->_isEnabled );
                }
            }
        }
    } else {
        throw std::runtime_error( ARIADNE_PRETTY_FUNCTION );
    }
}

Nat BinaryTreeNode::depth() const {
    //The depth of the sub-tree rooted to this node
    Nat result;

    if( ! this->is_leaf() ){
        //If the node is not a leaf, compute the depth of the sub-trees and take the maximum + 1
        //Note that, both left and right sub-nodes must exist, by to the way we construct the tree
        result = std::max(this->left_node()->depth(), this->right_node()->depth() ) + 1;
    } else {
        //If the node is a leaf then the depth of the sub-tree is zero
        result = 0;
    }

    return result;
}

SizeType BinaryTreeNode::count_enabled_leaf_nodes( const BinaryTreeNode* pNode ) {
    if(pNode->is_leaf()) {
        return pNode->is_enabled() ? 1u : 0u;
    } else {
        return count_enabled_leaf_nodes(pNode->left_node())
            + count_enabled_leaf_nodes(pNode->right_node());
    }
}

Void BinaryTreeNode::tree_to_binary_words( BinaryWord & tree, BinaryWord & leaves ) const {
    if( is_leaf() ) {
        tree.push_back( false );
        leaves.push_back( definitely( _isEnabled ) );
    } else {
        tree.push_back( true );
        _pLeftNode->tree_to_binary_words( tree, leaves );
        _pRightNode->tree_to_binary_words( tree, leaves );
    }
}

Void BinaryTreeNode::add_enabled( const BinaryTreeNode * pOtherSubTree, const BinaryWord & path ){
    //1. Locate the node, follow the path until it's end or until we meet an enabled node
    BinaryTreeNode * pCurrentSubTree = this;
    Nat position = 0;
    while( ( position < path.size() ) && ! pCurrentSubTree->is_enabled() ){
        //Split the node, if it is not a leaf it will not be changed
        pCurrentSubTree->split();
        //Follow the path step
        pCurrentSubTree = ( path[position] ? pCurrentSubTree->_pRightNode : pCurrentSubTree->_pLeftNode );
        //Go to the next path element
        position ++;
    }
    //2. Now we are in the right node of this tree or we have met an enabled node on the path,
    //   Thus, if this node is not enabled then we go on with adding subTree.
    if( ! pCurrentSubTree->is_enabled() ){
        add_enabled( pCurrentSubTree, pOtherSubTree );
    }
}

Void BinaryTreeNode::add_enabled( BinaryTreeNode* pToTreeRoot, const BinaryTreeNode* pFromTreeRoot ){
    if( pToTreeRoot->is_leaf() ){
        //If we are adding something to a leaf node
        if( pToTreeRoot->is_enabled() ){
            //Do nothing, adding to an enabled leaf node (nothing new can be added)
        } else {
            if( pFromTreeRoot->is_leaf() ){
                //If we are adding something to a disabled leaf node
                if( pFromTreeRoot->is_enabled() ){
                    //Adding an enabled node: Enable the node of pToTreeRoot
                    pToTreeRoot->set_enabled();
                } else {
                    //Do nothing, adding a disabled leaf node to a disabled leaf node
                }
            } else {
                //Adding a subtree pFromTreeRoot to a disabled leaf node pToTreeRoot
                //Using copy constructors here, to avoid memory collisions
                pToTreeRoot->_pLeftNode = new BinaryTreeNode( *pFromTreeRoot->_pLeftNode );
                pToTreeRoot->_pRightNode = new BinaryTreeNode( *pFromTreeRoot->_pRightNode );
                //Set the leaf node as unknown, since we do not know what is below
                pToTreeRoot->set_unknown();
            }
        }
    } else {
        //If we are adding something to a non-leaf node
        if( pFromTreeRoot->is_leaf() ){
            //Adding a leaf to a non-leaf node
            if( pFromTreeRoot->is_enabled() ){
                //Make the enabled leaf node
                pToTreeRoot->make_leaf(true);
            } else {
                //Do nothing, adding a disabled node to a sub tree (nothing new can be added)
            }
        } else {
            //Adding a non-leaf node to a non-leaf node, do recursion
            add_enabled( pToTreeRoot->_pLeftNode, pFromTreeRoot->_pLeftNode );
            add_enabled( pToTreeRoot->_pRightNode, pFromTreeRoot->_pRightNode );
        }
    }
}

Void BinaryTreeNode::add_enabled( BinaryTreeNode* pRootTreeNode, const BinaryWord& path, const Nat position ) {
    if( position < path.size() ) {
        //There is still something to do
        if( pRootTreeNode->is_leaf() ){
            if( pRootTreeNode->is_enabled() ) {
                //This leaf is enabled so adding path will not change anything
                return;
            } else {
                //Split the disabled node
                pRootTreeNode->split();
            }
        }
        //Go left-right depenting on the specified path
        add_enabled( ( path[position] ? pRootTreeNode->_pRightNode : pRootTreeNode->_pLeftNode ), path, position + 1 );
    } else {
        //We are at the destination node
        if( pRootTreeNode->is_leaf() ){
            //Mark the node as enabled
            pRootTreeNode->set_enabled();
        } else {
            //If this is not a leaf node, then make it leaf
            //The leafs below are not interesting any more
            pRootTreeNode->make_leaf(true);
        }
    }
}

BinaryTreeNode * BinaryTreeNode::prepend_tree( const BinaryWord & rootNodePath, BinaryTreeNode * oldRootNode){
    //Create the new binary tree node
    BinaryTreeNode * pRootBinaryTreeNode = new BinaryTreeNode(), * pCurrentBinaryTreeNode = pRootBinaryTreeNode;
    Nat i = 0;
    //Loop until the last path element, because it has to be treated in a different manner
    for( ; i < ( rootNodePath.size() - 1 ) ; i++ ){
        //Split the node
        pCurrentBinaryTreeNode->split();
        //Move to the appropriate subnode
        pCurrentBinaryTreeNode = (rootNodePath[i]) ? pCurrentBinaryTreeNode->_pRightNode : pCurrentBinaryTreeNode->_pLeftNode;
    }
    //Split the node for the last time
    pCurrentBinaryTreeNode->split();
    //Substitute the new primary cell with the one we had before
    if( rootNodePath[i] ){
        delete pCurrentBinaryTreeNode->_pRightNode;
        pCurrentBinaryTreeNode->_pRightNode = oldRootNode;
    } else {
        delete pCurrentBinaryTreeNode->_pLeftNode;
        pCurrentBinaryTreeNode->_pLeftNode = oldRootNode;
    }
    return pRootBinaryTreeNode;
}

Bool BinaryTreeNode::intersect( const BinaryTreeNode * pRootNodeOne, const BinaryTreeNode * pRootNodeTwo ) {
    Bool result = false;

    Bool isNodeOneALeaf = pRootNodeOne->is_leaf();
    Bool isNodeTwoALeaf = pRootNodeTwo->is_leaf();

    if( isNodeOneALeaf && isNodeTwoALeaf ) {
        //If both nodes are leaves, then the trees overlap if only both of the nodes are enabled
        result = pRootNodeOne->is_enabled() && pRootNodeTwo->is_enabled();
    } else {
        if( ! isNodeOneALeaf && isNodeTwoALeaf ){
            //If the second node is a leaf then the trees overlap is it is enabled
            //and the first node has an enabled sub-node
            result = pRootNodeTwo->is_enabled() && pRootNodeOne->has_enabled();
        } else {
            if( isNodeOneALeaf && ! isNodeTwoALeaf ){
                //If the first node is a leaf then the trees overlap is it is enabled
                //and the second node has an enabled sub-node
                result = pRootNodeOne->is_enabled() && pRootNodeTwo->has_enabled();
            } else {
                //Both nodes are non-lead nodes, then the trees overlap if
                //either their left or right branches overlap
                result = intersect( pRootNodeOne->left_node(), pRootNodeTwo->left_node() ) ||
                    intersect( pRootNodeOne->right_node(), pRootNodeTwo->right_node() );
            }
        }
    }

    return result;
}

Bool BinaryTreeNode::subset( const BinaryTreeNode * pRootNodeOne, const BinaryTreeNode * pRootNodeTwo ) {
    Bool result = false;

    Bool isNodeOneALeaf = pRootNodeOne->is_leaf();
    Bool isNodeTwoALeaf = pRootNodeTwo->is_leaf();

    if( isNodeOneALeaf && isNodeTwoALeaf ) {
        //If both nodes are leaves, then pRootNodeOne is a subset of pRootNodeTwo if:
        // 1. both of the nodes are enabled or
        // 2. pRootNodeOne is disabled (represents an empty set)
        result = ( ! pRootNodeOne->is_enabled() ) || pRootNodeTwo->is_enabled();
    } else {
        if( ! isNodeOneALeaf && isNodeTwoALeaf ){
            //If pRootNodeTwo is a leaf then pRootNodeOne is a subset of pRootNodeTwo if:
            // 1. pRootNodeTwo is enabled or
            // 2. pRootNodeOne has no enabled sub-nodes
            result = pRootNodeTwo->is_enabled() || ( ! pRootNodeOne->has_enabled() );
        } else {
            if( isNodeOneALeaf && ! isNodeTwoALeaf ){
                //If pRootNodeOne is a leaf then pRootNodeOne is a subset of pRootNodeTwo if:
                // 1. pRootNodeOne is disabled or
                // 2. all of the pRootNodeTwo's leaf nodes are enabled
                result = ( ! pRootNodeOne->is_enabled() ) || pRootNodeTwo->all_enabled();
            } else {
                //Both nodes are non-lead nodes, then pRootNodeOne is a subset of pRootNodeTwo
                //if sub-trees of pRootNodeOne are subsets of the subtrees of pRootNodeTwo
                result = subset( pRootNodeOne->left_node(), pRootNodeTwo->left_node() )
                         &&
                         subset( pRootNodeOne->right_node(), pRootNodeTwo->right_node() );
            }
        }
    }

    return result;
}

}
