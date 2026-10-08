/*
 * Copyright (C) 2000, Imperial College
 *
 * This file is part of the Imperial College Exact Real Arithmetic Library.
 * See the copyright notice included in the distribution for conditions
 * of use.
 */

#include <stdio.h>
#include "real.h"
#include "real-impl.h"

/*
 * For debugging purposes, we assign a unique identifier to each object in
 * the heap. This identifier is invariant under copying in garbage collection.
 * Also the names of the nodes in daVinci are strings formed from the 
 * assigned nodeId.
 */
int nodeId = 1;

/*
 * For debugging purposes, nodes are assigned id's which (unlike the address
 * of the node) is invariant under garbage collection.
 *
 * This file allocates ids and provides a function for mapping ids to
 * heap addresses.
 */
void
newNodeId(Generic *node)
{
	void addHashTableEntry(Generic *);

	node->tag.nodeId = nodeId++;
	addHashTableEntry(node);
}
 
/*
 * We use a naive hash table to map node identifiers to heap cells.
 */
typedef struct HTE {
	struct HTE *next;
	Generic *node;
} HashTableEntry;

/*
 * The following must be an integer power of 2
 */
#define HASH_TABLE_SIZE 512
#define HASH_TABLE_MASK (HASH_TABLE_SIZE - 1)

static HashTableEntry *hashTable[HASH_TABLE_SIZE] = {NULL};

void
addHashTableEntry(Generic *node)
{
	HashTableEntry *new;
	
	if ((new = (HashTableEntry *) malloc(sizeof(HashTableEntry))) == NULL)
		Error(FATAL, E_INT, "addHashTableEntry", "malloc failed");

	new->node = node;
	new->next = hashTable[node->tag.nodeId & HASH_TABLE_MASK];
	hashTable[node->tag.nodeId & HASH_TABLE_MASK] = new;
}

Generic *
mapNodeIdToHeapCell(int nodeId)
{
	HashTableEntry *p;

	p = hashTable[nodeId & HASH_TABLE_MASK];

	while (p != NULL) {
		if (p->node->tag.nodeId == nodeId)
			return p->node;
		else
			p = p->next;
	}
	Error(FATAL, E_INT, "mapNodeIdToHeapCell", "failed to find nodeId %d", nodeId);
}
