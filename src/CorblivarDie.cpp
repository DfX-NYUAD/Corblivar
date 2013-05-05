/*
 * =====================================================================================
 *
 *    Description:  Corblivar 2.5D representation wrapper; also encapsulates layout
 *    			generation functionality
 *
 *         Author:  Johann Knechtel, johann.knechtel@ifte.de
 *        Company:  Institute of Electromechanical and Electronic Design, www.ifte.de
 *
 * =====================================================================================
 */

// own Corblivar header
#include "CorblivarDie.hpp"

Block const* CorblivarDie::placeCurrentBlock(bool const& dbgStack) {
	vector<Block const*> relevBlocks;
	unsigned relevBlocksCount, b;
	double x, y;
	bool add_to_stack;
	list<Block const*> blocks_add_to_stack;

	// sanity check for empty dies
	if (this->CBL.empty()) {
		this->done = true;
		return nullptr;
	}

	// current tuple; only mutable block parameters can be edited
	Block const* cur_block = this->getBlock(this->pi);
	Direction const cur_dir = this->getDirection(this->pi);
	unsigned const cur_juncts = this->getJunctions(this->pi);

	// assign layer to block
	cur_block->layer = this->id;

	// horizontal placement
	if (cur_dir == Direction::HORIZONTAL) {
		// pop relevant blocks from stack
		relevBlocksCount = min(cur_juncts + 1, this->Hi.size());
		relevBlocks.reserve(relevBlocksCount);
		while (relevBlocksCount > relevBlocks.size()) {
			relevBlocks.push_back(move(this->Hi.top()));
			this->Hi.pop();
		}

		// determine y-coordinate for lower left corner of current block
		//
		// all rows are to be covered (according to T-juncts), thus place the
		// block at the bottom die boundary
		if (this->Hi.empty()) {
			y = 0;
		}
		// only some rows are to be covered, thus determine the lower front of
		// the related blocks
		else {
			y = relevBlocks[0]->bb.ll.y;
			for (b = 1; b < relevBlocks.size(); b++) {
				y = min(y, relevBlocks[b]->bb.ll.y);
			}
		}

		// update block's y-coordinates
		cur_block->bb.ll.y = y;
		cur_block->bb.ur.y = cur_block->bb.h + y;

		// determine x-coordinate for lower left corner of current block, consider
		// right front of blocks to be covered
		x = 0;
		for (Block const* b : relevBlocks) {
			// only consider blocks which intersect in y-direction
			if (Rect::rectsIntersectVertical(cur_block->bb, b->bb)) {
				// determine right front
				x = max(x, b->bb.ur.x);
			}
		}

		// update block's x-coordinates
		cur_block->bb.ll.x = x;
		cur_block->bb.ur.x = cur_block->bb.w + x;

		// update vertical stack; add cur_block when no other relevant blocks
		// are to its top side, indepent of overlap in x-direction
		add_to_stack = true;
		for (Block const* b : relevBlocks) {
			if (Rect::rectA_below_rectB(cur_block->bb, b->bb, false)) {
				add_to_stack = false;
			}
		}

		if (add_to_stack) {
			this->Vi.push(cur_block);
		}

		// update horizontal stack; add relevant blocks which have no block to the right,
		// can be simplified by checking against cur_block (only new block which
		// can be possibly right of others)
		for (Block const* b : relevBlocks) {
			if (!Rect::rectA_leftOf_rectB(b->bb, cur_block->bb, true)) {
				// prepending blocks to list retains the (implicit)
				// ordering of blocks popped from stack Hi regarding their
				// insertion order; required for proper stack manipulation
				blocks_add_to_stack.push_front(b);
			}
		}
		// always consider cur_block as it's current corner block, i.e., right to others
		blocks_add_to_stack.push_front(cur_block);

		for (Block const* b : blocks_add_to_stack) {
			this->Hi.push(b);
		}
	}
	// vertical placement
	else {
		// pop relevant blocks from stack
		relevBlocksCount = min(cur_juncts + 1, this->Vi.size());
		relevBlocks.reserve(relevBlocksCount);
		while (relevBlocksCount > relevBlocks.size()) {
			relevBlocks.push_back(move(this->Vi.top()));
			this->Vi.pop();
		}

		// determine x-coordinate for lower left corner of current block
		//
		// all columns are to be covered (according to T-juncts), thus place the
		// block at the left die boundary
		if (this->Vi.empty()) {
			x = 0;
		}
		// only some columns are to be covered, thus determine the left front of
		// the related blocks
		else {
			x = relevBlocks[0]->bb.ll.x;
			for (b = 1; b < relevBlocks.size(); b++) {
				x = min(x, relevBlocks[b]->bb.ll.x);
			}
		}

		// update block's x-coordinates
		cur_block->bb.ll.x = x;
		cur_block->bb.ur.x = cur_block->bb.w + x;

		// determine y-coordinate for lower left corner of current block, consider
		// upper front of blocks to be covered
		y = 0;
		for (Block const* b : relevBlocks) {
			// only consider blocks which intersect in x-direction
			if (Rect::rectsIntersectHorizontal(cur_block->bb, b->bb)) {
				// determine upper front
				y = max(y, b->bb.ur.y);
			}
		}

		// update block's y-coordinates
		cur_block->bb.ll.y = y;
		cur_block->bb.ur.y = cur_block->bb.h + y;

		// update horizontal stack; add cur_block when no other relevant blocks
		// are to its right side, indepent of overlap in y-direction
		add_to_stack = true;
		for (Block const* b : relevBlocks) {
			if (Rect::rectA_leftOf_rectB(cur_block->bb, b->bb, false)) {
				add_to_stack = false;
			}
		}

		if (add_to_stack) {
			this->Hi.push(cur_block);
		}

		// update vertical stack; add relevant blocks which have no block above,
		// can be simplified by checking against cur_block (only new block which
		// can be possibly above others)
		for (Block const* b : relevBlocks) {
			if (!Rect::rectA_below_rectB(b->bb, cur_block->bb, true)) {
				// prepending blocks to list retains the (implicit)
				// ordering of blocks popped from stack Vi regarding their
				// insertion order; required for proper stack manipulation
				blocks_add_to_stack.push_front(b);
			}
		}
		// always consider cur_block as it's current corner block, i.e., above others
		blocks_add_to_stack.push_front(cur_block);

		for (Block const* b : blocks_add_to_stack) {
			this->Vi.push(b);
		}
	}

	if (dbgStack) {
		cout << "DBG_CORB> ";
		cout << "Processed (placed) CBL tuple " << this->CBL.tupleString(this->pi) << " on die " << this->id << ": ";
		cout << "LL=(" << cur_block->bb.ll.x << ", " << cur_block->bb.ll.y << "), ";
		cout << "UR=(" << cur_block->bb.ur.x << ", " << cur_block->bb.ur.y << ")" << endl;

		stack<Block const*> tmp_Hi = this->Hi;
		cout << "DBG_CORB> stack Hi: ";
		while (!tmp_Hi.empty()) {
			if (tmp_Hi.size() > 1) {
				cout << tmp_Hi.top()->id << ", ";
			}
			else {
				cout << tmp_Hi.top()->id << endl;
			}
			tmp_Hi.pop();
		}

		stack<Block const*> tmp_Vi = this->Vi;
		cout << "DBG_CORB> stack Vi: ";
		while (!tmp_Vi.empty()) {
			if (tmp_Vi.size() > 1) {
				cout << tmp_Vi.top()->id << ", ";
			}
			else {
				cout << tmp_Vi.top()->id << endl;
			}
			tmp_Vi.pop();
		}
	}

	// increment progress pointer, consider next tuple (block) or mark die as done
	if (this->pi == (CBL.size() - 1)) {
		this->done = true;
	}
	else {
		this->pi++;
	}

	return cur_block;
}

// TODO consider alignment requirements; perform packing such that alignment is not
// undermined
void CorblivarDie::performPacking(Direction const& dir) {
	list<Block const*> blocks;
	double x, y;

	// sanity check for empty dies
	if (this->CBL.empty()) {
		return;
	}

	// store blocks in separate list, for subsequent sorting
	for (Block const* block : this->CBL.S) {
		blocks.push_back(block);
	}

	// sort blocks by lower-left corner coordinate, in ascending order
	if (dir == Direction::HORIZONTAL) {
		blocks.sort(
			// lambda expression for sorting by x-coordinate
			[&](Block const* b1, Block const* b2){
				return b1->bb.ll.x < b2->bb.ll.x;
			}
		);
	}
	else {
		blocks.sort(
			// lambda expression for sorting by y-coordinate
			[&](Block const* b1, Block const* b2){
				return b1->bb.ll.y < b2->bb.ll.y;
			}
		);
	}

	// for each block, perform packing by considering the gap b/w itself and the
	// left/lower adjacent block
	for (Block const* block : blocks) {

		// skip blocks at left/bottom boundary
		if (dir == Direction::HORIZONTAL) {
			if (block->bb.ll.x == 0.0) {
				continue;
			}
		}
		else {
			if (block->bb.ll.y == 0.0) {
				continue;
			}
		}

		// for remaining blocks, consider other blocks as neighbor block
		x = y = 0.0;
		for (Block const* neighbor : blocks) {

			// only consider blocks left/below the block itself, i.e., stop
			// walk on sorted list when we reach the block itself
			if (neighbor->id == block->id) {
				break;
			}

			// determine packed coordinates by considering the nearest,
			// adjacent block and its upper right corner as new lower left
			// corner for the block of interest
			if (dir == Direction::HORIZONTAL) {
				if (Rect::rectA_leftOf_rectB(neighbor->bb, block->bb, true)) {
					x = max(x, neighbor->bb.ur.x);
				}
			}
			else {
				if (Rect::rectA_below_rectB(neighbor->bb, block->bb, true)) {
					y = max(y, neighbor->bb.ur.y);
				}
			}
		}

		// update coordinate on block itself, effects the final layout as well as
		// the currently walked list (which is required for step-wise packing from
		// left/bottom to right/top boundary)
		if (dir == Direction::HORIZONTAL) {
			block->bb.ll.x = x;
			block->bb.ur.x = block->bb.w + x;
		}
		else {
			block->bb.ll.y = y;
			block->bb.ur.y = block->bb.h + y;
		}
	}
}
