/*
 * =====================================================================================
 *
 *    Description:  Corblivar 2.5D representation wrapper; also encapsulates layout
 *    generation functionality
 *
 *    Copyright (C) 2013 Johann Knechtel, johann.knechtel@ifte.de, www.ifte.de
 *
 *    This file is part of Corblivar.
 *    
 *    Corblivar is free software: you can redistribute it and/or modify it under the terms
 *    of the GNU General Public License as published by the Free Software Foundation,
 *    either version 3 of the License, or (at your option) any later version.
 *    
 *    Corblivar is distributed in the hope that it will be useful, but WITHOUT ANY
 *    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
 *    PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *    
 *    You should have received a copy of the GNU General Public License along with
 *    Corblivar.  If not, see <http://www.gnu.org/licenses/>.
 *
 * =====================================================================================
 */

// own Corblivar header
#include "CorblivarDie.hpp"
// required Corblivar headers
#include "Math.hpp"
#include "CorblivarCore.hpp"

void CorblivarDie::placeCurrentBlock(bool const& alignment_enabled) {
	list<Block const*> relevBlocks;

	// current tuple; only mutable block parameters can be edited
	Block const* cur_block = this->getCurrentBlock();

	// sanity check for previously placed blocks; may occur due to multiple alignment
	// requests in process covering this particular block
	if (cur_block->placed) {
		return;
	}

	// pop relevant blocks from related placement stack
	relevBlocks = this->popRelevantBlocks();

	// horizontal placement
	if (this->getCurrentDirection() == Direction::HORIZONTAL) {

		// first, determine block's y-coordinates
		this->determCurrentBlockCoords(Coordinate::Y, relevBlocks);
		// second, determine block's x-coordinates (depends on y-coord; extended
		// check depends on whether alignment is enabled, i.e., whether some
		// blocks might be shifted)
		this->determCurrentBlockCoords(Coordinate::X, relevBlocks, alignment_enabled);
	}
	// vertical placement
	else {

		// first, determine block's x-coordinates
		this->determCurrentBlockCoords(Coordinate::X, relevBlocks);
		// second, determine block's y-coordinates (depends on x-coord; extended
		// check depends on whether alignment is enabled, i.e., whether some
		// blocks might be shifted)
		this->determCurrentBlockCoords(Coordinate::Y, relevBlocks, alignment_enabled);
	}

	// update placement stacks
	this->updatePlacementStacks(relevBlocks);

	// mark block as placed
	cur_block->placed = true;

	// placement stacks debugging
	if (CorblivarDie::DBG_STACKS) {
		this->debugStacks();
	}
}

void CorblivarDie::debugStacks() {
	Block const* cur_block = this->getCurrentBlock();
	list<Block const*>::iterator iter;

	cout << "DBG_CORB> ";
	cout << "Processed (placed) CBL tuple " << this->getCBL().tupleString(this->pi) << " on die " << this->id + 1 << ": ";
	cout << "LL=(" << cur_block->bb.ll.x << ", " << cur_block->bb.ll.y << "), ";
	cout << "UR=(" << cur_block->bb.ur.x << ", " << cur_block->bb.ur.y << ")" << endl;

	cout << "DBG_CORB>  new stack Hi: ";
	for (iter = this->Hi.begin(); iter != this->Hi.end(); ++iter) {

		if (*iter != this->Hi.back()) {
			cout << (*iter)->id << ", ";
		}
		else {
			cout << (*iter)->id << endl;
			break;
		}
	}

	cout << "DBG_CORB>  new stack Vi: ";
	for (iter = this->Vi.begin(); iter != this->Vi.end(); ++iter) {

		if (*iter != this->Vi.back()) {
			cout << (*iter)->id << ", ";
		}
		else {
			cout << (*iter)->id << endl;
			break;
		}
	}
}

bool CorblivarDie::debugLayout() const {
	bool invalid = false;
	bool flag_inner;

	// check blocks against each other for (faulty) overlaps
	for (Block const* a : this->getCBL().S) {

		flag_inner = false;

		for (Block const* b : this->getCBL().S) {

			// ignore in outer loop checked blocks; start inner loop
			// after current block is self-checked
			if (a->id == b->id) {
				flag_inner = true;
				continue;
			}

			// check for block overlaps
			if (flag_inner && Rect::rectsIntersect(a->bb, b->bb)) {
				cout << "DBG_LAYOUT> Invalid layout! die: " << this->id + 1 << "; overlapping blocks: " << a->id << ", " << b->id << endl;

				invalid = true;
			}
		}
	}

	return invalid;
}

list<Block const*> CorblivarDie::popRelevantBlocks() {
	list<Block const*> ret;
	unsigned blocks_count;

	// horizontal placement; consider stack Hi
	if (this->getCurrentDirection() == Direction::HORIZONTAL) {

		// relevant blocks count depends on the T-junctions to be covered and the
		// current stack itself
		blocks_count = min<unsigned>(this->getJunctions(this->pi) + 1, this->Hi.size());

		// pop relevant blocks from stack into return list
		while (blocks_count > ret.size()) {
			ret.push_back(move(this->Hi.front()));
			this->Hi.pop_front();
		}
	}
	// vertical placement; consider stack Vi
	else {

		// relevant blocks count depends on the T-junctions to be covered and the
		// current stack itself
		blocks_count = min<unsigned>(this->getJunctions(this->pi) + 1, this->Vi.size());

		// pop relevant blocks from stack into return list
		while (blocks_count > ret.size()) {
			ret.push_back(move(this->Vi.front()));
			this->Vi.pop_front();
		}
	}

	return ret;
}

void CorblivarDie::updatePlacementStacks(list<Block const*>& relev_blocks_stack) {
	bool add_to_stack;
	Block const* b;

	// current block
	Block const* cur_block = this->getCurrentBlock();
	// current block's insertion direction
	Direction const& cur_dir = this->getCurrentDirection();

	if (CorblivarDie::DBG_STACKS) {
		cout << "DBG_CORB> Update stacks; current block: " << cur_block->id << "; block's dir: " << static_cast<unsigned>(cur_dir);

		cout << "; relevant blocks: ";
		for (Block const* b : relev_blocks_stack) {
			if (b->id != relev_blocks_stack.back()->id) {
				cout << b->id << ", ";
			}
			else {
				cout << b->id;
			}
		}

		cout << endl;
	}

	// horizontal placement
	if (cur_dir == Direction::HORIZONTAL) {

		// update vertical stack; add cur_block when no other relevant blocks
		// are to its top side
		//
		// note that this check is indepent of overlap in x-direction; this way,
		// we avoid situations where different blocks on both stacks Hi/Vi are
		// current corner block which would result in invalid layouts
		add_to_stack = true;
		for (Block const* b : relev_blocks_stack) {
			if (Rect::rectA_below_rectB(cur_block->bb, b->bb, false)) {
				add_to_stack = false;
				break;
			}
		}
		// actual stack update
		if (add_to_stack) {
			this->Vi.push_front(cur_block);
		}

		// update horizontal stack
		//
		// always consider cur_block since it's one of the right-most blocks now
		this->Hi.push_front(cur_block);
		//
		// add relevant blocks which have no block to the right, simplified by
		// checking against cur_block (only block which can be right of others);
		// by reverse iteration, we retain the (implicit) ordering of blocks
		// popped from stack Hi regarding their insertion order; required for
		// proper stack manipulation
		for (list<Block const*>::reverse_iterator r_iter = relev_blocks_stack.rbegin(); r_iter != relev_blocks_stack.rend(); ++r_iter) {
			b = *r_iter;

			if (!Rect::rectA_leftOf_rectB(b->bb, cur_block->bb, true)) {
				this->Hi.push_front(b);
			}
		}
	}
	// vertical placement
	else {

		// update horizontal stack; add cur_block when no other relevant blocks
		// are to its right side
		//
		// note that this check is indepent of overlap in y-direction; this way,
		// we avoid situations where different blocks on both stacks Hi/Vi are
		// current corner block which would result in invalid layouts
		add_to_stack = true;
		for (Block const* b : relev_blocks_stack) {
			if (Rect::rectA_leftOf_rectB(cur_block->bb, b->bb, false)) {
				add_to_stack = false;
				break;
			}
		}
		// actual stack update
		if (add_to_stack) {
			this->Hi.push_front(cur_block);
		}

		// update vertical stack
		//
		// always consider cur_block since it's one of the top-most blocks now
		this->Vi.push_front(cur_block);
		//
		// add relevant blocks which have no block above, simplified by checking
		// against cur_block (only block which can be above others); by reverse
		// iteration, we retain the (implicit) ordering of blocks popped from
		// stack Vi regarding their insertion order; required for proper stack
		// manipulation
		for (list<Block const*>::reverse_iterator r_iter = relev_blocks_stack.rbegin(); r_iter != relev_blocks_stack.rend(); ++r_iter) {
			b = *r_iter;

			if (!Rect::rectA_below_rectB(b->bb, cur_block->bb, true)) {
				this->Vi.push_front(b);
			}
		}
	}
}

void CorblivarDie::rebuildPlacementStacks(list<Block const*>& relev_blocks_stack) {
	list<Block const*>::iterator iter;
	list<Block const*> stack_backup;
	bool covered;

	// current block
	Block const* cur_block = this->getCurrentBlock();
	// current block's insertion direction
	Direction const& cur_dir = this->getCurrentDirection();

	if (CorblivarDie::DBG_STACKS) {
		cout << "DBG_CORB> Rebuild stacks; current block: " << cur_block->id << ", block's dir: " << static_cast<unsigned>(cur_dir);

		cout << "; relevant blocks: ";
		for (Block const* b : relev_blocks_stack) {
			if (b->id != relev_blocks_stack.back()->id) {
				cout << b->id << ", ";
			}
			else {
				cout << b->id;
			}
		}

		cout << endl;
	}

	// after block shifting, we cannot easily make assumption on the resulting layout
	// and thus how to update the stacks (note that even the insertion direction may
	// be implicitly different after shifting); thus, we simply a) update the stacks
	// w/ all relevant, uncovered blocks (includes previously popped blocks and blocks
	// still on stack), b) push back the current block itself and c) sort the stacks
	// by related dimensions, i.e., block-insertion order; note that this process has
	// to be done for both stacks

	// horizontal stack Hi
	//
	// a) check remaining blocks if they are covered now (by current block)
	for (iter = this->Hi.begin(); iter != this->Hi.end(); ++iter) {
		// block is now covered
		if (Rect::rectA_leftOf_rectB((*iter)->bb, cur_block->bb, true)) {
			// drop block from stack
			iter = this->Hi.erase(iter);
		}
	}

	// a) push back relevant blocks in case they are not covered (by current block);
	// only for related insertion direction
	if (cur_dir == Direction::HORIZONTAL) {

		for (Block const* b : relev_blocks_stack) {
			if (!Rect::rectA_leftOf_rectB(b->bb, cur_block->bb, true)) {
				this->Hi.push_front(b);
			}
		}
	}

	// b) push back the current block itself; only if not covered by any placed block
	covered = false;
	// walk all blocks (implicitly ordered such that placed blocks are first)
	for (unsigned b = 0; b < this->getCBL().size(); b++) {

		// if not yet placed block is reached, the following blocks are also not
		// placed, i.e., not relevant; break loop
		if (!this->getBlock(b)->placed) {
			break;
		}
		else {
			if (Rect::rectA_leftOf_rectB(cur_block->bb, this->getBlock(b)->bb, true)) {
				covered = true;
				break;
			}
		}
	}
	if (!covered) {
		this->Hi.push_back(cur_block);
	}

	// c) sort stack by y-dimension in descending order; retains the proper stack
	// structure for further horizontal block insertion
	this->Hi.sort(
		// lambda expression
		[&](Block const* b1, Block const* b2) {
			// descending order, b1 above b2
			return !Rect::rectA_below_rectB(b1->bb, b2->bb, false);
		}
	);

	// vertical stack Vi
	//
	// a) check remaining blocks if they are covered now (by current block)
	for (iter = this->Vi.begin(); iter != this->Vi.end(); ++iter) {
		// block is now covered
		if (Rect::rectA_below_rectB((*iter)->bb, cur_block->bb, true)) {
			// drop block from stack
			iter = this->Vi.erase(iter);
		}
	}

	// a) push back relevant blocks in case they are not covered (by current block);
	// only for related insertion direction
	if (cur_dir == Direction::VERTICAL) {

		for (Block const* b : relev_blocks_stack) {
			if (!Rect::rectA_below_rectB(b->bb, cur_block->bb, true)) {
				this->Vi.push_front(b);
			}
		}
	}

	// b) push back the current block itself; only if not covered by any placed block
	covered = false;
	// walk all blocks (implicitly ordered such that placed blocks are first)
	for (unsigned b = 0; b < this->getCBL().size(); b++) {

		// if not yet placed block is reached, the following blocks are also not
		// placed, i.e., not relevant; break loop
		if (!this->getBlock(b)->placed) {
			break;
		}
		else {
			if (Rect::rectA_below_rectB(cur_block->bb, this->getBlock(b)->bb, true)) {
				covered = true;
				break;
			}
		}
	}
	if (!covered) {
		this->Vi.push_back(cur_block);
	}

	// c) sort stack by x-dimension in descending order; retains the proper stack
	// structure for further vertical block insertion
	this->Vi.sort(
		// lambda expression
		[&](Block const* b1, Block const* b2) {
			// descending order, b1 right of b2
			return !Rect::rectA_leftOf_rectB(b1->bb, b2->bb, false);
		}
	);

	// sanity check for different corner blocks; may result due to shifting of blocks;
	// we need to try fixing both stacks since we cannot assume which is the correct
	// corner block in this case
	if (this->Hi.front() != this->Vi.front()) {

		// first, try to fix Hi
		//
		// local copy Hi for backup
		stack_backup = this->Hi;

		// try dropping blocks until corner blocks match
		while (this->Hi.front() != this->Vi.front()) {

			if (this->Hi.empty()) {
				break;
			}
			else {
				this->Hi.pop_front();
			}
		}

		// fixing this stack failed, retry w/ Vi
		if (this->Hi.empty()) {

			// restore Hi
			this->Hi = stack_backup;

			// local copy Vi for backup
			stack_backup = this->Vi;

			// try dropping blocks until corner blocks match
			while (this->Hi.front() != this->Vi.front()) {

				if (this->Vi.empty()) {
					break;
				}
				else {
					this->Vi.pop_front();
				}
			}

			// 2nd stack fix failed; this will most likely result in invalid
			// layouts
			if (this->Vi.empty()) {

				// restore Vi
				this->Vi = stack_backup;

				// dbg log for failure
				if (CorblivarDie::DBG_STACKS) {
					cout << "DBG_CORB>  Differing corner blocks on Hi, Vi; related stack fixing failed!" << endl;
				}
			}
		}
	}
}

void CorblivarDie::determCurrentBlockCoords(Coordinate const& coord, list<Block const*> const& relev_blocks_stack, bool const& extended_check) const {
	double x, y;

	// current block
	Block const* cur_block = this->getCurrentBlock();
	// current block's insertion direction
	Direction const& cur_dir = this->getCurrentDirection();

	// update x-coordinates
	if (coord == Coordinate::X) {

		// for vertical block insertion; x-coordinate is first coordinate, thus
		// not dependent on y-coordinate
		if (cur_dir == Direction::VERTICAL) {

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
				x = -1;
				for (Block const* b : relev_blocks_stack) {
					if (x == -1) {
						x = b->bb.ll.x;
					}
					else {
						x = min(x, b->bb.ll.x);
					}
				}
			}
		}
		// for horizontal insertion; x-coordinate is second coordinate, thus
		// dependent on y-coordinate
		else {
			x = 0;

			// extended check in case of block alignment is required: for
			// shifted blocks, and generally for all blocks to be placed w/
			// alignment enabled (relying on possibly shifted blocks given on
			// the placement stacks), we cannot assume that the current stacks
			// block are the relevant boundaries; thus, we need to check
			// against all previously placed blocks
			//
			if (extended_check) {

				// walk all blocks (implicitly ordered such that placed
				// blocks are first)
				for (unsigned b = 0; b < this->getCBL().size(); b++) {

					// if not yet placed block is reached, the
					// following blocks are also not placed, i.e., not
					// relevant; break loop
					if (!this->getBlock(b)->placed) {
						break;
					}
					else {
						// only consider blocks which intersect in y-direction
						if (Rect::rectsIntersectVertical(cur_block->bb, this->getBlock(b)->bb)) {
							// determine right front
							x = max(x, this->getBlock(b)->bb.ur.x);
						}
					}
				}
			}
			// non shifted block / trivial case w/o alignment; simply check
			// against blocks to be covered
			//
			else {
				// determine x-coordinate for lower left corner of current block, consider
				// right front of blocks to be covered
				for (Block const* b : relev_blocks_stack) {
					// only consider blocks which intersect in y-direction
					if (Rect::rectsIntersectVertical(cur_block->bb, b->bb)) {
						// determine right front
						x = max(x, b->bb.ur.x);
					}
				}
			}
		}

		// update block's x-coordinates
		cur_block->bb.ll.x = x;
		cur_block->bb.ur.x = cur_block->bb.w + x;
	}

	// update y-coordinates
	else {

		// for horizontal block insertion; y-coordinate is first coordinate, thus
		// not dependent on x-coordinate
		if (cur_dir == Direction::HORIZONTAL) {

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
				y = -1;
				for (Block const* b : relev_blocks_stack) {
					if (y == -1) {
						y = b->bb.ll.y;
					}
					else {
						y = min(y, b->bb.ll.y);
					}
				}
			}
		}
		// for vertical insertion; y-coordinate is second coordinate, thus
		// dependent on x-coordinate
		else {
			y = 0;

			// extended check in case of block alignment is required: for
			// shifted blocks, and generally for all blocks to be placed w/
			// alignment enabled (relying on possibly shifted blocks given on
			// the placement stacks), we cannot assume that the current stacks
			// block are the relevant boundaries; thus, we need to check
			// against all previously placed blocks
			//
			if (extended_check) {

				// walk all blocks (implicitly ordered such that placed
				// blocks are first)
				for (unsigned b = 0; b < this->getCBL().size(); b++) {

					// if not yet placed block is reached, the
					// following blocks are also not placed, i.e., not
					// relevant; break loop
					if (!this->getBlock(b)->placed) {
						break;
					}
					else {
						// only consider blocks which intersect in x-direction
						if (Rect::rectsIntersectHorizontal(cur_block->bb, this->getBlock(b)->bb)) {
							// determine upper front
							y = max(y, this->getBlock(b)->bb.ur.y);
						}
					}
				}
			}
			// non shifted block / trivial case w/o alignment; simply check
			// against blocks to be covered
			//
			else {
				// determine y-coordinate for lower left corner of current block, consider
				// upper front of blocks to be covered
				for (Block const* b : relev_blocks_stack) {
					// only consider blocks which intersect in x-direction
					if (Rect::rectsIntersectHorizontal(cur_block->bb, b->bb)) {
						// determine upper front
						y = max(y, b->bb.ur.y);
					}
				}
			}
		}

		// update block's y-coordinates
		cur_block->bb.ll.y = y;
		cur_block->bb.ur.y = cur_block->bb.h + y;
	}
}

bool CorblivarDie::shiftCurrentBlock(Direction const& dir, CorblivarAlignmentReq const* req, bool const& dry_run) {
	Block const* reference_block;
	double overlap_offset_x, overlap_offset_y;
	double shift_x, shift_y;
	double range_x, range_y;
	double neighbor_x, neighbor_y;
	bool neighbor_x_found, neighbor_y_found;
	bool shifted;

	// block to shift is current block
	Block const* shift_block = this->getCurrentBlock();

	// flag for monitoring shifting itself
	shifted = false;

	// first, determine reference block
	if (shift_block->id == req->s_i->id) {
		reference_block = req->s_j;
	}
	else {
		reference_block = req->s_i;
	}

	// second, perform actual shift
	//
	// shift in horizontal direction
	if (dir == Direction::HORIZONTAL) {

		overlap_offset_x = range_x = 0.0;

		// for shifting range, we need to ensure that the blocks have an overlap in
		// x-direction >= range
		if (req->range_x()) {

			// limit desired range, i.e., consider current block dimensions
			range_x = min(shift_block->bb.w, reference_block->bb.w);
			range_x = min(range_x, req->alignment_x);

			// determine inherent overlap; for non-overlapping blocks this
			// will be < 0
			overlap_offset_x = shift_block->bb.ur.x - reference_block->bb.ll.x;
		}

		// for shifting offset, we need to ensure that the blocks have an exact
		// offset w.r.t. their lower left corners
		else if (req->offset_x()) {

			// desired shifting range, i.e., fixed offset
			range_x = req->alignment_x;

			// determine inherent offset
			overlap_offset_x = shift_block->bb.ll.x - reference_block->bb.ll.x;
		}

		// try to shift block to the right
		if (overlap_offset_x < range_x) {

			// determine required additional shifting amount
			shift_x = range_x - overlap_offset_x;

			// check previously placed blocks; they might limit
			// shifting ranges in cases where the block to be shifted
			// is implicitly packed w/in the corner-blocks front
			//
			neighbor_x = 0.0;
			neighbor_x_found =  false;
			// walk all blocks (implicitly ordered such that placed blocks are first)
			for (unsigned b = 0; b < this->getCBL().size(); b++) {

				if (this->getBlock(b)->id == shift_block->id) {
					break;
				}
				else {
					// for shifting to the right, check for
					// right neigbhors
					if (Rect::rectA_leftOf_rectB(shift_block->bb, this->getBlock(b)->bb, true)) {

						if (!neighbor_x_found) {
							neighbor_x_found = true;
							neighbor_x = this->getBlock(b)->bb.ll.x;
						}
						else {
							neighbor_x = min(neighbor_x, this->getBlock(b)->bb.ll.x);
						}
					}
				}
			}

			// some relevant neighbor found, i.e., shifting is
			// restricted in order to avoid overlaps
			if (neighbor_x_found) {

				shift_x = min(shift_x, neighbor_x - shift_block->bb.ur.x);
			}

			// memorize that shifting is possible (and conducted)
			shifted = (shift_x > 0.0);

			// dbg logging
			if (CorblivarCore::DBG_ALIGNMENT_REQ) {
				cout << "DBG_ALIGNMENT>      Shift block " << shift_block->id;
				cout << " in x-direction by " << range_x - overlap_offset_x;
				cout << " (feasible: " << shift_x << ")";

				if (dry_run) {
					cout << " is required, but not performed now (dry run)";
				}

				cout << endl;
			}

			// apply shifting
			if (!dry_run) {
				shift_block->bb.ll.x += shift_x;
				shift_block->bb.ur.x += shift_x;
			}
		}

		// sanity check for impossible shifting, i.e., other block should be shifted
		else if (CorblivarCore::DBG_ALIGNMENT_REQ) {

			if (req->range_x()) {
				overlap_offset_x = reference_block->bb.ur.x - shift_block->bb.ll.x;
			}
			else if (req->offset_x()) {
				overlap_offset_x = reference_block->bb.ll.x - shift_block->bb.ll.x;
			}

			if (overlap_offset_x < range_x) {
				cout << "DBG_ALIGNMENT>      Shifting block " << shift_block->id << " in x-direction not effective; other block would need to be shifted!" << endl;
			}
		}
	}

	// shift in vertical direction
	else {
		overlap_offset_y = range_y = 0.0;

		// for shifting range, we need to ensure that the blocks have an overlap in
		// y-direction >= range
		if (req->range_y()) {

			// limit desired range, i.e., consider current block dimensions
			range_y = min(shift_block->bb.h, reference_block->bb.h);
			range_y = min(range_y, req->alignment_y);

			// determine inherent overlap; for non-overlapping blocks this
			// will be < 0
			overlap_offset_y = shift_block->bb.ur.y - reference_block->bb.ll.y;
		}

		// for shifting offset, we need to ensure that the blocks have an exact
		// offset w.r.t. their lower left corners
		else if (req->offset_y()) {

			// desired shifting range, i.e., fixed offset
			range_y = req->alignment_y;

			// determine inherent offset
			overlap_offset_y = shift_block->bb.ll.y - reference_block->bb.ll.y;
		}

		// try to shift block upwards
		if (overlap_offset_y < range_y) {

			// determine required additional shifting amount
			shift_y = range_y - overlap_offset_y;

			// check previously placed blocks; they might limit
			// shifting ranges in cases where the block to be shifted
			// is implicitly packed w/in the corner-blocks front
			//
			neighbor_y = 0.0;
			neighbor_y_found =  false;
			// walk all blocks (implicitly ordered such that placed blocks are first)
			for (unsigned b = 0; b < this->getCBL().size(); b++) {

				if (this->getBlock(b)->id == shift_block->id) {
					break;
				}
				else {
					// for shifting to the top, check for
					// neigbhors above
					if (Rect::rectA_below_rectB(shift_block->bb, this->getBlock(b)->bb, true)) {

						if (!neighbor_y_found) {
							neighbor_y_found = true;
							neighbor_y = this->getBlock(b)->bb.ll.y;
						}
						else {
							neighbor_y = min(neighbor_y, this->getBlock(b)->bb.ll.y);
						}
					}
				}
			}

			// some relevant neighbor found, i.e., shifting is
			// restricted in order to avoid overlaps
			if (neighbor_y_found) {

				shift_y = min(shift_y, neighbor_y - shift_block->bb.ur.y);
			}

			// memorize that shifting is possible (and conducted)
			shifted = (shift_y > 0.0);

			// dbg logging
			if (CorblivarCore::DBG_ALIGNMENT_REQ) {
				cout << "DBG_ALIGNMENT>      Shift block " << shift_block->id;
				cout << " in y-direction by " << range_y - overlap_offset_y;
				cout << " (feasible: " << shift_y << ")";

				if (dry_run) {
					cout << " is required, but not performed now (dry run)";
				}

				cout << endl;
			}

			// apply shifting
			if (!dry_run) {
				shift_block->bb.ll.y += shift_y;
				shift_block->bb.ur.y += shift_y;
			}
		}

		// sanity check for impossible shifting, i.e., other block should be shifted
		else if (CorblivarCore::DBG_ALIGNMENT_REQ) {

			if (req->range_y()) {
				overlap_offset_y = reference_block->bb.ur.y - shift_block->bb.ll.y;
			}
			else if (req->offset_y()) {
				overlap_offset_y = reference_block->bb.ll.y - shift_block->bb.ll.y;
			}

			if (overlap_offset_y < range_y) {
				cout << "DBG_ALIGNMENT>      Shifting block " << shift_block->id << " in y-direction not effective; other block would need to be shifted!" << endl;
			}
		}
	}

	return shifted;
}

// note that packing may undermine alignment requests; to avoid this, we call
// FloorPlanner::determCostAlignment before packing but after layout generation
// (FloorPlanner::determCostAlignment does annotate alignment success / failure to the
// blocks themselves)
void CorblivarDie::performPacking(Direction const& dir) {
	list<Block const*> blocks;
	list<Block const*>::iterator i1;
	list<Block const*>::reverse_iterator i2;
	Block const* block;
	Block const* neighbor;
	double x, y;
	vector<Rect> blocks_checked;
	double range_checked;
	Rect cur_intersect, cur_prev_intersect;

	// store blocks in separate list, for subsequent sorting
	blocks.insert(blocks.begin(), this->getCBL().S.begin(), this->getCBL().S.end());

	if (dir == Direction::HORIZONTAL) {

		// sort blocks by lower-left x-coordinate (ascending order)
		blocks.sort(
			// lambda expression
			[&](Block const* b1, Block const* b2){
				return (b1->bb.ll.x < b2->bb.ll.x)
					// for blocks on same column, sort additionally by
					// their width, putting the bigger back in the
					// list, thus consider them first during
					// subsequent checking for adjacent blocks
					// (reverse list traversal)
					|| ((b1->bb.ll.x == b2->bb.ll.x) && (b1->bb.ur.x < b2->bb.ur.x))
					// for blocks on same column and w/ same width,
					// order additionally by y-coordinate to ease list
					// traversal (relevant blocks are adjacent tuples
					// in list)
					|| ((b1->bb.ll.x == b2->bb.ll.x) && (b1->bb.ur.x == b2->bb.ur.x) && (b1->bb.ll.y < b2->bb.ll.y))
					;
			}
		);

		// for each block, check the adjacent blocks and perform packing by
		// considering the neighbors' nearest right front
		for (i1 = blocks.begin(); i1 != blocks.end(); ++i1) {
			block= *i1;

			// skip blocks at left boundary, they are implicitly packed
			if (block->bb.ll.x == 0.0) {
				continue;
			}

			// also skip blocks which are successfully aligned; this way, they
			// keep their position and alignment is not broken
			if (block->alignment == Block::AlignmentStatus::SUCCESS) {
				continue;
			}

			// init packed coordinate
			x = 0.0;
			// init search stop flag
			blocks_checked.clear();
			range_checked = 0.0;

			// check against other blocks; walk in reverse order since we only need to
			// consider the blocks to the left
			for (i2 = list<Block const*>::reverse_iterator(i1); i2 != blocks.rend(); ++i2) {
				neighbor = *i2;

				if (Rect::rectA_leftOf_rectB(neighbor->bb, block->bb, true)) {

					// determine the packed coordinate by considering
					// the neigbors nearest right front
					x = max(x, neighbor->bb.ur.x);

					// current blocks' intersection
					cur_intersect = Rect::determineIntersection(neighbor->bb, block->bb);

					// initially, consider the full intersection range
					// as relevant
					range_checked += cur_intersect.h;

					// check the intersection w/ previous
					// intersections in order to avoid redundant
					// considerations of ranges
					for (Rect const& r : blocks_checked) {

						cur_prev_intersect = Rect::determineIntersection(cur_intersect, r);
						// drop the additionally, redundant
						// covered range
						if (cur_prev_intersect.h > 0.0) {
							range_checked -= cur_prev_intersect.h;
						}
					}

					// memorize the actually covered range of the
					// block front; use a set of rectangles
					blocks_checked.push_back(cur_intersect);

					// in case the full block front was checked, we
					// can stop checking other blocks
					if (Math::doubleComp(block->bb.h, range_checked)) {
						break;
					}
				}
			}

			// update coordinate on block itself, effects the final layout as well as
			// the currently walked list (which is required for step-wise packing from
			// left to right boundary)
			block->bb.ll.x = x;
			block->bb.ur.x = block->bb.w + x;
		}
	}

	// vertical direction
	else {

		// sort blocks by lower-left y-coordinate (ascending order)
		blocks.sort(
			// lambda expression
			[&](Block const* b1, Block const* b2){
				return (b1->bb.ll.y < b2->bb.ll.y)
					// for blocks on same row, sort additionally by
					// their height, putting the bigger back in the
					// list, thus consider them first during
					// subsequent checking for adjacent blocks
					// (reverse list traversal)
					|| ((b1->bb.ll.y == b2->bb.ll.y) && (b1->bb.ur.y < b2->bb.ur.y))
					// for blocks on same row and w/ same height,
					// order additionally by x-coordinate to ease list
					// traversal (relevant blocks are adjacent tuples
					// in list)
					|| ((b1->bb.ll.y == b2->bb.ll.y) && (b1->bb.ur.y == b2->bb.ur.y) && (b1->bb.ll.x < b2->bb.ll.x))
					;
			}
		);

		// for each block, check the adjacent blocks and perform packing by
		// considering the neighbors' nearest upper front
		for (i1 = blocks.begin(); i1 != blocks.end(); ++i1) {
			block= *i1;

			// skip blocks at bottom boundary, they are implicitly packed
			if (block->bb.ll.y == 0.0) {
				continue;
			}

			// also skip blocks which are successfully aligned; this way, they
			// keep their position and alignment is not broken
			if (block->alignment == Block::AlignmentStatus::SUCCESS) {
				continue;
			}

			// init packed coordinate
			y = 0.0;
			// init search stop flag
			blocks_checked.clear();
			range_checked = 0.0;

			// check against other blocks; walk in reverse order since we only need to
			// consider the blocks below
			for (i2 = list<Block const*>::reverse_iterator(i1); i2 != blocks.rend(); ++i2) {
				neighbor = *i2;

				if (Rect::rectA_below_rectB(neighbor->bb, block->bb, true)) {

					// determine the packed coordinate by considering
					// the neigbors nearest right front
					y = max(y, neighbor->bb.ur.y);

					// current blocks' intersection
					cur_intersect = Rect::determineIntersection(neighbor->bb, block->bb);

					// initially, consider the full intersection range
					// as relevant
					range_checked += cur_intersect.w;

					// check the intersection w/ previous
					// intersections in order to avoid redundant
					// considerations of ranges
					for (Rect const& r : blocks_checked) {

						cur_prev_intersect = Rect::determineIntersection(cur_intersect, r);
						// drop the additionally, redundant
						// covered range
						if (cur_prev_intersect.w > 0.0) {
							range_checked -= cur_prev_intersect.w;
						}
					}

					// memorize the actually covered range of the
					// block front; use a set of rectangles
					blocks_checked.push_back(cur_intersect);

					// in case the full block front was checked, we
					// can stop checking other blocks
					if (Math::doubleComp(block->bb.w, range_checked)) {
						break;
					}
				}
			}

			// update coordinate on block itself, effects the final layout as
			// well as the currently walked list (which is required for
			// step-wise packing from bottom to top boundary)
			block->bb.ll.y = y;
			block->bb.ur.y = block->bb.h + y;
		}
	}
}
