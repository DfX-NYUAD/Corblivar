/*
 * =====================================================================================
 *
 *    Description:  Corblivar core (data structures, layout operations)
 *
 *    Copyright (C) 2013-2016 Johann Knechtel, johann aett jknechtel dot de
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
#include "CorblivarCore.hpp"
// required Corblivar headers
#include "Math.hpp"
#include "Block.hpp"

// memory allocation
constexpr int CorblivarCore::SORT_CBLS_BY_BLOCKS_SIZE;

void CorblivarCore::initCorblivarRandomly(bool const& log, int const& layers, std::vector<Block> const& blocks, bool const& power_aware_assignment) {
	Direction cur_dir;
	int die, cur_t, cur_layer;
	double blocks_area_per_layer, cur_blocks_area;
	std::vector<Block> blocks_copy;
	Block const* cur_block;

	if (log) {
		std::cout << "Corblivar> ";
		std::cout << "Initializing Corblivar data for corb on " << layers << " layers; ";
		if (power_aware_assignment) {
			std::cout << "w/ power-aware block handling..." << std::endl;
		}
		else {
			std::cout << "w/o power-aware block handling..." << std::endl;
		}
	}

	// local copy; used for blocks order
	blocks_copy = blocks;

	// prepare power-aware assignment
	if (power_aware_assignment) {

		// init vars
		blocks_area_per_layer = cur_blocks_area = 0.0;
		cur_layer = 0;

		// sort blocks by power density; use local (mutable) copy of blocks
		std::sort(blocks_copy.begin(), blocks_copy.end(),
			// lambda expression
			[](Block const& b1, Block const& b2) {
				// std::sort requires a _strict_ ordering, thus we have to make sure that same elements returns false
				// http://stackoverflow.com/a/1541909
				// also helps to make comparison short-cutting it early
				return (b1.numerical_id != b2.numerical_id) && (b1.power_density() < b2.power_density());
			}
		    );

		// determine blocks / die area ratio for balanced assignment
		blocks_area_per_layer = 0.0;
		for (Block const& cur_block : blocks) {
			blocks_area_per_layer += cur_block.bb.area;
		}
		blocks_area_per_layer /= layers;
	}

	// assign each block to one die, generate L and T as well; consider local, sorted
	// copy of blocks container
	for (Block& cur_block_copy : blocks_copy) {

		// determine related block from original blocks container
		cur_block = Block::findBlock(cur_block_copy.id, blocks);

		// for power-aware assignment, fill layers w/ (sorted) blocks until the
		// dies are evenly occupied
		if (power_aware_assignment) {

			cur_blocks_area += cur_block->bb.area;

			if (cur_blocks_area > blocks_area_per_layer) {
				cur_layer++;
				cur_blocks_area = 0.0;
			}

			// sanity check to limit die
			die = std::min(cur_layer, layers - 1);
		}
		else {
			// consider random die
			die = Math::randI(0, layers);
		}

		// memorize layer in block itself
		cur_block->layer = die;

		// generate direction L
		if (Math::randB()) {
			cur_dir = Direction::HORIZONTAL;
		}
		else {
			cur_dir = Direction::VERTICAL;
		}
		// init T-junction to be overlapped as zero, results in initial layout to
		// be placed ``somewhat diagonally'' into outline
		cur_t = 0;

		// store into separate CBL sequences
		this->dies[die].CBL.S.push_back(std::move(cur_block));
		this->dies[die].CBL.L.push_back(std::move(cur_dir));
		this->dies[die].CBL.T.push_back(std::move(cur_t));
	}

	if (CorblivarCore::DBG) {
		for (CorblivarDie const& die : this->dies) {
			std::cout << "DBG_CORE> ";
			std::cout << "Init CBL tuples for die " << die.id + 1 << "; " << die.CBL.size() << " tuples:" << std::endl;
			std::cout << die.CBL.CBLString() << std::endl;
			std::cout << "DBG_CORE> ";
			std::cout << std::endl;
		}
	}

	if (log) {
		std::cout << "Corblivar> ";
		std::cout << "Done" << std::endl << std::endl;
	}
}

bool CorblivarCore::generateLayout(bool const& perform_alignment) {
	Block const* cur_block;
	Block const* other_block;
	std::vector<CorblivarAlignmentReq const*> cur_block_alignment_reqs;
	CorblivarAlignmentReq const* req_processed;

	if (CorblivarCore::DBG) {
		std::cout << "DBG_CORE> ";
		std::cout << "Performing layout generation..." << std::endl;
	}

	// init die pointer
	this->p = &this->dies[0];

	// reset die data, i.e., layout generation handler data
	for (CorblivarDie& die : this->dies) {
		die.reset();
	}

	// reset alignments-in-process list
	AL.clear();

	if (CorblivarAlignmentReq::DBG_LAYOUT_GENERATION) {
		std::cout << "DBG_ALIGNMENT>" << std::endl;
		std::cout << "DBG_ALIGNMENT> New layout-generation run..." << std::endl;
		std::cout << "DBG_ALIGNMENT>" << std::endl;
	}

	// perform layout generation in loop (until all blocks are placed)
	while (true) {

		// sanity check for empty die
		if (this->p->getCBL().empty()) {
			this->p->done = true;
		}

		// die done in previous loop; or just marked done since it's empty
		if (this->p->done) {

			// continue layout generation on next, yet unfinished die; or
			// abort loop if all dies marked as done
			if (!this->switchDie()) {
				break;
			}
		}

		// dbg logging for current block
		if (CorblivarAlignmentReq::DBG_LAYOUT_GENERATION) {

			std::cout << "DBG_ALIGNMENT> Processing " << this->p->getCBL().tupleString(this->p->pi) << " on die " << this->p->id + 1 << std::endl;
		}

		// handle stalled die, i.e., resolve paused alignment process by placing
		// current block
		if (this->p->stalled) {

			if (CorblivarAlignmentReq::DBG_LAYOUT_GENERATION) {
				std::cout << "DBG_ALIGNMENT>  Resolving stalled die: " << this->p->id + 1;
				std::cout << " place current block : " << this->p->getCurrentBlock()->id << std::endl;
			}

			// place block, increment progress pointer
			this->p->placeCurrentBlock(perform_alignment);
			this->p->updateProgressPointerFlag();
			// mark die as not stalled anymore
			this->p->stalled = false;
		}
		// die is not stalled
		else {
			// handle current block on this die
			cur_block = this->p->getCurrentBlock();

			// handle block alignment only if desired
			if (perform_alignment) {

				// determine related requests for current block
				cur_block_alignment_reqs = this->findAlignmentReqs(cur_block);

				// some requests are given, handle them stepwise
				if (!cur_block_alignment_reqs.empty()) {

					// handle each request
					for (auto* cur_req : cur_block_alignment_reqs) {

						if (CorblivarAlignmentReq::DBG_LAYOUT_GENERATION) {
							std::cout << "DBG_ALIGNMENT>  Handling alignment request for block " << cur_block->id << std::endl;
							std::cout << "DBG_ALIGNMENT>   Request: " << cur_req->tupleString() << std::endl;
						}

						// determine other block of request
						if (cur_req->s_i->numerical_id == cur_block->numerical_id) {
							other_block = cur_req->s_j;
						}
						else {
							other_block = cur_req->s_i;
						}

						// check if request is already in process; if so,
						// the die related to the other block is currently
						// stalled, i.e., waiting for this block to be
						// placed / both blocks to be aligned
						req_processed = nullptr;
						for (auto* req_in_process : this->AL) {

							if (cur_req->id == req_in_process->id) {

								if (CorblivarAlignmentReq::DBG_LAYOUT_GENERATION) {
									std::cout << "DBG_ALIGNMENT>    Request in process; aligning related blocks" << std::endl;
								}

								// place/align blocks, but keep
								// progress pointer for now in
								// order to handle all alignment
								// requests for current blocks
								if (this->alignBlocks(cur_req)) {

									// in case the request was
									// handled, memorize
									// processed request
									req_processed = cur_req;

									// mark die related w/ other block
									// as not stalled any more;
									// re-enables further layout
									// generation on that die in next
									// iterations
									this->dies[other_block->layer].stalled = false;
								}

								break;
							}
						}

						// request is not in process yet;
						if (req_processed == nullptr) {

							// stall layout generation on this die;
							this->dies[cur_block->layer].stalled = true;
							// memorize alignment as in process;
							this->AL.push_back(cur_req);
							// continue layout generation on
							// die related to other block of
							// request; only applicable for
							// regular blocks, i.e., for RBOD,
							// continue on current die
							if (other_block->numerical_id != RBOD::NUMERICAL_ID) {
								this->p = &this->dies[other_block->layer];
							}

							if (CorblivarAlignmentReq::DBG_LAYOUT_GENERATION) {
								std::cout << "DBG_ALIGNMENT>    Request not (yet) in process" << std::endl;
								std::cout << "DBG_ALIGNMENT>     Mark request as in process; stall current die " << cur_block->layer + 1;
								std::cout << ", continue on die " << this->p->id + 1 << std::endl;
							}
						}
						// request is processed; drop from list of
						// requests-in-process
						else {
							this->AL.remove(req_processed);
						}
					}

					// all requests are handled and further layout generation
					// to be continued on this die; increment progress pointer
					// since block and related alignment requests are handled
					if (!this->dies[cur_block->layer].stalled) {

						if (CorblivarAlignmentReq::DBG_LAYOUT_GENERATION) {
							std::cout << "DBG_ALIGNMENT>  All requests handled for block " << cur_block->id << "; continue w/ next block" << std::endl;
						}

						this->p->updateProgressPointerFlag();
					}
				}

				// no alignment requested given for current block
				else {
					// place block, increment progress pointer
					this->p->placeCurrentBlock(perform_alignment);
					this->p->updateProgressPointerFlag();
				}
			}

			// handling block alignment is not desired, simply place blocks
			else {
				this->p->placeCurrentBlock(perform_alignment);
				this->p->updateProgressPointerFlag();
			}
		}
	}

	if (CorblivarCore::DBG) {
		std::cout << "DBG_CORE> ";
		std::cout << "Done" << std::endl;
	}

	return true;
}

bool CorblivarCore::alignBlocks(CorblivarAlignmentReq const* req) {
	Block const* b1;
	Block const* b2;
	CorblivarDie* die_b1;
	CorblivarDie* die_b2;
	std::list<Block const*> b1_relev_blocks, b2_relev_blocks;
	Direction dir_b1, dir_b2;
	bool b1_shifted, b2_shifted;
	bool b1_to_shift_horizontal, b1_to_shift_vertical, b2_to_shift_horizontal, b2_to_shift_vertical;

	// scenario I: both blocks are yet unplaced
	if (!req->s_i->placed && !req->s_j->placed) {

		if (CorblivarAlignmentReq::DBG_LAYOUT_GENERATION) {
			std::cout << "DBG_ALIGNMENT>     Both blocks not placed yet; consider adaptive alignment..." << std::endl;
		}

		// local mapping of blocks
		b1 = req->s_i;
		b2 = req->s_j;

		// retrieve related die pointers
		die_b1 = &this->dies[b1->layer];
		die_b2 = &this->dies[b2->layer];

		// pop relevant blocks from related placement stacks
		b1_relev_blocks = die_b1->popRelevantBlocks();
		b2_relev_blocks = die_b2->popRelevantBlocks();

		// first, we need to determine which insertion direction is to be applied
		// for each block
		dir_b1 = die_b1->getCurrentDirection();
		dir_b2 = die_b2->getCurrentDirection();

		// second, depending on insertion direction, there are various
		// subscenarios
		//
		// subscenario IIa: both blocks have the same insertion direction, i.e.,
		// can be processed in parallel
		if (dir_b1 == dir_b2) {

			// horizontal placement
			if (dir_b1 == Direction::HORIZONTAL) {

				// dbg placement direction
				if (CorblivarAlignmentReq::DBG_LAYOUT_GENERATION) {
					std::cout << "DBG_ALIGNMENT>     Blocks are handled in parallel, and to be placed horizontally" << std::endl;
				}

				// first, determine blocks' y-coordinates
				die_b1->determCurrentBlockCoords(Coordinate::Y, b1_relev_blocks);
				die_b2->determCurrentBlockCoords(Coordinate::Y, b2_relev_blocks);

				// perform shift in y-dir, if required and possible
				b1_shifted = die_b1->shiftCurrentBlock(Direction::VERTICAL, req);
				b2_shifted = die_b2->shiftCurrentBlock(Direction::VERTICAL, req);

				// second, determine block's x-coordinates (depends on
				// y-coord of relevant blocks and, more generally, on
				// previously placed, possibly shifted blocks)
				die_b1->determCurrentBlockCoords(Coordinate::X, b1_relev_blocks, true);
				die_b2->determCurrentBlockCoords(Coordinate::X, b2_relev_blocks, true);

				// perform shift in x-dir, if required and possible
				b1_shifted = die_b1->shiftCurrentBlock(Direction::HORIZONTAL, req) || b1_shifted;
				b2_shifted = die_b2->shiftCurrentBlock(Direction::HORIZONTAL, req) || b2_shifted;
			}
			// vertical placement
			else {

				// dbg placement direction
				if (CorblivarAlignmentReq::DBG_LAYOUT_GENERATION) {
					std::cout << "DBG_ALIGNMENT>     Blocks are handled in parallel, and to be placed vertically" << std::endl;
				}

				// first, determine blocks' x-coordinates
				die_b1->determCurrentBlockCoords(Coordinate::X, b1_relev_blocks);
				die_b2->determCurrentBlockCoords(Coordinate::X, b2_relev_blocks);

				// perform shift in x-dir, if required and possible
				b1_shifted = die_b1->shiftCurrentBlock(Direction::HORIZONTAL, req);
				b2_shifted = die_b2->shiftCurrentBlock(Direction::HORIZONTAL, req);

				// second, determine block's y-coordinates (depends on
				// x-coord of relevant blocks and, more generally, on
				// previously placed, possibly shifted blocks)
				die_b1->determCurrentBlockCoords(Coordinate::Y, b1_relev_blocks, true);
				die_b2->determCurrentBlockCoords(Coordinate::Y, b2_relev_blocks, true);

				// perform shift in y-dir, if required and possible
				b1_shifted = die_b1->shiftCurrentBlock(Direction::VERTICAL, req) || b1_shifted;
				b2_shifted = die_b2->shiftCurrentBlock(Direction::VERTICAL, req) || b2_shifted;
			}
		}
		// subscenario IIb: blocks have different insertion direction, i.e.,
		// cannot be processed in parallel
		else {

			// dbg placement direction
			if (CorblivarAlignmentReq::DBG_LAYOUT_GENERATION) {
				std::cout << "DBG_ALIGNMENT>     Blocks are to be handled sequentially" << std::endl;
			}

			// initially, determine all coordinates
			if (dir_b1 == Direction::HORIZONTAL) {
				die_b1->determCurrentBlockCoords(Coordinate::Y, b1_relev_blocks);
				die_b1->determCurrentBlockCoords(Coordinate::X, b1_relev_blocks, true);
			}
			else {
				die_b1->determCurrentBlockCoords(Coordinate::X, b1_relev_blocks);
				die_b1->determCurrentBlockCoords(Coordinate::Y, b1_relev_blocks, true);
			}
			if (dir_b2 == Direction::HORIZONTAL) {
				die_b2->determCurrentBlockCoords(Coordinate::Y, b2_relev_blocks);
				die_b2->determCurrentBlockCoords(Coordinate::X, b2_relev_blocks, true);
			}
			else {
				die_b2->determCurrentBlockCoords(Coordinate::X, b2_relev_blocks);
				die_b2->determCurrentBlockCoords(Coordinate::Y, b2_relev_blocks, true);
			}

			// second, determine which block is to be shifted in which direction
			b1_to_shift_horizontal = die_b1->shiftCurrentBlock(Direction::HORIZONTAL, req, true);
			b1_to_shift_vertical = die_b1->shiftCurrentBlock(Direction::VERTICAL, req, true);
			b2_to_shift_horizontal = die_b2->shiftCurrentBlock(Direction::HORIZONTAL, req, true);
			b2_to_shift_vertical = die_b2->shiftCurrentBlock(Direction::VERTICAL, req, true);

			// third, catch the various cases for possibly required shifting
			b1_shifted = b2_shifted = false;
			//
			// a) one block has to be shifted in both directions
			if (b1_to_shift_horizontal && b1_to_shift_vertical) {

				// perform shifting of b1; helper also considers to shift
				// b2 if required
				CorblivarCore::sequentialShiftingHelper(
						die_b1, die_b2, req, b1_relev_blocks, b2_relev_blocks, dir_b1, b1_shifted, b2_shifted);
			}

			else if (b2_to_shift_horizontal && b2_to_shift_vertical) {

				// perform shifting of b2; helper also considers to shift
				// b1 if required
				CorblivarCore::sequentialShiftingHelper(
						die_b2, die_b1, req, b2_relev_blocks, b1_relev_blocks, dir_b2, b2_shifted, b1_shifted);
			}

			// b) one or both blocks are to be shifted in their first direction
			if (!b1_shifted && b1_to_shift_vertical && dir_b1 == Direction::HORIZONTAL) {
				b1_shifted = die_b1->shiftCurrentBlock(Direction::VERTICAL, req);
			}
			else if (!b1_shifted && b1_to_shift_horizontal && dir_b1 == Direction::VERTICAL) {
				b1_shifted = die_b1->shiftCurrentBlock(Direction::HORIZONTAL, req);
			}

			if (!b2_shifted && b2_to_shift_vertical && dir_b2 == Direction::HORIZONTAL) {
				b2_shifted = die_b2->shiftCurrentBlock(Direction::VERTICAL, req);
			}
			else if (!b2_shifted && b2_to_shift_horizontal && dir_b2 == Direction::VERTICAL) {
				b2_shifted = die_b2->shiftCurrentBlock(Direction::HORIZONTAL, req);
			}

			// c) trivial cases where one or both blocks are to be shifted in
			// their second direction
			if (!b1_shifted && b1_to_shift_horizontal && dir_b1 == Direction::HORIZONTAL) {
				b1_shifted = die_b1->shiftCurrentBlock(Direction::HORIZONTAL, req);
			}
			else if (!b1_shifted && b1_to_shift_vertical && dir_b1 == Direction::VERTICAL) {
				b1_shifted = die_b1->shiftCurrentBlock(Direction::VERTICAL, req);
			}

			if (!b2_shifted && b2_to_shift_horizontal && dir_b2 == Direction::HORIZONTAL) {
				b2_shifted = die_b2->shiftCurrentBlock(Direction::HORIZONTAL, req);
			}
			else if (!b2_shifted && b2_to_shift_vertical && dir_b2 == Direction::VERTICAL) {
				b2_shifted = die_b2->shiftCurrentBlock(Direction::VERTICAL, req);
			}
		}

		// if a block was shifted, we need to rebuild the related placement stacks
		// since the corner-block front in both dimensions may be different now
		//
		if (b1_shifted) {
			die_b1->rebuildPlacementStacks(b1_relev_blocks);
		}
		// if a block was not shifted, we can simply update the placement stacks
		else {
			die_b1->updatePlacementStacks(b1_relev_blocks);
		}

		if (b2_shifted) {
			die_b2->rebuildPlacementStacks(b2_relev_blocks);
		}
		else {
			die_b2->updatePlacementStacks(b2_relev_blocks);
		}

		// mark (shifted) blocks as placed
		b1->placed = true;
		b2->placed = true;

		// placement stacks debugging
		if (CorblivarDie::DBG_STACKS) {
			die_b1->debugStacks();
			die_b2->debugStacks();
		}
	}

	// scenario II: both blocks are already placed, further shifting for block
	// alignment is not feasible
	else if (req->s_i->placed && req->s_j->placed) {

		if (CorblivarAlignmentReq::DBG_LAYOUT_GENERATION) {
			std::cout << "DBG_ALIGNMENT>     Both blocks previously placed; alignment not possible!" << std::endl;
		}
	}

	// scenario III: one block is already placed
	else {

		// determine yet unplaced block
		if (!req->s_i->placed) {
			b1 = req->s_i;
			b2 = req->s_j;
		}
		else {
			b1 = req->s_j;
			b2 = req->s_i;
		}

		if (CorblivarAlignmentReq::DBG_LAYOUT_GENERATION) {
			std::cout << "DBG_ALIGNMENT>     Block " << b2->id << " previously placed; try to shift block " << b1->id << std::endl;
		}

		// retrieve related die pointer
		die_b1 = &this->dies[b1->layer];

		// sanity check for diff b/w current CBL tuple and current block; that's
		// happening when the block to be shifted is not the current block, i.e.,
		// a block to be processed later on; thus, we skip the alignment for now
		if (b1->numerical_id != die_b1->getCurrentBlock()->numerical_id) {

			if (CorblivarAlignmentReq::DBG_LAYOUT_GENERATION) {
				std::cout << "DBG_ALIGNMENT>     Shift block is not current block; abort alignment" << std::endl;
			}

			return false;
		}

		// pop relevant blocks from related placement stack
		b1_relev_blocks = die_b1->popRelevantBlocks();

		// horizontal placement
		if (die_b1->getCurrentDirection() == Direction::HORIZONTAL) {

			// dbg placement direction
			if (CorblivarAlignmentReq::DBG_LAYOUT_GENERATION) {
				std::cout << "DBG_ALIGNMENT>     Block is to be placed horizontally" << std::endl;
			}

			// first, determine block's y-coordinates
			die_b1->determCurrentBlockCoords(Coordinate::Y, b1_relev_blocks);

			// perform shift in y-dir, if required and possible
			b1_shifted = die_b1->shiftCurrentBlock(Direction::VERTICAL, req);

			// second, determine block's x-coordinates (depends on y-coord of
			// relevant blocks and, more generally, on previously placed,
			// possibly shifted blocks)
			die_b1->determCurrentBlockCoords(Coordinate::X, b1_relev_blocks, true);

			// perform shift in x-dir, if required and possible
			b1_shifted = die_b1->shiftCurrentBlock(Direction::HORIZONTAL, req) || b1_shifted;
		}
		// vertical placement
		else {

			// dbg placement direction
			if (CorblivarAlignmentReq::DBG_LAYOUT_GENERATION) {
				std::cout << "DBG_ALIGNMENT>     Block is to be placed vertically" << std::endl;
			}

			// first, determine block's x-coordinates
			die_b1->determCurrentBlockCoords(Coordinate::X, b1_relev_blocks);

			// perform shift in x-dir, if required and possible
			b1_shifted = die_b1->shiftCurrentBlock(Direction::HORIZONTAL, req);

			// second, determine block's y-coordinates (depends on x-coord of
			// relevant blocks and, more generally, on previously placed,
			// possibly shifted blocks)
			die_b1->determCurrentBlockCoords(Coordinate::Y, b1_relev_blocks, true);

			// perform shift in y-dir, if required and possible
			b1_shifted = die_b1->shiftCurrentBlock(Direction::VERTICAL, req) || b1_shifted;
		}

		// if the block was shifted, we need to rebuild the placement stacks since
		// the corner-block front in both dimensions may be different now
		if (b1_shifted) {
			die_b1->rebuildPlacementStacks(b1_relev_blocks);
		}
		// if the block was not shifted, we can simply update the placement stacks
		else {
			die_b1->updatePlacementStacks(b1_relev_blocks);
		}

		// mark shifted block as placed
		b1->placed = true;

		// placement stacks debugging
		if (CorblivarDie::DBG_STACKS) {
			die_b1->debugStacks();
		}
	}

	return true;
}

void CorblivarCore::sequentialShiftingHelper(CorblivarDie* die_b1, CorblivarDie* die_b2, CorblivarAlignmentReq const* req, std::list<Block const*> b1_relev_blocks, std::list<Block const*> b2_relev_blocks, Direction const& dir_b1, bool& b1_shifted, bool& b2_shifted) {

	// annotate that b1 is shifted at least in one direction
	b1_shifted = true;

	// actual shifting depends on insertion direction
	if (dir_b1 == Direction::HORIZONTAL) {

		// perform shift in first direction, for
		// horizontal insertion that's the y-coordinate
		die_b1->shiftCurrentBlock(Direction::VERTICAL, req);

		// redetermine second coordinate, i.e.,
		// x-coordinate
		die_b1->determCurrentBlockCoords(Coordinate::X, b1_relev_blocks, true);

		// try to shift b1 also in second direction; if
		// that's not possible we need to try shifting b2
		if (!die_b1->shiftCurrentBlock(Direction::HORIZONTAL, req)) {

			// b2's insertion direction is vertical
			// (since b1's direction is horizontal and
			// different from b2's direction); thus we
			// shift b2 in its first direction
			// (horizontal)
			b2_shifted = die_b2->shiftCurrentBlock(Direction::HORIZONTAL, req);

			// redetermine b2's second coordinate,
			// i.e., y-coordinate
			die_b2->determCurrentBlockCoords(Coordinate::Y, b2_relev_blocks, true);

			// (TODO) since b2's second coordinate
			// might have changed, we would need to
			// check whether b1's previous shifting is
			// now undermined; this problem is a
			// circular problem and has to be consider
			// iteratively; thus, we ignore for
			// simplified handling such (rare?) cases
			if (CorblivarAlignmentReq::DBG_LAYOUT_GENERATION) {
				if (die_b1->shiftCurrentBlock(Direction::VERTICAL, req, true)) {
					std::cout << "DBG_ALIGNMENT>      Circular dependency occured during sequential shifiting of both blocks; alignment is undermined!" << std::endl;
				}
			}
		}
	}

	// b1's insertion direction is vertical
	else {

		// perform shift in first direction, for
		// vertical insertion that's the x-coordinate
		die_b1->shiftCurrentBlock(Direction::HORIZONTAL, req);

		// redetermine second coordinate, i.e.,
		// y-coordinate
		die_b1->determCurrentBlockCoords(Coordinate::Y, b1_relev_blocks, true);

		// try to shift b1 also in second direction; if
		// that's not possible we need to try shifting b2
		if (!die_b1->shiftCurrentBlock(Direction::VERTICAL, req)) {

			// b2's insertion direction is horizontal
			// (since b1's direction is vertical and
			// different from b2's direction); thus we
			// shift b2 in its first direction
			// (vertical)
			b2_shifted = die_b2->shiftCurrentBlock(Direction::VERTICAL, req);

			// redetermine b2's second coordinate,
			// i.e., x-coordinate
			die_b2->determCurrentBlockCoords(Coordinate::X, b2_relev_blocks, true);

			// (TODO) since b2's second coordinate might
			// have changed, we would need to check
			// whether b1's previous shifting is now
			// undermined; this problem is a circular
			// problem and has to be consider
			// iteratively; thus, we ignore for
			// simplified handling such (rare?) cases
			if (CorblivarAlignmentReq::DBG_LAYOUT_GENERATION) {
				if (die_b1->shiftCurrentBlock(Direction::HORIZONTAL, req, true)) {
					std::cout << "DBG_ALIGNMENT>      Circular dependency occured during sequential shifiting of both blocks; alignment is undermined!" << std::endl;
				}
			}
		}
	}
}

std::vector<CorblivarAlignmentReq const*> CorblivarCore::findAlignmentReqs(Block const* b) const {
	std::vector<CorblivarAlignmentReq const*> ret;

	// sanity check for no given requests
	if (this->A.empty()) {
		return ret;
	}

	// determine requests covering the given block
	for (CorblivarAlignmentReq const& req : this->A) {

		if (req.s_i->numerical_id == b->numerical_id || req.s_j->numerical_id == b->numerical_id) {

			// only consider request which are still in
			// process, i.e., not both blocks are placed yet
			if (!req.s_i->placed || !req.s_j->placed) {

				if (CorblivarAlignmentReq::DBG_LAYOUT_GENERATION) {
					std::cout << "DBG_ALIGNMENT>  Unhandled request: " << req.tupleString() << std::endl;
				}

				ret.push_back(&req);
			}
		}
	}

	// requests w/ placed blocks are considered first; eases handling
	// of alignment requests such that blocks ready for alignment are
	// placed/aligned first
	std::sort(ret.begin(), ret.end(),
		// lambda expression
		[](CorblivarAlignmentReq const* req1, CorblivarAlignmentReq const* req2) {
			return
				// std::sort requires a _strict_ ordering, thus we have to make sure that same elements returns false
				// http://stackoverflow.com/a/1541909
				// also helps to make comparison short-cutting it early
				(req1 != req2) && (
					(req1->s_i->placed || req1->s_j->placed) && (!req2->s_i->placed && !req2->s_j->placed)
				);
		}
	);

	return ret;
}

void CorblivarCore::sortCBLs(bool const& log, int const& mode) {
	std::vector<std::vector<CornerBlockList::Tuple>> tuples;
	std::vector<CornerBlockList::Tuple> tuples_die;
	CornerBlockList::Tuple cur_tuple;

	// log
	if (log) {
		switch (mode) {

			case CorblivarCore::SORT_CBLS_BY_BLOCKS_SIZE:
				std::cout << "Corblivar> ";
				std::cout << "Sorting CBL tuples by block sizes ..." << std::endl;

				break;
		}
	}

	// construct temp vectors w/ CBL tuples, assign separate CBL sequences to tuple vectors
	for (CorblivarDie& die : this->dies) {

		tuples_die.clear();

		for (unsigned t = 0; t < die.CBL.size(); t++) {

			cur_tuple.S = std::move(die.CBL.S[t]);
			cur_tuple.L = std::move(die.CBL.L[t]);
			cur_tuple.T = std::move(die.CBL.T[t]);

			tuples_die.push_back(std::move(cur_tuple));
		}

		tuples.push_back(std::move(tuples_die));
	}

	// perfom sorting
	switch (mode) {

		// sort tuple vector by blocks size, in descending order
		case CorblivarCore::SORT_CBLS_BY_BLOCKS_SIZE:

			for (auto& tuples_die : tuples) {
				std::sort(tuples_die.begin(), tuples_die.end(),
					// lambda expression to provide compare function
					[](CornerBlockList::Tuple const& t1, CornerBlockList::Tuple const& t2) {
						return
							// std::sort requires a _strict_ ordering, thus we have to make sure that same elements returns false
							// http://stackoverflow.com/a/1541909
							// also helps to make comparison short-cutting it early
							(t1.S->numerical_id != t2.S->numerical_id) & (t1.S->bb.area > t2.S->bb.area);
					}
				);
			}

			break;
	}

	// reassign CBL sequences from tuple vectors
	unsigned d = 0;
	for (CorblivarDie& die : this->dies) {

		for (unsigned t = 0; t < die.CBL.size(); t++) {

			die.CBL.S[t] = std::move(tuples[d][t].S);
			die.CBL.L[t] = std::move(tuples[d][t].L);
			die.CBL.T[t] = std::move(tuples[d][t].T);
		}

		d++;
	}

	// log
	if (log) {
		std::cout << "Corblivar> ";
		std::cout << "Done" << std::endl << std::endl;
	}
}
