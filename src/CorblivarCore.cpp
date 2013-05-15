/*
 * =====================================================================================
 *
 *    Description:  Corblivar core (data structures, layout operations)
 *
 *         Author:  Johann Knechtel, johann.knechtel@ifte.de
 *        Company:  Institute of Electromechanical and Electronic Design, www.ifte.de
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

void CorblivarCore::initCorblivarRandomly(bool const& log, int const& layers, vector<Block> const& blocks, bool const& power_aware_assignment) {
	Direction cur_dir;
	int die, cur_t, cur_layer;
	double blocks_area_per_layer, cur_blocks_area;
	vector<Block> blocks_copy;
	Block const* cur_block;

	if (log) {
		cout << "Corblivar> ";
		cout << "Initializing Corblivar data for corb on " << layers << " layers; ";
		if (power_aware_assignment) {
			cout << "w/ power-aware block handling..." << endl;
		}
		else {
			cout << "w/o power-aware block handling..." << endl;
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
		sort(blocks_copy.begin(), blocks_copy.end(),
			// lambda expression
			[&](Block b1, Block b2) {
				return b1.power_density < b2.power_density;
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
			die = min(cur_layer, layers - 1);
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
		this->dies[die].CBL.S.push_back(move(cur_block));
		this->dies[die].CBL.L.push_back(move(cur_dir));
		this->dies[die].CBL.T.push_back(move(cur_t));
	}

	if (CorblivarCore::DBG) {
		for (CorblivarDie const& die : this->dies) {
			cout << "DBG_CORE> ";
			cout << "Init CBL tuples for die " << die.id << "; " << die.CBL.size() << " tuples:" << endl;
			cout << die.CBL.CBLString() << endl;
			cout << "DBG_CORE> ";
			cout << endl;
		}
	}

	if (log) {
		cout << "Corblivar> ";
		cout << "Done" << endl << endl;
	}
}

// (TODO?) dbgStack as compile-time flag
void CorblivarCore::generateLayout(int const& packing_iterations, bool const& dbgStack) {
	Block const* cur_block;
	Block const* other_block;
	list<CorblivarAlignmentReq const*> cur_block_alignment_reqs;
	CorblivarAlignmentReq const* req_processed;
	bool loop;

	if (CorblivarCore::DBG) {
		cout << "DBG_CORE> ";
		cout << "Performing layout generation..." << endl;
	}

	// init die pointer
	this->p = &this->dies[0];

	// reset die data, i.e., layout generation handler data
	for (CorblivarDie& die : this->dies) {
		die.reset();
	}

	// reset alignments-in-process list
	AL.clear();

	if (CorblivarCore::DBG_ALIGNMENT_REQ) {
		cout << "DBG_ALIGNMENT>" << endl;
		cout << "DBG_ALIGNMENT> New layout-generation run..." << endl;
		cout << "DBG_ALIGNMENT>" << endl;
	}

	// perform layout generation in loop (until all blocks are placed)
	loop = true;
	while (loop) {

		if (CorblivarCore::DBG_ALIGNMENT_REQ) {
			cout << "DBG_ALIGNMENT> Processing block " << this->p->getCurrentBlock()->id << " on die " << this->p->id;
			cout << "; (#tuple w/in die: " << this->p->pi << ")" << endl;
		}

		// handle stalled die, i.e., resolve paused alignment process by placing
		// current block
		if (this->p->stalled) {

			if (CorblivarCore::DBG_ALIGNMENT_REQ) {
				cout << "DBG_ALIGNMENT>  Resolving stalled die: " << this->p->id;
				cout << " place current block : " << this->p->getCurrentBlock()->id << endl;
			}

			// place block, increment progress pointer
			this->p->placeCurrentBlock(dbgStack);
			this->p->updateProgressPointerFlag();
			// mark die as not stalled anymore
			this->p->stalled = false;
		}
		// die is not stalled
		else {
			// check for alignment tuples for current block
			// (TODO) hold pointer to reqs in blocks themselves for efficiency
			cur_block = this->p->getCurrentBlock();
			cur_block_alignment_reqs = this->findAlignmentReqs(cur_block);

			// some requests are given, handle them stepwise
			if (!cur_block_alignment_reqs.empty()) {

				// handle each request
				for (auto* cur_req : cur_block_alignment_reqs) {

					if (CorblivarCore::DBG_ALIGNMENT_REQ) {
						cout << "DBG_ALIGNMENT>  Handling alignment request for block " << cur_block->id << endl;
						cout << "DBG_ALIGNMENT>   Request: " << cur_req->tupleString() << endl;
					}

					// determine other block of request
					if (cur_req->s_i->id == cur_block->id) {
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

							if (CorblivarCore::DBG_ALIGNMENT_REQ) {
								cout << "DBG_ALIGNMENT>    Request in process; aligning related blocks" << endl;
							}

							// TODO replace w/
							// alignBlocks(cur_block,
							// other_block); in alignBlocks,
							// we need to check for previously
							// placed blocks and mark blocks
							// as placed afterwards; also, the
							// progress pointer must not be
							// increased
							//
							// TODO ok?
							this->dies[cur_block->layer].placeCurrentBlock(dbgStack);
							this->dies[other_block->layer].placeCurrentBlock(dbgStack);
							// simply marking as placed not
							// ok, some initial coordinates /
							// coordinates related to covering
							// blocks will be the result
							//cur_block->placed = true;
							//other_block->placed = true;

							// mark die related w/ other block
							// as not stalled any more;
							// re-enables further layout
							// generation on that die in next
							// iterations
							this->dies[other_block->layer].stalled = false;

							// memorize processed request
							req_processed = cur_req;

							break;
						}
					}

					// request is not in process yet;
					if (req_processed == nullptr) {

						// stall layout generation on this die;
						this->dies[cur_block->layer].stalled = true;
						// memorize alignment as in process;
						this->AL.push_back(cur_req);
						// continue layout generation on die
						// related to other block of request
						this->p = &this->dies[other_block->layer];

						if (CorblivarCore::DBG_ALIGNMENT_REQ) {
							cout << "DBG_ALIGNMENT>    Request not (yet) in process" << endl;
							cout << "DBG_ALIGNMENT>     Mark request as in process; stall current die " << cur_block->layer;
							cout << ", continue on die " << this->p->id << endl;
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

					if (CorblivarCore::DBG_ALIGNMENT_REQ) {
						cout << "DBG_ALIGNMENT>  All requests handled for block " << cur_block->id << "; continue w/ next block" << endl;
					}

					this->p->updateProgressPointerFlag();
				}
			}
			// no alignment requested for current block
			else {

				if (CorblivarCore::DBG_ALIGNMENT_REQ) {
					cout << "DBG_ALIGNMENT>  No unhandled request for block " << cur_block->id << "; place block" << endl;
				}

				// place block, increment progress pointer
				this->p->placeCurrentBlock(dbgStack);
				this->p->updateProgressPointerFlag();
			}
		}

		// die done
		if (this->p->done) {

			// perform packing if desired; perform for each dimension
			// separately and subsequently; multiple iterations may provide
			// denser packing configurations
			for (int i = 1; i <= packing_iterations; i++) {
				this->p->performPacking(Direction::HORIZONTAL);
				this->p->performPacking(Direction::VERTICAL);
			}

			// continue layout generation on next, yet unfinished die
			loop = this->switchDie();
		}
	}

	if (CorblivarCore::DBG) {
		cout << "DBG_CORE> ";
		cout << "Done" << endl;
	}
}

void CorblivarCore::sortCBLs(bool const& log, int const& mode) {
	vector<vector<CornerBlockList::Tuple>> tuples;
	vector<CornerBlockList::Tuple> tuples_die;
	CornerBlockList::Tuple cur_tuple;

	// log
	if (log) {
		switch (mode) {

			case CorblivarCore::SORT_CBLS_BY_BLOCKS_SIZE:
				cout << "Corblivar> ";
				cout << "Sorting CBL tuples by block sizes ..." << endl;

				break;
		}
	}

	// construct temp vectors w/ CBL tuples, assign separate CBL sequences to tuple vectors
	for (CorblivarDie& die : this->dies) {

		tuples_die.clear();

		for (unsigned t = 0; t < die.CBL.size(); t++) {

			cur_tuple.S = move(die.CBL.S[t]);
			cur_tuple.L = move(die.CBL.L[t]);
			cur_tuple.T = move(die.CBL.T[t]);

			tuples_die.push_back(move(cur_tuple));
		}

		tuples.push_back(move(tuples_die));
	}

	// perfom sorting
	switch (mode) {

		// sort tuple vector by blocks size, in descending order
		case CorblivarCore::SORT_CBLS_BY_BLOCKS_SIZE:

			for (auto& tuples_die : tuples) {
				sort(tuples_die.begin(), tuples_die.end(),
					// lambda expression to provide compare function
					[&](CornerBlockList::Tuple t1, CornerBlockList::Tuple t2) {
						return t1.S->bb.area > t2.S->bb.area;
					}
				);
			}

			break;
	}

	// reassign CBL sequences from tuple vectors
	unsigned d = 0;
	for (CorblivarDie& die : this->dies) {

		for (unsigned t = 0; t < die.CBL.size(); t++) {

			die.CBL.S[t] = move(tuples[d][t].S);
			die.CBL.L[t] = move(tuples[d][t].L);
			die.CBL.T[t] = move(tuples[d][t].T);
		}

		d++;
	}

	// log
	if (log) {
		cout << "Corblivar> ";
		cout << "Done" << endl << endl;
	}
}
