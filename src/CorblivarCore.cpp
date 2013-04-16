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

void CorblivarCore::initCorblivarRandomly(bool const& log, int const& layers, vector<Block> const& blocks) {
	Direction cur_dir;
	int rand, cur_t;

	if (log) {
		cout << "Layout> ";
		cout << "Initializing Corblivar data for corb on " << layers << " layers..." << endl;
	}

	// assign each block randomly to one die, generate L and T randomly as well
	for (Block const& cur_block : blocks) {

		// consider random die
		rand = Math::randI(0, layers);

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
		this->dies[rand].CBL.S.push_back(move(&cur_block));
		this->dies[rand].CBL.L.push_back(move(cur_dir));
		this->dies[rand].CBL.T.push_back(move(cur_t));
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
		cout << "Layout> ";
		cout << "Done" << endl << endl;
	}
}

void CorblivarCore::generateLayout(bool const& dbgStack) {
	Block const* cur_block;
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

	// perform layout generation in loop (until all blocks are placed)
	loop = true;
	while (loop) {
		// handle stalled die / resolve open alignment process by placing current block
		if (this->p->stalled) {
			// place block, increment progress pointer
			cur_block = this->p->placeCurrentBlock(dbgStack);
			// TODO mark current block as placed in AS
			if (cur_block != nullptr) {
			}

			// mark die as not stalled anymore
			this->p->stalled = false;
		}
		// die is not stalled
		else {
			// TODO check for alignment tuples for current block
			//
			if (false) {
			}
			// no alignment tuple assigned for current block
			else {
				// place block, increment progress pointer
				this->p->placeCurrentBlock(dbgStack);
			}
		}

		// die done
		if (this->p->done) {
			// continue loop on yet unfinished die
			for (CorblivarDie& die :  this->dies) {
				if (!die.done) {
					this->p = &die;
					break;
				}
			}
			// all dies handled, stop loop
			if (this->p->done) {
				loop = false;
			}
		}
	}

	if (CorblivarCore::DBG) {
		cout << "DBG_CORE> ";
		cout << "Done" << endl;
	}
}
