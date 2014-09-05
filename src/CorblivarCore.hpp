/*
 * =====================================================================================
 *
 *    Description:  Corblivar core (data structures, layout operations)
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
#ifndef _CORBLIVAR_CORE
#define _CORBLIVAR_CORE

// library includes
#include "Corblivar.incl.hpp"
// Corblivar includes, if any
#include "CorblivarDie.hpp"
#include "CorblivarAlignmentReq.hpp"
// forward declarations, if any
class Block;

class CorblivarCore {
	// debugging code switch (private)
	private:
		static constexpr bool DBG = false;

	// debugging code switch (public)
	public:
		static constexpr bool DBG_VALID_LAYOUT = false;
		static constexpr bool DBG_ALIGNMENT_REQ = false;

	// private data, functions
	private:
		// main data; encapsulated in CorblivarDie; Corblivar can thus be
		// considered as 2.5D layout representation
		std::vector<CorblivarDie> dies;

		// current-die pointer
		CorblivarDie* p;

		// die-selection handler
		inline bool switchDie() {

			// try to continue on unfinished die
			for (CorblivarDie& die :  this->dies) {
				if (!die.done) {
					this->p = &die;
					break;
				}
			}

			// all dies handled, continue w/ next die not possible
			if (this->p->done) {
				return false;
			}
			else {
				return true;
			}
		};

		// sequence A; alignment requests
		std::vector<CorblivarAlignmentReq> A;

		// alignments-in-process list
		std::list<CorblivarAlignmentReq const*> AL;

		// handler for block alignment
		bool alignBlocks(CorblivarAlignmentReq const* req);
		static void sequentialShiftingHelper(
				CorblivarDie* die_b1, CorblivarDie* die_b2,
				CorblivarAlignmentReq const* req,
				std::list<Block const*> b1_relev_blocks, std::list<Block const*> b2_relev_blocks,
				Direction const& dir_b1,
				bool& b1_shifted, bool& b2_shifted);
		std::list<CorblivarAlignmentReq const*> findAlignmentReqs(Block const* b) const;

	// constructors, destructors, if any non-implicit
	public:
		CorblivarCore(int const& layers, unsigned const& blocks) {

			// reserve mem for dies
			this->dies.reserve(layers);

			// init dies and their related structures
			for (int i = 0; i < layers; i++) {
				CorblivarDie cur_die = CorblivarDie(i);
				// reserve mem for worst case, i.e., all blocks in one particular die
				cur_die.CBL.reserve(blocks);

				this->dies.push_back(std::move(cur_die));
			}
		};

	// public data, functions
	public:

		// general operations
		void initCorblivarRandomly(bool const& log, int const& layers, std::vector<Block> const& blocks, bool const& power_aware_assignment);
		bool generateLayout(bool const& perform_alignment);

		// getter
		inline CorblivarDie& editDie(unsigned const& die) {
			return this->dies[die];
		};
		inline CorblivarDie const& getDie(unsigned const& die) const {
			return this->dies[die];
		};
		inline std::vector<CorblivarAlignmentReq>& editAlignments() {
			return this->A;
		};
		inline std::vector<CorblivarAlignmentReq> const& getAlignments() const {
			return this->A;
		};

		// abstract layout-modification operations
		//
		inline void swapBlocks(int const& die1, int const& die2, int const& tuple1, int const& tuple2) {

			// pre-update layer assignments if swapping across dies
			if (die1 != die2) {
				this->dies[die1].CBL.S[tuple1]->layer = die2;
				this->dies[die2].CBL.S[tuple2]->layer = die1;
			}

			// perform swap
			std::swap(this->dies[die1].CBL.S[tuple1], this->dies[die2].CBL.S[tuple2]);

			if (DBG) {
				std::cout << "DBG_CORE> swapBlocks;";
				std::cout << " d1=" << die1;
				std::cout << ", s1=" << this->dies[die1].CBL.S[tuple1]->id;
				std::cout << ", d2=" << die2;
				std::cout << ", s2=" << this->dies[die2].CBL.S[tuple2]->id;
				std::cout << std::endl;
			}
		};

		inline void moveTuples(int const& die1, int const& die2, int const& tuple1, int const& tuple2) {

			if (DBG) {
				std::cout << "DBG_CORE> moveTuples;";
				std::cout << " d1=" << die1;
				std::cout << ", t1=" << tuple1;
				std::cout << " (s1=" << this->dies[die1].CBL.S[tuple1]->id << ")";
				std::cout << ", d2=" << die2;
				std::cout << ", t2=" << tuple2;
				std::cout << " (s2=" << this->dies[die2].CBL.S[tuple2]->id << ")";
				std::cout << std::endl;
			}

			// move within same die: perform swaps
			if (die1 == die2) {
				std::swap(this->dies[die1].CBL.S[tuple1], this->dies[die2].CBL.S[tuple2]);
				std::swap(this->dies[die1].CBL.L[tuple1], this->dies[die2].CBL.L[tuple2]);
				std::swap(this->dies[die1].CBL.T[tuple1], this->dies[die2].CBL.T[tuple2]);
			}
			// move across dies: perform insert and delete
			else {
				// pre-update layer assignment for block to be moved
				this->dies[die1].CBL.S[tuple1]->layer = die2;

				// insert tuple1 from die1 into die2 w/ offset tuple2
				this->dies[die2].CBL.S.insert(this->dies[die2].CBL.S.begin() + tuple2, std::move(this->dies[die1].CBL.S[tuple1]));
				this->dies[die2].CBL.L.insert(this->dies[die2].CBL.L.begin() + tuple2, std::move(this->dies[die1].CBL.L[tuple1]));
				this->dies[die2].CBL.T.insert(this->dies[die2].CBL.T.begin() + tuple2, std::move(this->dies[die1].CBL.T[tuple1]));

				// erase tuple1 from die1
				this->dies[die1].CBL.S.erase(this->dies[die1].CBL.S.begin() + tuple1);
				this->dies[die1].CBL.L.erase(this->dies[die1].CBL.L.begin() + tuple1);
				this->dies[die1].CBL.T.erase(this->dies[die1].CBL.T.begin() + tuple1);
			}
		};

		inline void switchInsertionDirection(int const& die, int const& tuple) {
			if (this->dies[die].CBL.L[tuple] == Direction::VERTICAL) {
				this->dies[die].CBL.L[tuple] = Direction::HORIZONTAL;
			}
			else {
				this->dies[die].CBL.L[tuple] = Direction::VERTICAL;
			}

			if (DBG) {
				std::cout << "DBG_CORE> switchInsertionDirection;";
				std::cout << " d1=" << die;
				std::cout << ", t1=" << tuple;
				std::cout << " (s1=" << this->dies[die].CBL.S[tuple]->id << ")";
				std::cout << std::endl;
			}
		};

		inline void switchTupleJunctions(int const& die, int const& tuple, int const& juncts) {
			this->dies[die].CBL.T[tuple] = juncts;

			if (DBG) {
				std::cout << "DBG_CORE> switchTupleJunctions;";
				std::cout << " d1=" << die;
				std::cout << ", t1=" << tuple;
				std::cout << " (s1=" << this->dies[die].CBL.S[tuple]->id << ")";
				std::cout << ", juncts=" << juncts;
				std::cout << std::endl;
			}
		};

		// CBL logging
		inline std::string CBLsString() const {
			std::stringstream ret;

			ret << "# tuple format: ( BLOCK_ID DIRECTION T-JUNCTS BLOCK_WIDTH BLOCK_HEIGHT )" << std::endl;
			ret << "data_start" << std::endl;

			for (CorblivarDie const& die : this->dies) {
				ret << "CBL [ " << die.id << " ]" << std::endl;
				ret << die.CBL.CBLString() << std::endl;
			}

			return ret.str();
		};

		// CBL sorting handler
		static constexpr int SORT_CBLS_BY_BLOCKS_SIZE = 1;
		void sortCBLs(bool const& log, int const& mode);

		// CBL backup handler
		inline void backupCBLs() {

			for (CorblivarDie& die : this->dies) {

				die.CBLbackup.clear();
				die.CBLbackup.reserve(die.CBL.capacity());

				for (Block const* b : die.CBL.S) {

					// backup bb into block itself
					b->bb_backup = b->bb;

					die.CBLbackup.S.push_back(b);
				}
				for (Direction const& dir : die.CBL.L) {
					die.CBLbackup.L.push_back(dir);
				}
				for (unsigned const& t_juncts : die.CBL.T) {
					die.CBLbackup.T.push_back(t_juncts);
				}
			}
		};
		inline void restoreCBLs() {

			for (CorblivarDie& die : this->dies) {

				die.CBL.clear();
				die.CBL.reserve(die.CBLbackup.capacity());

				for (Block const* b : die.CBLbackup.S) {

					// restore bb from block itself
					b->bb = b->bb_backup;
					// update layer assignment
					b->layer = die.id;

					die.CBL.S.push_back(b);
				}
				for (Direction const& dir : die.CBLbackup.L) {
					die.CBL.L.push_back(dir);
				}
				for (unsigned const& t_juncts : die.CBLbackup.T) {
					die.CBL.T.push_back(t_juncts);
				}
			}
		};

		// CBL best-solution handler
		inline void storeBestCBLs() {

			for (CorblivarDie& die : this->dies) {

				die.CBLbest.clear();
				die.CBLbest.reserve(die.CBL.capacity());

				for (Block const* b : die.CBL.S) {

					// backup bb into block itself
					b->bb_best = b->bb;

					die.CBLbest.S.push_back(b);
				}
				for (Direction const& dir : die.CBL.L) {
					die.CBLbest.L.push_back(dir);
				}
				for (unsigned const& t_juncts : die.CBL.T) {
					die.CBLbest.T.push_back(t_juncts);
				}
			}
		};
		// returns false only if all dies from CBLbest are empty, i.e., no best
		// solution at all is available
		inline bool applyBestCBLs(bool const& log) {
			unsigned empty_dies = 0;
			bool ret;

			for (CorblivarDie& die : this->dies) {

				die.CBL.clear();
				die.CBL.reserve(die.CBLbest.capacity());

				if (die.CBLbest.empty()) {
					empty_dies++;
					continue;
				}

				for (Block const* b : die.CBLbest.S) {

					// restore bb from block itself
					b->bb = b->bb_best;
					// update layer assignment
					b->layer = die.id;

					die.CBL.S.push_back(b);
				}
				for (Direction const& dir : die.CBLbest.L) {
					die.CBL.L.push_back(dir);
				}
				for (unsigned const& t_juncts : die.CBLbest.T) {
					die.CBL.T.push_back(t_juncts);
				}
			}

			ret = (empty_dies != this->dies.size());

			if (!ret && log) {
				std::cout << "Corblivar> No best (fitting) solution available!" << std::endl << std::endl;
			}

			return ret;
		};
};

#endif
