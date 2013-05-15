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
		static constexpr bool DBG_ALIGNMENT_REQ = false;

	// private data, functions
	private:
		// main data; encapsulated in CorblivarDie; Corblivar can thus be
		// considered as 2.5D layout representation
		vector<CorblivarDie> dies;

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

		// alignments-in-process list
		list<CorblivarAlignmentReq const*> AL;

		// alignment-requests helper
		//
		// determine open requests covering block b
		inline list<CorblivarAlignmentReq const*> findAlignmentReqs(Block const* b) const {
			list<CorblivarAlignmentReq const*> ret;

			if (CorblivarCore::DBG_ALIGNMENT_REQ) {
				cout << "DBG_ALIGNMENT>  Determine unhandled alignment requests" << endl;
			}

			for (CorblivarAlignmentReq const& req : this->A) {

				if (req.s_i->id == b->id || req.s_j->id == b->id) {

					// only consider request which are still in
					// process, i.e., not both blocks are placed yet
					if (!req.s_i->placed || !req.s_j->placed) {

						if (CorblivarCore::DBG_ALIGNMENT_REQ) {
							cout << "DBG_ALIGNMENT>   request: " << req.tupleString() << endl;
						}

						ret.push_back(&req);
					}
				}
			}

			// sort such that requests w/ placed blocks are considered
			// first; eases handling of intersecting requests
			ret.sort(
				// lambda expression
				[&](CorblivarAlignmentReq const* req1, CorblivarAlignmentReq const* req2) {
					return (req1->s_i->placed || req2->s_j->placed);
				}
			);

			return ret;
		}

	// TODO declare private; public for testing only
	public:
		// sequence A; alignment requests
		vector<CorblivarAlignmentReq> A;

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

				this->dies.push_back(move(cur_die));
			}
		};

	// public data, functions
	public:
		// general operations
		void initCorblivarRandomly(bool const& log, int const& layers, vector<Block> const& blocks, bool const& power_aware_assignment);
		void generateLayout(int const& packing_iterations, bool const& dbgStack = false);

		// die getter
		inline CorblivarDie& editDie(unsigned const& die) {
			return this->dies[die];
		};
		inline CorblivarDie const& getDie(unsigned const& die) const {
			return this->dies[die];
		};

		inline void swapBlocks(int const& die1, int const& die2, int const& tuple1, int const& tuple2) {

			// pre-update layer assignments if swapping across dies
			if (die1 != die2) {
				this->dies[die1].CBL.S[tuple1]->layer = die2;
				this->dies[die2].CBL.S[tuple2]->layer = die1;
			}

			// perform swap
			swap(this->dies[die1].CBL.S[tuple1], this->dies[die2].CBL.S[tuple2]);

			if (DBG) {
				cout << "DBG_CORE> swapBlocks; d1=" << die1 << ", d2=" << die2;
				cout << ", s1=" << this->dies[die1].CBL.S[tuple1]->id;
				cout << ", s2=" << this->dies[die2].CBL.S[tuple2]->id << endl;
			}
		};
		inline void moveTuples(int const& die1, int const& die2, int const& tuple1, int const& tuple2) {

			// move within same die: perform swaps
			if (die1 == die2) {
				swap(this->dies[die1].CBL.S[tuple1], this->dies[die2].CBL.S[tuple2]);
				swap(this->dies[die1].CBL.L[tuple1], this->dies[die2].CBL.L[tuple2]);
				swap(this->dies[die1].CBL.T[tuple1], this->dies[die2].CBL.T[tuple2]);
			}
			// move across dies: perform insert and delete
			else {
				// pre-update layer assignment for block to be moved
				this->dies[die1].CBL.S[tuple1]->layer = die2;

				// insert tuple1 from die1 into die2 w/ offset tuple2
				this->dies[die2].CBL.S.insert(this->dies[die2].CBL.S.begin() + tuple2, move(this->dies[die1].CBL.S[tuple1]));
				this->dies[die2].CBL.L.insert(this->dies[die2].CBL.L.begin() + tuple2, move(this->dies[die1].CBL.L[tuple1]));
				this->dies[die2].CBL.T.insert(this->dies[die2].CBL.T.begin() + tuple2, move(this->dies[die1].CBL.T[tuple1]));

				// erase tuple1 from die1
				this->dies[die1].CBL.S.erase(this->dies[die1].CBL.S.begin() + tuple1);
				this->dies[die1].CBL.L.erase(this->dies[die1].CBL.L.begin() + tuple1);
				this->dies[die1].CBL.T.erase(this->dies[die1].CBL.T.begin() + tuple1);
			}

			if (DBG) {
				cout << "DBG_CORE> moveTuples; d1=" << die1 << ", d2=" << die2 << ", t1=" << tuple1 << ", t2=" << tuple2 << endl;
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
				cout << "DBG_CORE> switchInsertionDirection; d1=" << die << ", t1=" << tuple << endl;
			}
		};
		inline void switchTupleJunctions(int const& die, int const& tuple, int const& juncts) {
			this->dies[die].CBL.T[tuple] = juncts;

			if (DBG) {
				cout << "DBG_CORE> switchTupleJunctions; d1=" << die << ", t1=" << tuple << ", juncts=" << juncts << endl;
			}
		};

		// CBL logging
		inline string CBLsString() const {
			stringstream ret;

			ret << "# tuple format: ( BLOCK_ID DIRECTION T-JUNCTS BLOCK_WIDTH BLOCK_HEIGHT )" << endl;
			ret << "data_start" << endl;

			for (CorblivarDie const& die : this->dies) {
				ret << "CBL [ " << die.id << " ]" << endl;
				ret << die.CBL.CBLString() << endl;
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
					// backup (copy) block box into block itself
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
					// restore (copy) block box from block itself
					b->bb = b->bb_backup;
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
					b->bb = b->bb_best;
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
				cout << "Corblivar> No best (fitting) solution available!" << endl << endl;
			}

			return ret;
		};
};

#endif
