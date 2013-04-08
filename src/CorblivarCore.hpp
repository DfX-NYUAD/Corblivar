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
// forward declarations, if any
class CorblivarAlignmentReq;
class Block;

class CorblivarCore {
	private:
		// debugging code switch
		static constexpr bool DBG_CORE = false;

	private:
		// die pointer
		mutable CorblivarDie* p;

		// data; encapsulated in CorblivarDie; Corblivar can be considered as 2.5D
		// layout representation
		vector<CorblivarDie*> dies;

		// sequence A; alignment requirements
		vector<CorblivarAlignmentReq*> A;

	public:
		// general operations
		void initCorblivarRandomly(bool const& log, int const& layers, map<int, Block*> const& blocks);
		void generateLayout(bool const& dbgStack = false) const;
		//CorblivarDie* findDie(Block* Si);

		// die handler
		inline CorblivarDie* const& getDie(unsigned const& die) const {
			return this->dies[die];
		};
		inline unsigned diesSize() const {
			return this->dies.size();
		};

		// init handler
		inline void initCorblivarDies(int const& layers, unsigned const& blocks) {
			int i;
			CorblivarDie* cur_die;

			// clear and reserve mem for dies
			this->dies.clear();
			this->dies.reserve(layers);

			// init dies and their related structures
			for (i = 0; i < layers; i++) {
				cur_die = new CorblivarDie(i);
				// reserve mem for worst case, i.e., all blocks in one particular die
				cur_die->CBL.reserve(blocks);

				this->dies.push_back(cur_die);
			}
		};

		// layout operations for heuristic optimization
		static constexpr int OP_SWAP_BLOCKS = 1;
		static constexpr int OP_MOVE_TUPLE = 2;
		static constexpr int OP_SWITCH_TUPLE_DIR = 3;
		static constexpr int OP_SWITCH_TUPLE_JUNCTS = 4;
		static constexpr int OP_SWITCH_BLOCK_ORIENT = 5;
		// TODO implement; adapt FloorPlanner::performRandomLayoutOp
		static constexpr int OP_SWITCH_BLOCK_SHAPE = 6;

		inline void swapBlocks(int const& die1, int const& die2, int const& tuple1, int const& tuple2) const {
			swap(this->dies[die1]->CBL.S[tuple1], this->dies[die2]->CBL.S[tuple2]);

			if (DBG_CORE) {
				cout << "DBG_CORE> swapBlocks; d1=" << die1 << ", d2=" << die2;
				cout << ", s1=" << this->dies[die1]->CBL.S[tuple1]->id;
				cout << ", s2=" << this->dies[die2]->CBL.S[tuple2]->id << endl;
			}
		};
		inline void moveTuples(int const& die1, int const& die2, int const& tuple1, int const& tuple2) const {

			// move within same die: perform swaps
			if (die1 == die2) {
				swap(this->dies[die1]->CBL.S[tuple1], this->dies[die2]->CBL.S[tuple2]);
				swap(this->dies[die1]->CBL.L[tuple1], this->dies[die2]->CBL.L[tuple2]);
				swap(this->dies[die1]->CBL.T[tuple1], this->dies[die2]->CBL.T[tuple2]);
			}
			// move across dies: perform insert and delete
			else {
				// insert tuple1 from die1 into die2 w/ offset tuple2
				this->dies[die2]->CBL.S.insert(this->dies[die2]->CBL.S.begin() + tuple2, *(this->dies[die1]->CBL.S.begin() + tuple1));
				this->dies[die2]->CBL.L.insert(this->dies[die2]->CBL.L.begin() + tuple2, *(this->dies[die1]->CBL.L.begin() + tuple1));
				this->dies[die2]->CBL.T.insert(this->dies[die2]->CBL.T.begin() + tuple2, *(this->dies[die1]->CBL.T.begin() + tuple1));
				// erase tuple1 from die1
				this->dies[die1]->CBL.S.erase(this->dies[die1]->CBL.S.begin() + tuple1);
				this->dies[die1]->CBL.L.erase(this->dies[die1]->CBL.L.begin() + tuple1);
				this->dies[die1]->CBL.T.erase(this->dies[die1]->CBL.T.begin() + tuple1);
			}

			if (DBG_CORE) {
				cout << "DBG_CORE> moveTuples; d1=" << die1 << ", d2=" << die2 << ", t1=" << tuple1 << ", t2=" << tuple2 << endl;
			}
		};
		inline void switchTupleDirection(int const& die, int const& tuple) const {
			if (this->dies[die]->CBL.L[tuple] == Direction::VERTICAL) {
				this->dies[die]->CBL.L[tuple] = Direction::HORIZONTAL;
			}
			else {
				this->dies[die]->CBL.L[tuple] = Direction::VERTICAL;
			}

			if (DBG_CORE) {
				cout << "DBG_CORE> switchTupleDirection; d1=" << die << ", t1=" << tuple << endl;
			}
		};
		inline void switchTupleJunctions(int const& die, int const& tuple, int const& juncts) const {
			this->dies[die]->CBL.T[tuple] = juncts;

			if (DBG_CORE) {
				cout << "DBG_CORE> switchTupleJunctions; d1=" << die << ", t1=" << tuple << ", juncts=" << juncts << endl;
			}
		};
		inline void switchBlockOrientation(int const& die, int const& tuple) const {

			swap(this->dies[die]->CBL.S[tuple]->bb.w, this->dies[die]->CBL.S[tuple]->bb.h);

			if (DBG_CORE) {
				cout << "DBG_CORE> switchBlockOrientation; d1=" << die << ", t1=" << tuple << endl;
			}
		};

		// CBL logging
		inline string CBLsString() const {
			stringstream ret;

			ret << "# tuple format: ( BLOCK_ID DIRECTION T-JUNCTS BLOCK_WIDTH BLOCK_HEIGHT )" << endl;
			ret << "data_start" << endl;

			for (CorblivarDie* const& die : this->dies) {
				ret << "CBL [ " << die->id << " ]" << endl;
				ret << die->CBL.CBLString() << endl;
			}

			return ret.str();
		};

		// CBL backup handler
		inline void backupCBLs() const {

			for (CorblivarDie* const& die : this->dies) {

				die->CBLbackup.clear();
				die->CBLbackup.reserve(die->CBL.capacity());

				for (Block* const& b : die->CBL.S) {
					// backup block dimensions (block shape) into
					// block itself
					b->bb_backup = b->bb;
					die->CBLbackup.S.push_back(b);
				}
				for (Direction const& dir : die->CBL.L) {
					die->CBLbackup.L.push_back(dir);
				}
				for (unsigned const& t_juncts : die->CBL.T) {
					die->CBLbackup.T.push_back(t_juncts);
				}
			}
		};
		inline void restoreCBLs() const {

			for (CorblivarDie* const& die : this->dies) {

				die->CBL.clear();
				die->CBL.reserve(die->CBLbackup.capacity());

				for (Block* const& b : die->CBLbackup.S) {
					// restore block dimensions (block shape) from
					// block itself
					b->bb = b->bb_backup;
					die->CBL.S.push_back(b);
				}
				for (Direction const& dir : die->CBLbackup.L) {
					die->CBL.L.push_back(dir);
				}
				for (unsigned const& t_juncts : die->CBLbackup.T) {
					die->CBL.T.push_back(t_juncts);
				}
			}
		};

		// CBL best-solution handler
		inline void storeBestCBLs() const {

			for (CorblivarDie* const& die : this->dies) {

				die->CBLbest.clear();
				die->CBLbest.reserve(die->CBL.capacity());

				for (Block* const& b : die->CBL.S) {
					// backup block dimensions (block shape) into
					// block itself
					b->bb_best = b->bb;
					die->CBLbest.S.push_back(b);
				}
				for (Direction const& dir : die->CBL.L) {
					die->CBLbest.L.push_back(dir);
				}
				for (unsigned const& t_juncts : die->CBL.T) {
					die->CBLbest.T.push_back(t_juncts);
				}
			}
		};
		inline bool applyBestCBLs(bool const& log) const {

			for (CorblivarDie* const& die : this->dies) {

				// sanity check for existence of best solution
				if (die->CBLbest.empty()) {
					if (log) {
						cout << "Corblivar> No best (fitting) solution available!" << endl;
					}
					return false;
				}

				die->CBL.clear();
				die->CBL.reserve(die->CBLbest.capacity());

				for (Block* const& b : die->CBLbest.S) {
					// restore block dimensions (block shape) from
					// block itself
					b->bb = b->bb_best;
					die->CBL.S.push_back(b);
				}
				for (Direction const& dir : die->CBLbest.L) {
					die->CBL.L.push_back(dir);
				}
				for (unsigned const& t_juncts : die->CBLbest.T) {
					die->CBL.T.push_back(t_juncts);
				}
			}

			return true;
		};
};

#endif
