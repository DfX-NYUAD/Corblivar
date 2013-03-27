/*
 * =====================================================================================
 *
 *    Description:  Corblivar core header (data structures, layout operations)
 *
 *         Author:  Johann Knechtel, johann.knechtel@ifte.de
 *        Company:  Institute of Electromechanical and Electronic Design, www.ifte.de
 *
 * =====================================================================================
 */
#ifndef _CORBLIVAR_CORE_HPP
#define _CORBLIVAR_CORE_HPP

// debugging code switch
static constexpr bool DBG_CORB = false;

// CBL placement direction
class Direction {
	public:
		static constexpr unsigned DIRECTION_VERT = 0;
		static constexpr unsigned DIRECTION_HOR = 1;
};

class CornerBlockList {
	private:
		// CBL sequences
		mutable vector<Block*> S;
		mutable vector<unsigned> L;
		mutable vector<unsigned> T;

	public:
		friend class CorblivarCore;
		friend class CorblivarDie;

		// getter / setter
		inline unsigned size() const {

			if (DBG_CORB) {
				unsigned ret;
				bool mismatch = false;
				unsigned prev_ret;

				prev_ret = ret = this->S.size();
				ret = min(ret, this->L.size());
				mismatch = (ret != prev_ret);
				prev_ret = ret;
				ret = min(ret, this->T.size());
				mismatch = mismatch || (ret != prev_ret);

				if (mismatch) {
					cout << "DBG_CORB> CBL has sequences size mismatch!" << endl;
					cout << "DBG_CORB> CBL: " << endl;
					cout << this->itemString() << endl;
				}
			}

			return this->S.size();
		};

		inline unsigned capacity() const {
			return this->S.capacity();
		};

		inline bool empty() const {
			return this->S.empty();
		};

		inline void clear() {
			this->S.clear();
			this->L.clear();
			this->T.clear();
		};

		inline void reserve(unsigned const& elements) {
			this->S.reserve(elements);
			this->L.reserve(elements);
			this->T.reserve(elements);
		};

		inline void S_push_back(Block* const& block) const {
			this->S.push_back(block);
		};

		inline void L_push_back(unsigned const& l) const {
			this->L.push_back(l);
		};

		inline void T_push_back(unsigned const& t) const {
			this->T.push_back(t);
		};

		inline string itemString(unsigned const& i) const {
			stringstream ret;

			ret << "( " << S[i]->id << " " << L[i] << " " << T[i] << " " << S[i]->bb.w << " " << S[i]->bb.h << " )";

			return ret.str();
		};

		inline string itemString() const {
			unsigned i;
			stringstream ret;

			for (i = 0; i < this->size(); i++) {
				ret << this->itemString(i) << " , ";
			}

			return ret.str();
		};
};

class CorblivarDie {
	private:
		int id;
		// progress flags
		bool stalled;
		bool done;

		// progress pointer, CBL vector index
		unsigned pi;

		// placement stacks
		stack<Block*> Hi, Vi;

		// main CBL sequence
		CornerBlockList CBL;

		// backup CBL sequences
		CornerBlockList CBLbackup, CBLbest;

		// false: last CBL tuple; true: not last tuple
		inline bool incrementTuplePointer() {

			if (this->pi == (CBL.size() - 1)) {
				this->done = true;
				return false;
			}
			else {
				this->pi++;
				return true;
			}
		};

		inline void reset() {
			// reset progress pointer
			this->pi = 0;
			// reset done flag
			this->done = false;
			// reset placement stacks
			while (!this->Hi.empty()) {
				this->Hi.pop();
			}
			while (!this->Vi.empty()) {
				this->Vi.pop();
			}
		};

	public:
		friend class CorblivarCore;

		CorblivarDie(int const& i) {
			stalled = done = false;
			id = i;
		}

		// layout generation functions
		Block* placeCurrentBlock(bool const& dbgStack = false);

		// getter
		inline CornerBlockList const& getCBL() const {
			return this->CBL;
		};

		inline Block* const& currentBlock() const {
			return this->CBL.S[this->pi];
		};

		inline unsigned const& currentTupleDirection() const {
			return this->CBL.L[this->pi];
		};

		inline unsigned const& currentTupleJuncts() const {
			return this->CBL.T[this->pi];
		};

		inline unsigned const& tupleJuncts(unsigned const& tuple) const {
			return this->CBL.T[tuple];
		};

		inline string currentTupleString() const {
			return this->CBL.itemString(this->pi);
		};
};

class CorblivarCore {
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
		static constexpr int OP_SWAP_BLOCKS_WI_DIE = 0;
		static constexpr int OP_SWAP_BLOCKS_ACROSS_DIE = 1;
		static constexpr int OP_MOVE_TUPLE = 2;
		static constexpr int OP_SWITCH_TUPLE_DIR = 3;
		static constexpr int OP_SWITCH_TUPLE_JUNCTS = 4;
		static constexpr int OP_SWITCH_BLOCK_ORIENT = 5;

		inline void switchBlocksWithinDie(int const& die, int const& tuple1, int const& tuple2) const {
			swap(this->dies[die]->CBL.S[tuple1], this->dies[die]->CBL.S[tuple2]);

			if (DBG_CORB) {
				cout << "DBG_CORB> switchBlocksWithinDie; d1=" << die;
				cout << ", s1=" << this->dies[die]->CBL.S[tuple1]->id;
				cout << ", s2=" << this->dies[die]->CBL.S[tuple2]->id << endl;
			}
		};
		inline void switchBlocksAcrossDies(int const& die1, int const& die2, int const& tuple1, int const& tuple2) const {
			swap(this->dies[die1]->CBL.S[tuple1], this->dies[die2]->CBL.S[tuple2]);

			if (DBG_CORB) {
				cout << "DBG_CORB> switchBlocksAcrossDies; d1=" << die1 << ", d2=" << die2;
				cout << ", s1=" << this->dies[die1]->CBL.S[tuple1]->id;
				cout << ", s2=" << this->dies[die2]->CBL.S[tuple2]->id << endl;
			}
		};
		inline void moveTupleAcrossDies(int const& die1, int const& die2, int const& tuple1, int const& tuple2) const {

			// insert tuple1 from die1 into die2 w/ offset tuple2
			this->dies[die2]->CBL.S.insert(this->dies[die2]->CBL.S.begin() + tuple2, *(this->dies[die1]->CBL.S.begin() + tuple1));
			this->dies[die2]->CBL.L.insert(this->dies[die2]->CBL.L.begin() + tuple2, *(this->dies[die1]->CBL.L.begin() + tuple1));
			this->dies[die2]->CBL.T.insert(this->dies[die2]->CBL.T.begin() + tuple2, *(this->dies[die1]->CBL.T.begin() + tuple1));
			// erase tuple1 from die1
			this->dies[die1]->CBL.S.erase(this->dies[die1]->CBL.S.begin() + tuple1);
			this->dies[die1]->CBL.L.erase(this->dies[die1]->CBL.L.begin() + tuple1);
			this->dies[die1]->CBL.T.erase(this->dies[die1]->CBL.T.begin() + tuple1);

			if (DBG_CORB) {
				cout << "DBG_CORB> moveTupleAcrossDies; d1=" << die1 << ", d2=" << die2 << ", t1=" << tuple1 << ", t2=" << tuple2 << endl;
			}
		};
		inline void switchTupleDirection(int const& die, int const& tuple) const {
			if (this->dies[die]->CBL.L[tuple] == Direction::DIRECTION_VERT) {
				this->dies[die]->CBL.L[tuple] = Direction::DIRECTION_HOR;
			}
			else {
				this->dies[die]->CBL.L[tuple] = Direction::DIRECTION_VERT;
			}

			if (DBG_CORB) {
				cout << "DBG_CORB> switchTupleDirection; d1=" << die << ", t1=" << tuple << endl;
			}
		};
		inline void switchTupleJunctions(int const& die, int const& tuple, int const& juncts) const {
			this->dies[die]->CBL.T[tuple] = juncts;

			if (DBG_CORB) {
				cout << "DBG_CORB> switchTupleJunctions; d1=" << die << ", t1=" << tuple << ", juncts=" << juncts << endl;
			}
		};
		inline void switchBlockOrientation(int const& die, int const& tuple) const {
			double w_tmp;

			w_tmp = this->dies[die]->CBL.S[tuple]->bb.w;
			this->dies[die]->CBL.S[tuple]->bb.w = this->dies[die]->CBL.S[tuple]->bb.h;
			this->dies[die]->CBL.S[tuple]->bb.h = w_tmp;

			if (DBG_CORB) {
				cout << "DBG_CORB> switchBlockOrientation; d1=" << die << ", t1=" << tuple << endl;
			}
		};

		// CBL logging
		inline string CBLsString() const {
			stringstream ret;

			ret << "# tuple format: ( BLOCK_ID DIRECTION T-JUNCTS BLOCK_WIDTH BLOCK_HEIGHT )" << endl;
			ret << "data_start" << endl;

			for (CorblivarDie* const& die : this->dies) {
				ret << "CBL [ " << die->id << " ]" << endl;
				ret << die->CBL.itemString() << endl;
			}

			return ret.str();
		};

		// CBL backup handler
		inline void backupCBLs() const {

			for (CorblivarDie* const& die : this->dies) {

				die->CBLbackup.clear();
				die->CBLbackup.reserve(die->CBL.capacity());

				for (Block* &b : die->CBL.S) {
					// backup block dimensions (block shape) into
					// block itself
					b->bb_backup = b->bb;
					die->CBLbackup.S.push_back(b);
				}
				for (unsigned const& dir : die->CBL.L) {
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

				for (Block* &b : die->CBLbackup.S) {
					// restore block dimensions (block shape) from
					// block itself
					b->bb = b->bb_backup;
					die->CBL.S.push_back(b);
				}
				for (unsigned const& dir : die->CBLbackup.L) {
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

				for (Block* &b : die->CBL.S) {
					// backup block dimensions (block shape) into
					// block itself
					b->bb_best = b->bb;
					die->CBLbest.S.push_back(b);
				}
				for (unsigned const& dir : die->CBL.L) {
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

				for (Block* &b : die->CBLbest.S) {
					// restore block dimensions (block shape) from
					// block itself
					b->bb = b->bb_best;
					die->CBL.S.push_back(b);
				}
				for (unsigned const& dir : die->CBLbest.L) {
					die->CBL.L.push_back(dir);
				}
				for (unsigned const& t_juncts : die->CBLbest.T) {
					die->CBL.T.push_back(t_juncts);
				}
			}

			return true;
		};
};

class CorblivarAlignmentReq {
	private:
		// TODO realize as constexpr
		enum Alignment {ALIGNMENT_OFFSET, ALIGNMENT_RANGE, ALIGNMENT_UNDEF};

		Block* s_i;
		Block* s_j;
		Alignment type_x, type_y;
		double offset_range_x, offset_range_y;

	public:
		inline bool rangeX() const {
			return (this->type_x == ALIGNMENT_RANGE);
		};
		inline bool rangeY() const {
			return (this->type_y == ALIGNMENT_RANGE);
		};
		inline bool fixedOffsX() const {
			return (this->type_x == ALIGNMENT_OFFSET);
		};
		inline bool fixedOffsY() const {
			return (this->type_y == ALIGNMENT_OFFSET);
		};
		inline string tupleString() const {
			stringstream ret;

			ret << "(" << s_i->id << ", " << s_j->id << ", (" << offset_range_x << ", ";
			if (this->rangeX()) {
				ret << "1";
			}
			else if (this->fixedOffsX()) {
				ret << "0";
			}
			else {
				ret << "lambda";
			}
			ret << "), (" << offset_range_y << ", ";
			if (this->rangeY()) {
				ret << "1";
			}
			else if (this->fixedOffsY()) {
				ret << "0";
			}
			else {
				ret << "lambda";
			}
			ret << ") )";

			return ret.str();
		};

		CorblivarAlignmentReq(Block* si, Block* sj, Alignment typex, double offsetrangex, Alignment typey, double offsetrangey) {
			s_i = si;
			s_j = sj;
			type_x = typex;
			type_y = typey;
			offset_range_x = offsetrangex;
			offset_range_y = offsetrangey;

			// fix invalid negative range
			if ((this->rangeX() && offset_range_x < 0) || (this->rangeY() && offset_range_y < 0)) {
				cout << "CorblivarAlignmentReq> ";
				cout << "Fixing tuple (negative range):" << endl;
				cout << " " << this->tupleString() << " to" << endl;

				if (offset_range_x < 0) {
					offset_range_x = 0;
				}
				if (offset_range_y < 0) {
					offset_range_y = 0;
				}

				cout << " " << this->tupleString() << endl;
			}
		};
};

#endif
