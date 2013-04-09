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
#ifndef _CORBLIVAR_DIE
#define _CORBLIVAR_DIE

// library includes
#include "Corblivar.incl.hpp"
// Corblivar includes, if any
#include "CornerBlockList.hpp"
// forward declarations, if any
class Block;

class CorblivarDie {
	// debugging code switch (private)
	private:

	// private data, functions
	private:
		int id;
		// progress flags
		bool stalled;
		bool done;

		// progress pointer, CBL vector index
		unsigned pi;

		// placement stacks
		stack<Block const*> Hi, Vi;

		// main CBL sequence
		CornerBlockList CBL;

		// backup CBL sequences
		CornerBlockList CBLbackup, CBLbest;

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

	// constructors, destructors, if any non-implicit
	public:
		CorblivarDie(int const& i) {
			stalled = done = false;
			id = i;
		}

	// public data, functions
	public:
		friend class CorblivarCore;

		// layout generation functions
		//
		// may return nullptr
		Block const* placeCurrentBlock(bool const& dbgStack = false);

		// getter
		inline CornerBlockList const& getCBL() const {
			return this->CBL;
		};

		inline CornerBlockList& editCBL() {
			return this->CBL;
		};

		inline CornerBlockList::Tuple currentTuple() const {
			CornerBlockList::Tuple ret;

			ret.S = this->CBL.S[this->pi];
			ret.L = this->CBL.L[this->pi];
			ret.T = this->CBL.T[this->pi];

			return ret;
		};

		inline CornerBlockList::Tuple getTuple(unsigned const& tuple) const {
			CornerBlockList::Tuple ret;

			ret.S = this->CBL.S[tuple];
			ret.L = this->CBL.L[tuple];
			ret.T = this->CBL.T[tuple];

			return ret;
		};

		inline string currentTupleString() const {
			return this->CBL.tupleString(this->pi);
		};
};

#endif
