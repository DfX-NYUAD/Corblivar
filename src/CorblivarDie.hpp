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
#include "Coordinate.hpp"
// forward declarations, if any
class Block;

class CorblivarDie {
	// debugging code switch (private)
	private:
		static constexpr bool DBG = false;

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

		// reset handler
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

			// reset placed flags
			for (Block const* b : this->CBL.S) {
				b->placed = false;
			}
		};

		// handler for progress pointer, flag
		inline void updateProgressPointerFlag() {
			if (this->pi == (this->CBL.size() - 1)) {
				this->done = true;
			}
			else {
				this->pi++;
			}
		}

		// layout generation; place current block
		void placeCurrentBlock();

		// layout-generation helper: determine coordinates of block in process
		void inline determCurrentBlockCoords(Coordinate const& coord, vector<Block const*> relev_blocks_stack) const;
		// layout-generation helper: pop relevant blocks to consider during
		// placement from stacks
		vector<Block const*> inline popRelevantBlocks();
		// layout-generation helper: update placement stack (after placement)
		void inline updatePlacementStacks(vector<Block const*> relev_blocks_stack);

		// layout generation: packing, to be performed as post-placement operation
		void performPacking(Direction const& dir);

	// constructors, destructors, if any non-implicit
	public:
		CorblivarDie(int const& i) {
			stalled = done = false;
			id = i;
		}

	// public data, functions
	public:
		friend class CorblivarCore;

		// setter
		inline CornerBlockList& editCBL() {
			return this->CBL;
		};

		// getter
		inline CornerBlockList const& getCBL() const {
			return this->CBL;
		};
		inline Block const* getBlock(unsigned const& tuple) const {
			return this->CBL.S[tuple];
		};
		inline Block const* getCurrentBlock() const {
			return this->CBL.S[this->pi];
		};
		inline Direction const& getDirection(unsigned const& tuple) const {
			return this->CBL.L[tuple];
		};
		inline unsigned const& getJunctions(unsigned const& tuple) const {
			return this->CBL.T[tuple];
		};
};

#endif
