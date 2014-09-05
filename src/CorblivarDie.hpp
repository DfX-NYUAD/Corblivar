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
#ifndef _CORBLIVAR_DIE
#define _CORBLIVAR_DIE

// library includes
#include "Corblivar.incl.hpp"
// Corblivar includes, if any
#include "CornerBlockList.hpp"
#include "CorblivarAlignmentReq.hpp"
#include "Coordinate.hpp"
// forward declarations, if any
class Block;

class CorblivarDie {
	// debugging code switch (private)
	private:
		static constexpr bool DBG_STACKS = false;

	// private data, functions
	private:
		int id;
		// progress flags
		bool stalled;
		bool done;

		// progress pointer, CBL vector index
		unsigned pi;

		// placement stacks; for efficiency implemented as list
		std::list<Block const*> Hi, Vi;

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
			this->Hi.clear();
			this->Vi.clear();

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
		void placeCurrentBlock(bool const& alignment_enabled);
		// layout generation: block shifting
		bool shiftCurrentBlock(Direction const& dir, CorblivarAlignmentReq const* req, bool const& dry_run = false);

		// layout-generation helper: determine coordinates of block in process
		void determCurrentBlockCoords(Coordinate const& coord, std::list<Block const*> const& relev_blocks_stack, bool const& extended_check = false) const;
		// layout-generation helper: pop relevant blocks to consider during
		// placement from stacks
		std::list<Block const*> popRelevantBlocks();
		// layout-generation helper: update placement stack (after placement)
		void updatePlacementStacks(std::list<Block const*>& relev_blocks_stack);
		// layout-generation helper: rebuild placement stack (after block shifting)
		void rebuildPlacementStacks(std::list<Block const*>& relev_blocks_stack);
		// layout-generation helper: placement stacks debugging
		void debugStacks();

	// constructors, destructors, if any non-implicit
	public:
		CorblivarDie(int const& id) {
			this->stalled = false;
			this->done = false;
			this->id = id;
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
		inline std::vector<Block const*> const& getBlocks() const {
			return this->CBL.S;
		};
		inline Block const* getBlock(unsigned const& tuple) const {
			return this->CBL.S[tuple];
		};
		inline Block const* getCurrentBlock() const {
			return this->CBL.S[this->pi];
		};
		inline Direction const& getCurrentDirection() const {
			return this->CBL.L[this->pi];
		};
		inline unsigned const& getJunctions(unsigned const& tuple) const {
			return this->CBL.T[tuple];
		};
		inline int getTuple(Block const* block) const {
			int index;

			for (index = 0; index < static_cast<int>(this->CBL.S.size()); index++) {
				if (block->id == this->CBL.S[index]->id) {
					return index;
				}
			}
			return -1;
		};

		// layout generation: packing, to be performed as post-placement operation
		void performPacking(Direction const& dir);

		// layout-generation helper: sanity check and debugging for valid layout,
		// i.e., overlap-free block arrangement
		bool debugLayout() const;
};

#endif
