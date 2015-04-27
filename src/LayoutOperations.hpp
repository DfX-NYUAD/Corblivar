/*
 * =====================================================================================
 *
 *    Description:  Corblivar layout operations
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
#ifndef _CORBLIVAR_LAYOUT_OPERATIONS
#define _CORBLIVAR_LAYOUT_OPERATIONS

// library includes
#include "Corblivar.incl.hpp"
// Corblivar includes, if any
// forward declarations, if any
class CorblivarCore;
class Block;

class LayoutOperations {
	// debugging code switch
	private:
		static constexpr bool DBG = false;

	// constructors, destructors, if any non-implicit
	public:

	// public data, functions
	public:
		// layout-operation handler
		bool performLayoutOp(CorblivarCore& corb,
				int const& layout_fit_counter = 0,
				bool const& SA_phase_two = false,
				bool const& revertLastOp = false,
				bool const& cooling_phase_three = false
			);

		// layout-operation parameters
		struct parameters {

			// optimization flag; parsed along w/ FloorPlanner::SA_parameters
			// in IO::parseParametersFiles
			bool opt_alignment;

			// layer count; parsed along w/ FloorPlanner::IC in
			// IO::parseParametersFiles
			int layers;

			// layout generation options; parsed in IO::parseParametersFiles
			bool enhanced_hard_block_rotation, enhanced_soft_block_shaping;
			bool power_aware_block_handling, floorplacement, signal_TSV_clustering;
			int packing_iterations;
		} parameters;

	// private data, functions
	private:
		// layout operations op-codes
		static constexpr int OP_SWAP_BLOCKS = 1;
		static constexpr int OP_MOVE_TUPLE = 2;
		static constexpr int OP_SWITCH_INSERTION_DIR = 3;
		static constexpr int OP_SWITCH_TUPLE_JUNCTS = 4;
		static constexpr int OP_ROTATE_BLOCK__SHAPE_BLOCK = 5;
		// used only for soft block shaping
		static constexpr int OP_SHAPE_BLOCK__STRETCH_HORIZONTAL = 10;
		static constexpr int OP_SHAPE_BLOCK__STRETCH_VERTICAL = 11;
		static constexpr int OP_SHAPE_BLOCK__SHRINK_HORIZONTAL = 12;
		static constexpr int OP_SHAPE_BLOCK__SHRINK_VERTICAL = 13;
		static constexpr int OP_SHAPE_BLOCK__RANDOM_AR = 14;
		// used only for blocks related to failed alignment request
		static constexpr int OP_SWAP_BLOCKS_ENFORCE= 20;
		static constexpr int OP_SWAP_ALIGNMENT_COORDINATES = 21;

		// layout-operation handler variables
		mutable int last_op, last_op_die1, last_op_die2, last_op_tuple1, last_op_tuple2, last_op_juncts;
		// note that die and tuple parameters are return-by-reference; non-const
		// reference for CorblivarCore in order to enable operations on CBL-encode data
		inline bool prepareBlockSwappingFailedAlignment(CorblivarCore const& corb, int& die1, int& tuple1, int& die2, int& tuple2);
		inline bool prepareSwappingCoordinatesFailedAlignment(CorblivarCore const& corb, int& tuple1);
		inline bool performOpMoveOrSwapBlocks(int const& mode, bool const& revert, bool const& SA_phase_one, CorblivarCore& corb,
				int& die1, int& die2, int& tuple1, int& tuple2) const;
		inline bool performOpSwitchInsertionDirection(bool const& revert, CorblivarCore& corb, int& die1, int& tuple1) const;
		inline bool performOpSwitchTupleJunctions(bool const& revert, CorblivarCore& corb, int& die1, int& tuple1, int& juncts) const;
		inline bool performOpShapeBlock(bool const& revert, CorblivarCore& corb, int& die1, int& tuple1) const;
		inline bool performOpEnhancedHardBlockRotation(CorblivarCore const& corb, Block const* shape_block) const;
		inline bool performOpEnhancedSoftBlockShaping(CorblivarCore const& corb, Block const* shape_block) const;
		inline bool performOpSwapAlignmentCoordinates(bool const& revert, CorblivarCore& corb, int& tuple1) const;
		inline void determineOutermostBlock(CorblivarCore const& corb, int& die1, int& tuple1) const;
};

#endif
