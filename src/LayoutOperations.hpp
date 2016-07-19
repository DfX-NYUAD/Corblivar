/**
 * =====================================================================================
 *
 *    Description:  Corblivar layout operations
 *
 *    Copyright (C) 2013-2016 Johann Knechtel, johann aett jknechtel dot de
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
#include "Point.hpp"
// forward declarations, if any
class CorblivarCore;
class Block;
class Net;

/// Corblivar layout operations
class LayoutOperations {
	private:
		/// debugging code switch
		static constexpr bool DBG = false;

	// constructors, destructors, if any non-implicit
	public:

	// public data, functions
	public:
		/// layout-operation handler
		bool performLayoutOp(CorblivarCore& corb,
				int const& layout_fit_counter = 0,
				bool const& SA_phase_two = false,
				bool const& revertLastOp = false,
				bool const& cooling_phase_three = false
			);

		/// layout-operation parameters
		struct parameters {

			/// optimization flag; parsed along w/ FloorPlanner::SA_parameters
			/// in IO::parseParametersFiles
			bool opt_alignment;

			/// layer count; parsed along w/ FloorPlanner::IC in
			/// IO::parseParametersFiles
			int layers;

			/// outline; parsed along w/ FloorPlanner::IC in
			/// IO::parseParametersFiles
			Point outline;

			/// layout generation options; parsed in IO::parseParametersFiles
			bool enhanced_hard_block_rotation, enhanced_soft_block_shaping;
			/// layout generation options; parsed in IO::parseParametersFiles
			bool power_aware_block_handling, floorplacement, shrink_die, trivial_HPWL, signal_TSV_clustering;
			/// layout generation options; parsed in IO::parseParametersFiles
			int packing_iterations;

			/// block-selection guidance; the currently largest individual net;
			/// this net and the related modules are of particular interest to
			/// be rearranged; this parameter is updated during
			/// FloorPlanner::evaluateInterconnects
			Net const* largest_net = nullptr;
		} parameters;

	// private data, functions
	private:
		/// layout operations op-codes
		static constexpr int OP_SWAP_BLOCKS = 1;
		/// layout operations op-codes
		static constexpr int OP_MOVE_TUPLE = 2;
		/// layout operations op-codes
		static constexpr int OP_SWITCH_INSERTION_DIR = 3;
		/// layout operations op-codes
		static constexpr int OP_SWITCH_TUPLE_JUNCTS = 4;
		/// layout operations op-codes
		static constexpr int OP_ROTATE_BLOCK__SHAPE_BLOCK = 5;
		/// layout operations op-codes
		/// used only for soft block shaping
		static constexpr int OP_SHAPE_BLOCK__STRETCH_HORIZONTAL = 10;
		/// layout operations op-codes
		/// used only for soft block shaping
		static constexpr int OP_SHAPE_BLOCK__STRETCH_VERTICAL = 11;
		/// layout operations op-codes
		/// used only for soft block shaping
		static constexpr int OP_SHAPE_BLOCK__SHRINK_HORIZONTAL = 12;
		/// layout operations op-codes
		/// used only for soft block shaping
		static constexpr int OP_SHAPE_BLOCK__SHRINK_VERTICAL = 13;
		/// layout operations op-codes
		/// used only for soft block shaping
		static constexpr int OP_SHAPE_BLOCK__RANDOM_AR = 14;
		/// layout operations op-codes
		/// used only for blocks related to failed alignment request
		static constexpr int OP_SWAP_BLOCKS_ENFORCE= 20;
		/// layout operations op-codes
		static constexpr int OP_SWAP_ALIGNMENT_COORDINATES = 21;

		/// layout-operation handler variables
		mutable int last_op, last_op_die1, last_op_die2, last_op_tuple1, last_op_tuple2, last_op_juncts;
		/// layout-operation handler
		/// note that die and tuple parameters are return-by-reference; non-const
		/// reference for CorblivarCore in order to enable operations on CBL-encode data
		inline bool prepareBlockSwappingFailedAlignment(CorblivarCore const& corb, int& die1, int& tuple1, int& die2, int& tuple2);
		/// layout-operation handler
		/// note that die and tuple parameters are return-by-reference; non-const
		/// reference for CorblivarCore in order to enable operations on CBL-encode data
		inline bool prepareSwappingCoordinatesFailedAlignment(CorblivarCore const& corb, int& tuple1);
		/// layout-operation handler
		/// note that die and tuple parameters are return-by-reference; non-const
		/// reference for CorblivarCore in order to enable operations on CBL-encode data
		inline bool performOpMoveOrSwapBlocks(int const& mode, bool const& revert, bool const& SA_phase_one, CorblivarCore& corb,
				int& die1, int& die2, int& tuple1, int& tuple2) const;
		/// layout-operation handler
		/// note that die and tuple parameters are return-by-reference; non-const
		/// reference for CorblivarCore in order to enable operations on CBL-encode data
		inline bool performOpSwitchInsertionDirection(bool const& revert, CorblivarCore& corb, int& die1, int& tuple1) const;
		/// layout-operation handler
		/// note that die and tuple parameters are return-by-reference; non-const
		/// reference for CorblivarCore in order to enable operations on CBL-encode data
		inline bool performOpSwitchTupleJunctions(bool const& revert, CorblivarCore& corb, int& die1, int& tuple1, int& juncts) const;
		/// layout-operation handler
		/// note that die and tuple parameters are return-by-reference; non-const
		/// reference for CorblivarCore in order to enable operations on CBL-encode data
		inline bool performOpShapeBlock(bool const& revert, CorblivarCore& corb, int& die1, int& tuple1) const;
		/// layout-operation handler
		/// note that die and tuple parameters are return-by-reference; non-const
		/// reference for CorblivarCore in order to enable operations on CBL-encode data
		inline bool performOpEnhancedHardBlockRotation(CorblivarCore const& corb, Block const* shape_block) const;
		/// layout-operation handler
		/// note that die and tuple parameters are return-by-reference; non-const
		/// reference for CorblivarCore in order to enable operations on CBL-encode data
		inline bool performOpEnhancedSoftBlockShaping(CorblivarCore const& corb, Block const* shape_block) const;
		/// layout-operation handler
		/// note that die and tuple parameters are return-by-reference; non-const
		/// reference for CorblivarCore in order to enable operations on CBL-encode data
		inline bool performOpSwapAlignmentCoordinates(bool const& revert, CorblivarCore& corb, int& tuple1) const;
		/// layout-operation handler
		/// note that die and tuple parameters are return-by-reference; non-const
		/// reference for CorblivarCore in order to enable operations on CBL-encode data
		inline void prepareHandlingOutlineCriticalBlock(CorblivarCore const& corb, int& die1, int& tuple1) const;
		/// layout-operation handler
		/// note that die and tuple parameters are return-by-reference; non-const
		/// reference for CorblivarCore in order to enable operations on CBL-encode data
		inline void preselectBlockFromLargestNet(CorblivarCore const& corb, int& die1, int& tuple1) const;
};

#endif
