/*
 * =====================================================================================
 *
 *    Description:  Corblivar contiguity analysis, required for multiple-voltages feature
 *
 *    Copyright (C) 2015 Johann Knechtel, johann.knechtel@ifte.de, www.ifte.de
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
#ifndef _CORBLIVAR_CONTIGUITY
#define _CORBLIVAR_CONTIGUITY

// library includes
#include "Corblivar.incl.hpp"
// Corblivar includes, if any
#include "Point.hpp"
#include "Math.hpp"
// forward declarations, if any
class Block;

class ContiguityAnalysis {
	// debugging code switch (private)
	private:
		static constexpr bool DBG = false;

	// public data
	public:

	// PODs, to be declared early on
	public:
		struct ContiguousNeighbour {

			Block const* block;

			// (TODO) drop; not required as of now
//			// common boundaries; encode the intersecting range of abutting
//			// blocks
//			//
//			// by definition, this value is smaller than zero for the
//			// neighbour being left of the current block, and greater than
//			// zero for the neighbour being right of the current block
//			double common_boundary_vert = 0.0;
//			// by definition, this value is smaller than zero for the
//			// neighbour being below of the current block, and greater than
//			// zero for the neighbour being above of the current block
//			double common_boundary_hor = 0.0;
//
//			// these values are similar to the horizontal and vertical above,
//			// but may only refer to the neighbour block being stacked above
//			// the current block, in the next upper die
//			double common_boundary_inter_die_vert = 0.0;
//			double common_boundary_inter_die_hor = 0.0;
		};

		struct Boundary {

			Block const* block;
			
			Point low;
			Point high;
		};

		// these data structures are used for intra-die contiguity analysis, as
		// well as for checking MultipleVoltages::CompoundModules for intrusions
		// of any block (implemented in
		// MultipleVoltages::CompoundModule::updateOutlineCost); thus the data is public
		std::vector< std::vector<ContiguityAnalysis::Boundary> > boundaries_hor;
		std::vector< std::vector<ContiguityAnalysis::Boundary> > boundaries_vert;

	// private data, functions
	private:
		inline static bool boundaries_vert_comp(Boundary const& b1, Boundary const& b2);
		inline static bool boundaries_hor_comp(Boundary const& b1, Boundary const& b2);

		// (TODO) drop; not required as of now
//		inline static double common_boundary_vert(Boundary const& b1, Boundary const& b2);
//		inline static double common_boundary_hor(Boundary const& b1, Boundary const& b2);
//		inline static double common_boundary_hor(Block const* b1, Block const* b2);

	// constructors, destructors, if any non-implicit
	public:

	// public data, functions
	public:
		void analyseBlocks(int layers, std::vector<Block> const& blocks);
};

#endif
