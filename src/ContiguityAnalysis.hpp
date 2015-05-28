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

			Block const* neighbour;

			// common boundaries; encode the intersecting range of abutting
			// blocks
			//
			// by definition, this value is smaller than zero for the
			// neighbour being left of the current block, and greater than
			// zero for the neighbour being right of the current block
			double common_boundary_hor;
			// by definition, this value is smaller than zero for the
			// neighbour being below of the current block, and greater than
			// zero for the neighbour being above of the current block
			double common_boundary_vert;

			// these values are similar to the horizontal and vertical above,
			// but may only refer to the neighbour block being stacked above
			// the current block, in the next upper die
			double common_boundary_stacked_hor;
			double common_boundary_stacked_vert;
		};

		struct Boundary {

			Block const* block;
			
			Point p1;
			Point p2;
		};

	// private data, functions
	private:
		inline static bool boundaries_hor_comp(Boundary const& b1, Boundary const& b2) {
			return (
					// x-coordinates are the first criterion; note
					// that it's sufficient to compare the first
					// (lower) points
					(b1.p1.x < b2.p1.x)
					// for boundaries with same x-coordinate, resolve
					// equal values by considering the boundaries'
					// y-coordinate
					|| ((b1.p1.x == b2.p1.x) && (b1.p1.y < b2.p1.y))
			       );
		};

		inline static bool boundaries_vert_comp(Boundary const& b1, Boundary const& b2) {
			return (
					// y-coordinates are the first criterion; note
					// that it's sufficient to compare the first
					// (left) points
					(b1.p1.y < b2.p1.y)
					// for boundaries with same y-coordinate, resolve
					// equal values by considering the boundaries'
					// x-coordinate
					|| ((b1.p1.y == b2.p1.y) && (b1.p1.x < b2.p1.x))
			       );
		};


	// constructors, destructors, if any non-implicit
	public:

	// public data, functions
	public:
		static void analyseBlocks(int layers, std::vector<Block> const& blocks);
};

#endif
