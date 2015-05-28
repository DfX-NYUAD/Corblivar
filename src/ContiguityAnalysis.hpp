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
			double common_boundary_x;
			double common_boundary_y;
			double common_boundary_z;
		};

	// private data, functions
	private:

	// constructors, destructors, if any non-implicit
	public:

	// public data, functions
	public:
		static void analyseBlocks(std::vector<Block> const& blocks);
};

#endif
