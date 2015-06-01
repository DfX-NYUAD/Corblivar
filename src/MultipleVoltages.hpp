/*
 * =====================================================================================
 *
 *    Description:  Corblivar handler for multiple voltages
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
#ifndef _CORBLIVAR_MULTIPLEVOLTAGES
#define _CORBLIVAR_MULTIPLEVOLTAGES

// library includes
#include "Corblivar.incl.hpp"
// Corblivar includes, if any
#include "ContiguityAnalysis.hpp"
// forward declarations, if any
class Block;

class MultipleVoltages {
	// debugging code switch (private)
	private:
		static constexpr bool DBG = false;

	// public constants
	public:
		// dimension for feasible voltages;
		// represents the upper bound for globally available voltages
		static constexpr int MAX_VOLTAGES = 4;

	// PODs, to be declared early on
	public:
		struct CompoundModule {

			// TODO may be not required
			//std::vector<Block const*> blocks;

			// used to identify compound modules; since set is sorted, the
			// order of blocks added doesn't matter, each compound module is
			// unique in terms of blocks considered
			std::set<std::string> block_ids;

			std::bitset<MAX_VOLTAGES> feasible_voltages;
			std::vector<ContiguityAnalysis::ContiguousNeighbour> contiguous_neighbours;

			// TODO some terms of cost for power-domain routing, preferably
			// derived from contiguity analysis
		};

	// public data
	public:
		// set of unique compound modules
		std::unordered_map< std::string, CompoundModule> modules;

	// private data, functions
	private:

	// constructors, destructors, if any non-implicit
	public:

	// public data, functions
	public:
		void determineCompoundModules(int layers, std::vector<Block> const& blocks);
};

#endif
