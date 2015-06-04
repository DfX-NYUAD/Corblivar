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
		static constexpr bool DBG_FLOORPLAN = true;

		// dimension for feasible voltages;
		// represents the upper bound for globally available voltages
		static constexpr int MAX_VOLTAGES = 4;

	// PODs, to be declared early on
	public:
		struct CompoundModule {

			// used to identify compound modules; since the ids are in a
			// sorted set, the order of blocks added doesn't matter, each
			// compound module is unique in terms of blocks considered , i.e.,
			// different merging steps will results in the same compound
			// module as long as the same set of blocks is underlying
			std::set<std::string> block_ids;

			// TODO pointers to blocks may be required

			// feasible voltages for whole module; defined by intersection of
			// all comprised blocks
			std::bitset<MAX_VOLTAGES> feasible_voltages;

			// key: neighbour block id
			//
			// with an unsorted_map, redundant neighbours which may arise
			// during stepwise build-up of compound modules are ignored
			//
			// TODO if the actual contiguity changes for compound modules,
			// this should be reflected in an own copy of
			// ContiguityAnalysis::ContiguousNeighbour
			std::unordered_map<std::string, ContiguityAnalysis::ContiguousNeighbour*> contiguous_neighbours;

			// TODO some terms of cost for power-domain routing, preferably
			// derived from contiguity analysis
		};

	// private data, functions
	private:
		// note that the comparator has to be implemented as type, for proper map
		// template handling
		//
		// also note that the less-than order is important for providing hints
		// (iterators) during stepwise insertion, where elements to insert will
		// always be larger, thus to be added after the hinting iterator
		struct modules_comp {
			bool operator() (std::set<std::string> const& s1, std::set<std::string> const& s2) const {

				return (
					// size of the sets is the first criterion; this
					// also facilitates stepwise insertion of modules,
					// since based on the set's size (i.e., the number
					// of blocks in the module), a hint for insertion
					// can be given which reduces complexity for
					// actual insertion
					(s1.size() < s2.size())
					// if they have the same size, perform regular
					// (lexicographical) comparison
					|| (s1.size() == s2.size() && s1 < s2)
				       );
			}
		};

	// public data
	public:
		// set of unique compound modules; the respective keys are the (sorted)
		// ids of all comprised blocks
		typedef std::map< std::set<std::string>, CompoundModule, modules_comp> modules_type;
		modules_type modules;

	// constructors, destructors, if any non-implicit
	public:

	// public data, functions
	public:
		void determineCompoundModules(std::vector<Block> const& blocks);

	// private helper functions
	private:
		void buildCompoundModulesHelper(CompoundModule& module, modules_type::iterator hint);
};

#endif
