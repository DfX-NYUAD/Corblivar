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

// own Corblivar header
#include "MultipleVoltages.hpp"
// required Corblivar headers
#include "Block.hpp"
#include "ContiguityAnalysis.hpp"

// TODO consider compound modules across dies; this requires to determine contiguity
// across dies first, see ContiguityAnalysis::analyseBlocks
void MultipleVoltages::determineCompoundModules(std::vector<Block> const& blocks) {

	this->modules.clear();

	// consider each block as starting point for a compound module
	for (Block const& start : blocks) {

		// init the trivial compound module, containing only the block itself
		MultipleVoltages::CompoundModule module;
		// copy feasible voltages
		module.feasible_voltages = start.feasible_voltages;
		// init sorted set of block ids, they will be used for key generation of
		// compound modules
		module.block_ids.insert(start.id);
		// init neighbours; pointers to block's neighbour is sufficient
		for (auto& neighbour : start.contiguous_neighbours) {
			module.contiguous_neighbours.insert({neighbour.block->id, &neighbour});
		}

		// stepwise, recursive consideration of all blocks for merging into
		// compound module
		this->buildCompoundModulesHelper(module);

		// store trivial compound module; the module can be moved now, it will be
		// accessed from the map later on
		this->modules.insert({start.id, std::move(module)});
	}

	// fill container with sorted compound modules; consider only pointers to actual
	// modules
	this->modules_sorted.clear();
	for (auto it = this->modules.begin(); it != this->modules.end(); ++it) {
		// note that the number of comprised blocks can be retrieved from the size
		// of block_ids
		this->modules_sorted.insert({it->second.block_ids.size(), &(it->second)});
	}

	if (MultipleVoltages::DBG) {

		std::cout << "DBG_VOLTAGES> Compound modules (in total " << this->modules.size() << "); view ordered by number of comprised blocks:" << std::endl;

		for (auto it = this->modules_sorted.begin(); it != this->modules_sorted.end(); ++it) {

			std::cout << "DBG_VOLTAGES>  Module;" << std::endl;
			std::cout << "DBG_VOLTAGES>   Comprised blocks #: " << it->first << std::endl;
			std::cout << "DBG_VOLTAGES>   Comprised blocks ids: ";
			for (auto& id : it->second->block_ids) {
				std:: cout << id << ", ";
			}
			std::cout << std::endl;

			std::cout << "DBG_VOLTAGES>   Module voltages bitset: " << it->second->feasible_voltages << std::endl;
		}
	}
}

// stepwise consider adding single blocks into the compound module until all blocks are
// considered; note that this implies recursive calls to determine transitive neighbours
void MultipleVoltages::buildCompoundModulesHelper(MultipleVoltages::CompoundModule& module) {
	std::bitset<MAX_VOLTAGES> feasible_voltages;
	ContiguityAnalysis::ContiguousNeighbour* neighbour;
	std::pair<
		std::unordered_map< std::string, CompoundModule>::iterator,
		bool> new_module_insertion;

	// walk all current neighbours
	for (auto it = module.contiguous_neighbours.begin(); it != module.contiguous_neighbours.end(); ++it) {

		neighbour = it->second;

		// first, determine if adding this neighbour would lead to an trivial
		// solution, i.e., only the highest possible voltage is assignable; such
		// modules are ignored and thus we can achieve notable reduction in memory
		// and runtime by pruning trivial solutions early on during recursive
		// bottom-up phase
		//
		// bit-wise AND to obtain the intersection of feasible voltages
		feasible_voltages = module.feasible_voltages & neighbour->block->feasible_voltages;

		// only one voltage is feasible, which is trivially the highest possible;
		// ignore this potential compound module
		if (feasible_voltages.count() == 1) {
			continue;
		}
		// more than one voltage is feasible; generate the related compound module
		else {
			// init the new compound module, comprising the previous module
			// and the relevant neigbhour
			MultipleVoltages::CompoundModule new_module;

			// move feasible voltages
			new_module.feasible_voltages = std::move(feasible_voltages);

			// init sorted set of block ids with copy from previous compound
			// module
			new_module.block_ids = module.block_ids;
			// add id of now additionally considered block
			new_module.block_ids.insert(neighbour->block->id);

			// init pointers to neighbours with copy from previous compound
			// module
			new_module.contiguous_neighbours = module.contiguous_neighbours;
			// add (pointers to) neighbours of now additionally considered
			// block; note that only non yet considered neighbours are added
			// effectively into the map
			for (auto& n : neighbour->block->contiguous_neighbours) {
				new_module.contiguous_neighbours.insert({n.block->id, &n});
			}

			// copy the separate (sorted) block ids into one large string,
			// which defines the unique key for this compound module
			std::string compound_id;
			for (std::string id : new_module.block_ids) {
				compound_id += id + ",";
			}

			// store new compound module; note that it is only inserted if not
			// existing before; this avoids storage of redundant modules for
			// commutative orders of blocks, which will arise from different
			// start points / initial modules; e.g., a compound module of
			// "sb1,sb2" is the same as "sb2,sb1", and by 1) sorting the block
			// ids and 2) inserting compound modules into a map, "sb2,sb1" is
			// effectively ignored
			//
			new_module_insertion = this->modules.insert(std::make_pair<std::string, MultipleVoltages::CompoundModule>(
						std::move(compound_id),
						std::move(new_module)
					));

			// only if this compound module was successfully inserted, i.e.,
			// not already previously inserted, we also consider it for
			// recursive calls, to determine next-level compound modules by
			// considering the currently added block's neighbours
			if (new_module_insertion.second) {
				// first: iterator to just into this->modules inserted
				// element;
				// first->second: actual reference to compound module
				this->buildCompoundModulesHelper(new_module_insertion.first->second);
			}
		}
	}
}
