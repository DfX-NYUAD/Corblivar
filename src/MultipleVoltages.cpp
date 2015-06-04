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
		this->buildCompoundModulesHelper(module, this->modules.begin());

		// store trivial compound module; the module can be moved now, it will be
		// accessed from the map later on
		this->modules.insert({module.block_ids, std::move(module)});
	}

	if (MultipleVoltages::DBG) {

		std::cout << "DBG_VOLTAGES> Compound modules (in total " << this->modules.size() << "); view ordered by number of comprised blocks:" << std::endl;

		for (auto it = this->modules.begin(); it != this->modules.end(); ++it) {

			std::cout << "DBG_VOLTAGES>  Module;" << std::endl;
			std::cout << "DBG_VOLTAGES>   Comprised blocks #: " << it->first.size() << std::endl;
			std::cout << "DBG_VOLTAGES>   Comprised blocks ids: ";
			for (auto& id : it->second.block_ids) {
				std:: cout << id << ", ";
			}
			std::cout << std::endl;

			std::cout << "DBG_VOLTAGES>   Module voltages bitset: ";
			// TODO test case w/ 3 max voltages
			for (unsigned v = 0; v < 3 && v < MultipleVoltages::MAX_VOLTAGES; v++) {
				std::cout << it->second.feasible_voltages[v];
			}
			std::cout << std::endl;
		}
	}
}

// stepwise consider adding single blocks into the compound module until all blocks are
// considered; note that this implies recursive calls to determine transitive neighbours
void MultipleVoltages::buildCompoundModulesHelper(MultipleVoltages::CompoundModule& module, MultipleVoltages::modules_type::iterator hint) {
	std::bitset<MAX_VOLTAGES> feasible_voltages;
	ContiguityAnalysis::ContiguousNeighbour* neighbour;
	MultipleVoltages::modules_type::iterator inserted;
	unsigned modules_before, modules_after;

	// walk all current neighbours
	for (auto it = module.contiguous_neighbours.begin(); it != module.contiguous_neighbours.end(); ++it) {

		neighbour = it->second;

		// first, we determine if adding this neighbour would lead to an trivial
		// solution, i.e., only the highest possible voltage is assignable; such
		// modules are ignored and thus we can achieve notable reduction in memory
		// and runtime by pruning trivial solutions early on during recursive
		// bottom-up phase
		//
		// bit-wise AND to obtain the intersection of feasible voltages
		feasible_voltages = module.feasible_voltages & neighbour->block->feasible_voltages;

		// only one voltage is feasible, which is trivially the highest possible;
		// ignore this trivial compound module
		if (feasible_voltages.count() == 1) {
			continue;
		}
		// more than one voltage is feasible; generate the potential compound module
		else {
			// first, we have to check whether this compound module was
			// already considered previously, i.e., during consideration of
			// another starting block; only if the compound module is a new
			// one, we continue (recursively)
			//
			// to check if the module already exits, we could a) search it
			// first, and if not found insert it (2x constant or worst case
			// linear time), or b) try to insert and only proceed if insertion
			// was successful (1x constant or linear time)
			//
			// init the potential compound module which, if considered,
			// comprises the previous module and the current neighbour
			MultipleVoltages::CompoundModule potential_new_module;

			// initially, to try insertion, we have to build up at least the
			// sorted set of block ids; init sorted set of block ids with copy
			// from previous compound module
			potential_new_module.block_ids = module.block_ids;
			// add id of now additionally considered block
			potential_new_module.block_ids.insert(neighbour->block->id);

			// store new compound module; note that it is only inserted if not
			// existing before; this avoids storage of redundant modules for
			// commutative orders of blocks, which will arise from different
			// start points / initial modules; e.g., a compound module of
			// "sb1,sb2" is the same as "sb2,sb1", and by 1) sorting the block
			// ids and 2) inserting compound modules into a map, "sb2,sb1" is
			// effectively ignored
			//
			// hint provided is the iterator to the previously inserted
			// module; given that modules are ordered in ascending size, this
			// new module should follow (with some offset, depending on actual
			// string ids) the hint
			modules_before = this->modules.size();
			inserted = this->modules.insert(hint, {potential_new_module.block_ids, std::move(potential_new_module)});
			modules_after = this->modules.size();

			// only if this compound module was successfully inserted, i.e.,
			// not already previously inserted, we also consider it for
			// recursive calls, to determine next-level compound modules by
			// considering the currently added block's neighbours
			if (modules_after > modules_before) {

				MultipleVoltages::CompoundModule& inserted_new_module = (*inserted).second;

				// we also have to initialize remaining members of the
				// compound module, which was deferred until now, when
				// it's clear that the module has to be considered at all
				//

				// move feasible voltages
				inserted_new_module.feasible_voltages = std::move(feasible_voltages);

				// init pointers to neighbours with copy from previous
				// compound module
				inserted_new_module.contiguous_neighbours = module.contiguous_neighbours;
				// add (pointers to) neighbours of now additionally
				// considered block; note that only non yet considered
				// neighbours are added effectively into the map
				for (auto& n : neighbour->block->contiguous_neighbours) {
					inserted_new_module.contiguous_neighbours.insert({n.block->id, &n});
				}

				// recursive call; provide iterator to just inserted
				// module as hint for next insertion
				this->buildCompoundModulesHelper(inserted_new_module, inserted);
			}
		}
	}
}
