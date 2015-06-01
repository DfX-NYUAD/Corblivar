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

void MultipleVoltages::determineCompoundModules(int layers, std::vector<Block> const& blocks) {

	// consider each block as starting point for a compound module
	for (Block const& block : blocks) {

		// init the trivial compound module, containing only the block itself
		MultipleVoltages::CompoundModule module;
		// TODO may be not required
		//module.blocks.push_back(&block);
		module.feasible_voltages = block.feasible_voltages;
		module.block_ids.insert(block.id);
		module.contiguous_neighbours = block.contiguous_neighbours;

		// store trivial compound module
		this->modules.insert(std::make_pair<std::string, MultipleVoltages::CompoundModule>(
					// make_pair has move semantics, thus we need a
					// copy of the key string
					std::string(block.id),
					// the module can be moved, it will be accessed
					// from the map later on
					std::move(module)
				));

		// consider adding single blocks stepwise into each compound module until
		// all blocks are merged
		//
		// TODO consider all blocks; this requires to determine contiguity across
		// dies first, see ContiguityAnalysis::analyseBlocks
		for (unsigned b = 0; b < 10; b++) {
		}
	}
}
