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
#include "Rect.hpp"
#include "ContiguityAnalysis.hpp"

void MultipleVoltages::determineCompoundModules(int layers, std::vector<Block> const& blocks) {

	this->modules.clear();

	// consider each block as starting point for a compound module
	for (Block const& start : blocks) {

		// init the trivial compound module, containing only the block itself
		MultipleVoltages::CompoundModule module;

		// copy feasible voltages
		module.feasible_voltages = start.feasible_voltages;

		// init pointers to blocks
		module.blocks.insert({start.id, &start});

		// init sorted set of block ids, they will be used for key generation of
		// compound modules
		module.block_ids.insert(start.id);

		// init neighbours; pointers to block's neighbour is sufficient
		for (auto& neighbour : start.contiguous_neighbours) {
			module.contiguous_neighbours.insert({neighbour.block->id, &neighbour});
		}

		// init die-wise data
		module.bb.reserve(layers);
		module.blocks_area.reserve(layers);
		module.outline_cost_die.reserve(layers);
		for (int l = 0; l < layers; l++) {

			if (start.layer == l) {
				module.bb.emplace_back(start.bb);
				module.blocks_area.emplace_back(start.bb.area);
				module.outline_cost_die.emplace_back(1.0);
			}
			else {
				module.bb.emplace_back(Rect());
				module.blocks_area.emplace_back(0.0);
				module.outline_cost_die.emplace_back(0.0);
			}
		}

		// init overall outline cost
		module.outline_cost = 1.0;

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
			std::cout << "DBG_VOLTAGES>   Comprised blocks ids: " << it->second.id() << std::endl;
			std::cout << "DBG_VOLTAGES>   Module voltages bitset: " << it->second.feasible_voltages << std::endl;
			std::cout << "DBG_VOLTAGES>    Index of min voltage: " << it->second.min_voltage_index() << std::endl;
			std::cout << "DBG_VOLTAGES>   Module (local) cost: " << it->second.outline_cost << std::endl;
		}
		std::cout << "DBG_VOLTAGES>" << std::endl;
	}
}

void MultipleVoltages::selectCompoundModules() {
	// set of compound modules; the modules are sorted by the overall cost; multiset
	// since cost may be equal for some modules, especially for trivial modules
	// comprising one block
	struct modules_cost_comp {
		bool operator() (CompoundModule const* m1, CompoundModule const* m2) const {

			return (m1->cost() > m2->cost());
		}
	};
	std::multiset<CompoundModule*, modules_cost_comp> modules_w_cost;

	MultipleVoltages::CompoundModule* cur_selected_module;
	MultipleVoltages::CompoundModule* module_to_check;
	bool module_to_remove;

	unsigned count;

	// first, insert all modules into (by cost sorted) set
	//
	for (auto it = this->modules.begin(); it != this->modules.end(); ++it) {
		modules_w_cost.insert(&(it->second));
	}

	// stepwise select module with best cost, assign module's voltage to all related
	// modules, remove the other (candidate) modules which comprise any of the already
	// assigned blocks (to avoid redundant assignments with non-optimal cost for any
	// block); proceed until all modules have been considered, which implies until all
	// blocks have a cost-optimal voltage assignment
	//
	this->selected_modules.clear();
	while (!modules_w_cost.empty()) {

		if (MultipleVoltages::DBG_VERBOSE) {

			std::cout << "DBG_VOLTAGES> Current set of compound modules to be considered (in total " << modules_w_cost.size() << "); view ordered by total cost:" << std::endl;

			for (auto* module : modules_w_cost) {

				std::cout << "DBG_VOLTAGES>  Module;" << std::endl;
				std::cout << "DBG_VOLTAGES>   Comprised blocks #: " << module->blocks.size() << std::endl;
				std::cout << "DBG_VOLTAGES>   Comprised blocks ids: " << module->id() << std::endl;
				std::cout << "DBG_VOLTAGES>   Module voltages bitset: " << module->feasible_voltages << std::endl;
				std::cout << "DBG_VOLTAGES>    Index of min voltage: " << module->min_voltage_index() << std::endl;
				std::cout << "DBG_VOLTAGES>   Module (total) cost: " << module->cost() << std::endl;

			}
			std::cout << "DBG_VOLTAGES>" << std::endl;
		}

		// select module with currently best cost
		cur_selected_module = *(modules_w_cost.begin());

		// memorize this module as selected
		this->selected_modules.push_back(cur_selected_module);

		// assign (index of) lowest applicable voltage to all blocks comprised in
		// this module
		//
		for (auto it = cur_selected_module->blocks.begin(); it != cur_selected_module->blocks.end(); ++it) {

			it->second->assigned_voltage_index = cur_selected_module->min_voltage_index();
		}

		if (MultipleVoltages::DBG_VERBOSE) {

			std::cout << "DBG_VOLTAGES> Selected compound module (out of " << modules_w_cost.size() << " modules);" << std::endl;
			std::cout << "DBG_VOLTAGES>   Comprised blocks #: " << cur_selected_module->blocks.size() << std::endl;
			std::cout << "DBG_VOLTAGES>   Comprised blocks ids: " << cur_selected_module->id() << std::endl;
			std::cout << "DBG_VOLTAGES>   Module voltages bitset: " << cur_selected_module->feasible_voltages << std::endl;
			std::cout << "DBG_VOLTAGES>    Index of min voltage: " << cur_selected_module->min_voltage_index() << std::endl;
			std::cout << "DBG_VOLTAGES>   Module (total) cost: " << cur_selected_module->cost() << std::endl;
		}

		// remove other modules which contain some already contained blocks; start
		// with 1st module in set to also remove the just considered module
		//
		if (MultipleVoltages::DBG_VERBOSE) {
			count = 0;
		}

		for (auto it = modules_w_cost.begin(); it != modules_w_cost.end();) {

			module_to_check = *it;
			module_to_remove = false;

			for (auto& id : cur_selected_module->block_ids) {

				// the module to check contains a block which is assigned
				// in the current module; thus, we drop the module
				if (module_to_check->block_ids.find(id) != module_to_check->block_ids.end()) {

					if (MultipleVoltages::DBG_VERBOSE) {

						count++;

						std::cout << "DBG_VOLTAGES>     Module to be deleted after selecting the module above: " << module_to_check->id() << std::endl;
					}

					// also update iterator; pointing to next element
					// after erased element
					it = modules_w_cost.erase(it);
					module_to_remove = true;

					break;
				}
			}

			// no module to remove; simply increment iterator
			if (!module_to_remove) {
				++it;
			}
		}

		if (MultipleVoltages::DBG_VERBOSE) {
			std::cout << "DBG_VOLTAGES>     Deleted modules count: " << count << std::endl;
		}
	}

	if (MultipleVoltages::DBG) {

		count = 0;

		std::cout << "DBG_VOLTAGES> Selected compound modules (in total " << this->selected_modules.size() << "); view ordered by total cost:" << std::endl;

		for (auto* module : this->selected_modules) {

			std::cout << "DBG_VOLTAGES>  Module;" << std::endl;
			std::cout << "DBG_VOLTAGES>   Comprised blocks #: " << module->blocks.size() << std::endl;
			std::cout << "DBG_VOLTAGES>   Comprised blocks ids: " << module->id() << std::endl;
			std::cout << "DBG_VOLTAGES>   Module voltages bitset: " << module->feasible_voltages << std::endl;
			std::cout << "DBG_VOLTAGES>    Index of min voltage: " << module->min_voltage_index() << std::endl;
			std::cout << "DBG_VOLTAGES>   Module (total) cost: " << module->cost() << std::endl;

			count += module->blocks.size();
		}
		std::cout << "DBG_VOLTAGES>" << std::endl;
		std::cout << "DBG_VOLTAGES> In total assigned blocks to modules: " << count << std::endl;
		std::cout << "DBG_VOLTAGES>" << std::endl;
	}
}

// stepwise consider adding single blocks into the compound module until all blocks are
// considered; note that this implies recursive calls to determine transitive neighbours;
// also note that a breadth-first search is applied to determine which is the best block
// to be merged such that total cost (sum of local cost, where the sum differs for
// different starting blocks) cost remain low
void MultipleVoltages::buildCompoundModulesHelper(MultipleVoltages::CompoundModule& module, MultipleVoltages::modules_type::iterator hint) {
	std::bitset<MultipleVoltages::MAX_VOLTAGES> feasible_voltages;
	ContiguityAnalysis::ContiguousNeighbour* neighbour;
	std::vector<ContiguityAnalysis::ContiguousNeighbour*> candidates;
	double best_candidate_cost;
	std::vector<double> candidates_cost;

	// walk all current neighbours; perform breadth-first search for each next-level
	// compound module with same set of applicable voltages
	//
	for (auto it = module.contiguous_neighbours.begin(); it != module.contiguous_neighbours.end(); ++it) {

		neighbour = it->second;

		// first, we determine if adding this neighbour would lead to an trivial
		// solution, i.e., only the highest possible voltage is assignable; such
		// modules are mainly ignored (one exception, see below) and thus we can
		// achieve notable reduction in memory and runtime by pruning trivial
		// solutions early on during recursive bottom-up phase
		//
		// the only exception where modules with only highest voltage shall be
		// further investigated is in case adjacent trivial compound modules
		// (single blocks) can be merged
		//

		// bit-wise AND to obtain the intersection of feasible voltages
		feasible_voltages = module.feasible_voltages & neighbour->block->feasible_voltages;

		if (MultipleVoltages::DBG) {

			std::cout << "DBG_VOLTAGES> Current module (" << module.id() << "),(" << module.feasible_voltages << ");";
			std::cout << " consider neighbour block: (" << neighbour->block->id << "),(" << neighbour->block->feasible_voltages << ")" << std::endl;
		}

		// only one voltage is applicable _afterwards_, i.e., only after adding
		// the current neighbour; this voltage is trivially the highest possible;
		// ignore this trivial compound module
		if (module.feasible_voltages.count() > 1 && feasible_voltages.count() == 1) {

			if (MultipleVoltages::DBG) {
				std::cout << "DBG_VOLTAGES>  Trivial solution; only highest possible voltage applicable (" << feasible_voltages << ");";
				std::cout << " skip this neighbour block" << std::endl;
			}

			continue;
		}
		// two similar cases: a) more than one voltage is applicable _afterwards_,
		// but the resulting set of voltages is the same as _before_ for the
		// previous module or b) merging trivial compound modules w/ only highest
		// voltage applicable; both cases can be simplified to comparing previous
		// and new set of voltages, ignoring the count of applicable voltages
		//
		// here, we don't insert the new module immediately, but rather memorize
		// all such candidate modules / neighbours and then consider only the one
		// with the lowest cost for further branching
		else if (feasible_voltages == module.feasible_voltages) {

			if (MultipleVoltages::DBG) {
				std::cout << "DBG_VOLTAGES>  No change in applicable voltages (" << module.feasible_voltages << ")";
				std::cout << "; consider neighbour block as candidate" << std::endl;
			}

			candidates.push_back(neighbour);
		}
		// more than one voltage is applicable, and the set of voltages has
		// changed; such a module should be considered without notice of cost,
		// since it impacts the overall set of possible voltage islands
		else {

			if (MultipleVoltages::DBG) {
				std::cout << "DBG_VOLTAGES>  Change in applicable voltages: " << module.feasible_voltages << " before, " << feasible_voltages << " now;";
				std::cout << " non-trivial solution; try insertion of related new module" << std::endl;
			}

			// previous neighbours shall be considered, since the related new
			// module has a different set of voltages, i.e., no tie-braking
			// was considered among some candidate neighbours
			this->insertCompoundModuleHelper(module, neighbour, true, feasible_voltages, hint);
		}
	}

	if (MultipleVoltages::DBG) {
		std::cout << "DBG_VOLTAGES> Current module (" << module.id() << "),(" << module.feasible_voltages << "); all neighbour blocks considered" << std::endl;
	}

	// some neighbours may be added such that there is no change in the set of
	// applicable voltages; out of the related candidates, proceed only with the
	// lowest-cost candidate (w.r.t. outline_cost); this way, the solution space is
	// notably reduced, and the top-down process would select partial solutions
	// (compound modules) of lowest cost anyway, thus this decision can already be
	// applied here
	//
	//
	if (!candidates.empty()) {

		if (MultipleVoltages::DBG) {
				std::cout << "DBG_VOLTAGES> Current module (" << module.id() << "),(" << module.feasible_voltages << "); evaluate candidates" << std::endl;
		}

		// init with zero dummy cost; max cost is to be determined
		best_candidate_cost = 0.0;

		// 1) determine all cost and max cost
		candidates_cost.reserve(candidates.size());
		for (auto& candidate : candidates) {

			// apply_update = false; i.e., only calculate cost of potentially
			// adding the candidate block, don't add block yet
			//
			// memorize cost since more than one module may show best cost
			//
			candidates_cost.push_back(module.updateOutlineCost(candidate, false));

			if (MultipleVoltages::DBG) {
				std::cout << "DBG_VOLTAGES>  Candidate block " << candidate->block->id <<"; cost: " << candidates_cost.back() << std::endl;
			}

			// determine max cost
			if (candidates_cost.back() > best_candidate_cost) {
				best_candidate_cost = candidates_cost.back();
			}
		}

		// consider all candidates w/ best cost
		//
		for (unsigned c = 0; c < candidates.size(); c++) {

			if (Math::doubleComp(candidates_cost[c], best_candidate_cost)) {

				ContiguityAnalysis::ContiguousNeighbour* best_candidate = candidates[c];

				// redetermine intersection of feasible voltages
				feasible_voltages = module.feasible_voltages & best_candidate->block->feasible_voltages;

				if (MultipleVoltages::DBG) {
					std::cout << "DBG_VOLTAGES> Current module (" << module.id() << "),(" << module.feasible_voltages << ");";
					std::cout << " best candidate block " << best_candidate->block->id;
					std::cout << "; cost: " << best_candidate_cost << "; try insertion of related new module" << std::endl;
				}

				// insert the candidate with the best cost; continue
				// recursively with this new module; other neighbours
				// shall not be considered anymore, otherwise the
				// selection of best-cost candidate would be undermined;
				// note that in practice some blocks will still be
				// (rightfully) considered since they are also contiguous
				// neighbours with the now considered best-cost candidate
				this->insertCompoundModuleHelper(module, best_candidate, false, feasible_voltages, hint);
			}
		}
	}
}

inline void MultipleVoltages::insertCompoundModuleHelper(MultipleVoltages::CompoundModule& module, ContiguityAnalysis::ContiguousNeighbour* neighbour, bool consider_prev_neighbours, std::bitset<MultipleVoltages::MAX_VOLTAGES>& feasible_voltages, MultipleVoltages::modules_type::iterator& hint) {
	MultipleVoltages::modules_type::iterator inserted;
	unsigned modules_before, modules_after;

	// first, we have to check whether this compound module was already considered
	// previously, i.e., during consideration of another starting block; only if the
	// compound module is a new one, we continue
	//
	// to check if the module already exits, we could a) search for it first, and if
	// not found insert it (2x constant or worst case linear time), or b) try to
	// insert it and only proceed if insertion was successful (1x constant or linear
	// time)
	//
	// init the potential compound module which, if considered, comprises the previous
	// module and the current neighbour
	MultipleVoltages::CompoundModule potential_new_module;

	// initially, to try insertion, we have to build up at least the sorted set of
	// block ids; init sorted set of block ids with copy from previous compound module
	potential_new_module.block_ids = module.block_ids;
	// add id of now additionally considered block
	potential_new_module.block_ids.insert(neighbour->block->id);

	// store new compound module; note that it is only inserted if not existing
	// before; this avoids storage of redundant modules for commutative orders of
	// blocks, which will arise from different start points / initial modules; e.g., a
	// compound module of "sb1,sb2" is the same as "sb2,sb1", and by 1) sorting the
	// block ids and 2) inserting compound modules into a map, "sb2,sb1" is
	// effectively ignored
	//
	// hint provided is the iterator to the previously inserted module; given that
	// modules are ordered in ascending size, this new module should follow (with some
	// offset, depending on actual string ids) the hint
	modules_before = this->modules.size();
	inserted = this->modules.insert(hint, {potential_new_module.block_ids, std::move(potential_new_module)});
	modules_after = this->modules.size();

	// only if this compound module was successfully inserted, i.e., not already
	// previously inserted, we proceed with proper initialization of remaining
	// members/data of the compound module
	if (modules_after > modules_before) {

		MultipleVoltages::CompoundModule& inserted_new_module = (*inserted).second;

		// assign feasible voltages
		inserted_new_module.feasible_voltages = std::move(feasible_voltages);

		// init block pointers from previous module
		inserted_new_module.blocks = module.blocks;
		// insert now additionally considered neighbour
		inserted_new_module.blocks.insert({neighbour->block->id, neighbour->block});

		// init bounding box, blocks area and cost from the previous module
		inserted_new_module.outline_cost_die = module.outline_cost_die;
		inserted_new_module.bb = module.bb;
		inserted_new_module.blocks_area = module.blocks_area;

		// update bounding box, blocks area, and recalculate outline cost; all
		// w.r.t. added (neighbour) block
		inserted_new_module.updateOutlineCost(neighbour);

		// if previous neighbours shall be considered, init the related pointers
		// as copy from the previous module
		if (consider_prev_neighbours) {

			inserted_new_module.contiguous_neighbours = module.contiguous_neighbours;

			// ignore the just considered neighbour (inserted_new_module);
			// deleting afterwards is computationally less expansive than
			// checking each neighbor's id during copying
			inserted_new_module.contiguous_neighbours.erase(neighbour->block->id);
		}

		// add (pointers to) neighbours of the now additionally considered block;
		// note that only yet not considered neighbours are effectively added to
		// the map
		for (auto& n : neighbour->block->contiguous_neighbours) {

			// ignore any neighbour which is already comprised in the module
			if (module.block_ids.find(n.block->id) != module.block_ids.end()) {
				continue;
			}

			inserted_new_module.contiguous_neighbours.insert({n.block->id, &n});
		}

		if (MultipleVoltages::DBG) {
			std::cout << "DBG_VOLTAGES> Insertion successful; continue recursively with this module" << std::endl;
		}

		// recursive call; provide iterator to just inserted module as hint for
		// next insertion
		this->buildCompoundModulesHelper(inserted_new_module, inserted);
	}
	else if (MultipleVoltages::DBG) {
		std::cout << "DBG_VOLTAGES> Insertion not successful; module was already inserted previously" << std::endl;
	}
}

// TODO consider contiguity, or similar measure to estimate power-domain synthesis cost
//
// current cost term: blocks area over bounding box; i.e., packing density; i.e., the
// higher the cost the better
inline double MultipleVoltages::CompoundModule::updateOutlineCost(ContiguityAnalysis::ContiguousNeighbour* neighbour, bool apply_update) {
	double overall_cost = 0.0;
	double dies_to_consider = 0;

	int neighbour_layer = neighbour->block->layer;

	// these data structures are used to calculate the changes resulting from adding
	// ContiguousNeighbour* block to the module; they are eventually only applied if
	// apply_update == true
	//
	// init them from the module's current state
	std::vector<double> outline_cost_die = this->outline_cost_die;
	std::vector<Rect> bb = this->bb;
	std::vector<double> blocks_area = this->blocks_area;

	// update bounding box and blocks area on (by added block) affected die;
	// note that the added block may be the first on its related die which is
	// assigned to this module
	if (blocks_area[neighbour_layer] == 0.0) {
		// init new bb
		bb[neighbour_layer] = neighbour->block->bb;
		// init blocks area
		blocks_area[neighbour_layer] = neighbour->block->bb.area;
	}
	else {
		// update existing bb
		bb[neighbour_layer] = Rect::determBoundingBox(this->bb[neighbour_layer], neighbour->block->bb);
		// update blocks area
		blocks_area[neighbour_layer] += neighbour->block->bb.area;
	}

	// update affected layer's and overall cost; walk all dies
	//
	for (unsigned l = 0; l < blocks_area.size(); l++) {

		// sanity check for some block being assigned to the die
		//
		if (blocks_area[l] > 0.0) {

			// affected die; update die-wise cost first
			if (l == static_cast<unsigned>(neighbour_layer)) {

				// actual cost term; blocks area over bounding box; i.e.,
				// packing density
				outline_cost_die[l] = blocks_area[l] / bb[l].area;
			}

			// in any case (die affected or not), update intermediate terms,
			// i.e., sum up cost for total cost
			overall_cost += outline_cost_die[l];
			dies_to_consider++;
		}
	}

	// normalize cost to avg cost; note that div by zero cannot occur since each
	// module has at least one block assigned
	overall_cost /= dies_to_consider;

	// apply updates to module only if requested; this way, both the actual change for
	// new modules and the potential change for candidate modules can be unified in
	// this helper
	//
	// note that the whole data structures have to be moved, not only the values for
	// the affected die; this is because CompoundModule& module is a new module
	// instance
	if (apply_update) {
		this->bb = std::move(bb);
		this->blocks_area = std::move(blocks_area);
		this->outline_cost_die = std::move(outline_cost_die);
		this->outline_cost = overall_cost;
	}

	return overall_cost;
}
