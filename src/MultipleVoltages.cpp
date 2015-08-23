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

void MultipleVoltages::determineCompoundModules(int layers, std::vector<Block> const& blocks, ContiguityAnalysis& cont) {

	this->modules.clear();

	// consider each block as starting point for a compound module
	for (Block const& start : blocks) {

		// init the base compound module, containing only the block itself
		MultipleVoltages::CompoundModule module;

		// copy feasible voltages
		module.feasible_voltages = start.feasible_voltages;

		// init pointers to blocks
		module.blocks.push_back(&start);

		// init power saving, based on feasible voltages and current block; note
		// that previous values are not defined, thus the regular case to reset
		// and recalculate power saving over all (here one) blocks is applied
		module.update_power_saving();

		// init block ids such that they may encode all blocks' numerical ids;
		// also account for the offset of one, introduced by Block::DUMMY_NUM_ID
		module.block_ids.reserve(blocks.size() + 1);
		for (unsigned b = 0; b < blocks.size() + 1; b++) {
			module.block_ids.push_back(false);
		}

		// also, set the block-ids' flag for the current block
		module.block_ids[start.numerical_id] = true;

		// init neighbours; pointers to block's neighbour is sufficient
		for (auto& neighbour : start.contiguous_neighbours) {
			module.contiguous_neighbours.insert({neighbour.block->numerical_id, &neighbour});
		}

		// init outline and corners for power rings
		module.outline.reserve(layers);
		module.corners_powerring.reserve(layers);

		for (int l = 0; l < layers; l++) {

			// empty bb
			module.outline.emplace_back(std::vector<Rect>());

			// any layer, also not affected layers, may be initialized with
			// the trivial min number of corners, i.e., 4
			module.corners_powerring.emplace_back(4);

			if (start.layer == l) {
				module.outline[l].emplace_back(start.bb);
			}
			// note that outline[l] shall remain empty otherwise
		}

		// store base compound module
		auto inserted_it = this->modules.insert(this->modules.begin(), {module.block_ids, std::move(module)});

		// perform stepwise and recursive merging of base module into larger
		// compound modules
		this->buildCompoundModulesHelper(inserted_it->second, inserted_it, cont);
	}

	if (MultipleVoltages::DBG) {

		std::cout << "DBG_VOLTAGES> Compound modules (in total " << this->modules.size() << "):" << std::endl;

		for (auto it = this->modules.begin(); it != this->modules.end(); ++it) {

			CompoundModule& module = it->second;

			std::cout << "DBG_VOLTAGES>  Module;" << std::endl;
			std::cout << "DBG_VOLTAGES>   Comprised blocks #: " << module.blocks.size() << std::endl;
			std::cout << "DBG_VOLTAGES>   Comprised blocks ids: " << module.id() << std::endl;
			std::cout << "DBG_VOLTAGES>   Module voltages bitset: " << module.feasible_voltages << std::endl;
			std::cout << "DBG_VOLTAGES>    Index of min voltage: " << module.min_voltage_index() << std::endl;
			std::cout << "DBG_VOLTAGES>   Module (local) cost: " << module.outline_cost << std::endl;
		}
		std::cout << "DBG_VOLTAGES>" << std::endl;
	}
}

std::vector<MultipleVoltages::CompoundModule*> const& MultipleVoltages::selectCompoundModules(bool const& merge_selected_modules) {
	MultipleVoltages::CompoundModule* cur_selected_module;
	MultipleVoltages::CompoundModule* module_to_check;

	bool module_to_remove;
	unsigned count;

	double max_power_saving;
	unsigned max_corners;

	unsigned min_voltage_index;

	// comparator for local multiset of selected compound modules; multiset since cost
	// may be equal for some modules, especially for trivial modules comprising one
	// block; the comparator can be parametrized for proper cost normalization
	//
	class modules_cost_comp {

		double max_power_saving;
		unsigned max_corners;
		MultipleVoltages::Parameters parameters;

		public:
			// initializer for comparator w/ parameters
			modules_cost_comp(double const& max_power_saving, unsigned const& max_corners, MultipleVoltages::Parameters const& parameters) {
				this->max_power_saving = max_power_saving;
				this->max_corners = max_corners;
				this->parameters = parameters;
			}

			bool operator()(CompoundModule const* m1, CompoundModule const* m2) const {

				double c1 = m1->cost(this->max_power_saving, this->max_corners, this->parameters);
				double c2 = m2->cost(this->max_power_saving, this->max_corners, this->parameters);

				return (
						// the smaller the cost, the better
						(c1 < c2)
						// if cost are similar, consider larger modules in
						// the sense of modules covering more blocks; this
						// is especially relevant to discourage trivial
						// modules comprising only one block; these blocks
						// will have same cost
						|| (Math::doubleComp(c1, c2) && m1->blocks.size() > m2->blocks.size())
				       );
			}
	};

	// first, determine max values for corners and power saving, required for
	// normalization of related cost terms
	//
	max_power_saving = this->modules.begin()->second.power_saving();
	max_corners = this->modules.begin()->second.corners_powerring_max();
	for (auto it = this->modules.begin(); it != this->modules.end(); ++it) {
		max_power_saving = std::max(max_power_saving, it->second.power_saving());
		max_corners = std::max(max_corners, it->second.corners_powerring_max());
	}
	
	// now, a multiset can be initialized with the proper comparator parameters
	//
	std::multiset<CompoundModule*, modules_cost_comp> modules_w_cost (modules_cost_comp(max_power_saving, max_corners, this->parameters));
	
	// second, insert all modules' pointers into this multiset
	//
	for (auto it = this->modules.begin(); it != this->modules.end(); ++it) {
		modules_w_cost.insert(&(it->second));
	}

	// third, stepwise select module with best cost, assign module's voltage to all
	// related modules, remove the other (candidate) modules which comprise any of the
	// already assigned blocks (to avoid redundant assignments with non-optimal cost
	// for any block); proceed until all modules have been considered, which implies
	// until all blocks have a cost-optimal voltage assignment
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
				std::cout << "DBG_VOLTAGES>   Module (total) cost: " << module->cost(max_power_saving, max_corners, parameters) << std::endl;
				std::cout << "DBG_VOLTAGES>    Gain minus ``wasted gain'' in power reduction: " << module->power_saving() << std::endl;
				std::cout << "DBG_VOLTAGES>    Gain in power reduction: " << module->power_saving(false) << std::endl;
				std::cout << "DBG_VOLTAGES>    Estimated max number of corners for power rings: " << module->corners_powerring_max() << std::endl;
				std::cout << "DBG_VOLTAGES>    Covered blocks (not modeled in cost, but considered during selection): " << module->blocks.size() << std::endl;

			}
			std::cout << "DBG_VOLTAGES>" << std::endl;
		}

		// select module with currently best cost
		cur_selected_module = *(modules_w_cost.begin());

		// memorize this module as selected
		this->selected_modules.push_back(cur_selected_module);

		// assign related values to all blocks comprised in this module: (index
		// of) lowest applicable voltage, and pointer to module itself
		//
		min_voltage_index = cur_selected_module->min_voltage_index();
		for (Block const* b : cur_selected_module->blocks) {

			b->assigned_voltage_index = min_voltage_index;
			b->assigned_module = cur_selected_module;
		}

		if (MultipleVoltages::DBG_VERBOSE) {

			std::cout << "DBG_VOLTAGES> Selected compound module (out of " << modules_w_cost.size() << " modules);" << std::endl;
			std::cout << "DBG_VOLTAGES>   Comprised blocks #: " << cur_selected_module->blocks.size() << std::endl;
			std::cout << "DBG_VOLTAGES>   Comprised blocks ids: " << cur_selected_module->id() << std::endl;
			std::cout << "DBG_VOLTAGES>   Module voltages bitset: " << cur_selected_module->feasible_voltages << std::endl;
			std::cout << "DBG_VOLTAGES>    Index of min voltage: " << min_voltage_index << std::endl;
			std::cout << "DBG_VOLTAGES>   Module (total) cost: " << cur_selected_module->cost(max_power_saving, max_corners, parameters) << std::endl;
			std::cout << "DBG_VOLTAGES>    Gain minus ``wasted gain'' in power reduction: " << cur_selected_module->power_saving() << std::endl;
			std::cout << "DBG_VOLTAGES>    Gain in power reduction: " << cur_selected_module->power_saving(false) << std::endl;
			std::cout << "DBG_VOLTAGES>    Estimated max number of corners for power rings: " << cur_selected_module->corners_powerring_max() << std::endl;
			std::cout << "DBG_VOLTAGES>    Covered blocks (not modeled in cost, but considered during selection): " << cur_selected_module->blocks.size() << std::endl;
		}

		// remove other modules which contain some already assigned blocks; start
		// with 1st module in set to also remove the just considered module
		//
		if (MultipleVoltages::DBG_VERBOSE) {
			count = 0;
		}

		for (auto it = modules_w_cost.begin(); it != modules_w_cost.end();) {

			module_to_check = *it;
			module_to_remove = false;

			for (Block const* assigned_block : cur_selected_module->blocks) {

				// the module to check contains a block which is assigned
				// in the current module; thus, we drop the module
				//
				if (module_to_check->block_ids[assigned_block->numerical_id] == true) {

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

	// fourth, merge selected modules whenever possible, i.e., when some of the
	// modules' blocks are contiguous to another module sharing the same voltage
	//
	// note that such merging will a) impact the corners and b) undermine the cost
	// normalization and thus the actual top-down selection. For a), it most likely
	// increases the estimated max number of corners to an unreasonably high value
	// since most merges will not increase the numbers of corners, especially not when
	// many modules nearby (with same voltages) are merged. For b), the actual cost of
	// merged modules shall be different and may be inferior than other cost,
	// suggesting that the merge is not beneficial
	//
	// thus, it shall only be applied when requested, e.g., for final logging
	if (merge_selected_modules) {

		if (MultipleVoltages::DBG) {
			std::cout << "DBG_VOLTAGES>  Start merging modules" << std::endl;
		}

		// for-loop instead of iterator, since we edit this very data structure
		//
		for (unsigned m = 0; m < this->selected_modules.size(); m++) {

			MultipleVoltages::CompoundModule* module = this->selected_modules[m];

			// walk all contiguous blocks of current module
			//
			for (auto it = module->contiguous_neighbours.begin(); it != module->contiguous_neighbours.end(); ++it) {

				MultipleVoltages::CompoundModule* n_module = it->second->block->assigned_module;

				// if the related module of the contiguous block has the same
				// voltage index as this module, they can be merged; merging means
				// to extend the current module with the blocks from the
				// contiguous block's module
				//
				if (n_module->min_voltage_index() == module->min_voltage_index()) {

					// sanity check; avoid merging with itself
					if (n_module->block_ids == module->block_ids) {
						continue;
					}

					if (MultipleVoltages::DBG) {
						std::cout << "DBG_VOLTAGES>   Merging modules;" << std::endl;
						std::cout << "DBG_VOLTAGES>    " << module->id() << std::endl;
						std::cout << "DBG_VOLTAGES>    " << n_module->id() << std::endl;
					}

					// update the actual blocks
					for (Block const* b : n_module->blocks) {

						module->blocks.push_back(b);
						module->block_ids[b->numerical_id] = true;

						// also update the module pointer for merged
						// module's blocks
						b->assigned_module = module;
					}

					// sum up the power saving values
					module->power_saving_ += n_module->power_saving_;
					module->power_saving_wasted_ += n_module->power_saving_wasted_;

					// add the outline rects from the module to be merged
					for (unsigned l = 0; l < module->outline.size(); l++) {

						for (auto& rect : n_module->outline[l]) {
							module->outline[l].push_back(std::move(rect));
						}

						// also sum up the power-ring corners, but
						// subtract two under the assumption that the
						// previous module's outline can be extended
						// without further corners; this is an somewhat
						// optimistic but simple estimation
						module->corners_powerring[l] += n_module->corners_powerring[l] - 2;
					}

					// add (pointers to) now additionally considered
					// contiguous neighbours; note that only yet not
					// considered neighbours are effectively added
					for (auto n : n_module->contiguous_neighbours) {

						// ignore any neighbour which is already comprised in the module
						if (module->block_ids[n.first] == true) {
							continue;
						}

						module->contiguous_neighbours.insert(n);
					}

					// erase the just merged module
					//
					for (auto it = this->selected_modules.begin(); it != this->selected_modules.end(); ++it) {

						if ((*it)->block_ids == n_module->block_ids) {
							this->selected_modules.erase(it);
							break;
						}
					}

					// finally, reset the iterator for contiguous blocks as
					// well; required to capture transitive merges and,
					// furthermore, iterator it is invalid after the above
					// insertions
					//
					it = module->contiguous_neighbours.begin();
				}
			}
		}

		if (MultipleVoltages::DBG) {
			std::cout << "DBG_VOLTAGES>  Done merging modules" << std::endl;
			std::cout << "DBG_VOLTAGES>" << std::endl;
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
			std::cout << "DBG_VOLTAGES>   Module (total) cost: " << module->cost(max_power_saving, max_corners, parameters) << std::endl;
			std::cout << "DBG_VOLTAGES>    Gain minus ``wasted gain'' in power reduction: " << module->power_saving() << std::endl;
			std::cout << "DBG_VOLTAGES>    Gain in power reduction: " << module->power_saving(false) << std::endl;
			std::cout << "DBG_VOLTAGES>    Estimated max number of corners for power rings: " << module->corners_powerring_max() << std::endl;
			std::cout << "DBG_VOLTAGES>    Covered blocks (not modeled in cost, but considered during selection): " << module->blocks.size() << std::endl;

			count += module->blocks.size();
		}
		std::cout << "DBG_VOLTAGES>" << std::endl;
		std::cout << "DBG_VOLTAGES> In total assigned blocks to modules: " << count << std::endl;
		std::cout << "DBG_VOLTAGES>" << std::endl;
	}

	return this->selected_modules;
}

// stepwise consider adding single blocks into the compound module until all blocks are
// considered; note that this implies recursive calls to determine transitive neighbours;
// also note that a breadth-first search is applied to determine which is the best block
// to be merged such that total cost (sum of local cost, where the sum differs for
// different starting blocks) cost remain low
void MultipleVoltages::buildCompoundModulesHelper(MultipleVoltages::CompoundModule& module, MultipleVoltages::modules_type::iterator hint, ContiguityAnalysis& cont) {
	std::bitset<MultipleVoltages::MAX_VOLTAGES> feasible_voltages;
	ContiguityAnalysis::ContiguousNeighbour* neighbour;
	std::vector<ContiguityAnalysis::ContiguousNeighbour*> candidates;
	double best_candidate_cost, cur_candidate_cost;
	ContiguityAnalysis::ContiguousNeighbour* best_candidate;

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

		// more than one voltage is applicable _afterwards_ but the resulting set
		// of voltages is the same as _before_ for the previous module
		//
		// here, we don't insert the new module immediately, but rather memorize
		// all such candidate modules / neighbours and then consider only the one
		// with the lowest cost for further branching
		if (feasible_voltages.count() > 1 && feasible_voltages == module.feasible_voltages) {

			if (MultipleVoltages::DBG) {
				std::cout << "DBG_VOLTAGES>  No change in applicable voltages (" << module.feasible_voltages << ")";
				std::cout << "; consider neighbour block as candidate" << std::endl;
			}

			candidates.push_back(neighbour);
		}
		// only one voltage was applicable, i.e., handle a trivial compound
		// module; consider only for merging with another trivial block/neighbour;
		// this way, largest possible islands for the trivial voltage can be
		// obtained; in order to limit the search space, branching is not allowed
		// here, i.e., modules are stepwise added as long as some contiguous and
		// trivial modules are available, but once such a module is selected for
		// merging, no further modules on this branching level are considered
		//
		// same candidate principle as for other cases with unchanged
		// set of voltages applies here, in order to limit the search space
		else if (module.feasible_voltages.count() == 1 && neighbour->block->feasible_voltages.count() == 1) {

			if (MultipleVoltages::DBG) {
				std::cout << "DBG_VOLTAGES>  Consider trivial module to merge with another trivial block/neighbour;";
				std::cout << " consider neighbour block as candidate" << std::endl;
			}

			// previous neighbours shall not be considered, in order to limit
			// the search space such that only ``forward merging'' of new
			// contiguous trivial modules is considered
			this->insertCompoundModuleHelper(module, neighbour, false, feasible_voltages, hint, cont);

			// this break is the ``trick'' for disabling branching: once a
			// contiguous trivial module is extended by this relevant
			// neighbour and once the recursive calls (for building up
			// resulting larger modules) return to this point, no further
			// neighbours are considered; the resulting stepwise merging of
			// only one trivial neighbour is sufficient to capture
			// largest-possible contiguous trivial modules
			break;
		}
		// more than one voltage is applicable, and the set of voltages has
		// changed; such a module should be considered without notice of cost,
		// since it impacts the overall set of possible voltage islands
		//
		else if (feasible_voltages.count() > 1 && feasible_voltages != module.feasible_voltages) {

			if (MultipleVoltages::DBG) {
				std::cout << "DBG_VOLTAGES>  Change in applicable voltages: " << module.feasible_voltages << " before, " << feasible_voltages << " now;";
				std::cout << " non-trivial solution; try insertion of related new module" << std::endl;
			}

			// previous neighbours shall be considered, since the related new
			// module has a different set of voltages, i.e., no tie-braking
			// was considered among some candidate neighbours
			this->insertCompoundModuleHelper(module, neighbour, true, feasible_voltages, hint, cont);
		}
		// any other case, i.e., only one (trivially the highest possible) voltage
		// applicable for the new module; to be ignored
		else {
			if (MultipleVoltages::DBG) {
				std::cout << "DBG_VOLTAGES>  Trivial partial solution, with only highest voltage applicable (" << feasible_voltages << ");";
				std::cout << " skip this neighbour block" << std::endl;
			}

			continue;
		}
	}

	if (MultipleVoltages::DBG) {
		std::cout << "DBG_VOLTAGES> Current module (" << module.id() << "),(" << module.feasible_voltages << "); all neighbour blocks considered" << std::endl;
	}

	// some neighbours may be added such that there is no change in the set of
	// applicable voltages; out of the related candidates, proceed only with the
	// lowest-cost candidate (w.r.t. local outline_cost); this way, the solution space
	// is notably reduced, and the top-down process would select compound modules of
	// lowest cost (global cost, somewhat related to this local cost) anyway, thus
	// this decision can already be applied here
	//
	if (!candidates.empty()) {

		if (MultipleVoltages::DBG) {
				std::cout << "DBG_VOLTAGES> Current module (" << module.id() << "),(" << module.feasible_voltages << "); evaluate candidates" << std::endl;
		}

		// init with dummy cost (each bb cannot be more intruded than by factor
		// 1.0); min cost is to be determined
		best_candidate_cost = 1.0;

		// determine best candidate
		for (auto* candidate : candidates) {

			// apply_update = false; i.e., only calculate cost of potentially
			// adding the candidate block, don't add block yet
			//
			cur_candidate_cost = module.updateOutlineCost(candidate, cont, false);

			if (MultipleVoltages::DBG) {
				std::cout << "DBG_VOLTAGES>  Candidate block " << candidate->block->id <<"; cost: " << cur_candidate_cost << std::endl;
			}

			// determine min cost and related best candidate
			if (cur_candidate_cost < best_candidate_cost) {
				best_candidate_cost = cur_candidate_cost;
				best_candidate = candidate;
			}
		}

		// redetermine intersection of feasible voltages for best-cost candidate
		feasible_voltages = module.feasible_voltages & best_candidate->block->feasible_voltages;

		if (MultipleVoltages::DBG) {
			std::cout << "DBG_VOLTAGES> Current module (" << module.id() << "),(" << module.feasible_voltages << ");";
			std::cout << " best candidate block " << best_candidate->block->id;
			std::cout << "; cost: " << best_candidate_cost << "; try insertion of related new module" << std::endl;
		}

		// merge only the best-cost candidate into the module; continue
		// recursively with this new module; other neighbours shall not be
		// considered anymore, otherwise the selection of best-cost candidate
		// would be undermined; note that in practice some blocks will still be
		// (rightfully) considered since they are also contiguous neighbours with
		// the now considered best-cost candidate
		this->insertCompoundModuleHelper(module, best_candidate, false, feasible_voltages, hint, cont);
	}
}

inline void MultipleVoltages::insertCompoundModuleHelper(MultipleVoltages::CompoundModule& module, ContiguityAnalysis::ContiguousNeighbour* neighbour, bool consider_prev_neighbours, std::bitset<MultipleVoltages::MAX_VOLTAGES>& feasible_voltages, MultipleVoltages::modules_type::iterator& hint, ContiguityAnalysis& cont) {
	MultipleVoltages::modules_type::iterator inserted;

	// first, we have to check whether this potential compound module was already
	// considered previously, i.e., during consideration of another starting module;
	// only if the compound module is really a new one, then we continue
	//
	// to check if the potential module already exits, we ``leverage'' the previous
	// module, by assigning the now-to-consider neighbour's block to the previous
	// module's set of considered blocks; thus we avoid copying the whole set of
	// blocks just for checking the potential module's existence; for further improved
	// efficiency, we leverage the bool flags instead of the actual set blocks
	//
	module.block_ids[neighbour->block->numerical_id] = true;

	// now, perform the actual check
	if (this->modules.find(module.block_ids) != this->modules.end()) {

		// the potential module does already exit; revert the just assigned
		// neighbour from the previous module again; and return
		module.block_ids[neighbour->block->numerical_id] = false;

		if (MultipleVoltages::DBG) {
			std::cout << "DBG_VOLTAGES> Insertion not successful; module was already inserted previously" << std::endl;
		}

		return;
	}

	// at this point, it's clear that we have to generate the new compound module; it
	// comprises the previous module and the neighbour
	//
	MultipleVoltages::CompoundModule new_module;

	// the blocks assignment is contained in the previous module, since the
	// neighbour's block was already assigned; simply copy these flags
	new_module.block_ids = module.block_ids;

	// only now we shall revert the neighbour's block assignment to the previous module
	module.block_ids[neighbour->block->numerical_id] = false;

	// copy block pointers from previous module
	new_module.blocks = module.blocks;
	// consider neighbour block for new module
	new_module.blocks.push_back(neighbour->block);

	// assign feasible voltages, may be moved since they were generated only for this
	// callee and will not be reused in the caller
	new_module.feasible_voltages = std::move(feasible_voltages);

	// recalculate the power saving for all comprised modules if required, i.e.,
	// whenever the set of applicable voltages changes
	//
	if (new_module.feasible_voltages != module.feasible_voltages) {

		new_module.update_power_saving();
	}
	// otherwise, copy previous values and update only according to the specific new
	// block to be considered
	//
	else {
		// copy previous values
		new_module.power_saving_ = module.power_saving_;
		new_module.power_saving_wasted_ = module.power_saving_wasted_;

		// update copied values to consider new block
		new_module.update_power_saving(neighbour->block);
	}

	// copy outline from the previous module
	new_module.outline = module.outline;

	// copy corners from the previous module
	new_module.corners_powerring = module.corners_powerring;

	// update bounding box, blocks area, and recalculate outline cost; all w.r.t.
	// added (neighbour) block
	new_module.updateOutlineCost(neighbour, cont);

	// if previous neighbours shall be considered, copy the related pointers from the
	// previous module
	if (consider_prev_neighbours) {

		new_module.contiguous_neighbours = module.contiguous_neighbours;

		// we have to ignore the just considered neighbour; deleting afterwards is
		// computationally less expansive than checking each neighbor's id during
		// copying
		new_module.contiguous_neighbours.erase(neighbour->block->numerical_id);
	}

	// add (pointers to) neighbours of the now additionally considered block; note
	// that only yet not considered neighbours are effectively added
	//
	for (auto& n : neighbour->block->contiguous_neighbours) {

		// we have to ignore any neighbour which is already comprised in the
		// module itself
		if (module.block_ids[n.block->numerical_id] == true) {
			continue;
		}

		new_module.contiguous_neighbours.insert({n.block->numerical_id, &n});
	}


	// perform actual insertion; hint provided is the iterator to the previously
	// inserted module
	//
	inserted = this->modules.insert(hint, {new_module.block_ids, std::move(new_module)});

	if (MultipleVoltages::DBG) {
		std::cout << "DBG_VOLTAGES> Insertion successful; continue recursively with this module" << std::endl;
	}

	// recursive call; provide iterator to just inserted new module as hint for next
	// insertion
	this->buildCompoundModulesHelper(inserted->second, inserted, cont);
}

// local cost, used during bottom-up merging
//
// cost term: ratio of (by other blocks with non-compatible voltage) intruded area of the
// module's bb; the lower the better
//
// note that the cost always considers the amount of _current_ intrusion (after adding
// neighbour to module), despite the fact that only the non-intruded bb's are memorized;
// this is required in order to model the amount of intrusion as local cost, required for
// local tree-pruning decisions during bottom-up phase
//
// also, extended bbs with minimized number of corners for power-ring synthesis are
// generated here; note that the die-wise container for power-ring corners is updated here
// as well
double MultipleVoltages::CompoundModule::updateOutlineCost(ContiguityAnalysis::ContiguousNeighbour* neighbour, ContiguityAnalysis& cont, bool apply_update) {
	double cost;
	int n_l = neighbour->block->layer;
	double intrusion_area = 0.0;
	bool checked_boundaries = false;
	Rect ext_bb;
	Rect neighbour_ext_bb;
	Rect prev_bb_ext;
	std::vector<Block const*> intruding_blocks;

	if (MultipleVoltages::DBG) {
		if (apply_update) {
			std::cout << "DBG_VOLTAGES>  Update outline cost and power-ring corners; module " << this->id() << ";";
		}
		else {
			std::cout << "DBG_VOLTAGES>  Determine (but don't update) outline cost and power-ring corners; module " << this->id() << ";";
		}
		std::cout << " neighbour block " << neighbour->block->id << "; affected die " << n_l << std::endl;
	}

	// update bounding boxes on (by added block) affected die; note that the added
	// block may be the first on its related die which is assigned to this module;
	// then init new bb
	//
	if (this->outline[n_l].empty()) {

		// apply update only when required
		if (apply_update) {
			this->outline[n_l].emplace_back(neighbour->block->bb);
		}

		// power-ring corners can safely be ignored; adding one rectangular block
		// will not increase the previous max value for power-ring corners

		// the first module's bb will not be intruded per se
		cost = 0.0;
	}
	// update existing bb; try to extend bb to cover previous blocks and the new
	// neighbour block; check for intrusion by any other block
	//
	else {
		// consider the previous bb to be extended by the neighbour; the relevant
		// bb is the last one of the vector (by this just introduced definition);
		// consider local copy and only store in case update shall be applied
		//
		ext_bb = Rect::determBoundingBox(this->outline[n_l].back(), neighbour->block->bb);

		if (MultipleVoltages::DBG) {
			std::cout << "DBG_VOLTAGES>   Currently considered extended bb ";
			std::cout << "(" << ext_bb.ll.x << "," << ext_bb.ll.y << ")";
			std::cout << "(" << ext_bb.ur.x << "," << ext_bb.ur.y << ")" << std::endl;
		}

		// walking vertical boundaries, provided by ContiguityAnalysis; the
		// boundaries of the relevant die will be checked whether they intrude the
		// extended bb, and if so to what degree; note that walking the vertical
		// boundaries is sufficient for determining overlaps in x- and
		// y-dimension; also see ContiguityAnalysis::analyseBlocks
		for (auto i1 = cont.boundaries_vert[n_l].begin(); i1 != cont.boundaries_vert[n_l].end(); ++i1) {

			ContiguityAnalysis::Boundary const& b1 = (*i1);

			// the boundary b2, to be compared to b1, should be within the
			// x-range of the extended bb; thus we initially search for the
			// first boundary (slightly larger than) the extended bb's left
			// x-coordinate
			if (b1.low.x <= ext_bb.ll.x) {
				continue;
			}

			// at this point, a boundary b1 is found which is greater than the
			// lower x-coordinate of the extended bb; check for intruding
			// boundaries/blocks
			for (auto i2 = i1; i2 != cont.boundaries_vert[n_l].end(); ++i2) {

				ContiguityAnalysis::Boundary const& b2 = (*i2);

				// break condition; if b2 is just touching (or later on
				// outside to) the right of extended bb, no intersection
				// if feasible anymore
				if (b2.low.x >= ext_bb.ur.x) {

					checked_boundaries = true;
					break;
				}

				// some intersection _may_ exist, but only for a) blocks
				// not covered in the module yet or not being the
				// neighbour and b) if there is some overlap in
				// y-direction
				//
				// negation of a), ignore such block
				if (b2.block->numerical_id == neighbour->block->numerical_id) {
					continue;
				}
				if (this->block_ids[b2.block->numerical_id] == true) {
					continue;
				}
				// b)
				if (ext_bb.ll.y < b2.high.y && b2.low.y < ext_bb.ur.y) {

					// at this point, we know that b2 is intersecting
					// with extended bb to some degree in _both_
					// dimensions; we may memorize the _potentially_
					// intruding block (in a map to avoid considering
					// blocks two times which may happen when walking
					// the two vertical boundaries of all blocks)
					//
					// finally, look ahead whether this blocks
					// represents an relevant intrusion, i.e., whether
					// the voltages will be different; this can only
					// be addressed conservatively, since the actual
					// assignment is not done yet: when the intruding
					// block has a different set of voltages
					// applicable than the module's current set of
					// voltage we shall assume this block to be
					// intruding at this point, since such neighbours,
					// if merged into the module, will be altering the
					// set of applicable voltages and thus change the
					// module's properties altogether, or, if not
					// merged, will be intruding the module; note that
					// this consideration will always result in
					// trivial neighbours (with only highest voltage
					// applicable) to be rightfully considered as
					// intruding; such modules will not be generated
					// where trivial neighbours are merged into
					//
					if (this->feasible_voltages != b2.block->feasible_voltages) {
						intruding_blocks.push_back(b2.block);
					}
				}
			}

			// break condition, all relevant boundaries have been considered
			if (checked_boundaries) {
				break;
			}
		}
	
		// in case no intrusion would occur, consider the extended bb
		if (intruding_blocks.empty()) {

			if (apply_update) {
				this->outline[n_l].back() = ext_bb;
			}

			if (MultipleVoltages::DBG) {
				std::cout << "DBG_VOLTAGES>   Extended bb is not intruded by any block; consider this extended bb as is" << std::endl;
			}

			// note that no increase in corners for the power rings occurs in
			// such cases; thus they are ignored
		}
		// in case any intrusion would occur, consider only the separate,
		// non-intruded boxes
		//
		// also handle the estimated number of corners in the power rings
		//
		else {
			// consider each intruding block only once, i.e., sort and make
			// vector unique; for efficiency, a vector which is later on
			// sorted and made unique is usually still better than using a
			// sorted set or a hashed map
			//
			std::sort(intruding_blocks.begin(), intruding_blocks.end(),
				// lambda expression for sorting
				[&](Block const* b1, Block const* b2) {
					return b1->numerical_id < b2->numerical_id;
				}
			);
			auto it_last = std::unique(intruding_blocks.begin(), intruding_blocks.end(),
				// lambda expression for equality
				[&](Block const* b1, Block const* b2) {
					return b1->numerical_id == b2->numerical_id;
				}
			);
			// note that the vector must be resized, otherwise its range is
			// not valid in the sense that it may still refer to some
			// redundant entries
			intruding_blocks.resize(std::distance(intruding_blocks.begin(), it_last));

			// add the neighbours (extended) bb and extend the previous bb
			// separately; the extension shall be applied such that number of
			// corners will be minimized, i.e., the bbs should be sized to
			// match the overall bb (enclosing previous bb and neighbour) as
			// close as possible but still considering intruding blocks
			//
			// init copy for extended neighbour bb with actual neighbour bb
			neighbour_ext_bb = neighbour->block->bb;
			// hold reference to actual prev bb; the last in the container by
			// definition
			Rect const& prev_bb = this->outline[n_l].back();
			// init copy for extended prev bb with actual prev bb
			prev_bb_ext = prev_bb;

			// extent both bbs to meet boundaries of overall bb; to do so,
			// increase the bbs separately in the relevant dimensions
			//
			// prev bb and neighbour are vertically intersecting, thus extend
			// the vertical dimensions
			if (Rect::rectsIntersectVertical(neighbour->block->bb, prev_bb)) {
				neighbour_ext_bb.ll.y = prev_bb_ext.ll.y = std::min(neighbour->block->bb.ll.y, prev_bb.ll.y);
				neighbour_ext_bb.ur.y = prev_bb_ext.ur.y = std::max(neighbour->block->bb.ur.y, prev_bb.ur.y);
			}
			// prev bb and neighbour are horizontally intersecting, thus
			// extend the horizontal dimensions
			else if (Rect::rectsIntersectHorizontal(neighbour->block->bb, prev_bb)) {
				neighbour_ext_bb.ll.x = prev_bb_ext.ll.x = std::min(neighbour->block->bb.ll.x, prev_bb.ll.x);
				neighbour_ext_bb.ur.x = prev_bb_ext.ur.x = std::max(neighbour->block->bb.ur.x, prev_bb.ur.x);
			}

			// determine the amount of intersection/intrusion; and ``cut''
			// parts of extended bbs which are intruded
			//
			for (Block const* intruding_block : intruding_blocks) {

				Rect const& cur_intruding_bb = intruding_block->bb;

				// if the intruding block is below the neighbour, consider
				// to limit the lower boundary of the extended bb
				//
				// note that checking for intersection is not required
				// since neighbours are continuous by definition, the same
				// applies to the other cases below
				//
				if (Rect::rectA_below_rectB(cur_intruding_bb, neighbour->block->bb, false)) {
					neighbour_ext_bb.ll.y = std::max(cur_intruding_bb.ur.y, neighbour_ext_bb.ll.y);
				}
				// if the intruding block is above the neighbour, consider
				// to limit the upper boundary of the extended bb
				else if (Rect::rectA_below_rectB(neighbour->block->bb, cur_intruding_bb, false)) {
					neighbour_ext_bb.ur.y = std::min(cur_intruding_bb.ll.y, neighbour_ext_bb.ur.y);
				}

				// if the intruding block is left of the neighbour,
				// consider to limit the left boundary of the extended bb
				if (Rect::rectA_leftOf_rectB(cur_intruding_bb, neighbour->block->bb, false)) {
					neighbour_ext_bb.ll.x = std::max(cur_intruding_bb.ur.x, neighbour_ext_bb.ll.x);
				}
				// if the intruding block is right of the neighbour,
				// consider to limit the right boundary of the extended bb
				else if (Rect::rectA_leftOf_rectB(neighbour->block->bb, cur_intruding_bb, false)) {
					neighbour_ext_bb.ur.x = std::min(cur_intruding_bb.ll.x, neighbour_ext_bb.ur.x);
				}

				// same as above also applies to the prev bb
				//
				// if the intruding block is below the prev bb, consider
				// to limit the lower boundary of the extended bb
				if (Rect::rectA_below_rectB(cur_intruding_bb, prev_bb, false)) {
					prev_bb_ext.ll.y = std::max(cur_intruding_bb.ur.y, prev_bb.ll.y);
				}
				// if the intruding block is above the prev bb, consider
				// to limit the upper boundary of the extended bb
				else if (Rect::rectA_below_rectB(prev_bb, cur_intruding_bb, false)) {
					prev_bb_ext.ur.y = std::min(cur_intruding_bb.ll.y, prev_bb.ur.y);
				}

				// if the intruding block is left of the prev bb,
				// consider to limit the left boundary of the extended bb
				if (Rect::rectA_leftOf_rectB(cur_intruding_bb, prev_bb, false)) {
					prev_bb_ext.ll.x = std::max(cur_intruding_bb.ur.x, prev_bb.ll.x);
				}
				// if the intruding block is right of the prev bb,
				// consider to limit the right boundary of the extended bb
				else if (Rect::rectA_leftOf_rectB(prev_bb, cur_intruding_bb, false)) {
					prev_bb_ext.ur.x = std::min(cur_intruding_bb.ll.x, prev_bb.ur.x);
				}

				// determine the amount of intrusion; only consider the
				// actual intersection
				intrusion_area += Rect::determineIntersection(ext_bb, cur_intruding_bb).area;

				if (MultipleVoltages::DBG) {
					std::cout << "DBG_VOLTAGES>   Extended bb is intruded by block " << intruding_block->id;
					std::cout << "; block bb (" << cur_intruding_bb.ll.x << "," << cur_intruding_bb.ll.y << ")";
					std::cout << "(" << cur_intruding_bb.ur.x << "," << cur_intruding_bb.ur.y << ")";
					std::cout << "; amount of intrusion / area of intersection: " << Rect::determineIntersection(ext_bb, cur_intruding_bb).area << std::endl;
				}
			}

			// memorize the extended bbs if required
			//
			if (apply_update) {

				// recall that prev_bb refers to the previous bb in the
				// outline[n_l] by definition; thus, the extended prev bb
				// shall replace this very previous bb
				this->outline[n_l].back() = prev_bb_ext;

				// store the new, extended bb for the neighbour
				this->outline[n_l].emplace_back(neighbour_ext_bb);
			}

			// also update the number of corners if required
			//
			if (apply_update) {
				// whenever the extended bbs have different coordinates in
				// the extended dimension (due to intruding blocks
				// considered above), two new corners will be introduced
				//
				// prev bb and neighbour are vertically intersecting, thus
				// the vertical dimensions were extended
				if (Rect::rectsIntersectVertical(neighbour->block->bb, prev_bb)) {

					// check both boundaries separately
					if (!Math::doubleComp(neighbour_ext_bb.ll.y, prev_bb_ext.ll.y)) {
						this->corners_powerring[n_l] += 2;
					}
					if (!Math::doubleComp(neighbour_ext_bb.ur.y, prev_bb_ext.ur.y)) {
						this->corners_powerring[n_l] += 2;
					}
				}
				// prev bb and neighbour are horizontally intersecting,
				// thus the horizontal dimensions were extended
				else if (Rect::rectsIntersectHorizontal(neighbour->block->bb, prev_bb)) {

					// check both boundaries separately
					if (!Math::doubleComp(neighbour_ext_bb.ll.x, prev_bb_ext.ll.x)) {
						this->corners_powerring[n_l] += 2;
					}
					if (!Math::doubleComp(neighbour_ext_bb.ur.x, prev_bb_ext.ur.x)) {
						this->corners_powerring[n_l] += 2;
					}
				}
			}
		}

		// calculate cost (amount of intrusion); only required for adapted bbs
		//
		// note that only _current_ intrusion is considered, i.e. the amount of
		// intrusion in any previous merging step is ignored; this is valid since
		// the module was already selected as best-cost module, despite any amount
		// of intrusion, and the separated bb have been memorized, i.e., the
		// starting condition, before considering the neighbour, was a
		// non-intruded module
		//
		cost = intrusion_area / ext_bb.area;
	}

	// update cost if required
	if (apply_update) {
		this->outline_cost = cost;
	}

	return cost;
}

// helper to estimate gain in power reduction
//
// this is done by comparing lowest applicable to highest (trivial solution) voltage /
// power for all (or one specific) comprised blocks; the flag subtract_wasted_saving flag
// required for cost calculation, the flag update is used to perform actual evaluation
// only when required, and the Block pointer may be used to update power saving only
// w.r.t. one specific block
//
inline void MultipleVoltages::CompoundModule::update_power_saving(Block const* block_to_consider) {
	unsigned min_voltage_index = this->min_voltage_index();

	// all blocks shall be considered
	//
	if (block_to_consider == nullptr) {

		// reset previous values
		this->power_saving_ = this->power_saving_wasted_ = 0.0;

		for (Block const* b : this->blocks) {

			// for each block, its power saving is given by the theoretical
			// max power consumption minus the power consumption achieved
			// within this module
			this->power_saving_ += (b->power_max() - b->power(min_voltage_index));

			// consider also the ``wasted saving'', that is the difference in
			// power saving which is not achievable anymore since each block
			// has been assigned to this module which is potentially not the
			// best-case / lowest-voltage / lowest-power module
			this->power_saving_wasted_ += (b->power(min_voltage_index) - b->power_min());
		}
	}
	// only one specific block shall be considered; update values only, no complete
	// recalculation
	//
	else {
		this->power_saving_ += (block_to_consider->power_max() - block_to_consider->power(min_voltage_index));
		this->power_saving_wasted_ += (block_to_consider->power(min_voltage_index) - block_to_consider->power_min());
	}
}

// global cost, required during top-down selection
//
// cost terms: normalized power reduction/saving and number of corners in power rings; the
// smaller the cost the better
//
inline double MultipleVoltages::CompoundModule::cost(double const& max_power_saving, unsigned const& max_corners, MultipleVoltages::Parameters const& parameters) const {
	// for the normalization, the min values are fixed: zero for power-saving (for
	// trivial modules w/ only highest voltage applicable) and four for corners of
	// trivially-shaped(rectangular) modules; add small epsilon to both min values in
	// order to avoid division by zero
	//
	static constexpr double min_corners = 4 + Math::epsilon;
	static constexpr double min_power_saving = Math::epsilon;
	//
	// the max values are derived from all candidate modules; this enables proper
	// judgment of quality of any module in terms of weighted sum of cost terms;
	// however, this does _not_ allow comparisons between different solutions, i.e.,
	// different sets of selected best modules; this will be handled in
	// FloorPlanner::evaluateVoltageAssignment where cost are normalized to initial
	// values, similar like WL or thermal management cost terms
	//

	// this term models the normalized inverse power reduction, with 0 representing
	// max power reduction and 1 representing min power reduction, i.e., smaller cost
	// represents better solutions
	//
	double power_saving_term = 1.0 - ((this->power_saving() - min_power_saving) / (max_power_saving - min_power_saving));

	// this term models the normalized number of corners; 0 represents min corners and
	// 1 represents max corners, i.e., the less corners the smaller the cost term
	//
	double corners_term = (static_cast<double>(this->corners_powerring_max()) - min_corners) / (static_cast<double>(max_corners) - min_corners);

	// return weighted sum of terms
	//
	return (parameters.weight_power_saving * power_saving_term) + (parameters.weight_corners * corners_term);
}

std::string MultipleVoltages::CompoundModule::id() const {
	std::string ret;

	for (unsigned b = 0; b < this->blocks.size() - 1; b++) {
		ret += this->blocks[b]->id + ", ";
	}

	// the last id shall not be followed by a comma
	ret += this->blocks.back()->id;

	return ret;
};
