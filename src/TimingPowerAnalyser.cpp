/*
 * =====================================================================================
 *
 *    Description:  Corblivar handler for timing, delay and power analysis
 *
 *    Copyright (C) 2015-2016 Johann Knechtel, johann aett jknechtel dot de
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
#include "TimingPowerAnalyser.hpp"
// required Corblivar headers
#include "Net.hpp"

// memory allocation
constexpr const char* TimingPowerAnalyser::DAG_Node::SOURCE_ID;
constexpr const char* TimingPowerAnalyser::DAG_Node::SINK_ID;

/// generate DAG (direct acyclic graph) from nets
void TimingPowerAnalyser::initSLSTA(std::vector<Block> const& blocks, std::vector<Pin> const& terminals, std::vector<Net> const& nets, unsigned const& voltages_count, bool const& log, std::string const& benchmark) {

	if (log) {
		std::cout << "TimingPowerAnalyser> ";
		std::cout << "Generate DAG from nets for STA..." << std::endl;
	}

	// reset DAG
	this->nets_DAG.clear();
	this->nets_DAG_sorted.clear();

	// allocate memory for DAG
	this->nets_DAG.reserve(blocks.size() + terminals.size() + 2);
	this->nets_DAG_sorted.reserve(blocks.size() + terminals.size() + 2);

	// init DAG nodes from all the blocks; also allocate slack vectors
	for (Block const& cur_block : blocks) {

		this->nets_DAG.emplace(std::make_pair(
					cur_block.id,
					// index yet unknown
					TimingPowerAnalyser::DAG_Node(&cur_block, voltages_count)
				));

		cur_block.potential_slacks = std::vector<double>(voltages_count, 0.0);
	}

	// also put all terminals (both input/output) into the DAG
	for (Pin const& cur_pin : terminals) {
		this->nets_DAG.emplace(std::make_pair(
					cur_pin.id,
					// index yet unknown
					TimingPowerAnalyser::DAG_Node(&cur_pin, voltages_count)
				));

		// allocate slack vectors as well
		cur_pin.potential_slacks = std::vector<double>(voltages_count, 0.0);
		// for pins, we additionally need to allocate dummy delay factors
		cur_pin.voltages_delay_factors = std::vector<double>(voltages_count, 0.0);
		// for pins, we additionally need to reset a dummy voltage assignment
		cur_pin.resetVoltageAssignment();
	}

	// put global sink
	this->nets_DAG.emplace(std::make_pair(
				// dummy id for sink
				TimingPowerAnalyser::DAG_Node::SINK_ID,
				// index yet unknown
				TimingPowerAnalyser::DAG_Node(&this->dummy_block_DAG_sink, voltages_count)
			));

	// put global source
	this->nets_DAG.emplace(std::make_pair(
				// dummy id for global source
				TimingPowerAnalyser::DAG_Node::SOURCE_ID,
				// has always index 0
				TimingPowerAnalyser::DAG_Node(&this->dummy_block_DAG_source, voltages_count, 0)
			));

	// allocate slack vectors for global source/sink as well
	this->dummy_block_DAG_sink.potential_slacks = std::vector<double>(voltages_count, 0.0);
	this->dummy_block_DAG_source.potential_slacks = std::vector<double>(voltages_count, 0.0);
	// also for global source/sinks, we need to allocate dummy delay factors
	this->dummy_block_DAG_sink.voltages_delay_factors = std::vector<double>(voltages_count, 0.0);
	this->dummy_block_DAG_source.voltages_delay_factors = std::vector<double>(voltages_count, 0.0);
	// also for global source/sinks, we need to reset a dummy voltage assignment
	this->dummy_block_DAG_sink.resetVoltageAssignment();
	this->dummy_block_DAG_source.resetVoltageAssignment();

	// construct the links/pointers for the DAG; simply walk all nets and translate them to parents-children relationships
	//
	for (Net const& n : nets) {

		// input nets have no block as source/driver, but a pin
		//
		if  (n.inputNet) {

			// look into the node representing the input pin
			TimingPowerAnalyser::DAG_Node &pin_node = this->nets_DAG.at(n.terminals.front()->id);

			// memorize pin node as child for the global source
			this->nets_DAG.at(TimingPowerAnalyser::DAG_Node::SOURCE_ID).children.emplace(std::make_pair(
					pin_node.block->id,
					&pin_node
				));

			// also memorize global source as parent of pin node
			pin_node.parents.emplace(std::make_pair(
						TimingPowerAnalyser::DAG_Node::SOURCE_ID,
						&this->nets_DAG.at(TimingPowerAnalyser::DAG_Node::SOURCE_ID)
					));

			// check all the children of the node, i.e., the blocks driven by this net
			//
			for (Block const* block : n.blocks) {

				// look into the node representing the block/child
				TimingPowerAnalyser::DAG_Node &child_node = this->nets_DAG.at(block->id);

				// memorize node/block as child for pin
				pin_node.children.emplace(std::make_pair(
							child_node.block->id,
							&child_node
						));

				// also memorize pin node within the child
				child_node.parents.emplace(std::make_pair(
							pin_node.block->id,
							&pin_node
						));

				// finally, memorize the block<->global_sink relations; for regular netlists with proper output pins, this is not required but also won't hurt, but
				// for netlists without outputs, this is essential; then, all blocks which would otherwise drive nothing are considered to connect to the
				// global_sink, mimicking output drivers
				//
				this->nets_DAG.at(TimingPowerAnalyser::DAG_Node::SINK_ID).parents.emplace(std::make_pair(
						child_node.block->id,
						&child_node
					));
				child_node.children.emplace(std::make_pair(
							TimingPowerAnalyser::DAG_Node::SINK_ID,
							&this->nets_DAG.at(TimingPowerAnalyser::DAG_Node::SINK_ID)
						));
			}

			// also check all the output pins driven by this input net; note that such nets might be rare in practice
			//
			for (Pin const* output_pin : n.terminals) {

				// ignore node representing the input pin
				//
				if (output_pin->id == pin_node.block->id) {
					continue;
				}

				// look into the node representing the output pin
				TimingPowerAnalyser::DAG_Node &child_node = this->nets_DAG.at(output_pin->id);

				// memorize node/pin as child for input pin
				pin_node.children.emplace(std::make_pair(
							child_node.block->id,
							&child_node
						));

				// also memorize input pin within the child/output pin
				child_node.parents.emplace(std::make_pair(
							pin_node.block->id,
							&pin_node
						));

				// finally, memorize the pin<->global_sink relations
				//
				this->nets_DAG.at(TimingPowerAnalyser::DAG_Node::SINK_ID).parents.emplace(std::make_pair(
						child_node.block->id,
						&child_node
					));
				child_node.children.emplace(std::make_pair(
							TimingPowerAnalyser::DAG_Node::SINK_ID,
							&this->nets_DAG.at(TimingPowerAnalyser::DAG_Node::SINK_ID)
						));
			}
		}
		// other regular or output nets have a block as source/driver
		//
		else {
			// look into the node representing the driver
			TimingPowerAnalyser::DAG_Node &driver_node = this->nets_DAG.at(n.source->id);

			// check all the children of the node, i.e., the driven blocks of this net
			//
			for (Block const* block : n.blocks) {

				// ignore node representing the driver
				//
				if (block->id == driver_node.block->id) {
					continue;
				}

				// look into the node representing the child/block
				TimingPowerAnalyser::DAG_Node &child_node = this->nets_DAG.at(block->id);

				// memorize node/block as child for driver
				driver_node.children.emplace(std::make_pair(
							child_node.block->id,
							&child_node
						));

				// also memorize driver node within the child
				child_node.parents.emplace(std::make_pair(
							driver_node.block->id,
							&driver_node
						));

				// finally, memorize the block<->global_sink relations; for regular netlists with proper output pins, this is not required but also won't hurt, but
				// for netlists without outputs, this is essential; then, all blocks which would otherwise drive nothing are considered to connect to the
				// global_sink, mimicking output drivers
				//
				this->nets_DAG.at(TimingPowerAnalyser::DAG_Node::SINK_ID).parents.emplace(std::make_pair(
						child_node.block->id,
						&child_node
					));
				child_node.children.emplace(std::make_pair(
							TimingPowerAnalyser::DAG_Node::SINK_ID,
							&this->nets_DAG.at(TimingPowerAnalyser::DAG_Node::SINK_ID)
						));
			}

			// also check all the output pins driven by this net; note that no input pins are found here, as the related nets are handled separately above
			//
			for (Pin const* output_pin : n.terminals) {

				// look into the node representing the pin
				TimingPowerAnalyser::DAG_Node &child_node = this->nets_DAG.at(output_pin->id);

				// memorize node (output pin) as child for driver
				driver_node.children.emplace(std::make_pair(
							child_node.block->id,
							&child_node
						));

				// also memorize driver node within the child/output pin
				child_node.parents.emplace(std::make_pair(
							driver_node.block->id,
							&driver_node
						));

				// further, memorize output pin (child) as parent for the global sink
				this->nets_DAG.at(TimingPowerAnalyser::DAG_Node::SINK_ID).parents.emplace(std::make_pair(
						child_node.block->id,
						&child_node
					));

				// finally, memorize global sink as child of output pin (child)
				child_node.children.emplace(std::make_pair(
							TimingPowerAnalyser::DAG_Node::SINK_ID,
							&this->nets_DAG.at(TimingPowerAnalyser::DAG_Node::SINK_ID)
						));
			}
		}
	}

	// check for cycles (and resolve them) in the graph
	//
	if (TimingPowerAnalyser::DBG) {
		std::cout << "DBG_TimingPowerAnalyser> Check DAG for cycles (and resolve them)" << std::endl;
	}
	this->resolveCyclesDAG(&this->nets_DAG.at(TimingPowerAnalyser::DAG_Node::SOURCE_ID), log);

	// now, determine all the DAG node topological indices by depth-first search; start with global source
	//
	if (TimingPowerAnalyser::DBG) {
		std::cout << "DBG_TimingPowerAnalyser> Determine topological order/indices for DAG; global source is first (index = 0)" << std::endl;
	}
	this->determIndicesDAG(&this->nets_DAG.at(TimingPowerAnalyser::DAG_Node::SOURCE_ID));

	// finally, order DAG nodes by indices; put the nodes' pointers into separate container
	//
	for (auto const& pair : this->nets_DAG) {
		this->nets_DAG_sorted.push_back(&pair.second);
	}
	std::sort(this->nets_DAG_sorted.begin(), this->nets_DAG_sorted.end(),
			// lambda expression
			[](DAG_Node const* n1, DAG_Node const* n2) {

				// early sanity check to enable that same-element comparisons return false
				return (n1 != n2) && (
						// sort in ascending order of topological indices
						(n1->index < n2->index) ||
						// in case indices are the same, also consider the ID; this way a more natural representation of the ordering will arise assuming
						// that the notations for pins/blocks follow a regular scheme
						((n1->index == n2->index) && (n1->block->id < n2->block->id)) ||
						// in case even the IDs are the same, we can still resort to numerical IDs
						((n1->index == n2->index) && (n1->block->id == n2->block->id) && (n1->block->numerical_id < n2->block->numerical_id))
				       );
			}
		 );

	if (TimingPowerAnalyser::DBG) {

		std::cout << "DBG_TimingPowerAnalyser> Parsed DAG for nets:" << std::endl;

		for (DAG_Node const* node : this->nets_DAG_sorted) {

			std::cout << "DBG_TimingPowerAnalyser>  Node for block/pin " << node->block->id << std::endl;
			std::cout << "DBG_TimingPowerAnalyser>   Topological index of node: " << node->index << std::endl;

			if (!node->children.empty()) {
				std::cout << "DBG_TimingPowerAnalyser>   Children: " << node->children.size() << std::endl;
				for (auto const& child : node->children) {
					std::cout << "DBG_TimingPowerAnalyser>    Child: " << child.first << std::endl;
					std::cout << "DBG_TimingPowerAnalyser>     Index of child: " << child.second->index << std::endl;
				}
			}

			if (!node->parents.empty()) {
				std::cout << "DBG_TimingPowerAnalyser>   Parents: " << node->parents.size() << std::endl;
				for (auto const& parent : node->parents) {
					std::cout << "DBG_TimingPowerAnalyser>    Parent: " << parent.first << std::endl;
					std::cout << "DBG_TimingPowerAnalyser>     Index of parent: " << parent.second->index << std::endl;
				}
			}
		}
	}

	if (log) {
		std::cout << "TimingPowerAnalyser> Done; " << this->nets_DAG_sorted.size() << " nodes created, ";

		// count all edges/children
		int children = 0;
		for (DAG_Node const* node : this->nets_DAG_sorted) {
			children += node->children.size();
		}
		std::cout << children << " unique edges created (not accounting for multiple same-net instances)" << std::endl;
		std::cout << std::endl;
	}

	if (TimingPowerAnalyser::DBG_DAG_DOT) {

		std::cout << "DBG_TimingPowerAnalyser> Output of DAG as it would be required as input for dot graphviz:" << std::endl;
		std::cout << std::endl;

		std::cout << "strict digraph " << benchmark << " {" << std::endl;
		for (DAG_Node const* node : this->nets_DAG_sorted) {
			for (auto const& child : node->children) {
				std::cout << "	" << node->block->id << " -> " << child.first << ";" << std::endl;
			}
		}
		std::cout << "}" << std::endl;
		std::cout << std::endl;
	}
}

bool TimingPowerAnalyser::resolveCyclesDAG(DAG_Node *cur_node, bool const& log) {
	bool cycle_found = false;

	if (TimingPowerAnalyser::DBG_VERBOSE) {
		std::cout << "DBG_TimingPowerAnalyser>  Depth-first traversal of DAG; consider node: " << cur_node->block->id << std::endl;
	}

	// node not visited/checked yet
	//
	if (!cur_node->visited) {

		if (TimingPowerAnalyser::DBG_VERBOSE) {
			std::cout << "DBG_TimingPowerAnalyser>   Proceed with node " << cur_node->block->id <<"; not visited yet; mark as visited and as part of recursion" << std::endl;
		}

		// mark as visited/checked, and also mark as part of this recursion
		cur_node->visited = cur_node->recursion = true;

		// now, check all children in depth-first manner
		//
		unsigned child_id = 1;
		unsigned orig_child_count = cur_node->children.size();
		for (auto it = cur_node->children.begin(); it != cur_node->children.end(); ) {

			DAG_Node* child = (*it).second;

			if (TimingPowerAnalyser::DBG_VERBOSE) {
				std::cout << "DBG_TimingPowerAnalyser>    Consider node " << cur_node->block->id << "'s child: " << child->block->id;
				std::cout << "; child " << child_id << " out of " << orig_child_count << " in total" << std::endl;
				child_id++;
			}

			// child not visited yet; check recursively whether some cycle can be found
			//
			if (!child->visited && this->resolveCyclesDAG(child, log)) {

				if (TimingPowerAnalyser::DBG_VERBOSE) {
					std::cout << "DBG_TimingPowerAnalyser>     Return from recursive check of node " << cur_node->block->id << "'s child: " << child->block->id;
					std::cout << "; at least one cycle was found and resolved; continue with next child ..." << std::endl;
				}

				cycle_found = true;
				++it;
			}
			// child already visited; in case it has been visited during the current recursive call, then we found a cycle/backedge
			// http://www.geeksforgeeks.org/detect-cycle-in-a-graph/
			//
			else if (child->recursion) {

				if (TimingPowerAnalyser::DBG_VERBOSE) {
					std::cout << "DBG_TimingPowerAnalyser>     Cycle found; passed this node " << child->block->id << " already during recursion; resolve and proceed with next child..." << std::endl;
				}

				if (log) {
					std::cout << "TimingPowerAnalyser>  A cycle was found! The following driver-sink relation is deleted to resolve: ";
					std::cout << cur_node->block->id << " -> " << child->block->id << std::endl;
				}

				cycle_found = true;

				// resolve the cycle by deleting the child from the parent (cur_node) which is inducing the cycle; continue with next child
				//
				it = cur_node->children.erase(it);
			}
			// child already visited, but not part of the recursion anymore; represents a transitive edge from one parent node to some child node, which is fine
			// 
			else {
				if (TimingPowerAnalyser::DBG_VERBOSE) {
					std::cout << "DBG_TimingPowerAnalyser>     Cleared node " << cur_node->block->id << "'s child: " << child->block->id << std::endl;
				}

				++it;
			}
		}
	}

	if (TimingPowerAnalyser::DBG_VERBOSE) {
		std::cout << "DBG_TimingPowerAnalyser>  Depth-first traversal of DAG; considering the node " << cur_node->block->id << " is done; mark it as _not_ part of the recursion anymore" << std::endl;
		if (cycle_found) {
			std::cout << "DBG_TimingPowerAnalyser>   At least one cycle was found and resolved" << std::endl;
		}
		else {
			std::cout << "DBG_TimingPowerAnalyser>   No cycle was found" << std::endl;
		}
	}

	// after return from recursion; mark as "not anymore part of a recursion"
	cur_node->recursion = false;

	return cycle_found;
}

void TimingPowerAnalyser::determIndicesDAG(DAG_Node *cur_node) {

	// derive index for current node from maximum among parents
	//
	for (auto const& parent : cur_node->parents) {
		cur_node->index = std::max(cur_node->index, parent.second->index + 1);
	}
	
	if (TimingPowerAnalyser::DBG_VERBOSE) {

		std::cout << "DBG_TimingPowerAnalyser>  Depth-first traversal of DAG; cur_node: " << cur_node->block->id << std::endl;
		std::cout << "DBG_TimingPowerAnalyser>   Topological index: " << cur_node->index << std::endl;

		if (!cur_node->children.empty()) {
			std::cout << "DBG_TimingPowerAnalyser>   Children: " << cur_node->children.size() << std::endl;
			for (auto const& child : cur_node->children) {
				std::cout << "DBG_TimingPowerAnalyser>    Child: " << child.first << std::endl;
				std::cout << "DBG_TimingPowerAnalyser>     Current topological index of child: " << child.second->index << std::endl;
			}
		}
	}

	// traverse all children in depth-first manner
	//
	for (auto &child : cur_node->children) {

		// only traverse when useful; in case the child's index is already larger than the index of the current node, no updates will be possible
		//
		if (child.second->index <= cur_node->index) {

			if (TimingPowerAnalyser::DBG_VERBOSE) {
				std::cout << "DBG_TimingPowerAnalyser>  Depth-first traversal of DAG; continue with child of cur_node: " << cur_node->block->id << std::endl;
			}

			this->determIndicesDAG(child.second);
		}
	}
	
	if (TimingPowerAnalyser::DBG_VERBOSE) {

		std::cout << "DBG_TimingPowerAnalyser>  Depth-first traversal of DAG; done (for now) with cur_node: " << cur_node->block->id << std::endl;
	}
}

void TimingPowerAnalyser::updateTiming(bool const& voltage_assignment, double const& global_arrival_time, int const& voltage_index) {
	DAG_Node* child;
	DAG_Node* parent;
	Rect bb_driver_sink;

	DAG_Node &global_sink = this->nets_DAG.at(TimingPowerAnalyser::DAG_Node::SINK_ID);
	DAG_Node &global_source = this->nets_DAG.at(TimingPowerAnalyser::DAG_Node::SOURCE_ID);

	if (TimingPowerAnalyser::DBG_VERBOSE) {
		if (voltage_index == -1) {
			std::cout << "DBG_TimingPowerAnalyser> Determine timing values for DAG, considering all the block's currently assigned voltages" << std::endl;
		}
		else {
			std::cout << "DBG_TimingPowerAnalyser> Determine timing values for DAG, considering the voltage index " << voltage_index << " for all blocks" << std::endl;
		}
		if (!voltage_assignment) {
			std::cout << "DBG_TimingPowerAnalyser>  No voltage assignment is applied, so we determine only the actual arrival time / system-level latency here..." << std::endl;
		}
	}

	// reset AAT, RAT
	//
	for (auto &pair : this->nets_DAG) {
		pair.second.setAAT(voltage_index, 0);
		pair.second.setRAT(voltage_index, global_arrival_time);
	}

	// first and in any case, compute all arrival times over sorted DAG
	//
	// ignore the very first node, i.e., the global source; there is no physical delay between the global source and the input pins, which are following right after in
	// nets_DAG_sorted
	//
	// also ignore here the very last node, i.e., the global sink; this node is handled as special case below
	//
	for (auto iter = (this->nets_DAG_sorted.begin() + 1); iter != (this->nets_DAG_sorted.end() - 1); ++iter) {

		DAG_Node const* node = *iter;

		if (TimingPowerAnalyser::DBG_VERBOSE) {

			std::cout << "DBG_TimingPowerAnalyser>  Determine AAT for all " << node->children.size() << " children of node: " << node->block->id << std::endl;
			std::cout << "DBG_TimingPowerAnalyser>  (AAT of this node: " << node->getAAT(voltage_index) << ")" << std::endl;
		}

		// propagate AAT from this node to all children
		//
		// note that the global sink is still considered here every now and then, namely when we have an output pin as node; however, always checking whether the child is
		// the global sink is more costly than just recalculating the proper AAT for the global sink as we do below
		//
		for (auto &pair : node->children) {
			child = pair.second;

			if (TimingPowerAnalyser::DBG_VERBOSE) {

				std::cout << "DBG_TimingPowerAnalyser>   Current AAT for node " << child->block->id << ": " << child->getAAT(voltage_index) << std::endl;
			}

			// to estimate the interconnects delay (wires and TSVs), we consider the projected bounding box; it is reasonable to assume that all wires and TSVs will be
			// placed within that box; also consider the centers of the blocks, as we do for interconnect estimation in general
			//
			bb_driver_sink = Rect::determBoundingBox(node->block->bb, child->block->bb, true);

			// now, the AAT for the child is to be calculated considering the driver's AAT, the interconnect delay, and the delay of the child itself
			//
			child->setAAT(voltage_index, std::max(child->getAAT(voltage_index),
					node->getAAT(voltage_index)
					+ TimingPowerAnalyser::elmoreDelay(bb_driver_sink.w + bb_driver_sink.h, std::abs(node->block->layer - child->block->layer))
					+ child->block->delay(voltage_index)
				));

			if (TimingPowerAnalyser::DBG_VERBOSE) {

				std::cout << "DBG_TimingPowerAnalyser>   Updated AAT for node " << child->block->id << ": " << child->getAAT(voltage_index) << std::endl;
				std::cout << "DBG_TimingPowerAnalyser>    Inherent delay for this node: " << child->block->delay(voltage_index) << std::endl;
				std::cout << "DBG_TimingPowerAnalyser>    Elmore delay for connecting " << node->block->id << " to this node: ";
				std::cout << TimingPowerAnalyser::elmoreDelay(bb_driver_sink.w + bb_driver_sink.h, std::abs(node->block->layer - child->block->layer)) << std::endl;
				std::cout << "DBG_TimingPowerAnalyser>     Related HPWL: " << bb_driver_sink.w + bb_driver_sink.h << std::endl;
				std::cout << "DBG_TimingPowerAnalyser>     Related TSVs: " << std::abs(node->block->layer - child->block->layer) << std::endl;
			}
		}
	}
	// now, solve the special case for the global sink; its AAT is simply the maximum among all parents, as there is no physical delay between those parents (the output pins)
	// and the global sink
	//
	// also note that the AAT for the global sink has been set already above; reset first
	global_sink.setAAT(voltage_index, 0);
	for (auto &pair : global_sink.parents) {

		global_sink.setAAT(voltage_index, std::max(
				global_sink.getAAT(voltage_index),
				pair.second->getAAT(voltage_index)
			));
	}

	// the other calculations (for RAT and slack) are only required in case voltage assignment is applied
	//
	if (voltage_assignment) {

		// next, compute the required arrival times over sorted DAG, considering the given critical delay
		//
		// same principle as AAT, just reversed, walking DAG backwards and using parents
		//
		// also ignore the very first node and last nodes (global sink and source), with the same reasoning as for the AAT
		//
		for (auto r_iter = (this->nets_DAG_sorted.rbegin() + 1); r_iter != (this->nets_DAG_sorted.rend() - 1); ++r_iter) {

			DAG_Node const* node = *r_iter;

			if (TimingPowerAnalyser::DBG_VERBOSE) {

				std::cout << "DBG_TimingPowerAnalyser>  Determine RAT for all " << node->parents.size() << " parents of node: " << node->block->id << std::endl;
				std::cout << "DBG_TimingPowerAnalyser>  (RAT of this node: " << node->getRAT(voltage_index) << ")" << std::endl;
			}

			// propagate RAT from this node to all parents
			//
			// note that the global source is still considered here every now and then, namely when we have an input pin as node; however, always checking whether the parent is
			// the global source is more costly than just recalculating the proper RAT for the global source as we do below
			//
			for (auto &pair : node->parents) {
				parent = pair.second;

				if (TimingPowerAnalyser::DBG_VERBOSE) {

					std::cout << "DBG_TimingPowerAnalyser>   Current RAT for node " << parent->block->id << ": " << parent->getRAT(voltage_index) << std::endl;
				}

				// to estimate the interconnects delay (wires and TSVs), we consider the projected bounding box; it is reasonable to assume that all wires and TSVs will be
				// placed within that box; also consider the centers of the blocks, as we do for interconnect estimation in general
				//
				bb_driver_sink = Rect::determBoundingBox(parent->block->bb, node->block->bb, true);

				// now, the RAT for the parent is to be calculated considering the node's RAT, the interconnect delay, and the delay of the parent itself
				//
				parent->setRAT(voltage_index, std::min(parent->getRAT(voltage_index),
						node->getRAT(voltage_index)
						- TimingPowerAnalyser::elmoreDelay(bb_driver_sink.w + bb_driver_sink.h, std::abs(parent->block->layer - node->block->layer))
						- parent->block->delay(voltage_index)
					));

				if (TimingPowerAnalyser::DBG_VERBOSE) {

					std::cout << "DBG_TimingPowerAnalyser>   Updated RAT for node " << parent->block->id << ": " << parent->getRAT(voltage_index) << std::endl;
					std::cout << "DBG_TimingPowerAnalyser>    Inherent delay for this node: " << parent->block->delay(voltage_index) << std::endl;
					std::cout << "DBG_TimingPowerAnalyser>    Elmore delay for connecting this node to node " << node->block->id << ": ";
					std::cout << TimingPowerAnalyser::elmoreDelay(bb_driver_sink.w + bb_driver_sink.h, std::abs(parent->block->layer - node->block->layer)) << std::endl;
					std::cout << "DBG_TimingPowerAnalyser>     Related HPWL: " << bb_driver_sink.w + bb_driver_sink.h << std::endl;
					std::cout << "DBG_TimingPowerAnalyser>     Related TSVs: " << std::abs(parent->block->layer - node->block->layer) << std::endl;
				}
			}
		}
		// now, solve the special case for the global source; its RAT is simply the minimum among all children, as there is no physical delay between those children (the input
		// pins) and the global source
		//
		// also note that the RAT for the global source has been set already above; reset first
		global_source.setRAT(voltage_index, global_arrival_time);
		for (auto &pair : global_source.children) {

			global_source.setRAT(voltage_index, std::min(
					global_source.getRAT(voltage_index),
					pair.second->getRAT(voltage_index)
				));
		}

		// finally, compute the slack for all DAG nodes
		//
		for (auto &pair : this->nets_DAG) {

			DAG_Node &node = pair.second;

			node.setSlack(voltage_index, node.getRAT(voltage_index)- node.getAAT(voltage_index));

			// also memorize the _potential_ slack in the blocks themselves; only required for the cases where we pre-calculate the conservative slack models for all blocks
			// having the same voltage index
			if (voltage_index != -1) {
				node.block->potential_slacks[voltage_index] = node.getSlack(voltage_index);
			}
		}
	}

	if (TimingPowerAnalyser::DBG_VERBOSE) {

		if (voltage_index == -1) {
			std::cout << "DBG_TimingPowerAnalyser> Final timing values for DAG, considering all the block's currently assigned voltages:" << std::endl;
		}
		else {
			std::cout << "DBG_TimingPowerAnalyser> Final timing values for DAG, considering the voltage index " << voltage_index << " for all blocks" << std::endl;
		}
		if (!voltage_assignment) {
			std::cout << "DBG_TimingPowerAnalyser>  No voltage assignment is applied, so only the actual arrival time / system-level latency is valid" << std::endl;
		}

		for (DAG_Node const* node : this->nets_DAG_sorted) {

			std::cout << "DBG_TimingPowerAnalyser>  Node for block/pin " << node->block->id << std::endl;
			std::cout << "DBG_TimingPowerAnalyser>   Topological index: " << node->index << std::endl;
			std::cout << "DBG_TimingPowerAnalyser>   Actual arrival time: " << node->getAAT(voltage_index) << std::endl;
			std::cout << "DBG_TimingPowerAnalyser>   Required arrival time: " << node->getRAT(voltage_index) << std::endl;
			std::cout << "DBG_TimingPowerAnalyser>   Timing slack: " << node->getSlack(voltage_index) << std::endl;
		}
	}
}
