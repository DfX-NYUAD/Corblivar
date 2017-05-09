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
void TimingPowerAnalyser::initSLSTA(std::vector<Block> const& blocks, std::vector<Pin> const& terminals, std::vector<Net> const& nets, bool const& log) {

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

	// init DAG nodes from all the blocks
	for (Block const& cur_block : blocks) {
		this->nets_DAG.emplace(std::make_pair(
					cur_block.id,
					// index yet unknown
					TimingPowerAnalyser::DAG_Node(&cur_block)
				));
	}

	// also put all terminals (both input/output) into the DAG
	for (Pin const& cur_pin : terminals) {
		this->nets_DAG.emplace(std::make_pair(
					cur_pin.id,
					// index yet unknown
					TimingPowerAnalyser::DAG_Node(&cur_pin)
				));
	}

	// put global sink
	this->nets_DAG.emplace(std::make_pair(
				// dummy id for sink
				TimingPowerAnalyser::DAG_Node::SINK_ID,
				// index yet unknown
				TimingPowerAnalyser::DAG_Node(&this->dummy_block_DAG_sink)
			));

	// put global source
	this->nets_DAG.emplace(std::make_pair(
				// dummy id for global source
				TimingPowerAnalyser::DAG_Node::SOURCE_ID,
				// has always index 0
				TimingPowerAnalyser::DAG_Node(&this->dummy_block_DAG_source, 0)
			));

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

				return (
						// sort in ascending order of topological indices
						(n1->index < n2->index) ||
						// in case indices are the same, also consider the ID; this way a more natural representation of the ordering will arise assuming
						// that the notations for pins/blocks follow a regular scheme
						((n1->index == n2->index) && (n1->block->id < n2->block->id))
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
}

bool TimingPowerAnalyser::resolveCyclesDAG(DAG_Node *cur_node, bool const& log) {

	// node not visited/checked yet
	//
	if (!cur_node->visited) {

		if (TimingPowerAnalyser::DBG_VERBOSE) {
			std::cout << "DBG_TimingPowerAnalyser>  Depth-first traversal of DAG; cur_node: " << cur_node->block->id << std::endl;
		}

		// mark as visited/checked, and also mark as part of this recursion
		cur_node->visited = cur_node->recursion = true;

		// now, check all children in depth-first manner
		//
		for (auto &child : cur_node->children) {

			// child not visited yet; check recursively whether some cycle can be found
			//
			if (!child.second->visited && this->resolveCyclesDAG(child.second, log)) {

				// in case a cycle was found, resolve it naively by deleting the child which was associated with the cycle
				cur_node->children.erase(child.first);

				if (log) {
					std::cout << "TimingPowerAnalyser> ";
					std::cout << " A cycle was found in the DAG/netlist! The following driver-sink relation was deleted: ";
					std::cout << cur_node->block->id << "->" << child.first << std::endl;
					std::cout << "TimingPowerAnalyser> ";
					std::cout << "  Please check and revise the netlist accordingly!" << std::endl;
				}

				return true;
			}
			// child already visited; in case it has been visited during the current recursive call, then we found a cycle/backedge
			// http://www.geeksforgeeks.org/detect-cycle-in-a-graph/
			//
			else if (child.second->recursion) {

				// in case a cycle was found, resolve it naively by deleting the child which was associated with the cycle
				cur_node->children.erase(child.first);

				if (log) {
					std::cout << "TimingPowerAnalyser> ";
					std::cout << " A cycle was found in the DAG/netlist! The following driver-sink relation was deleted: ";
					std::cout << cur_node->block->id << "->" << child.first << std::endl;
					std::cout << "TimingPowerAnalyser> ";
					std::cout << "  Please check and revise the netlist accordingly!" << std::endl;
				}

				return true;
			}
		}
	}

	// after return from recursion; mark as "not anymore part of a recursion"
	cur_node->recursion = false;

	// also, at this point, it's clear that this node is not part of a cycle
	return false;
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

void TimingPowerAnalyser::updateTiming() {
	DAG_Node* child;
	Rect bb_driver_sink;

	if (TimingPowerAnalyser::DBG_VERBOSE) {
		std::cout << "DBG_TimingPowerAnalyser> Determine timing values for DAG" << std::endl;
	}

	// reset AAT, RAT
	//
	for (auto &pair : this->nets_DAG) {
		pair.second.AAT = 0;
		// TODO
		pair.second.RAT = 15;
	}

	// first, compute all arrival times over sorted DAG
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
			std::cout << "DBG_TimingPowerAnalyser>  (AAT of this node: " << node->AAT << ")" << std::endl;
		}

		// propagate AAT from this node to all children
		//
		// note that the global sink is still considered here every now and then, namely when we have an output pin as node; however, always checking whether the child is
		// the global sink is more costly than just recalculating the proper AAT for the global sink as we do below
		//
		for (auto &pair : node->children) {
			child = pair.second;

			if (TimingPowerAnalyser::DBG_VERBOSE) {

				std::cout << "DBG_TimingPowerAnalyser>   Current AAT for node " << child->block->id << ": " << child->AAT << std::endl;
			}

			// to estimate the interconnects delay (wires and TSVs), we consider the projected bounding box; it is reasonable to assume that all wires and TSVs will be
			// placed within that box
			//
			bb_driver_sink = Rect::determBoundingBox(node->block->bb, child->block->bb);

			// now, the AAT for the child is to be calculated considering the driver's AAT, the interconnect delay, and the delay of the child itself
			//
			child->AAT = std::max(child->AAT,
					node->AAT
					+ TimingPowerAnalyser::elmoreDelay(bb_driver_sink.w + bb_driver_sink.h, std::abs(node->block->layer - child->block->layer))
					+ child->block->delay()
				);

			if (TimingPowerAnalyser::DBG_VERBOSE) {

				std::cout << "DBG_TimingPowerAnalyser>   Updated AAT for node " << child->block->id << ": " << child->AAT << std::endl;
				std::cout << "DBG_TimingPowerAnalyser>    Inherent delay for this node: " << child->block->delay() << std::endl;
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
	this->nets_DAG.at(TimingPowerAnalyser::DAG_Node::SINK_ID).AAT = 0;
	for (auto &pair : this->nets_DAG.at(TimingPowerAnalyser::DAG_Node::SINK_ID).parents) {

		this->nets_DAG.at(TimingPowerAnalyser::DAG_Node::SINK_ID).AAT = std::max(
				this->nets_DAG.at(TimingPowerAnalyser::DAG_Node::SINK_ID).AAT,
				pair.second->AAT
			);
	}

	// next, compute the required arrival times over sorted DAG, considering the given critical delay
	// TODO

	// finally, compute the slack for all DAG nodes
	//
	for (auto &pair : this->nets_DAG) {
		pair.second.slack = pair.second.RAT - pair.second.AAT;
	}

	if (TimingPowerAnalyser::DBG_VERBOSE) {

		std::cout << "DBG_TimingPowerAnalyser> Final timing values for DAG:" << std::endl;

		for (DAG_Node const* node : this->nets_DAG_sorted) {

			std::cout << "DBG_TimingPowerAnalyser>  Node for block/pin " << node->block->id << std::endl;
			std::cout << "DBG_TimingPowerAnalyser>   Topological index: " << node->index << std::endl;
			std::cout << "DBG_TimingPowerAnalyser>   Actual arrival time: " << node->AAT << std::endl;
			std::cout << "DBG_TimingPowerAnalyser>   Required arrival time: " << node->RAT << std::endl;
			std::cout << "DBG_TimingPowerAnalyser>   Timing slack: " << node->slack << std::endl;
		}
	}
}
