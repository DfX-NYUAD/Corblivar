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
constexpr const char* TimingPowerAnalyser::DAG_SOURCE_ID;
constexpr const char* TimingPowerAnalyser::DAG_SINK_ID;

/// generate DAG (direct acyclic graph) from nets
void TimingPowerAnalyser::initSLSTA(std::vector<Block> const& blocks, std::vector<Pin> const& terminals, std::vector<Net> const& nets, bool const& log) {

	if (log) {
		std::cout << "TimingPowerAnalyser> ";
		std::cout << "Generate DAG from nets for STA..." << std::endl;
	}

	// reset DAG
	this->nets_DAG.clear();

	// allocate memory for DAG
	this->nets_DAG.reserve(blocks.size() + terminals.size() + 2);

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
				TimingPowerAnalyser::DAG_SINK_ID,
				// index yet unknown
				TimingPowerAnalyser::DAG_Node(&this->nets_DAG_sink)
			));

	// put global source
	this->nets_DAG.emplace(std::make_pair(
				// dummy id for global source
				TimingPowerAnalyser::DAG_SOURCE_ID,
				// has always index 0
				TimingPowerAnalyser::DAG_Node(&this->nets_DAG_source, 0)
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
			this->nets_DAG.at(TimingPowerAnalyser::DAG_SOURCE_ID).children.emplace(std::make_pair(
					pin_node.block->id,
					&pin_node
				));

			// also memorize global source as parent of pin node
			pin_node.parents.emplace(std::make_pair(
						TimingPowerAnalyser::DAG_SOURCE_ID,
						&this->nets_DAG.at(TimingPowerAnalyser::DAG_SOURCE_ID)
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
				this->nets_DAG.at(TimingPowerAnalyser::DAG_SINK_ID).parents.emplace(std::make_pair(
						child_node.block->id,
						&child_node
					));

				// finally, memorize global sink as child of output pin (child)
				child_node.children.emplace(std::make_pair(
							TimingPowerAnalyser::DAG_SINK_ID,
							&this->nets_DAG.at(TimingPowerAnalyser::DAG_SINK_ID)
						));
			}
		}
	}

	// now, determine all the DAG node topological indices by depth-first search; start with global source
	//
	this->determIndicesDAG(&this->nets_DAG.at(TimingPowerAnalyser::DAG_SOURCE_ID));

	// finally, order DAG nodes by indices
	// TODO

	if (TimingPowerAnalyser::DBG) {

		std::cout << "DBG_TimingPowerAnalyser> Parsed DAG for nets:" << std::endl;

		for (auto const& pair : this->nets_DAG) {

			TimingPowerAnalyser::DAG_Node const& node = pair.second;

			std::cout << "DBG_TimingPowerAnalyser>  Node for block/pin " << node.block->id << std::endl;
			std::cout << "DBG_TimingPowerAnalyser>   Topological index of node: " << node.index << std::endl;

			if (!node.children.empty()) {
				std::cout << "DBG_TimingPowerAnalyser>   Children: " << node.children.size() << std::endl;
				for (auto const& child : node.children) {
					std::cout << "DBG_TimingPowerAnalyser>    Child: " << child.first << std::endl;
					std::cout << "DBG_TimingPowerAnalyser>     Index of child: " << child.second->index << std::endl;
				}
			}

			if (!node.parents.empty()) {
				std::cout << "DBG_TimingPowerAnalyser>   Parents: " << node.parents.size() << std::endl;
				for (auto const& parent : node.parents) {
					std::cout << "DBG_TimingPowerAnalyser>    Parent: " << parent.first << std::endl;
					std::cout << "DBG_TimingPowerAnalyser>     Index of parent: " << parent.second->index << std::endl;
				}
			}
		}
	}

	if (log) {
		std::cout << "TimingPowerAnalyser> Done; " << this->nets_DAG.size() << " nodes created" << std::endl;
		std::cout << std::endl;
	}
}

void TimingPowerAnalyser::determIndicesDAG(DAG_Node const* cur_node) {

	// derive index for current node from maximum among parents
	//
	for (auto const& parent : cur_node->parents) {
		cur_node->index = std::max(cur_node->index, parent.second->index + 1);
	}
	
	if (TimingPowerAnalyser::DBG_VERBOSE) {

		std::cout << "DBG_TimingPowerAnalyser> Depth-first traversal of DAG; cur_node: " << cur_node->block->id << std::endl;
		std::cout << "DBG_TimingPowerAnalyser>  Topological index: " << cur_node->index << std::endl;

		if (!cur_node->children.empty()) {
			std::cout << "DBG_TimingPowerAnalyser>  Children: " << cur_node->children.size() << std::endl;
			for (auto const& child : cur_node->children) {
				std::cout << "DBG_TimingPowerAnalyser>   Child: " << child.first << std::endl;
				std::cout << "DBG_TimingPowerAnalyser>    Current topological index of child: " << child.second->index << std::endl;
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
				std::cout << "DBG_TimingPowerAnalyser> Depth-first traversal of DAG; continue with child of cur_node: " << cur_node->block->id << std::endl;
			}

			this->determIndicesDAG(child.second);
		}
	}
	
	if (TimingPowerAnalyser::DBG_VERBOSE) {

		std::cout << "DBG_TimingPowerAnalyser> Depth-first traversal of DAG; done (for now) with cur_node: " << cur_node->block->id << std::endl;
	}
}
