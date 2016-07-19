/**
 * =====================================================================================
 *
 *    Description:  Corblivar design net
 *
 *    Copyright (C) 2013-2016 Johann Knechtel, johann aett jknechtel dot de
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
#ifndef _CORBLIVAR_NET
#define _CORBLIVAR_NET

// library includes
#include "Corblivar.incl.hpp"
// Corblivar includes, if any
#include "Block.hpp"
#include "Rect.hpp"
#include "TimingPowerAnalyser.hpp"
// forward declarations, if any

/// Corblivar design net
class Net {
	public:
		/// debugging code switch (public; access e.g. from class FloorPlanner)
		static constexpr bool DBG = false;

	// private data, functions
	private:

	// constructors, destructors, if any non-implicit
	public:
		/// default constructor
		Net(std::string const& id) {
			this->id = id;
			this->hasExternalPin = false;
			this->layer_bottom = -1;
			this->layer_top = -1;
			this->clustered = false;
			this->inputNet = this->outputNet = false;
			this->source = nullptr;
		};

	// public data, functions
	public:
		std::string id;
		bool hasExternalPin;
		std::vector<Block const*> blocks;
		std::vector<TSV_Island> TSVs;
		std::vector<Pin const*> terminals;
		mutable int layer_bottom, layer_top;
		mutable bool clustered;

		/// the first block of a net is considered the source/driver, the remaining
		/// blocks/terminals are sinks
		Block const* source;
		/// flag whether nets are global input/output nets, being connected to some
		/// terminal pin
		bool inputNet, outputNet;

		/// the delay value is calculated as max value from source to any sink;
		/// consider only net delay and for this only bbs and number of TSVs, no
		/// precise location of previously placed TSVs since we don't require TSVs
		/// to be placed at this point; assign max value to source block
		///
		inline void assignSourceMaxDelay() const {
			Rect source_sink_bb;

			// sanity check; input nets will have no block assigned as source;
			// assume zero delay
			if (this->inputNet) {
				return;
			}

			if (Net::DBG) {
				std::cout << "DBG_NET> Update _net_ delay for net " << this->id << std::endl;
				std::cout << "DBG_NET>  Driving block: " << this->source->id << std::endl;
			}

			// determine net delay for all possible block-pair relations:
			// analyse bb covering source block to any sink block/terminal
			for (auto const* sink_block : this->blocks) {

				source_sink_bb = Rect::determBoundingBox(this->source->bb, sink_block->bb);

				this->source->net_delay_max = std::max(this->source->net_delay_max,
						// consider HPWL of bb connecting source
						// to sink, also consider number of
						// required TSVs; memorize only max value
						TimingPowerAnalyser::elmoreDelay(source_sink_bb.w + source_sink_bb.h, std::abs(this->source->layer - sink_block->layer)));
			}
			// also consider terminal sinks
			for (auto const* sink_terminal : this->terminals) {

				source_sink_bb = Rect::determBoundingBox(this->source->bb, sink_terminal->bb);

				this->source->net_delay_max = std::max(this->source->net_delay_max,
						TimingPowerAnalyser::elmoreDelay(source_sink_bb.w + source_sink_bb.h, std::abs(this->source->layer - Pin::LAYER)));
			}

			if (Net::DBG) {
				std::cout << "DBG_NET>  Current max delay: " << this->source->net_delay_max << std::endl;
			}
		};

		/// reset helper
		inline void resetSourceMaxDelay() {

			// sanity check; input nets are ignored since they have no driving block
			if (this->inputNet) {
				return;
			}

			this->source->net_delay_max = 0.0;
		};

		/// reset helper
		inline void resetLayerBoundaries() const {

			if (this->blocks.empty()) {
				return;
			}
			else {
				this->layer_bottom = this->layer_top = this->blocks[0]->layer;

				for (Block const* b : this->blocks) {
					this->layer_bottom = std::min(this->layer_bottom, b->layer);
					this->layer_top = std::max(this->layer_top, b->layer);
				}

				// terminals are fixed onto a specific die; consider this
				// die if pins are given
				if (!this->terminals.empty()) {
					this->layer_bottom = std::min(this->layer_bottom, Pin::LAYER);
					this->layer_top = std::max(this->layer_top, Pin::LAYER);
				}
			}
		};


		/// helper to determine net's bb accurately, with consideration of TSVs
		/// and terminal pins
		inline Rect determBoundingBox(int const& layer, bool const& consider_center = false) const {
			int i;
			std::vector<Rect const*> blocks_to_consider;
			bool blocks_above_considered;
			bool TSV_in_layer;
			// dummy return value
			Rect bb;

			if (Net::DBG) {
				std::cout << "DBG_NET>   Determine bb for net " << this->id << " on layer " << layer << std::endl;
			}

			// blocks for cur_net on this layer
			for (Block const* b : this->blocks) {

				// blocks
				if (b->layer == layer) {
					blocks_to_consider.push_back(&b->bb);

					if (Net::DBG) {
						std::cout << "DBG_NET> 	Consider block " << b->id << " on layer " << layer << std::endl;
					}
				}
			}

			// TSV for cur_net on this layer
			TSV_in_layer = false;
			for (TSV_Island const& t : this->TSVs) {

				// TSVs
				if (t.layer == layer) {

					blocks_to_consider.push_back(&t.bb);

					TSV_in_layer = true;

					if (Net::DBG) {
						std::cout << "DBG_NET> 	Consider TSV island " << t.id << " on layer " << layer << std::endl;
					}
				}
			}

			// also consider terminal pins; on fixed layer
			if (layer == Pin::LAYER) {
				for (Pin const* pin :  this->terminals) {

					blocks_to_consider.push_back(&pin->bb);

					if (Net::DBG) {
						std::cout << "DBG_NET> 	Consider terminal pin " << pin->id << std::endl;
					}
				}
			}

			// ignore cases with no blocks on current layer
			if (blocks_to_consider.empty()) {
				return bb;
			}

			// consider blocks on the layer above; required to assume a
			// reasonable bounding box on current layer w/o actual placement
			// of TSVs; the layer to consider is not necessarily the adjacent
			// one, thus stepwise consider layers until some blocks are found
			//
			// note that this is only required when no TSV is placed yet on
			// this layer
			if (!TSV_in_layer) {

				blocks_above_considered = false;
				i = layer + 1;

				while (i <= this->layer_top) {

					for (Block const* b : this->blocks) {
						if (b->layer == i) {
							blocks_to_consider.push_back(&b->bb);
							blocks_above_considered = true;

							if (Net::DBG) {
								std::cout << "DBG_NET> 	Consider block " << b->id << " on layer " << i << std::endl;
							}
						}
					}

					// loop handler
					if (blocks_above_considered) {
						break;
					}
					else {
						i++;
					}
				}
			}

			// also consider TSV from layer below; required to estimated routing to the respective landing pad
			if (layer > 0) {

				for (TSV_Island const& t : this->TSVs) {

					if (t.layer == layer - 1) {
						blocks_to_consider.push_back(&t.bb);

						if (Net::DBG) {
							std::cout << "DBG_NET> 	Consider TSV island " << t.id << " on layer " << layer - 1 << std::endl;
						}
					}
				}
			}

			// ignore cases where only one block on the uppermost
			// layer needs to be considered; these cases are already
			// covered while considering layers below
			if (blocks_to_consider.size() == 1 && layer == this->layer_top) {

				if (Net::DBG) {
					std::cout << "DBG_NET> 	  Ignore single block on uppermost layer" << std::endl;
				}

				return bb;
			}

			return Rect::determBoundingBox(blocks_to_consider, consider_center);
		}
};

#endif
