/*
 * =====================================================================================
 *
 *    Description:  Corblivar design net
 *
 *    Copyright (C) 2013 Johann Knechtel, johann.knechtel@ifte.de, www.ifte.de
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
// forward declarations, if any

class Net {
	// debugging code switch (public; access e.g. from class FloorPlanner)
	public:
		static constexpr bool DBG = false;

	// private data, functions
	private:

	// constructors, destructors, if any non-implicit
	public:
		Net(int const& id) {
			this->id = id;
			this->hasExternalPin = false;
			this->layer_bottom = -1;
			this->layer_top = -1;
			this->clustered = false;
		};

	// public data, functions
	public:
		int id;
		bool hasExternalPin;
		vector<Block const*> blocks;
		vector<Pin const*> terminals;
		mutable int layer_bottom, layer_top;
		mutable bool clustered;

		inline void setLayerBoundaries() const {

			if (this->blocks.empty()) {
				return;
			}
			else {
				this->layer_bottom = this->layer_top = this->blocks[0]->layer;

				for (Block const* b : this->blocks) {
					this->layer_bottom = min(this->layer_bottom, b->layer);
					this->layer_top = max(this->layer_top, b->layer);
				}

				// terminals have to be routed through die 0, that means
				// when terminals exist for this net, the lowermost die is
				// die 0
				if (!this->terminals.empty()) {
					this->layer_bottom = 0;
				}
			}
		};


		inline Rect determBoundingBox(int const& layer) const {
			int i;
			vector<Rect const*> blocks_to_consider;
			bool blocks_above_considered;
			// dummy return value
			Rect bb;

			if (Net::DBG) {
				cout << "DBG_NET>   Determine bb for net " << this->id << " on layer " << layer << endl;
			}

			// blocks / pins for cur_net on this layer
			for (Block const* b : this->blocks) {

				// blocks
				if (b->layer == layer) {
					blocks_to_consider.push_back(&b->bb);

					if (Net::DBG) {
						cout << "DBG_NET> 	Consider block " << b->id << " on layer " << layer << endl;
					}
				}

				// also consider routes to terminal pins; only on lowest
				// die of stack since connections b/w terminal pins and
				// blocks on upper dies are routed through the TSV in that
				// lowermost die
				if (layer == 0) {
					for (Pin const* pin :  this->terminals) {
						blocks_to_consider.push_back(&pin->bb);

						if (Net::DBG) {
							cout << "DBG_NET> 	Consider terminal pin " << pin->id << endl;
						}
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
			blocks_above_considered = false;
			i = layer + 1;
			while (i <= this->layer_top) {
				for (Block const* b : this->blocks) {
					if (b->layer == i) {
						blocks_to_consider.push_back(&b->bb);
						blocks_above_considered = true;

						if (Net::DBG) {
							cout << "DBG_NET> 	Consider block " << b->id << " on layer " << i << endl;
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

			// ignore cases where only one block on the uppermost
			// layer needs to be considered; these cases are already
			// covered while considering layers below
			if (blocks_to_consider.size() == 1 && layer == this->layer_top) {

				if (Net::DBG) {
					cout << "DBG_NET> 	Ignore single block on uppermost layer" << endl;
				}

				return bb;
			}

			return Rect::determBoundingBox(blocks_to_consider);
		}
};

#endif
