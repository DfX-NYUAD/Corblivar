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
// forward declarations, if any

class Net {
	// debugging code switch (private)
	private:

	// private data, functions
	private:

	// constructors, destructors, if any non-implicit
	public:
		Net(int const& id) {
			this->id = id;
			this->hasExternalPin = false;
			this->layer_bottom = -1;
			this->layer_top = -1;
		};

	// public data, functions
	public:
		int id;
		bool hasExternalPin;
		vector<Block const*> blocks;
		vector<Block const*> terminals;
		int layer_bottom, layer_top;

		inline void setLayerBoundaries() {

			if (this->blocks.empty()) {
				return;
			}
			else {
				this->layer_bottom = this->layer_top = this->blocks[0]->layer;

				for (Block const* b : this->blocks) {
					this->layer_bottom = min(this->layer_bottom, b->layer);
					this->layer_top = max(this->layer_top, b->layer);
				}
			}
		};
};

#endif
