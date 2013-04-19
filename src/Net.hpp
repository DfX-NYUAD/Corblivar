/*
 * =====================================================================================
 *
 *    Description:  Corblivar design net
 *
 *         Author:  Johann Knechtel, johann.knechtel@ifte.de
 *        Company:  Institute of Electromechanical and Electronic Design, www.ifte.de
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
		Net(int const& id_i) {
			id = id_i;
			hasExternalPin = false;
			layer_bottom = layer_top = -1;
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
