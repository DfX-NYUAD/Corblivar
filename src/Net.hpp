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
	public:

		int id;
		bool hasExternalPin;
		vector<Block*> blocks;
		int layer_bottom, layer_top;

		Net(int const& id_i) {
			id = id_i;
			hasExternalPin = false;
		};

		inline void setLayerBoundaries(int const& globalUpperLayer) {
			this->layer_bottom = globalUpperLayer;
			this->layer_top = 0;

			if (this->blocks.empty()) {
				return;
			}
			else {
				for (Block* b : this->blocks) {
					this->layer_bottom = min(this->layer_bottom, b->layer);
					this->layer_top = max(this->layer_top, b->layer);

				}
			}
		};
};

#endif
