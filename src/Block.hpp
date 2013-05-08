/*
 * =====================================================================================
 *
 *    Description:  Corblivar design block
 *
 *         Author:  Johann Knechtel, johann.knechtel@ifte.de
 *        Company:  Institute of Electromechanical and Electronic Design, www.ifte.de
 *
 * =====================================================================================
 */
#ifndef _CORBLIVAR_BLOCK
#define _CORBLIVAR_BLOCK

// library includes
#include "Corblivar.incl.hpp"
// Corblivar includes, if any
#include "Rect.hpp"
#include "Math.hpp"
// forward declarations, if any

class Block {
	// debugging code switch (private)
	private:

	// private data, functions
	private:

	// constructors, destructors, if any non-implicit
	public:
		Block(string const& id_i) {
			id = id_i;
			layer = -1;
			power_density= 0.0;
			AR.min = AR.max = 1.0;
			soft = false;
			floorplacement = false;
			//x_slack_backward = y_slack_backward = 0.0;
			//x_slack_forward = y_slack_forward = 0.0;
		};

	// public data, functions
	public:
		string id;
		mutable int layer;

		// density in [uW/(um^2)]
		double power_density;

		//double x_slack_backward, y_slack_backward;
		//double x_slack_forward, y_slack_forward;

		// rectangle, represents block geometry and placement
		mutable Rect bb, bb_backup, bb_best;

		// aspect ratio AR, relates to blocks' dimensions by x / y
		struct AR {
			double min;
			double max;
		} AR;
		// AR is only relevant for soft blocks
		bool soft;

		// large macro, flag for floorplacement handling
		bool floorplacement;

		// layout-generation related helper; perform operations on mutable bb,
		// thus marked const
		//
		inline void rotate() const {
			swap(this->bb.w, this->bb.h);
		};
		inline void shapeRandomlyByAR() const {

			// reshape block randomly w/in AR range; note that x^2 = AR * A
			this->bb.w = sqrt(Math::randF(this->AR.min, this->AR.max) * this->bb.area);
			this->bb.h = this->bb.area / this->bb.w;
		}

		// power in [W]
		inline double power() const {
			// power density is given in uW/um^2, area is given in um^2, thus
			// we have to convert uW to W
			return this->power_density * this->bb.area * 1.0e-6;
		}

		// search blocks
		static inline Block const* findBlock(string const& id, vector<Block> const& container) {

			for (Block const& b : container) {
				if (b.id == id) {
					return &b;
				}
			}

			return nullptr;
		};
};

#endif
