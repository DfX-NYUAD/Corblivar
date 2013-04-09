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
// forward declarations, if any

class Block {
	// debugging code switch (private)
	private:

	// private data, functions
	private:

	// constructors, destructors, if any non-implicit
	public:
		Block(int const& id_i) {
			id = id_i;
			layer = -1;
			power_density= 0.0;
			//x_slack_backward = y_slack_backward = 0.0;
			//x_slack_forward = y_slack_forward = 0.0;
		};

	// public data, functions
	public:
		int id;
		mutable int layer;
		// density in [uW/(um^2)]
		double power_density;
		//double x_slack_backward, y_slack_backward;
		//double x_slack_forward, y_slack_forward;
		mutable Rect bb, bb_backup, bb_best;

		// power in [W]
		inline double power() const {
			return this->power_density * this->bb.area * 1.0e-6;
		}
};

#endif
