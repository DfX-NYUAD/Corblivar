/*
 * =====================================================================================
 *
 *    Description:  Corblivar layout point
 *
 *         Author:  Johann Knechtel, johann.knechtel@ifte.de
 *        Company:  Institute of Electromechanical and Electronic Design, www.ifte.de
 *
 * =====================================================================================
 */
#ifndef _CORBLIVAR_POINT
#define _CORBLIVAR_POINT

// library includes
#include "Corblivar.incl.hpp"
// Corblivar includes, if any
// forward declarations, if any

class Point {
	// debugging code switch (private)
	private:

	// private data, functions
	private:

	// constructors, destructors, if any non-implicit
	public:
		Point() {
			x = y = UNDEF;
		};

	// public data, functions
	public:
		static constexpr int UNDEF = -1;

		double x, y;

		inline static double dist(Point const& a, Point const& b) {
			return sqrt(pow(abs(a.x - b.x), 2) + pow(abs(a.y - b.y), 2));
		};
};

#endif
