/*
 * =====================================================================================
 *
 *    Description:  Corblivar layout point
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
			this->x = Point::UNDEF;
			this->y = Point::UNDEF;
		};

	// public data, functions
	public:
		static constexpr int UNDEF = -1;

		double x, y;

		inline static double dist(Point const& a, Point const& b) {
			return std::sqrt(std::pow(abs(a.x - b.x), 2) + std::pow(abs(a.y - b.y), 2));
		};
};

#endif
