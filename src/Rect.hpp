/*
 * =====================================================================================
 *
 *    Description:  Corblivar layout box
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
#ifndef _CORBLIVAR_RECT
#define _CORBLIVAR_RECT

// library includes
#include "Corblivar.incl.hpp"
// Corblivar includes, if any
#include "Point.hpp"
#include "Math.hpp"
// forward declarations, if any

class Rect {
	// debugging code switch (private)
	private:

	// private data, functions
	private:

	// constructors, destructors, if any non-implicit
	public:
		Rect() {
			this->h = 0.0;
			this->w = 0.0;
			this->area = 0.0;
		};

	// public data, functions
	public:
		Point ll, ur;
		double h, w;
		double area;

		inline static Rect determBoundingBox(std::vector<Rect const*> const& rects, bool const& consider_center = false) {
			Rect ret;

			if (rects.empty()) {
				ret.ll.x = ret.ll.y = ret.ur.x = ret.ur.y = Point::UNDEF;
				ret.h = ret.w = ret.area = Point::UNDEF;
			}
			else {
				if (consider_center) {

					// init w/ center point of first rect
					ret.ll.x = ret.ur.x = rects[0]->ll.x + rects[0]->w / 2.0;
					ret.ll.y = ret.ur.y = rects[0]->ll.y + rects[0]->h / 2.0;

					// determine bounding box off all rects based on
					// center points of rects
					for (Rect const* r : rects) {
						ret.ll.x = std::min(ret.ll.x, r->ll.x + r->w / 2.0);
						ret.ur.x = std::max(ret.ur.x, r->ll.x + r->w / 2.0);
						ret.ll.y = std::min(ret.ll.y, r->ll.y + r->h / 2.0);
						ret.ur.y = std::max(ret.ur.y, r->ll.y + r->h / 2.0);
					}
				}
				else {

					// init w/ box of first rect
					ret.ll = rects[0]->ll;
					ret.ur = rects[0]->ur;

					// determine bounding box of all rects based on
					// boxes of rects
					for (Rect const* r : rects) {
						ret.ll.x = std::min(ret.ll.x, r->ll.x);
						ret.ll.y = std::min(ret.ll.y, r->ll.y);
						ret.ur.x = std::max(ret.ur.x, r->ur.x);
						ret.ur.y = std::max(ret.ur.y, r->ur.y);
					}
				}

				ret.w = ret.ur.x - ret.ll.x;
				ret.h = ret.ur.y - ret.ll.y;
				ret.area = ret.w * ret.h;
			}

			return ret;
		};

		inline static Rect determBoundingBox(Rect const& r1, Rect const& r2, bool const& consider_center = false) {
			Rect ret;

			// determine bounding box considering minx, max ranges of both
			// rects; possibly consider center points as well
			if (consider_center) {
				ret.ll.x = std::min(r1.ll.x + r1.w / 2.0, r2.ll.x + r2.w / 2.0);
				ret.ll.y = std::min(r1.ll.y + r1.h / 2.0, r2.ll.y + r2.h / 2.0);
				ret.ur.x = std::max(r1.ll.x + r1.w / 2.0, r2.ll.x + r2.w / 2.0);
				ret.ur.y = std::max(r1.ll.y + r1.h / 2.0, r2.ll.y + r2.h / 2.0);
			}
			else {
				ret.ll.x = std::min(r1.ll.x, r2.ll.x);
				ret.ll.y = std::min(r1.ll.y, r2.ll.y);
				ret.ur.x = std::max(r1.ur.x, r2.ur.x);
				ret.ur.y = std::max(r1.ur.y, r2.ur.y);
			}

			// determine rect properties
			ret.w = ret.ur.x - ret.ll.x;
			ret.h = ret.ur.y - ret.ll.y;
			ret.area = ret.w * ret.h;

			return ret;
		};

		inline static Rect determineIntersection(Rect const& a, Rect const& b) {
			Rect ret;

			// left edge of b within a
			if (a.ll.x <= b.ll.x && b.ll.x <= a.ur.x) {
				ret.ll.x = b.ll.x;
				// right edge: minimum of ur.x
				ret.ur.x = std::min(a.ur.x, b.ur.x);
			}
			// left edge of a within b
			else if (b.ll.x <= a.ll.x && a.ll.x <= b.ur.x) {
				ret.ll.x = a.ll.x;
				// right edge: minimum of ur.x
				ret.ur.x = std::min(a.ur.x, b.ur.x);
			}
			// no intersection
			else {
				ret.ll.x = ret.ur.x = Point::UNDEF;
			}

			// bottom edge of b within a
			if (a.ll.y <= b.ll.y && b.ll.y <= a.ur.y) {
				ret.ll.y = b.ll.y;
				// top edge: minimum of ur.y
				ret.ur.y = std::min(a.ur.y, b.ur.y);
			}
			// bottom edge of a within b
			else if (b.ll.y <= a.ll.y && a.ll.y <= b.ur.y) {
				ret.ll.y = a.ll.y;
				// top edge: minimum of ur.y
				ret.ur.y = std::min(a.ur.y, b.ur.y);
			}
			// no intersection
			else {
				ret.ll.y = ret.ur.y = Point::UNDEF;
			}

			ret.w = ret.ur.x - ret.ll.x;
			ret.h = ret.ur.y - ret.ll.y;
			ret.area = ret.w * ret.h;

			return ret;
		};

		inline static void greedyShiftingRemoveIntersection(Rect& a, Rect& b, bool shift_only_a = true) {
			Rect intersect;

			intersect = Rect::determineIntersection(a, b);

			// the intersection is larger in y-dimension, shift in the
			// y-dimension; although that's counterintuitive and should imply
			// shifting in x-dimension, the latter results in ``elongated
			// rows'' of shifted blocks in case many are initially nearby and
			// are stepwise shifted
			//
			if (intersect.h > intersect.w) {

				// A is below B and shifting B is allowed; shift B to the top
				if (Rect::rectA_below_rectB(a, b, false) && !shift_only_a) {

					b.ll.y += intersect.h;
					b.ur.y += intersect.h;
				}
				// B is below A and/or shifting B is not allowed; shift A to the top
				else {
					a.ll.y += intersect.h;
					a.ur.y += intersect.h;
				}
			}
			// the intersection is larger in x-dimension, shift in the
			// x-dimension
			else {
				// A is left of B and shifting B is allowed; shift B to the right
				if (Rect::rectA_leftOf_rectB(a, b, false) && !shift_only_a) {

					b.ll.x += intersect.w;
					b.ur.x += intersect.w;
				}
				// B is left of A and/or shifting B is not allowed; shift A to the right
				else {
					a.ll.x += intersect.w;
					a.ur.x += intersect.w;
				}
			}
		};

		inline static bool rectsIntersectVertical(Rect const& a, Rect const& b) {
			return (
					(a.ll.y <= b.ll.y && b.ll.y < a.ur.y) ||
					(b.ll.y <= a.ll.y && a.ll.y < b.ur.y)
				);
		};

		inline static bool rectsIntersectHorizontal(Rect const& a, Rect const& b) {
			return (
					(a.ll.x <= b.ll.x && b.ll.x < a.ur.x) ||
					(b.ll.x <= a.ll.x && a.ll.x < b.ur.x)
				);
		};

		inline static bool rectsIntersect(Rect const& a, Rect const& b) {
			return rectsIntersectVertical(a, b) && rectsIntersectHorizontal(a, b);
		};

		inline static bool rectA_leftOf_rectB(Rect const& a, Rect const& b, bool const& considerVerticalIntersect) {
			return (a.ur.x <= b.ll.x) && (!considerVerticalIntersect || rectsIntersectVertical(a, b));
		};

		inline static bool rectA_below_rectB(Rect const& a, Rect const& b, bool const& considerHorizontalIntersect) {
			return (a.ur.y <= b.ll.y) && (!considerHorizontalIntersect || rectsIntersectHorizontal(a, b));
		};
};

#endif
