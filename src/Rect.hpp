/**
 * =====================================================================================
 *
 *    Description:  Corblivar layout box
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
#ifndef _CORBLIVAR_RECT
#define _CORBLIVAR_RECT

// library includes
#include "Corblivar.incl.hpp"
// Corblivar includes, if any
#include "Point.hpp"
// forward declarations, if any

/// Corblivar layout box
class Rect {
	// debugging code switch (private)
	private:

	// private data, functions
	private:

	// constructors, destructors, if any non-implicit
	public:
		/// default constructor
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

		/// helper to determine the overall bounding box of multiple rectangles
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

		/// helper to determine the bounding box of two rectangles
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

		/// helper to determine the bounding box of the intersection of two rectangles
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

		/// helper to shift and mitigate any intersection of two rectangles
		//
		/// note that any conditional shifting, e.g., to check for violating the
		/// outline, to apply only shifting in the direction of the lowest shift
		/// required etc may (and will) result in circular moves between different
		/// bbs; for example, when shifting considers the lowest shift and a new bb
		/// has to be shifted between two abutting bbs, the new bb will be shifted
		/// back and forth; thus, greedy shifting has to follow a strict shifting
		/// direction, i.e., upwards
		///
		inline static void greedyShiftingRemoveIntersection(Rect& to_shift, Rect& fixed) {
			Rect intersect;

			intersect = Rect::determineIntersection(to_shift, fixed);

			// the intersection is larger in x-dimension; thus shift in the
			// y-dimension to minimize shifting
			if (intersect.w > intersect.h) {

				// shift to the top
				to_shift.ll.y = fixed.ur.y;
				to_shift.ur.y = to_shift.ll.y + to_shift.h;
			}
			// the intersection is larger in y-dimension; thus shift in the
			// x-dimension to minimize shifting
			else {
				// shift to the right
				to_shift.ll.x = fixed.ur.x;
				to_shift.ur.x = to_shift.ll.x + to_shift.w;
			}
		};

		/// helper to check whether two rectangles have an intersection in the
		/// vertical direction
		inline static bool rectsIntersectVertical(Rect const& a, Rect const& b) {
			return (
					(a.ll.y <= b.ll.y && b.ll.y < a.ur.y) ||
					(b.ll.y <= a.ll.y && a.ll.y < b.ur.y)
				);
		};

		/// helper to check whether two rectangles have an intersection in the
		/// horizontal direction
		inline static bool rectsIntersectHorizontal(Rect const& a, Rect const& b) {
			return (
					(a.ll.x <= b.ll.x && b.ll.x < a.ur.x) ||
					(b.ll.x <= a.ll.x && a.ll.x < b.ur.x)
				);
		};

		/// helper to check whether two rectangles have an intersection in the
		/// horizontal direction
		inline static bool rectsIntersect(Rect const& a, Rect const& b) {
			return rectsIntersectVertical(a, b) && rectsIntersectHorizontal(a, b);
		};

		/// helper to check whether one rectangle is left of the other
		inline static bool rectA_leftOf_rectB(Rect const& a, Rect const& b, bool const& considerVerticalIntersect) {
			return (a.ur.x <= b.ll.x) && (!considerVerticalIntersect || rectsIntersectVertical(a, b));
		};

		/// helper to check whether one rectangle is below the other
		inline static bool rectA_below_rectB(Rect const& a, Rect const& b, bool const& considerHorizontalIntersect) {
			return (a.ur.y <= b.ll.y) && (!considerHorizontalIntersect || rectsIntersectHorizontal(a, b));
		};
};

#endif
