/*
 * =====================================================================================
 *
 *    Description:  Corblivar layout box
 *
 *         Author:  Johann Knechtel, johann.knechtel@ifte.de
 *        Company:  Institute of Electromechanical and Electronic Design, www.ifte.de
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

class Rect {
	// debugging code switch (private)
	private:

	// private data, functions
	private:

	// constructors, destructors, if any non-implicit
	public:
		Rect() {
			h = w = area = 0.0;
		};

	// public data, functions
	public:
		Point ll, ur;
		double h, w;
		double area;

		inline static Rect determBoundingBox(vector<Rect const*> const& rects) {
			Rect ret;

			if (rects.empty()) {
				ret.ll.x = ret.ll.y = ret.ur.x = ret.ur.y = Point::UNDEF;
				ret.h = ret.w = ret.area = Point::UNDEF;
			}
			else {
				ret.ll = rects[0]->ll;
				ret.ur = rects[0]->ur;
				for (Rect const* r : rects) {
					ret.ll.x = min(ret.ll.x, r->ll.x);
					ret.ll.y = min(ret.ll.y, r->ll.y);
					ret.ur.x = max(ret.ur.x, r->ur.x);
					ret.ur.y = max(ret.ur.y, r->ur.y);
				}
				ret.w = ret.ur.x - ret.ll.x;
				ret.h = ret.ur.y - ret.ll.y;
				ret.area = ret.w * ret.h;
			}

			return ret;
		};

		inline static Rect determineIntersection(Rect const& a, Rect const& b) {
			Rect ret;

			// left edge of b within a
			if (a.ll.x <= b.ll.x && b.ll.x <= a.ur.x) {
				ret.ll.x = b.ll.x;
				// right edge: minimum of ur.x
				ret.ur.x = min(a.ur.x, b.ur.x);
			}
			// left edge of a within b
			else if (b.ll.x <= a.ll.x && a.ll.x <= b.ur.x) {
				ret.ll.x = a.ll.x;
				// right edge: minimum of ur.x
				ret.ur.x = min(a.ur.x, b.ur.x);
			}
			// no intersection
			else {
				ret.ll.x = ret.ur.x = Point::UNDEF;
			}

			// bottom edge of b within a
			if (a.ll.y <= b.ll.y && b.ll.y <= a.ur.y) {
				ret.ll.y = b.ll.y;
				// top edge: minimum of ur.y
				ret.ur.y = min(a.ur.y, b.ur.y);
			}
			// bottom edge of a within b
			else if (b.ll.y <= a.ll.y && a.ll.y <= b.ur.y) {
				ret.ll.y = a.ll.y;
				// top edge: minimum of ur.y
				ret.ur.y = min(a.ur.y, b.ur.y);
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
			bool const leftOf = (a.ur.x <= b.ll.x);
			bool const verticalIntersect = rectsIntersectVertical(a, b);

			return ((leftOf && verticalIntersect) || (leftOf && !considerVerticalIntersect));
		};

		inline static bool rectA_below_rectB(Rect const& a, Rect const& b, bool const& considerHorizontalIntersect) {
			bool const below = (a.ur.y <= b.ll.y);
			bool const horizontalIntersect = rectsIntersectHorizontal(a, b);

			return ((below && horizontalIntersect) || (below && !considerHorizontalIntersect));
		};
};

#endif
