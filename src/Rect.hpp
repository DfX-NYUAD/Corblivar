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
			this->h = 0.0;
			this->w = 0.0;
			this->area = 0.0;
		};

	// public data, functions
	public:
		Point ll, ur;
		double h, w;
		double area;

		inline static Rect determBoundingBox(vector<Rect const*> const& rects, bool const& consider_center = false) {
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
						ret.ll.x = min(ret.ll.x, r->ll.x + r->w / 2.0);
						ret.ur.x = max(ret.ur.x, r->ll.x + r->w / 2.0);
						ret.ll.y = min(ret.ll.y, r->ll.y + r->h / 2.0);
						ret.ur.y = max(ret.ur.y, r->ll.y + r->h / 2.0);
					}
				}
				else {

					// init w/ box of first rect
					ret.ll = rects[0]->ll;
					ret.ur = rects[0]->ur;

					// determine bounding box of all rects based on
					// boxes of rects
					for (Rect const* r : rects) {
						ret.ll.x = min(ret.ll.x, r->ll.x);
						ret.ll.y = min(ret.ll.y, r->ll.y);
						ret.ur.x = max(ret.ur.x, r->ur.x);
						ret.ur.y = max(ret.ur.y, r->ur.y);
					}
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
			return (a.ur.x <= b.ll.x) && (!considerVerticalIntersect || rectsIntersectVertical(a, b));
		};

		inline static bool rectA_below_rectB(Rect const& a, Rect const& b, bool const& considerHorizontalIntersect) {
			return (a.ur.y <= b.ll.y) && (!considerHorizontalIntersect || rectsIntersectHorizontal(a, b));
		};
};

#endif
