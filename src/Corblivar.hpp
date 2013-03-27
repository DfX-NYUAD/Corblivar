/*
 * =====================================================================================
 *
 *    Description:  Main header for Corblivar
 *    			(Corner Block List for Varied [block] Alignment Requests)
 *
 *         Author:  Johann Knechtel, johann.knechtel@ifte.de
 *        Company:  Institute of Electromechanical and Electronic Design, www.ifte.de
 *
 * =====================================================================================
 */
#ifndef _CORBLIVAR_HPP
#define _CORBLIVAR_HPP

/* standard includes */
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cmath>
#include <array>
#include <vector>
#include <list>
#include <map>
#include <deque>
#include <stack>
#include <algorithm>
#include <sys/timeb.h>
#include <ctime>
#include <cstdlib>

/* consider standard namespace */
using namespace std;

/* forward declarations */
class FloorPlanner;
class CorblivarCore;

/* typed enum classes */
enum class Direction : unsigned {VERTICAL = 0, HORIZONTAL = 1};
enum class Alignment : unsigned {OFFSET, RANGE, UNDEF};

/* general classes */
class Math {
	public:

		// random functions
		// note: range is [min, max)
		inline static int randI(int const& min, int const& max) {
			if (max == min) {
				return min;
			}
			else {
				return min + (rand() % (max - min));
			}
		};
		inline static bool randB() {
			int r;

			r = rand();
			return (r < (RAND_MAX / 2));
		};
		inline static double randF01() {
			return ((double) rand() / RAND_MAX);
		}

		// standard deviation of samples
		inline static double stdDev(vector<double> const& samples) {
			double avg, sq_diffs;

			// determine avg of samples
			avg = 0.0;
			for (double const& s : samples) {
				avg += s;
			}
			avg /= samples.size();

			// determine sum of squared diffs for std dev
			sq_diffs = 0.0;
			for (double const& s : samples) {
				sq_diffs += pow(s - avg, 2.0);
			}

			// determine std dev
			return sqrt(sq_diffs / ((double) samples.size()));
		}

		// 1D gauss function; used for separated convolution w/ 2D gauss function,
		// provides the impulse response function for power blurring
		inline static double gauss1D(double const& value, double const& factor, double const& spread) {
			return sqrt(factor) * exp(-spread * pow(value, 2.0));
		}
};

class Point {
	public:
		static constexpr int UNDEF = -1;

		double x, y;

		Point() {
			x = y = UNDEF;
		};

		inline static double dist(Point const& a, Point const& b) {
			return sqrt(pow(abs(a.x - b.x), 2) + pow(abs(a.y - b.y), 2));
		};
};

class Rect {
	public:
		Point ll, ur;
		double h, w;
		double area;

		Rect() {
			h = w = area = 0.0;
		};

		inline static Rect determBoundingBox(vector<Rect*> const& rects) {
			Rect ret;
			unsigned r;

			if (rects.empty()) {
				ret.ll.x = ret.ll.y = ret.ur.x = ret.ur.y = Point::UNDEF;
				ret.h = ret.w = ret.area = Point::UNDEF;
			}
			else {
				ret.ll = rects[0]->ll;
				ret.ur = rects[0]->ur;
				for (r = 1; r < rects.size(); r++) {
					ret.ll.x = min(ret.ll.x, rects[r]->ll.x);
					ret.ll.y = min(ret.ll.y, rects[r]->ll.y);
					ret.ur.x = max(ret.ur.x, rects[r]->ur.x);
					ret.ur.y = max(ret.ur.y, rects[r]->ur.y);
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
			bool leftOf = (a.ur.x <= b.ll.x);
			bool verticalIntersect = rectsIntersectVertical(a, b);

			return ((leftOf && verticalIntersect) || (leftOf && !considerVerticalIntersect));
		};

		inline static bool rectA_below_rectB(Rect const& a, Rect const& b, bool const& considerHorizontalIntersect) {
			bool below = (a.ur.y <= b.ll.y);
			bool horizontalIntersect = rectsIntersectHorizontal(a, b);

			return ((below && horizontalIntersect) || (below && !considerHorizontalIntersect));
		};
};

class Block {
	public:
		int id;
		int layer;
		double power;
		//double x_slack_backward, y_slack_backward;
		//double x_slack_forward, y_slack_forward;
		Rect bb, bb_backup, bb_best;

		Block(int const& id_i) {
			id = id_i;
			layer = -1;
			power = 0.0;
			//x_slack_backward = y_slack_backward = 0.0;
			//x_slack_forward = y_slack_forward = 0.0;
		};
};

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
