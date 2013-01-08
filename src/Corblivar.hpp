/*
 * =====================================================================================
 *
 *    Description:  Main header for Corblivar (corner block list for varied [block] alignment requests)
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
#include <vector>
#include <list>
#include <map>
#include <algorithm>
#include <queue>
#include <stack>
#include <sys/timeb.h>
#include <ctime>
#include <cstdlib>

// debugging code switch
//#define DBG_IO
//#define DBG_CORB
//#define DBG_SA

/* standard namespace */
using namespace std;

/* div stuff */
enum Region {REGION_LEFT, REGION_RIGHT, REGION_BOTTOM, REGION_TOP, REGION_UNDEF};
enum Corner {CORNER_LL, CORNER_UL, CORNER_LR, CORNER_UR, CORNER_UNDEF};
enum Alignment {ALIGNMENT_OFFSET, ALIGNMENT_RANGE, ALIGNMENT_UNDEF};
enum Direction {DIRECTION_VERT, DIRECTION_HOR};

/* forward declarations */
class CorblivarFP;
class CorblivarLayoutRep;
class CorblivarDie;
class CorblivarAlignmentReq;
class CBLitem;
class IO;
class Point;
class Block;
class Net;
class Rect;

/* classes */

class CorblivarFP {
	public:
		// main vars
		string benchmark, blocks_file, power_file, nets_file;
		ofstream results;
		map<int, Block*> blocks;
		vector<Net*> nets;

		// config parameters
		int conf_log;
		int conf_layer;
		double conf_outline_x, conf_outline_y;
		double conf_SA_minT, conf_SA_coolingT, conf_SA_loopFactor;
		// fixed config parameters
		static const int SA_INIT_T_FACTOR = 20;
		// values see Corblivar.cpp
		static const double COST_FACTOR_TEMP;
		static const double COST_FACTOR_IR;
		static const double COST_FACTOR_WL;
		static const double COST_FACTOR_TSVS;
		// handled via outline
		//static const double COST_FACTOR_AREA;
		static const double COST_FACTOR_OUTLINE_X;
		static const double COST_FACTOR_OUTLINE_Y;
		static const double COST_FACTOR_ALIGNMENTS;

		// SA parameters: max cost values
		double max_cost_temp, max_cost_IR, max_cost_WL, max_cost_TSVs, max_cost_alignments;

		// logging
		static const int LOG_MINIMAL = 1;
		static const int LOG_MEDIUM = 2;
		static const int LOG_MAXIMUM = 3;
		bool logMin() {
			return logMin(this->conf_log);
		};
		bool logMed() {
			return logMed(this->conf_log);
		};
		bool logMax() {
			return logMax(this->conf_log);
		};
		static bool logMin(int log) {
			return (log >= LOG_MINIMAL);
		};
		static bool logMed(int log) {
			return (log >= LOG_MEDIUM);
		};
		static bool logMax(int log) {
			return (log >= LOG_MAXIMUM);
		};

		// FP functions
		bool SA(CorblivarLayoutRep &chip);
		double determLayoutCost();
		vector<double> determCostOutline();
		vector<double> determCostInterconnects();

		// random functions
		// note: range is [min, max)
		static double randF(int min, int max) {
			double r;

			r = (double) std::rand() / RAND_MAX;
			return min + r * (double) (max - min);
		};
		// note: range is [min, max)
		static int randI(int min, int max) {
			return (int) randF(min, max);
		};
		static bool randB() {
			int r;

			r = std::rand();
			return (r < (RAND_MAX / 2));
		};

		// test suites
};

class IO {
	public:
		static void parseParameterConfig(CorblivarFP &corb, int argc, char** argv);
		static void parseBlocks(CorblivarFP &corb);
		static void parseNets(CorblivarFP &corb);
		static void writeFloorplanGP(CorblivarFP &corb);
		static void writeFloorplanGP(CorblivarFP &corb, string file_suffix);
};

class Point {
	public:
		static const int UNDEF = -1;

		double x, y;

		Point() {
			x = y = UNDEF;
		};

		static double dist(Point a, Point b) {
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

		static Rect determBoundingBox(vector<Rect> rects) {
			Rect ret;
			unsigned b;

			if (rects.empty()) {
				ret.ll.x = ret.ll.y = ret.ur.x = ret.ur.y = Point::UNDEF;
				ret.h = ret.w = ret.area = Point::UNDEF;
			}
			else {
				ret.ll = rects[0].ll;
				ret.ur = rects[0].ur;
				for (b = 1; b < rects.size(); b++) {
					ret.ll.x = min(ret.ll.x, rects[b].ll.x);
					ret.ll.y = min(ret.ll.y, rects[b].ll.y);
					ret.ur.x = max(ret.ur.x, rects[b].ur.x);
					ret.ur.y = max(ret.ur.y, rects[b].ur.y);
				}
				ret.w = ret.ur.x - ret.ll.x;
				ret.h = ret.ur.y - ret.ll.y;
				ret.area = ret.w * ret.h;
			}

			return ret;
		};
};

class Block {
	public:
		int id;
		int layer;
		double power;
		//double x_slack_backward, y_slack_backward;
		//double x_slack_forward, y_slack_forward;
		Rect bb;

		Block(int id_i) {
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

		Net(int id_i) {
			id = id_i;
			hasExternalPin = false;
		};

		double determHPWL();
};

class CBLitem {
	public:

		Block* Si;
		Direction Li;
		unsigned Ti;

		CBLitem (Block *si, Direction li, unsigned ti) {
			Si = si;
			Li = li;
			Ti = ti;
		}

		string itemString() {
			stringstream ret;

			ret << "(" << Si->id << ", " << Li << ", " << Ti << ")";

			return ret.str();
		}
};

class CorblivarLayoutRep {
	public:

		vector<CorblivarDie*> dies;
		vector<CorblivarAlignmentReq*> A;
		// die pointer
		CorblivarDie* p;

		void initCorblivar(CorblivarFP &corb);
		void generateLayout(int log);
		CorblivarDie* findDie(Block* Si);
};


class CorblivarDie {
	public:
		int id;

		// CBL data
		list<CBLitem*> CBL;
		// progress pointer, list iterator
		list<CBLitem*>::iterator pi;
		// placement stacks
		stack<Block*> Hi, Vi;

		bool stalled;
		bool done;

		CorblivarDie(int i) {
			stalled = done = false;
			id = i;
		}

		Block* placeCurrentBlock();

		// false: last CBL tuple; true: not last tuple
		bool incrementTuplePointer() {

			if ((* this->pi) == this->CBL.back()) {
				this->done = true;
				return false;
			}
			else {
				++(this->pi);
				return true;
			}
		}

		void resetTuplePointer() {
			this->pi = this->CBL.begin();
		}

		Block* currentBlock() {
			return (* this->pi)->Si;
		}
};

class CorblivarAlignmentReq {
	public:

		Block *s_i, *s_j;
		Alignment type_x, type_y;
		double offset_range_x, offset_range_y;

		bool rangeX() {
			return (type_x == ALIGNMENT_RANGE);
		};
		bool rangeY() {
			return (type_y == ALIGNMENT_RANGE);
		};
		bool fixedOffsX() {
			return (type_x == ALIGNMENT_OFFSET);
		};
		bool fixedOffsY() {
			return (type_y == ALIGNMENT_OFFSET);
		};
		string tupleString() {
			stringstream ret;

			ret << "(" << s_i->id << ", " << s_j->id << ", (" << offset_range_x << ", ";
			if (this->rangeX()) {
				ret << "1";
			}
			else if (this->fixedOffsX()) {
				ret << "0";
			}
			else {
				ret << "lambda";
			}
			ret << "), (" << offset_range_y << ", ";
			if (this->rangeY()) {
				ret << "1";
			}
			else if (this->fixedOffsY()) {
				ret << "0";
			}
			else {
				ret << "lambda";
			}
			ret << ") )";

			return ret.str();
		};

		CorblivarAlignmentReq(Block* si, Block* sj, Alignment typex, double offsetrangex, Alignment typey, double offsetrangey) {
			s_i = si;
			s_j = sj;
			type_x = typex;
			type_y = typey;
			offset_range_x = offsetrangex;
			offset_range_y = offsetrangey;

			// fix invalid negative range
			if ((this->rangeX() && offset_range_x < 0) || (this->rangeY() && offset_range_y < 0)) {
				cout << "CorblivarAlignmentReq> ";
				cout << "Fixing tuple (negative range):" << endl;
				cout << " " << this->tupleString() << " to" << endl;

				if (offset_range_x < 0) {
					offset_range_x = 0;
				}
				if (offset_range_y < 0) {
					offset_range_y = 0;
				}

				cout << " " << this->tupleString() << endl;
			}
		};
};

#endif
