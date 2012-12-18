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

// debugging code switch
//#define DBG_IO

/* standard namespace */
using namespace std;

/* div stuff */
enum Region {REGION_LEFT, REGION_RIGHT, REGION_BOTTOM, REGION_TOP, REGION_UNDEF};
enum Corner {CORNER_LL, CORNER_UL, CORNER_LR, CORNER_UR, CORNER_UNDEF};
enum Alignment {ALIGNMENT_OFFSET, ALIGNMENT_RANGE, ALIGNMENT_UNDEF};

/* forward declarations */
class CorblivarFP;
class CorblivarCore;
class CorblivarAlignmentReq;
class IO;
class Point;
//class Pin;
class Block;
class Net;
class Rect;

/* classes */

class CorblivarFP {
	public:
		static const int LOG_MINIMAL = 1;
		static const int LOG_MEDIUM = 2;
		static const int LOG_MAXIMUM = 3;

		// main vars
		string benchmark, blocks_file, power_file, nets_file;
		ofstream results;
		map<int, Block*> blocks;
		vector<Net*> nets;
		//vector<Net*> inter_nets;
		//vector<Net*> intra_nets;
		int maxBlockId;

		// config parameters
		int conf_log;
		int conf_layer;
		double conf_outline_x, conf_outline_y;

		// logging
		bool logMin() {
			return (this->conf_log >= LOG_MINIMAL);
		};
		bool logMed() {
			return (this->conf_log >= LOG_MEDIUM);
		};
		bool logMax() {
			return (this->conf_log >= LOG_MAXIMUM);
		};

		// var stuff

		// test suites
};

class IO {
	public:
		static void parseParameterConfig(CorblivarFP &corb, int argc, char** argv);
		static void parseBlocks(CorblivarFP &corb);
		static void parseNets(CorblivarFP &corb);
		static void writeFloorplanGP(CorblivarFP &corb);
		static void writeFloorplanGP(CorblivarFP &corb, string file_suffix);

	private:
		static int netId;
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

//class Pin: public Point {
//	public:
//		int id;
//
//		Pin() : Point() {
//			id = 0;
//		};
//};

class Rect {
	public:
		Point ll, ur;
		double h, w;
		double area;

		Rect() {
			h = w = area = 0.0;
		};

		static bool RectsAB_identical(Rect a, Rect b);
		static bool RectsAB_overlap_vert(Rect a, Rect b);
		static bool RectsAB_overlap_hori(Rect a, Rect b);
		static bool RectsAB_overlap(Rect a, Rect b);
		static bool RectA_leftOf_RectB(Rect a, Rect b);
		static bool RectA_below_RectB(Rect a, Rect b);
		//static Region nearest_border_of_a_to_b(Rect a, Rect b);
		//static Rect determineBoundingBox(vector<Point*> points);
		//static Rect determineBoundingBox(vector<Rect> rects);
		//static Rect determineIntersection(Rect a, Rect b);
};

class Block {
	public:
		static const int INSERT_FIXED_POS = 1;
		static const int INSERT_SHIFTING = 2;
		static const int TYPE_BLOCK = 1;
		static const int TYPE_TSV = 2;

		int insert_modifier;
		int id;
		int layer;
		int type;
		double power;
		//double x_slack_backward, y_slack_backward;
		//double x_slack_forward, y_slack_forward;
		//Rect bb, init_bb, tmp_bb, shift_window;
		Rect bb, tmp_bb;

		Block(int id_i) {
			insert_modifier = INSERT_SHIFTING;
			id = id_i;
			layer = -1;
			type = TYPE_BLOCK;
			power = 0.0;
			//x_slack_backward = y_slack_backward = 0.0;
			//x_slack_forward = y_slack_forward = 0.0;
		};

		//bool invalidId() {
		//	return (this->id <= 0);
		//};
};

class Net {
	public:
		//static const int TYPE_INTRALAYER = 0;
		//static const int TYPE_INTERLAYER = 1;

		int id;
		//int type;
		//int lower_layer, upper_layer;
		bool hasExternalPin;
		//// assignment flags for each related layer: [lower_layer..upper_layer]
		//vector<bool> assigned;
		//// outer vector for var layers
		//vector< vector<Block*> > blocks;
		vector<Block*> blocks;

		Net(int id_i) {
			id = id_i;
			//type = TYPE_INTRALAYER;
			//lower_layer = upper_layer = -1;
			hasExternalPin = false;
		};

		//static void determineAndLog_HPWL_Cong(MoDo &modo);

		//Rect determineBoundingBox(int l_layer);
		//bool requiresTSV(int layer);
		//Rect determineBoundingBox(int l_layer, int u_layer);
		//double determineHPWL_Cong(MoDo &modo, vector< vector< vector<int> > > &grid, double bin_size);
};

class CorblivarCore {
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
