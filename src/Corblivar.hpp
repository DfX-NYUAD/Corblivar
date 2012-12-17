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

/* forward declarations */
class Corblivar_FP;
class Corblivar_Core;
class IO;
class Point;
class Pin;
class Block;
class Net;
class Rect;

/* classes */

class Corblivar_FP {
	public:
		static const int LOG_MINIMAL = 1;
		static const int LOG_MEDIUM = 2;
		static const int LOG_MAXIMUM = 3;

		// main vars
		string benchmark, config_file, blocks_file, nets_file, results_file;
		ofstream results;
		//TODO blocks w/o dependencie to layers
		vector< map<int, Block*> > blocks;
		vector<Net*> inter_nets;
		vector<Net*> intra_nets;
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

class Corblivar_Core {
};

class IO {
	public:
		static void parseProgramParameter(Corblivar_FP &corb, int argc, char** argv);
		static void parseConfig(Corblivar_FP &corb, string file);
		static void parseBlocks(Corblivar_FP &corb, string file);
		static void parseNets(Corblivar_FP &corb, string file);
		static void writeFloorplanGP(Corblivar_FP &corb);
		static void writeFloorplanGP(Corblivar_FP &corb, string file_suffix);

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

class Pin: public Point {
	public:
		int id;

		Pin() : Point() {
			id = 0;
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

		Block() {
			insert_modifier = INSERT_SHIFTING;
			id = 0;
			layer = -1;
			type = TYPE_BLOCK;
			power = 0.0;
			//x_slack_backward = y_slack_backward = 0.0;
			//x_slack_forward = y_slack_forward = 0.0;
		};

		bool invalidId() {
			return (this->id <= 0);
		};
};

class Net {
	public:
		static const int TYPE_INTRALAYER = 0;
		static const int TYPE_INTERLAYER = 1;

		int id;
		int type;
		int lower_layer, upper_layer;
		bool hasExternalPin;
		// assignment flags for each related layer: [lower_layer..upper_layer]
		vector<bool> assigned;
		// outer vector for var layers
		vector< vector<Block*> > blocks;

		Net(int i) {
			id = i;
			type = TYPE_INTRALAYER;
			lower_layer = upper_layer = -1;
			hasExternalPin = false;
		};

		//static void determineAndLog_HPWL_Cong(MoDo &modo);

		//Rect determineBoundingBox(int l_layer);
		//bool requiresTSV(int layer);
		//Rect determineBoundingBox(int l_layer, int u_layer);
		//double determineHPWL_Cong(MoDo &modo, vector< vector< vector<int> > > &grid, double bin_size);
};

#endif
