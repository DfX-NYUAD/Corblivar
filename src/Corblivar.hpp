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
#include <deque>
#include <stack>
#include <algorithm>
#include <sys/timeb.h>
#include <ctime>
#include <cstdlib>

// debugging code switch
//#define DBG_IO
//#define DBG_CORB
//#define DBG_LAYOUT
//#define DBG_SA

/* consider standard namespace */
using namespace std;

/* global enumerations */
enum Direction {DIRECTION_VERT, DIRECTION_HOR};

/* forward declarations */
class FloorPlanner;
class CorblivarCore;
class CorblivarDie;
class CorblivarAlignmentReq;
class CornerBlockList;
class ThermalAnalyzer;
class IO;
class Point;
class Block;
class Net;
class Rect;
class Math;

/* classes */
class IO {
	public:
		static void parseParameterConfig(FloorPlanner& fp, const int& argc, char** argv);
		static void parseBlocks(FloorPlanner& fp);
		static void parseNets(FloorPlanner& fp);
		static void parseCorblivarFile(FloorPlanner& fp, CorblivarCore& corb);
		static void writeFloorplanGP(const FloorPlanner& fp, const string& file_suffix = "");
		static void writeHotSpotFiles(const FloorPlanner& fp);
		static void writePowerThermalMaps(const FloorPlanner& fp);
};

class ThermalAnalyzer {
	private:
		// material parameters for HotSpot thermal 3D-IC simulation
		static const double HEAT_CAPACITY_SI;
		static const double THERMAL_RESISTIVITY_SI;
		static const double THICKNESS_SI;
		static const double THICKNESS_SI_ACTIVE;
		static const double HEAT_CAPACITY_BEOL;
		static const double THERMAL_RESISTIVITY_BEOL;
		static const double THICKNESS_BEOL;
		static const double HEAT_CAPACITY_BOND;
		static const double THERMAL_RESISTIVITY_BOND;
		static const double THICKNESS_BOND;

		// thermal modeling: vector masks and maps
		// mask[i][x][y], whereas mask[0] relates to the mask for layer 0 obtained
		// by considering heat source in layer 0, mask[1] relates to the mask for
		// layer 0 obtained by considering heat source in layer 1 and so forth.
		vector<vector<vector<double>>> thermal_masks;
		// map[i][x][y], whereas map[0] relates to the map for layer 0 and so forth.
		vector<vector<vector<double>>> power_maps;
		// thermal map for layer 0, i.e., lowest layer, i.e., hottest layer
		vector<vector<double>> thermal_map;

		// thermal modeling: handlers
		void generatePowerMaps(const FloorPlanner& fp, const int& maps_dim);

	public:
		friend class IO;

		// thermal modeling: handlers
		void initThermalMasks(const FloorPlanner& fp);
		// thermal-analyzer routine based on power blurring,
		// i.e., convolution of thermals masks and power maps
		// returns max value of convoluted 2D matrix
		double performPowerBlurring(const FloorPlanner& fp, const bool& set_max_cost = false, const bool& normalize = true);
};

class Math {
	public:

		// random functions
		// note: range is [min, max)
		inline static int randI(const int& min, const int& max) {
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
		inline static double stdDev(const vector<double>& samples) {
			double avg, sq_diffs;

			// determine avg of samples
			avg = 0.0;
			for (const double& s : samples) {
				avg += s;
			}
			avg /= samples.size();

			// determine sum of squared diffs for std dev
			sq_diffs = 0.0;
			for (const double& s : samples) {
				sq_diffs += pow(s - avg, 2.0);
			}

			// determine std dev
			return sqrt(sq_diffs / ((double) samples.size()));
		}

		// 2D gauss function; used for power blurring as impulse response fuction
		inline static double gauss2D(const double& x, const double& y, const double& factor, const double& spread) {
			return factor * exp(-spread * pow(x, 2.0)) * exp(-spread * pow(y, 2.0));
		}
};

class Point {
	public:
		static const int UNDEF = -1;

		double x, y;

		Point() {
			x = y = UNDEF;
		};

		inline static double dist(const Point& a, const Point& b) {
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

		inline static Rect determBoundingBox(const vector<Rect>& rects) {
			Rect ret;
			unsigned r;

			if (rects.empty()) {
				ret.ll.x = ret.ll.y = ret.ur.x = ret.ur.y = Point::UNDEF;
				ret.h = ret.w = ret.area = Point::UNDEF;
			}
			else {
				ret.ll = rects[0].ll;
				ret.ur = rects[0].ur;
				for (r = 1; r < rects.size(); r++) {
					ret.ll.x = min(ret.ll.x, rects[r].ll.x);
					ret.ll.y = min(ret.ll.y, rects[r].ll.y);
					ret.ur.x = max(ret.ur.x, rects[r].ur.x);
					ret.ur.y = max(ret.ur.y, rects[r].ur.y);
				}
				ret.w = ret.ur.x - ret.ll.x;
				ret.h = ret.ur.y - ret.ll.y;
				ret.area = ret.w * ret.h;
			}

			return ret;
		};

		inline static Rect determineIntersection(const Rect& a, const Rect& b) {
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

		inline static bool rectsIntersectVertical(const Rect& a, const Rect& b) {
			return (
					(a.ll.y <= b.ll.y && b.ll.y < a.ur.y) ||
					(b.ll.y <= a.ll.y && a.ll.y < b.ur.y)
				);
		};

		inline static bool rectsIntersectHorizontal(const Rect& a, const Rect& b) {
			return (
					(a.ll.x <= b.ll.x && b.ll.x < a.ur.x) ||
					(b.ll.x <= a.ll.x && a.ll.x < b.ur.x)
				);
		};

		inline static bool rectsIntersect(const Rect& a, const Rect& b) {
			return rectsIntersectVertical(a, b) && rectsIntersectHorizontal(a, b);
		};

		inline static bool rectA_leftOf_rectB(const Rect& a, const Rect& b, const bool& considerVerticalIntersect) {
			bool leftOf = (a.ur.x <= b.ll.x);
			bool verticalIntersect = rectsIntersectVertical(a, b);

			return ((leftOf && verticalIntersect) || (leftOf && !considerVerticalIntersect));
		};

		inline static bool rectA_below_rectB(const Rect& a, const Rect& b, const bool& considerHorizontalIntersect) {
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

		Block(const int& id_i) {
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

		Net(const int& id_i) {
			id = id_i;
			hasExternalPin = false;
		};

		double determHPWL();
};

class FloorPlanner {
	private:
		// PODs for cost functions
		struct Cost {
			double cost;
			bool fits_fixed_outline;
		};
		struct CostInterconn {
			double HPWL;
			double TSVs;
		};

		// IO
		string benchmark, blocks_file, power_file, nets_file;
		ofstream results, solution_out;

		// IO: scaling factor for block dimensions
		static const int BLOCKS_SCALE_UP = 50;

		// SA config parameters
		double conf_SA_loopFactor, conf_SA_loopLimit;
		double conf_SA_cost_temp, conf_SA_cost_WL, conf_SA_cost_TSVs, conf_SA_cost_area_outline;

		// SA parameters: temperature-scaling factors
		double conf_SA_temp_factor_phase1, conf_SA_temp_factor_phase2, conf_SA_temp_factor_phase3;
		// SA parameters: temperature-phase-transition factos
		double conf_SA_temp_phase_trans_12_factor, conf_SA_temp_phase_trans_23_factor;

		// SA parameter: scaling factor for loops during solution-space sampling
		static const int SA_SAMPLING_LOOP_FACTOR = 2;

		// SA: layout-operation handler variables
		int last_op, last_op_die1, last_op_die2, last_op_tuple1, last_op_tuple2, last_op_juncts;

		// SA: layout-operation handler
		bool performRandomLayoutOp(const CorblivarCore& corb, const bool& revertLastOp = false);

		// SA: cost functions, i.e., layout-evalutions
		Cost determCost(const double& ratio_feasible_solutions_fixed_outline = 0.0, const bool& phase_two = false, const bool& set_max_cost = false);
		inline double determCostThermalDistr(const bool& set_max_cost = false, const bool& normalize = true) {
			return this->thermalAnalyzer.performPowerBlurring(*this, set_max_cost, normalize);
		}
		Cost determCostAreaOutline(const double& ratio_feasible_solutions_fixed_outline = 0.0);
		CostInterconn determCostInterconnects(const bool& set_max_cost = false, const bool& normalize = true);

	public:
		friend class IO;

		// chip data
		map<int, Block*> blocks;
		vector<Net*> nets;

		// thermal analyzer
		ThermalAnalyzer thermalAnalyzer;

		// IO
		struct timeb start;
		ifstream solution_in;

		// config parameters
		int conf_layer;
		int conf_log;
		double conf_outline_x, conf_outline_y, outline_AR;

		// logging
		static const int LOG_MINIMAL = 1;
		static const int LOG_MEDIUM = 2;
		static const int LOG_MAXIMUM = 3;
		inline bool logMin() const {
			return (this->conf_log >= LOG_MINIMAL);
		};
		inline bool logMed() const {
			return (this->conf_log >= LOG_MEDIUM);
		};
		inline bool logMax() const {
			return (this->conf_log >= LOG_MAXIMUM);
		};

		// SA parameters: max cost values
		mutable double max_cost_temp, max_cost_WL, max_cost_TSVs, max_cost_alignments;

		// SA: floorplanning handler
		bool performSA(const CorblivarCore& corb);
		void finalize(const CorblivarCore& corb);
};

class CornerBlockList {
	public:

		// CBL sequences
		vector<Block*> S;
		vector<Direction> L;
		vector<unsigned> T;

		inline unsigned size() const {
			unsigned ret;

			ret = this->S.size();
#ifdef DBG_CORB
			bool mismatch = false;
			unsigned prev_ret;

			prev_ret = ret;
			ret = min(ret, this->L.size());
			mismatch = (ret != prev_ret);
			prev_ret = ret;
			ret = min(ret, this->T.size());
			mismatch = mismatch || (ret != prev_ret);

			if (mismatch) {
				cout << "DBG_CORB> CBL has sequences size mismatch!" << endl;
				cout << "DBG_CORB> CBL: " << endl;
				cout << this->itemString() << endl;
			}

#endif
			return ret;
		};

		inline unsigned capacity() const {
			return this->S.capacity();
		};

		inline bool empty() const {
			return (this->size() == 0);
		};

		inline void clear() {
			this->S.clear();
			this->L.clear();
			this->T.clear();
		};

		inline void reserve(const unsigned& elements) {
			this->S.reserve(elements);
			this->L.reserve(elements);
			this->T.reserve(elements);
		};

		inline string itemString(const unsigned& i) const {
			stringstream ret;

			ret << "( " << S[i]->id << " " << L[i] << " " << T[i] << " " << S[i]->bb.w << " " << S[i]->bb.h << " )";

			return ret.str();
		};

		inline string itemString() const {
			unsigned i;
			stringstream ret;

			for (i = 0; i < this->size(); i++) {
				ret << this->itemString(i) << " , ";
			}

			return ret.str();
		};
};

class CorblivarDie {
	private:
		// progress pointer, CBL vector index
		unsigned pi;
		// placement stacks
		stack<Block*> Hi, Vi;

		// false: last CBL tuple; true: not last tuple
		inline bool incrementTuplePointer() {

			if (this->pi == (CBL.size() - 1)) {
				this->done = true;
				return false;
			}
			else {
				this->pi++;
				return true;
			}
		};

		inline void resetTuplePointer() {
			this->pi = 0;
		};

	public:
		int id;
		bool stalled;
		bool done;

		// main CBL sequence
		CornerBlockList CBL;

		// backup CBL sequences
		CornerBlockList CBLbackup, CBLbest;

		CorblivarDie(const int& i) {
			stalled = done = false;
			id = i;
		}

		// layout generation functions
		Block* placeCurrentBlock(const bool& dbgStack = false);

		inline Block* currentBlock() const {
			return this->CBL.S[this->pi];
		};

		inline Direction currentTupleDirection() const {
			return this->CBL.L[this->pi];
		};

		inline unsigned currentTupleJuncts() const {
			return this->CBL.T[this->pi];
		};

		inline string currentTupleString() const {
			return this->CBL.itemString(this->pi);
		};

		inline void reset() {
			// reset progress pointer
			this->resetTuplePointer();
			// reset done flag
			this->done = false;
			// reset placement stacks
			while (!this->Hi.empty()) {
				this->Hi.pop();
			}
			while (!this->Vi.empty()) {
				this->Vi.pop();
			}
		};
};

class CorblivarCore {
	private:
		// die pointer
		mutable CorblivarDie* p;
		// sequence A; alignment requirements
		vector<CorblivarAlignmentReq*> A;

	public:
		vector<CorblivarDie*> dies;

		// general operations
		void initCorblivarRandomly(const FloorPlanner& fp);
		void generateLayout(const bool& dbgStack = false) const;
		//CorblivarDie* findDie(Block* Si);

		// init handler
		inline void initCorblivarDies(const int& layer, const unsigned& blocks) {
			int i;
			CorblivarDie* cur_die;

			// clear and reserve mem for dies
			this->dies.clear();
			this->dies.reserve(layer);

			// init dies and their related structures
			for (i = 0; i < layer; i++) {
				cur_die = new CorblivarDie(i);
				// reserve mem for worst case, i.e., all blocks in one particular die
				cur_die->CBL.reserve(blocks);

				this->dies.push_back(cur_die);
			}
		};

		// layout operations for heuristic optimization
		static const int OP_SWAP_BLOCKS_WI_DIE = 0;
		static const int OP_SWAP_BLOCKS_ACROSS_DIE = 1;
		static const int OP_MOVE_TUPLE = 2;
		static const int OP_SWITCH_TUPLE_DIR = 3;
		static const int OP_SWITCH_TUPLE_JUNCTS = 4;
		static const int OP_SWITCH_BLOCK_ORIENT = 5;

		inline void switchBlocksWithinDie(const int& die, const int& tuple1, const int& tuple2) const {
			swap(this->dies[die]->CBL.S[tuple1], this->dies[die]->CBL.S[tuple2]);
#ifdef DBG_CORB
			cout << "DBG_CORB> switchBlocksWithinDie; d1=" << die;
			cout << ", s1=" << this->dies[die]->CBL.S[tuple1]->id;
			cout << ", s2=" << this->dies[die]->CBL.S[tuple2]->id << endl;
#endif
		};
		inline void switchBlocksAcrossDies(const int& die1, const int& die2, const int& tuple1, const int& tuple2) const {
			swap(this->dies[die1]->CBL.S[tuple1], this->dies[die2]->CBL.S[tuple2]);
#ifdef DBG_CORB
			cout << "DBG_CORB> switchBlocksAcrossDies; d1=" << die1 << ", d2=" << die2;
			cout << ", s1=" << this->dies[die1]->CBL.S[tuple1]->id;
			cout << ", s2=" << this->dies[die2]->CBL.S[tuple2]->id << endl;
#endif
		};
		inline void moveTupleAcrossDies(const int& die1, const int& die2, const int& tuple1, const int& tuple2) const {

			// insert tuple1 from die1 into die2 w/ offset tuple2
			this->dies[die2]->CBL.S.insert(this->dies[die2]->CBL.S.begin() + tuple2, *(this->dies[die1]->CBL.S.begin() + tuple1));
			this->dies[die2]->CBL.L.insert(this->dies[die2]->CBL.L.begin() + tuple2, *(this->dies[die1]->CBL.L.begin() + tuple1));
			this->dies[die2]->CBL.T.insert(this->dies[die2]->CBL.T.begin() + tuple2, *(this->dies[die1]->CBL.T.begin() + tuple1));
			// erase tuple1 from die1
			this->dies[die1]->CBL.S.erase(this->dies[die1]->CBL.S.begin() + tuple1);
			this->dies[die1]->CBL.L.erase(this->dies[die1]->CBL.L.begin() + tuple1);
			this->dies[die1]->CBL.T.erase(this->dies[die1]->CBL.T.begin() + tuple1);

#ifdef DBG_CORB
			cout << "DBG_CORB> moveTupleAcrossDies; d1=" << die1 << ", d2=" << die2 << ", t1=" << tuple1 << ", t2=" << tuple2 << endl;
#endif
		};
		inline void switchTupleDirection(const int& die, const int& tuple) const {
			if (this->dies[die]->CBL.L[tuple] == DIRECTION_VERT) {
				this->dies[die]->CBL.L[tuple] = DIRECTION_HOR;
			}
			else {
				this->dies[die]->CBL.L[tuple] = DIRECTION_VERT;
			}
#ifdef DBG_CORB
			cout << "DBG_CORB> switchTupleDirection; d1=" << die << ", t1=" << tuple << endl;
#endif
		};
		inline void switchTupleJunctions(const int& die, const int& tuple, const int& juncts) const {
			this->dies[die]->CBL.T[tuple] = juncts;
#ifdef DBG_CORB
			cout << "DBG_CORB> switchTupleJunctions; d1=" << die << ", t1=" << tuple << ", juncts=" << juncts << endl;
#endif
		};
		inline void switchBlockOrientation(const int& die, const int& tuple) const {
			double w_tmp;

			w_tmp = this->dies[die]->CBL.S[tuple]->bb.w;
			this->dies[die]->CBL.S[tuple]->bb.w = this->dies[die]->CBL.S[tuple]->bb.h;
			this->dies[die]->CBL.S[tuple]->bb.h = w_tmp;
#ifdef DBG_CORB
			cout << "DBG_CORB> switchBlockOrientation; d1=" << die << ", t1=" << tuple << endl;
#endif
		};

		// CBL logging
		inline string CBLsString() const {
			stringstream ret;

			ret << "# tuple format: ( BLOCK_ID DIRECTION T-JUNCTS BLOCK_WIDTH BLOCK_HEIGHT )" << endl;
			ret << "data_start" << endl;

			for (CorblivarDie* const& die : this->dies) {
				ret << "CBL [ " << die->id << " ]" << endl;
				ret << die->CBL.itemString() << endl;
			}

			return ret.str();
		};

		// CBL backup handler
		inline void backupCBLs() const {

			for (CorblivarDie* const& die : this->dies) {

				die->CBLbackup.clear();
				die->CBLbackup.reserve(die->CBL.capacity());

				for (Block* &b : die->CBL.S) {
					// backup block dimensions (block shape) into
					// block itself
					b->bb_backup = b->bb;
					die->CBLbackup.S.push_back(b);
				}
				for (const Direction& dir : die->CBL.L) {
					die->CBLbackup.L.push_back(dir);
				}
				for (const unsigned& t_juncts : die->CBL.T) {
					die->CBLbackup.T.push_back(t_juncts);
				}
			}
		};
		inline void restoreCBLs() const {

			for (CorblivarDie* const& die : this->dies) {

				die->CBL.clear();
				die->CBL.reserve(die->CBLbackup.capacity());

				for (Block* &b : die->CBLbackup.S) {
					// restore block dimensions (block shape) from
					// block itself
					b->bb = b->bb_backup;
					die->CBL.S.push_back(b);
				}
				for (const Direction& dir : die->CBLbackup.L) {
					die->CBL.L.push_back(dir);
				}
				for (const unsigned& t_juncts : die->CBLbackup.T) {
					die->CBL.T.push_back(t_juncts);
				}
			}
		};

		// CBL best-solution handler
		inline void storeBestCBLs() const {

			for (CorblivarDie* const& die : this->dies) {

				die->CBLbest.clear();
				die->CBLbest.reserve(die->CBL.capacity());

				for (Block* &b : die->CBL.S) {
					// backup block dimensions (block shape) into
					// block itself
					b->bb_best = b->bb;
					die->CBLbest.S.push_back(b);
				}
				for (const Direction& dir : die->CBL.L) {
					die->CBLbest.L.push_back(dir);
				}
				for (const unsigned& t_juncts : die->CBL.T) {
					die->CBLbest.T.push_back(t_juncts);
				}
			}
		};
		inline bool applyBestCBLs(const bool& log) const {

			for (CorblivarDie* const& die : this->dies) {

				// sanity check for existence of best solution
				if (die->CBLbest.empty()) {
					if (log) {
						cout << "Corblivar> No best (fitting) solution available!" << endl;
					}
					return false;
				}

				die->CBL.clear();
				die->CBL.reserve(die->CBLbest.capacity());

				for (Block* &b : die->CBLbest.S) {
					// restore block dimensions (block shape) from
					// block itself
					b->bb = b->bb_best;
					die->CBL.S.push_back(b);
				}
				for (const Direction& dir : die->CBLbest.L) {
					die->CBL.L.push_back(dir);
				}
				for (const unsigned& t_juncts : die->CBLbest.T) {
					die->CBL.T.push_back(t_juncts);
				}
			}

			return true;
		};
};

class CorblivarAlignmentReq {
	private:
		enum Alignment {ALIGNMENT_OFFSET, ALIGNMENT_RANGE, ALIGNMENT_UNDEF};

	public:

		Block *s_i, *s_j;
		Alignment type_x, type_y;
		double offset_range_x, offset_range_y;

		inline bool rangeX() const {
			return (this->type_x == ALIGNMENT_RANGE);
		};
		inline bool rangeY() const {
			return (this->type_y == ALIGNMENT_RANGE);
		};
		inline bool fixedOffsX() const {
			return (this->type_x == ALIGNMENT_OFFSET);
		};
		inline bool fixedOffsY() const {
			return (this->type_y == ALIGNMENT_OFFSET);
		};
		inline string tupleString() const {
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
