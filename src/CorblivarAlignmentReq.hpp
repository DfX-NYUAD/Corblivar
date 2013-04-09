/*
 * =====================================================================================
 *
 *    Description:  Corblivar alignment requirements data
 *
 *         Author:  Johann Knechtel, johann.knechtel@ifte.de
 *        Company:  Institute of Electromechanical and Electronic Design, www.ifte.de
 *
 * =====================================================================================
 */
#ifndef _CORBLIVAR_ALIGNMENT_REQUIREMENT
#define _CORBLIVAR_ALIGNMENT_REQUIREMENT

// library includes
#include "Corblivar.incl.hpp"
// Corblivar includes, if any
#include "Block.hpp"
// forward declarations, if any

// alignment types
enum class Alignment : unsigned {OFFSET, RANGE, UNDEF};

class CorblivarAlignmentReq {
	// debugging code switch (private)
	private:

	// private data, functions
	private:
		Block* s_i;
		Block* s_j;
		Alignment type_x, type_y;
		double offset_range_x, offset_range_y;

	// constructors, destructors, if any non-implicit
	public:
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

	// public data, functions
	public:
		inline bool rangeX() const {
			return (this->type_x == Alignment::RANGE);
		};
		inline bool rangeY() const {
			return (this->type_y == Alignment::RANGE);
		};
		inline bool fixedOffsX() const {
			return (this->type_x == Alignment::OFFSET);
		};
		inline bool fixedOffsY() const {
			return (this->type_y == Alignment::OFFSET);
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
};

#endif
