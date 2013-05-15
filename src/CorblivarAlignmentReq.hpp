/*
 * =====================================================================================
 *
 *    Description:  Corblivar alignment requests data
 *
 *         Author:  Johann Knechtel, johann.knechtel@ifte.de
 *        Company:  Institute of Electromechanical and Electronic Design, www.ifte.de
 *
 * =====================================================================================
 */
#ifndef _CORBLIVAR_ALIGNMENT_REQUEST
#define _CORBLIVAR_ALIGNMENT_REQUEST

// library includes
#include "Corblivar.incl.hpp"
// Corblivar includes, if any
#include "Block.hpp"
// forward declarations, if any

// alignment types
enum class AlignmentType : int {OFFSET = 0, RANGE = 1, UNDEF = -1};

class CorblivarAlignmentReq {
	// debugging code switch (private)
	private:

	// private data, functions
	private:
		int id;
		Block const* s_i;
		Block const* s_j;
		AlignmentType type_x, type_y;
		double offset_range_x, offset_range_y;

	// constructors, destructors, if any non-implicit
	public:
		CorblivarAlignmentReq(int const& id_i, Block const* si, Block const* sj, AlignmentType const& typex, double const& offsetrangex, AlignmentType const& typey, double const& offsetrangey) {
			id = id_i;
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
		friend class CorblivarCore;

		inline bool rangeX() const {
			return (this->type_x == AlignmentType::RANGE);
		};
		inline bool rangeY() const {
			return (this->type_y == AlignmentType::RANGE);
		};
		inline bool fixedOffsX() const {
			return (this->type_x == AlignmentType::OFFSET);
		};
		inline bool fixedOffsY() const {
			return (this->type_y == AlignmentType::OFFSET);
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
