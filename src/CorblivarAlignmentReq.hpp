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
		CorblivarAlignmentReq(int const& id, Block const* s_i, Block const* s_j,
				AlignmentType const& type_x, double const& offset_range_x,
				AlignmentType const& type_y, double const& offset_range_y) {
			this->id = id;
			this->s_i = s_i;
			this->s_j = s_j;
			this->type_x = type_x;
			this->type_y = type_y;
			this->offset_range_x = offset_range_x;
			this->offset_range_y = offset_range_y;

			// fix invalid negative range
			// TODO check for allowed range, offsets; negative can be
			// transformed to positive via block swapping
			if ((this->rangeX() && this->offset_range_x < 0) || (this->rangeY() && this->offset_range_y < 0)) {

				// logging
				cout << "Corblivar> ";
				cout << "Fixing alignment request (negative range):" << endl;
				cout << " " << this->tupleString() << " to" << endl;

				// actual fixing
				this->offset_range_x = abs(this->offset_range_x);
				this->offset_range_y = abs(this->offset_range_y);

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
