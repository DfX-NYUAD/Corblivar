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

class CorblivarAlignmentReq {
	// debugging code switch (private)
	private:

	// enum class for alignment types; has to be defined first
	public:
		enum class Type : int {OFFSET = 0, RANGE = 1, RANGE_MAX = 2, UNDEF = -1};

	// private data, functions
	private:
		CorblivarAlignmentReq::Type type_x;
		CorblivarAlignmentReq::Type type_y;

	// constructors, destructors, if any non-implicit
	public:
		CorblivarAlignmentReq(int const& id, Block const* s_i, Block const* s_j,
				CorblivarAlignmentReq::Type const& type_x, double const& offset_range_x,
				CorblivarAlignmentReq::Type const& type_y, double const& offset_range_y) {
			this->id = id;
			this->s_i = s_i;
			this->s_j = s_j;
			this->type_x = type_x;
			this->type_y = type_y;
			this->offset_range_x = offset_range_x;
			this->offset_range_y = offset_range_y;

			// fix negative range, if required
			if (
				(this->range_x() && this->offset_range_x < 0) ||
				(this->range_y() && this->offset_range_y < 0)
			   ) {

				// negative range can be trivially resolved
				this->offset_range_x = abs(this->offset_range_x);
				this->offset_range_y = abs(this->offset_range_y);
			}
		};

	// public data, functions
	public:
		int id;
		Block const* s_i;
		Block const* s_j;
		double offset_range_x, offset_range_y;

		friend ostream& operator<< (ostream& out, CorblivarAlignmentReq::Type const& type) {
			out << static_cast<int>(type);
			return out;
		}

		inline bool range_x() const {
			return (this->type_x == CorblivarAlignmentReq::Type::RANGE && this->offset_range_x != 0.0);
		}
		inline bool range_y() const {
			return (this->type_y == CorblivarAlignmentReq::Type::RANGE && this->offset_range_y != 0.0);
		}
		inline bool range_max_x() const {
			return (this->type_x == CorblivarAlignmentReq::Type::RANGE_MAX && this->offset_range_x != 0.0);
		}
		inline bool range_max_y() const {
			return (this->type_y == CorblivarAlignmentReq::Type::RANGE_MAX && this->offset_range_y != 0.0);
		}
		inline bool offset_x() const {
			return (this->type_x == CorblivarAlignmentReq::Type::OFFSET);
		}
		inline bool offset_y() const {
			return (this->type_y == CorblivarAlignmentReq::Type::OFFSET);
		}

		inline string tupleString() const {
			stringstream ret;

			ret << "(" << this->s_i->id << ", " << this->s_j->id << ", ";
			ret << "(" << this->type_x << ", " << this->offset_range_x << "), ";
			ret << "(" << this->type_y << ", " << this->offset_range_y << ") )";

			return ret.str();
		};
};

#endif
