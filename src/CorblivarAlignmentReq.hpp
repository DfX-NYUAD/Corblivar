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
		Type type_x;
		Type type_y;

	// constructors, destructors, if any non-implicit
	public:
		CorblivarAlignmentReq(int const& id, Block const* s_i, Block const* s_j,
				Type const& type_x, double const& offset_range_x,
				Type const& type_y, double const& offset_range_y) {
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

		friend ostream& operator<< (ostream& out, Type const& type) {

			switch (type) {

				case Type::RANGE:
					out << "RANGE";
					break;
				case Type::RANGE_MAX:
					out << "RANGE_MAX";
					break;
				case Type::OFFSET:
					out << "OFFSET";
					break;
				default:
					out << "UNDEF";
					break;
			}

			return out;
		}

		inline bool range_x() const {
			return (this->type_x == Type::RANGE && this->offset_range_x != 0.0);
		}
		inline bool range_y() const {
			return (this->type_y == Type::RANGE && this->offset_range_y != 0.0);
		}
		inline bool range_max_x() const {
			return (this->type_x == Type::RANGE_MAX && this->offset_range_x != 0.0);
		}
		inline bool range_max_y() const {
			return (this->type_y == Type::RANGE_MAX && this->offset_range_y != 0.0);
		}
		inline bool offset_x() const {
			return (this->type_x == Type::OFFSET);
		}
		inline bool offset_y() const {
			return (this->type_y == Type::OFFSET);
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
