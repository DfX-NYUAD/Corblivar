/*
 * =====================================================================================
 *
 *    Description:  Corblivar alignment requests data
 *
 *    Copyright (C) 2013 Johann Knechtel, johann.knechtel@ifte.de, www.ifte.de
 *
 *    This file is part of Corblivar.
 *    
 *    Corblivar is free software: you can redistribute it and/or modify it under the terms
 *    of the GNU General Public License as published by the Free Software Foundation,
 *    either version 3 of the License, or (at your option) any later version.
 *    
 *    Corblivar is distributed in the hope that it will be useful, but WITHOUT ANY
 *    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
 *    PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *    
 *    You should have received a copy of the GNU General Public License along with
 *    Corblivar.  If not, see <http://www.gnu.org/licenses/>.
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

			fulfilled = false;

			// fix negative range, if required; only for offsets, a negative
			// value is applicable
			if (
				(this->offset_range_x < 0 && !this->offset_x()) ||
				(this->offset_range_y < 0 && !this->offset_y())
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
		mutable bool fulfilled;

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
