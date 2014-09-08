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
	// debugging code switch
	public:
		static constexpr bool DBG = false;

	// enum classes; have to be defined first
	public:
		enum class Type : int {OFFSET = 0, MIN = 1, MAX = 2, UNDEF = -1};
		enum class Global_Type : int {STRICT = 0, FLEXIBLE = 1};

	// private data, functions
	private:
		Type type_x;
		Type type_y;

	// constructors, destructors, if any non-implicit
	public:
		CorblivarAlignmentReq(int const& id,
				Global_Type const& type,
				int const& signals,
				Block const* s_i, Block const* s_j,
				Type const& type_x, double const& alignment_x,
				Type const& type_y, double const& alignment_y) {

			this->id = id;

			this->type = type;

			this->signals = signals;

			this->s_i = s_i;
			this->s_j = s_j;

			this->type_x = type_x;
			this->type_y = type_y;

			this->alignment_x = alignment_x;
			this->alignment_y = alignment_y;

			fulfilled = false;

			// fix negative range, if required; only for offsets, a negative
			// value is applicable
			if (
				(this->alignment_x < 0 && !this->offset_x()) ||
				(this->alignment_y < 0 && !this->offset_y())
			   ) {

				// negative range can be trivially resolved
				this->alignment_x = abs(this->alignment_x);
				this->alignment_y = abs(this->alignment_y);
			}
		};

	// public data, functions
	public:
		int id;
		Block const* s_i;
		Block const* s_j;
		double alignment_x, alignment_y;
		Global_Type type;
		int signals;
		mutable bool fulfilled;

		friend std::ostream& operator<< (std::ostream& out, Type const& type) {

			switch (type) {

				case Type::MIN:
					out << "MIN";
					break;
				case Type::MAX:
					out << "MAX";
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

		friend std::ostream& operator<< (std::ostream& out, Global_Type const& type) {

			switch (type) {

				case Global_Type::STRICT:
					out << "STRICT";
					break;
				case Global_Type::FLEXIBLE:
					out << "FLEXIBLE";
					break;
				default:
					out << "UNDEF";
					break;
			}

			return out;
		}

		inline bool range_x() const {
			return (this->type_x == Type::MIN && this->alignment_x != 0.0);
		}
		inline bool range_y() const {
			return (this->type_y == Type::MIN && this->alignment_y != 0.0);
		}
		inline bool range_max_x() const {
			return (this->type_x == Type::MAX && this->alignment_x != 0.0);
		}
		inline bool range_max_y() const {
			return (this->type_y == Type::MAX && this->alignment_y != 0.0);
		}
		inline bool offset_x() const {
			return (this->type_x == Type::OFFSET);
		}
		inline bool offset_y() const {
			return (this->type_y == Type::OFFSET);
		}

		inline std::string tupleString() const {
			std::stringstream ret;

			ret << "(" << this->type << ", " << this->signals << ", ";
			ret << "(" << this->s_i->id << ", " << this->s_j->id << ", ";
			ret << "(" << this->type_x << ", " << this->alignment_x << "), ";
			ret << "(" << this->type_y << ", " << this->alignment_y << ") )";

			return ret.str();
		};

		double evaluate() const;
};

#endif
