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
		static constexpr bool DBG_LAYOUT_GENERATION = false;

	// enum classes; have to be defined first
	public:
		enum class Type : int {OFFSET = 0, MIN = 1, MAX = 2, UNDEF = -1};
		enum class Handling : int {STRICT = 0, FLEXIBLE = 1};

	// constructors, destructors, if any non-implicit
	public:
		CorblivarAlignmentReq(int const& id,
				Handling const& handling,
				int const& signals,
				Block const* s_i, Block const* s_j,
				Type const& type_x, double const& alignment_x,
				Type const& type_y, double const& alignment_y) {

			this->id = id;

			this->handling = handling;

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
		Type type_x;
		Type type_y;
		double alignment_x, alignment_y;
		Handling handling;
		int signals;
		mutable bool fulfilled;

		struct Evaluate {
			double cost;
			double actual_mismatch;
		};

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

		friend std::ostream& operator<< (std::ostream& out, Handling const& handling) {

			switch (handling) {

				case Handling::STRICT:
					out << "STRICT";
					break;
				case Handling::FLEXIBLE:
					out << "FLEXIBLE";
					break;
				default:
					out << "UNDEF";
					break;
			}

			return out;
		}

		// alignment-type getter
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

		// alignment evaluation helper
		//
		inline bool vertical_bus() const {
			return (
				// zero-offset fixed alignment in both coordinates
				(this->offset_x() && this->alignment_x == 0 && this->offset_y() && this->alignment_y == 0) ||
				// min overlap in both dimensions
				(this->range_x() && this->range_y())
			);
		}
		inline bool partner_blocks(Block const* b1, Block const* b2) const {
			return (
				(b1->id == this->s_i->id && b2->id == this->s_j->id) ||
				(b1->id == this->s_j->id && b2->id == this->s_i->id)
			       );
		}

		inline std::string tupleString() const {
			std::stringstream ret;

			ret << "(" << this->handling << ", " << this->signals << ", ";
			ret << "(" << this->s_i->id << ", " << this->s_j->id << ", ";
			ret << "(" << this->type_x << ", " << this->alignment_x << "), ";
			ret << "(" << this->type_y << ", " << this->alignment_y << ") )";

			return ret.str();
		};

		Evaluate evaluate() const;
};

#endif
