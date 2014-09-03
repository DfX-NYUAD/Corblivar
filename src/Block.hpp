/*
 * =====================================================================================
 *
 *    Description:  Corblivar design block
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
#ifndef _CORBLIVAR_BLOCK
#define _CORBLIVAR_BLOCK

// library includes
#include "Corblivar.incl.hpp"
// Corblivar includes, if any
#include "Rect.hpp"
#include "Math.hpp"
// forward declarations, if any

class Block {
	// debugging code switch (private)
	private:

	// private data, functions
	private:

	// enum class for alignment status; has to be defined first
	public:
		// flags to indicate whether the block is associated with some alignment
		// and also if this alignment is successful or the block is too far of in
		// a particular direction; note that only one (failing) direction at one
		// time is considered
		enum class AlignmentStatus : unsigned {UNDEF, SUCCESS, FAIL_HOR_TOO_LEFT, FAIL_HOR_TOO_RIGHT, FAIL_VERT_TOO_LOW, FAIL_VERT_TOO_HIGH};

	// constructors, destructors, if any non-implicit
	public:
		Block(string const& id) {
			this->id = id;
			this->layer = -1;
			this->power_density= 0.0;
			this->AR.min = AR.max = 1.0;
			this->placed = false;
			this->soft = false;
			this->floorplacement = false;
			this->alignment = AlignmentStatus::UNDEF;
		};

	// public data, functions
	public:
		string id;
		mutable int layer;

		// flag to monitor placement; also required for alignment handling
		mutable bool placed;

		// flag to monitor block alignment
		mutable AlignmentStatus alignment;

		// density in [uW/(um^2)]
		double power_density;

		// rectangle, represents block geometry and placement
		mutable Rect bb, bb_backup, bb_best;

		// aspect ratio AR, relates to blocks' dimensions by x / y
		struct AR {
			double min;
			double max;
		} AR;
		// AR is only relevant for soft blocks
		bool soft;

		// large macro, flag for floorplacement handling
		bool floorplacement;

		// layout-generation related helper; perform operations on mutable bb,
		// thus marked const
		//
		inline void rotate() const {
			swap(this->bb.w, this->bb.h);
		};
		inline void shapeRandomlyByAR() const {

			// reshape block randomly w/in AR range; note that x^2 = AR * A
			this->bb.w = sqrt(Math::randF(this->AR.min, this->AR.max) * this->bb.area);
			this->bb.h = this->bb.area / this->bb.w;
			this->bb.ur.x = this->bb.ll.x + this->bb.w;
			this->bb.ur.y = this->bb.ll.y + this->bb.h;
		};
		inline bool shapeByWidthHeight(double const& width, double const& height) const {
			double AR;

			AR = width / height;

			// apply new dimensions in case the resulting AR is allowed
			if (this->AR.min <= AR && AR <= this->AR.max) {

				this->bb.ur.x = this->bb.ll.x + width;
				this->bb.ur.y = this->bb.ll.y + height;
				this->bb.w = width;
				this->bb.h = height;

				return true;
			}
			else {
				return false;
			}
		};

		// power in [W]
		inline double power() const {
			// power density is given in uW/um^2, area is given in um^2, thus
			// we have to convert uW to W
			return this->power_density * this->bb.area * 1.0e-6;
		}

		// search blocks
		inline static Block const* findBlock(string const& id, vector<Block> const& container) {

			for (Block const& b : container) {
				if (b.id == id) {
					return &b;
				}
			}

			return nullptr;
		};

		friend ostream& operator<< (ostream& out, AlignmentStatus const& status) {

			switch (status) {

				case AlignmentStatus::SUCCESS:
					out << "SUCCESS";
					break;
				case AlignmentStatus::FAIL_HOR_TOO_LEFT:
					out << "FAIL_HOR_TOO_LEFT";
					break;
				case AlignmentStatus::FAIL_HOR_TOO_RIGHT:
					out << "FAIL_HOR_TOO_RIGHT";
					break;
				case AlignmentStatus::FAIL_VERT_TOO_LOW:
					out << "FAIL_VERT_TOO_LOW";
					break;
				case AlignmentStatus::FAIL_VERT_TOO_HIGH:
					out << "FAIL_VERT_TOO_HIGH";
					break;
				default:
					out << "UNDEF";
					break;
			}

			return out;
		}
};

// derived pin class
class Pin : public Block {

	// constructors, destructors, if any non-implicit
	//
	public:
		Pin (string const& id) : Block(id) {
		};

		// search pins
		inline static Pin const* findPin(string const& id, vector<Pin> const& container) {

			for (Pin const& b : container) {
				if (b.id == id) {
					return &b;
				}
			}

			return nullptr;
		};
};

// derived TSVs class; encapsulates TSV island / bundle of TSVs
class TSV_Group : public Block {

	// constructors, destructors, if any non-implicit
	//
	public:
		// TODO unify
		TSV_Group (string const& id, int const& TSVs_count, Rect const& bb, int const& layer) : Block(id) {

			this->TSVs_count = TSVs_count;
			this->layer = layer;
			this->bb = bb;
		};

		TSV_Group (string const& id, int const& TSVs_count, int const& layer) : Block(id) {

			this->TSVs_count = TSVs_count;
			this->layer = layer;
		};

	// public data, functions
	public:
		int TSVs_count;
};

// derived dummy block "RBOD" as ``Reference Block On Die'' for fixed offsets
class RBOD : public Block {

	// constructors, destructors, if any non-implicit
	//
	// inherits properties of block and defines coordinates as 0,0, i.e., the
	// lower-left corner of the die
	public:
		RBOD () : Block("RBOD") {

			this->bb.ll.x = 0.0;
			this->bb.ll.y = 0.0;
			this->bb.ur.x = 0.0;
			this->bb.ur.y = 0.0;

			// also consider this dummy block as placed, i.e., not to be
			// shifted for alignment
			this->placed = true;
		};
};
#endif
