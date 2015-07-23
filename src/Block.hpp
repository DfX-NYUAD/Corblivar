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
#include "MultipleVoltages.hpp"
// forward declarations, if any
class CorblivarAlignmentReq;
class ContiguityAnalysis;

class Block {
	// debugging code switch (private)
	private:

	// private data, functions
	private:
		friend class IO;

		// these values are required for multi-voltage domains; the
		// power_density_unscaled is read in from the benchmarks, while the
		// factors are read in from the Technology.conf, base_delay is calculated
		// according to [Lin10] during block parsing; the power_density() and
		// delay() are calculated accordingly to these factors and the current
		// assigned voltage, given in assigned_voltage_index
		//
		double power_density_unscaled;
		std::vector<double> voltages_power_factors;
		double base_delay;
		std::vector<double> voltages_delay_factors;

		// delay in [ns]; relates to net delay and currently assigned voltage;
		// theoretical value to be obtained for given voltage index
		inline double delay(unsigned voltage_index) const {
			return
				this->base_delay * this->voltages_delay_factors[voltage_index]
				+ this->net_delay_max;
		}

	// enum class for alignment status; has to be defined first
	public:
		// flags to indicate whether the block is associated with some alignment
		// and also if this alignment is successful or the block is too far of in
		// a particular direction; note that only one (failing) direction at one
		// time is considered
		enum class AlignmentStatus : unsigned {UNDEF, SUCCESS, FAIL_HOR_TOO_LEFT, FAIL_HOR_TOO_RIGHT, FAIL_VERT_TOO_LOW, FAIL_VERT_TOO_HIGH};

	// constructors, destructors, if any non-implicit
	public:
		Block(std::string const& id) {
			this->id = id;
			this->layer = -1;
			this->power_density_unscaled = this->base_delay = 0.0;
			this->AR.min = AR.max = 1.0;
			this->placed = false;
			this->soft = false;
			this->floorplacement = false;
			this->alignment = AlignmentStatus::UNDEF;
			this->rotatable = true;
			this->net_delay_max = 0.0;
		};

	// public data, functions
	public:
		std::string id;
		mutable int layer;

		// flag to monitor placement; also required for alignment handling
		mutable bool placed;

		// flag to monitor block alignment; only considers status of most recently
		// evaluated request but not all associated request
		mutable AlignmentStatus alignment;

		// pointers to alignments representing vertical bus, if any
		mutable std::list<CorblivarAlignmentReq*> alignments_vertical_bus;

		// density in [uW/(um^2)]; relates to given voltage index or currently
		// assigned voltage
		inline double power_density(unsigned index) const {
			return this->power_density_unscaled * this->voltages_power_factors[index];
		}
		inline double power_density() const {
			return this->power_density_unscaled * this->voltages_power_factors[this->assigned_voltage_index];
		}
		inline double power_density_max() const {
			return this->power_density_unscaled * this->voltages_power_factors.back();
		}

		// delay in [ns]; relates to net delay and currently assigned voltage
		inline double delay() const {
			return
				// the module delay resulting from current voltage
				// assignment
				this->base_delay * this->voltages_delay_factors[this->assigned_voltage_index]
				// the net delay, greater zero for any driving block
				+ this->net_delay_max;
		}

		// this delay value is the max value for any net where this block is the
		// source/driving block
		mutable double net_delay_max;

		// bit-wise flags for applicable voltages, where feasible_voltages[k]
		// encodes the highest voltage V_k, and remaining bits encode the lower
		// voltages in descending order; note that if less than
		// MultipleVoltages::MAX_VOLTAGES are globally available, the non-required
		// bits are left as is, i.e., zero by constructor definition
		mutable std::bitset<MultipleVoltages::MAX_VOLTAGES> feasible_voltages;

		// current voltage assignment; this index refers to the lowest applicable
		// voltage, according the lowest set bit of feasible_voltages; this index
		// will also be used for proper delay and power calculations
		mutable unsigned assigned_voltage_index;

		// this is the module where the block is finally assigned to
		mutable MultipleVoltages::CompoundModule* assigned_module;

		// reset feasible voltages and voltage assignment; the highest possible
		// voltage shall be considered as set; this enables all the related
		// functions to return correct values even if no assignment is performed
		// and/or only one voltage is globally available
		inline void resetVoltageAssignment() {
			this->feasible_voltages.reset();
			this->feasible_voltages[this->voltages_power_factors.size() - 1] = 1;
			this->assigned_voltage_index = this->voltages_power_factors.size() - 1;
		}

		// helper to set/update feasible voltages; considers a delay threshold; a
		// voltage is considered feasible as long as setting it will not violate
		// the delay threshold (by increasing the module delay too much)
		//
		inline void setFeasibleVoltages(double delay_threshold) {
			unsigned index;

			// the first index, i.e., the index for the highest applicable
			// voltage; this voltage is set per definition
			index = this->voltages_power_factors.size() - 1;

			// try to consider the next-lower index / voltage as long as the
			// resulting delay is not violating the threshold; also consider
			// lower limit for index
			//
			while (index > 0 && this->delay(index - 1) <= delay_threshold) {

				// consider this voltage as feasible; memorize it
				index--;
				this->feasible_voltages[index] = 1;
			}
		}

		// vector of contiguous neighbours, required for voltage assignment
		mutable std::vector<ContiguityAnalysis::ContiguousNeighbour> contiguous_neighbours;

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

		// blocks related to STRICT alignment requests will not be rotatable
		mutable bool rotatable;

		// layout-generation related helper; perform operations on mutable bb,
		// thus marked const
		//
		inline bool rotate() const {

			if (this->rotatable) {
				std::swap(this->bb.w, this->bb.h);
				return true;
			}
			else {
				return false;
			}
		};
		inline bool shapeRandomlyByAR() const {

			if (this->rotatable) {
				// reshape block randomly w/in AR range; note that x^2 = AR * A
				this->bb.w = std::sqrt(Math::randF(this->AR.min, this->AR.max) * this->bb.area);
				this->bb.h = this->bb.area / this->bb.w;
				this->bb.ur.x = this->bb.ll.x + this->bb.w;
				this->bb.ur.y = this->bb.ll.y + this->bb.h;

				return true;
			}
			else {
				return false;
			}
		};
		inline bool shapeByWidthHeight(double const& width, double const& height) const {
			double AR;

			AR = width / height;

			// apply new dimensions in case the resulting AR is allowed; also
			// consider whether block should be allowed to rotated / reshaped
			// at all
			if (this->AR.min <= AR && AR <= this->AR.max && this->rotatable) {

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

		// power in [W]; returns power according to currently assigned voltage
		inline double power() const {

			return this->power_density()
				// power density is given in uW/um^2, area is given in
				// um^2, thus we have to convert uW to W
				* this->bb.area	* 1.0e-6;
		}
		// power in [W]; returns power according to given voltage index
		inline double power(unsigned voltage_index) const {

			return this->power_density(voltage_index)
				// power density is given in uW/um^2, area is given in
				// um^2, thus we have to convert uW to W
				* this->bb.area	* 1.0e-6;
		}

		// the theoretical max power, for highest applicable voltage; note that
		// this value is static, i.e., does not depend on current set of feasible
		// voltages derived from voltage assignment
		inline double power_max() const {

			return this->power_density_max()
				// power density is given in uW/um^2, area is given in
				// um^2, thus we have to convert uW to W
				* this->bb.area	* 1.0e-6;
		}

		// the theoretical min power, for lowest applicable voltage; considers
		// current set of feasible voltages which, in turn, will be affected by
		// the current delay values
		inline double power_min() const {

			for (unsigned v = 0; v < MultipleVoltages::MAX_VOLTAGES; v++) {

				if (this->feasible_voltages[v]) {
					return this->power(v);
				}
			}

			return this->power_max();
		}

		// search blocks
		inline static Block const* findBlock(std::string const& id, std::vector<Block> const& container) {

			for (Block const& b : container) {
				if (b.id == id) {
					return &b;
				}
			}

			return nullptr;
		};

		friend std::ostream& operator<< (std::ostream& out, AlignmentStatus const& status) {

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
		// terminal pins are by definition to be placed onto lowermost die 0
		static constexpr int LAYER = 0;

		Pin (std::string const& id) : Block(id) {
			this->layer = LAYER;
		};

		// search pins
		inline static Pin const* findPin(std::string const& id, std::vector<Pin> const& container) {

			for (Pin const& b : container) {
				if (b.id == id) {
					return &b;
				}
			}

			return nullptr;
		};
};

// derived TSVs class; encapsulates TSV island / bundle of TSVs
class TSV_Island : public Block {
	// debugging code switch (private)
	private:
		static constexpr bool DBG = false;

	// constructors, destructors, if any non-implicit
	//
	public:
		TSV_Island (std::string const& id, int const& TSVs_count, double const& TSV_pitch, Rect const& bb, int const& layer, double width = -1.0) : Block(id) {

			this->TSVs_count = TSVs_count;
			this->layer = layer;
			this->bb = bb;

			this->resetOutline(TSV_pitch, width);
		};

	// public data, functions
	public:
		int TSVs_count;

		// reset TSV group's outline according to area required for given TSVs
		//
		// note that the following code does _not_ consider a sanity check where
		// the required area for TSVs is larger than the provided bb; since TSVs
		// are assumed to be embedded into blocks later on anyway, such over-usage
		// of area is not critical
		void resetOutline(double TSV_pitch, double width) {
			double TSV_rows, TSV_cols;
			Rect new_bb;

			// if width is given, orient the island's dimension based on that
			if (width > 0.0) {
				new_bb.w = width;
				new_bb.h = this->TSVs_count * pow(TSV_pitch, 2.0) / width;
			}
			// else plan for square island
			else {
				// determine number of TSV rows and cols from number of
				// required TSVs; define a square TSV island
				TSV_rows = std::sqrt(this->TSVs_count);
				TSV_cols = std::sqrt(this->TSVs_count);

				// round up rows and cols, spare TSVs are not as ``bad''
				// as missing TSVs for signal routing; this way it's also
				// guaranteed that at least one row and col are considered
				TSV_rows = std::ceil(TSV_rows);
				TSV_cols = std::ceil(TSV_cols);

				new_bb.w = TSV_rows * TSV_pitch;
				new_bb.h = TSV_cols * TSV_pitch;
			}

			// calculate island's area
			new_bb.area = new_bb.w * new_bb.h;

			// place new bb such into the given bb that their center points
			// are (roughly) aligned
			new_bb.ll.x = std::max(0.0, this->bb.ll.x + (this->bb.w - new_bb.w) / 2.0);
			new_bb.ll.y = std::max(0.0, this->bb.ll.y + (this->bb.h - new_bb.h) / 2.0);

			// determine new bb's upper bound
			new_bb.ur.x = new_bb.ll.x + new_bb.w;
			new_bb.ur.y = new_bb.ll.y + new_bb.h;

			// replace bb w/ new bb
			this->bb = new_bb;

			// dbg logging for TSV scaling
			if (TSV_Island::DBG) {

				std::cout << "DBG_TSVS> TSV group" << std::endl;
				std::cout << "DBG_TSVS>  " << this->id << std::endl;
				std::cout << "DBG_TSVS>  (" << this->bb.ll.x << "," << this->bb.ll.y << ")";
				std::cout << "(" << this->bb.ur.x << "," << this->bb.ur.y << ")" << std::endl;
			}
		}
};

// derived dummy block "RBOD" as ``Reference Block On Die'' for fixed offsets
class RBOD : public Block {
	// public data, functions
	public:
		static constexpr const char* ID = "RBOD";

	// constructors, destructors, if any non-implicit
	//
	// inherits properties of block and defines coordinates as 0,0, i.e., the
	// lower-left corner of the die
	public:
		RBOD () : Block(ID) {

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
