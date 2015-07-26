/*
 * =====================================================================================
 *
 *    Description:  Corblivar handler for multiple voltages
 *
 *    Copyright (C) 2015 Johann Knechtel, johann.knechtel@ifte.de, www.ifte.de
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
#ifndef _CORBLIVAR_MULTIPLEVOLTAGES
#define _CORBLIVAR_MULTIPLEVOLTAGES

// library includes
#include "Corblivar.incl.hpp"
// Corblivar includes, if any
#include "ContiguityAnalysis.hpp"
// forward declarations, if any
class Block;
class Rect;

class MultipleVoltages {
	// debugging code switch (private)
	private:
		static constexpr bool DBG = false;
		static constexpr bool DBG_VERBOSE = false;

	// public constants
	public:
		static constexpr bool DBG_FLOORPLAN = true;

		// dimension for feasible voltages;
		// represents the upper bound for globally available voltages
		static constexpr int MAX_VOLTAGES = 4;

	// public POD, to be declared early on
	public:
		struct Parameters {

			// voltages and related scaling factors for power consumption and
			// module delays
			std::vector<double> voltages;
			std::vector<double> voltages_power_factors;
			std::vector<double> voltages_delay_factors;

			// internal weights, used for internal cost terms
			double weight_power_saving;
			double weight_corners;
			double weight_modules_count;
		} parameters;

		// max evaluation values have to memorized as well, in order to enable
		// comparison during different SA iterations
		struct max_values {
			double inv_power_saving, corners_avg;
			unsigned module_count;
		} max_values;

	// inner class, to be declared early on
	class CompoundModule {

		// private data
		private:
			friend class MultipleVoltages;
			friend class IO;

			// note that the comparator has to be implemented as dedicated
			// class, for proper template instantiation; also note that the
			// actual implementation for operator() is in
			// MultipleVoltages.cpp, to avoid circular dependencies between
			// MultipleVoltages.hpp and Block.hpp
			//
			class blocks_comp {

				public:
					bool operator() (Block const* b1, Block const* b2) const;
			};

			// sorted set of comprised blocks; the compound module's id will
			// be derived from this sorted set
			std::set<Block const*, blocks_comp> blocks;

			// blocks' numerical ids; stored separately for simplified
			// handling of compound modules; also, numerical ids are probably
			// more efficient than using the string ids
			std::unordered_set<unsigned> block_ids;

			// die-wise bounding boxes for whole module; only the set/vector of
			// by other blocks not covered partial boxes are memorized; thus,
			// the die-wise voltage islands' proper outlines are captured here
			std::vector< std::vector<Rect> > outline;

			// (local) cost term: outline cost is ratio of (by other blocks
			// with non-compatible voltage) intruded area of the module's bb;
			// the lower the better; current cost value is calculated via
			// updateOutlineCost()
			//
			double outline_cost = 0.0;

			// container for estimated max number of corners in power rings
			// per die
			//
			// each rectangle / partial bb of the die outline will introduce
			// four corners; however, if for example two rectangles are
			// sharing a boundary then only six corners are found for these
			// two rectangles; thus, we only consider the unique boundaries
			// and estimate that each unique boundary introduces two corners;
			// a shared boundary may arise in vertical or horizontal
			// direction, the actual number of corners will be given by the
			// maximum of both estimates
			std::vector<unsigned> corners_powerring;

			// feasible voltages for whole module; defined by intersection of
			// all comprised blocks
			std::bitset<MAX_VOLTAGES> feasible_voltages;

			// key: neighbour's numerical block id
			//
			// with an map, redundant neighbours which may arise during
			// stepwise build-up of compound modules are ignored; order not
			// required and thus related computations be can avoided
			//
			std::unordered_map<unsigned, ContiguityAnalysis::ContiguousNeighbour*> contiguous_neighbours;

		// private functions
		private:
			// local cost; required during bottom-up construction
			double updateOutlineCost(ContiguityAnalysis::ContiguousNeighbour* neighbour, ContiguityAnalysis& cont, bool apply_update = true);

			// helper to return id comprising all (sorted) block ids;
			// numerical_ids = true provides a simplified string comprising
			// only numerical block ids instead of actual string ids
			std::string id(bool numerical_ids = false) const;

			// helper to return index of minimal assignable voltage, note that
			// this is not necessarily the globally minimal index but rather
			// depends on the intersection of all comprised blocks' voltages
			inline unsigned min_voltage_index() const {

				for (unsigned v = 0; v < MAX_VOLTAGES; v++) {

					if (this->feasible_voltages[v]) {
						return v;
					}
				}

				return MAX_VOLTAGES - 1;
			}

			// global cost, required during top-down selection
			//
			inline double cost(double const& max_power_saving, unsigned const& max_corners, MultipleVoltages::Parameters const& parameters) const;

		// public functions
		public:
			// helper to estimate gain in power reduction
			//
			double power_saving(bool subtract_wasted_saving = true) const;

			// helper to obtain overall (over all dies) max number of corners
			// in power rings
			inline unsigned corners_powerring_max() const {
				unsigned ret = this->corners_powerring[0];

				for (unsigned i = 1; i < this->corners_powerring.size(); i++) {
					ret = std::max(ret, this->corners_powerring[i]);
				}

				return ret;
			}
	};

	// private data, functions
	private:
		friend class IO;

		// map of unique compound modules; keys are the compound modules' string
		// id; unordered map is more efficient in accessing individual elements
		typedef std::unordered_map< std::string, CompoundModule > modules_type;
		modules_type modules;

		// vector of selected modules, filled by selectCompoundModules()
		std::vector<CompoundModule*> selected_modules;

	// constructors, destructors, if any non-implicit
	public:

	// public data, functions
	public:
		void determineCompoundModules(int layers, std::vector<Block> const& blocks, ContiguityAnalysis& contig);
		std::vector<CompoundModule*> const& selectCompoundModules();

	// private helper data, functions
	private:
		void buildCompoundModulesHelper(CompoundModule& module, modules_type::iterator hint, ContiguityAnalysis& cont);
		inline void insertCompoundModuleHelper(
				CompoundModule& module,
				ContiguityAnalysis::ContiguousNeighbour* neighbour,
				bool consider_prev_neighbours,
				std::bitset<MAX_VOLTAGES>& feasible_voltages,
				modules_type::iterator& hint,
				ContiguityAnalysis& cont
			);
};

#endif
