/*
 * =====================================================================================
 *
 *    Description:  Corblivar floorplanner (SA operations and related handler)
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
#ifndef _CORBLIVAR_FLOORPLANNER
#define _CORBLIVAR_FLOORPLANNER

// library includes
#include "Corblivar.incl.hpp"
// Corblivar includes, if any
#include "Block.hpp"
#include "Net.hpp"
#include "ThermalAnalyzer.hpp"
// forward declarations, if any
class CorblivarCore;
class CorblivarAlignmentReq;

class FloorPlanner {
	// debugging code switch (private)
	private:
		static constexpr bool DBG_SA = false;
		static constexpr bool DBG_CALLS_SA = false;
		static constexpr bool DBG_LAYOUT = false;

	// private data, functions
	private:
		// chip data
		vector<Block> blocks;
		vector<Block> terminals;
		vector<Net> nets;
		// dummy reference block, represents lower-left corner of dies
		RBOD const RBOD;

		// 3D IC config parameters
		int conf_layer;
		double conf_outline_x, conf_outline_y;
		double conf_blocks_scale;
		bool conf_outline_shrink;

		// 3D IC characteristica, resulting from config
		double die_AR, die_area;
		// these parameters cover all dies
		double blocks_area;
		double stack_area, stack_deadspace;

		// POD declarations
		struct Cost {
			double cost;
			bool fits_fixed_outline;

			// http://www.learncpp.com/cpp-tutorial/93-overloading-the-io-operators/
			friend ostream& operator<< (ostream& out, Cost const& cost) {
				out << "cost=" << cost.cost << ", fits_fixed_outline=" << cost.fits_fixed_outline;
				return out;
			}
		};
		struct CostInterconn {
			double HPWL;
			int TSVs;
			double TSVs_area_deadspace_ratio;

			friend ostream& operator<< (ostream& out, CostInterconn const& cost) {
				out << "HPWL=" << cost.HPWL << ", TSVs=" << cost.TSVs;
				return out;
			}
		};
		struct TempStep {
			int step;
			double temp;
			double avg_cost;
			bool new_best_sol_found;
			double cost_best_sol;
		};

		// IO
		string benchmark, blocks_file, alignments_file, pins_file, power_density_file, nets_file, thermal_masks_file;
		ofstream results, solution_out;
		ifstream solution_in;
		struct timeb start;
		// flag whether power density file is available / was handled / thermal
		// analysis should be performed / thermal files should be generated
		bool power_density_file_avail;
		// similar flags for other files
		bool alignments_file_avail, thermal_masks_file_avail;

		// logging
		int conf_log;
		static constexpr int LOG_MINIMAL = 1;
		static constexpr int LOG_MEDIUM = 2;
		static constexpr int LOG_MAXIMUM = 3;

		// SA parameters: loop control
		double conf_SA_loopFactor, conf_SA_loopLimit;

		// SA parameters: optimization flags
		bool conf_SA_opt_thermal, conf_SA_opt_interconnects, conf_SA_opt_alignment;

		// SA parameters: cost factors
		double conf_SA_cost_thermal, conf_SA_cost_WL, conf_SA_cost_TSVs, conf_SA_cost_alignment;

		// SA cost variables: max cost values
		double max_cost_thermal, max_cost_WL, max_cost_alignments;
		int max_cost_TSVs;

		// SA cost paramters: global weights, enforce that area and outline
		// violation is constantly considered; related weight should be >= 0.5 in
		// order to enforce guiding into outline during whole optimization run
		static constexpr double SA_COST_WEIGHT_AREA_OUTLINE = 0.5;
		static constexpr double SA_COST_WEIGHT_OTHERS = 1.0 - SA_COST_WEIGHT_AREA_OUTLINE;

		// SA: cost functions, i.e., layout-evalutions
		Cost determCost(vector<CorblivarAlignmentReq> const& alignments, double const& ratio_feasible_solutions_fixed_outline = 0.0,
				bool const& SA_phase_two = false, bool const& set_max_cost = false);
		inline double determCostThermalDistr(bool const& set_max_cost = false, bool const& normalize = true, bool const& return_max_temp = false) {
			this->thermalAnalyzer.generatePowerMaps(this->conf_layer, this->blocks, this->getOutline(),
					this->conf_power_blurring_parameters, this->benchmark);
			return this->thermalAnalyzer.performPowerBlurring(this->conf_layer, this->conf_power_blurring_parameters, this->max_cost_thermal,
					set_max_cost, normalize, return_max_temp);
		};
		double determCostAlignment(vector<CorblivarAlignmentReq> const& alignments, bool const& set_max_cost = false, bool const& normalize = true);
		Cost determWeightedCostAreaOutline(double const& ratio_feasible_solutions_fixed_outline = 0.0) const;
		CostInterconn determCostInterconnects(bool const& set_max_cost = false, bool const& normalize = true);

		// SA: parameters for cost functions
		//
		// trivial HPWL refers to one global bounding box for each net;
		// non-trivial considers the bounding boxes on each layer separately
		static constexpr bool SA_COST_INTERCONNECTS_TRIVIAL_HPWL = false;

		// SA parameter: scaling factor for loops during solution-space sampling
		static constexpr int SA_SAMPLING_LOOP_FACTOR = 1;

		// SA parameter: scaling factor for initial temp
		double conf_SA_temp_init_factor;

		// SA parameters: temperature-scaling factors
		double conf_SA_temp_factor_phase1, conf_SA_temp_factor_phase1_limit, conf_SA_temp_factor_phase2, conf_SA_temp_factor_phase3;

		// SA: temperature-schedule log data
		vector<TempStep> tempSchedule;

		// SA: reheating parameters, for SA phase 3
		static constexpr int SA_REHEAT_COST_SAMPLES = 3;
		static constexpr double SA_REHEAT_STD_DEV_COST_LIMIT = 1.0e-6;

		// SA parameters: layout generation options
		bool conf_SA_layout_enhanced_hard_block_rotation, conf_SA_layout_enhanced_soft_block_shaping;
		bool conf_SA_layout_power_aware_block_handling, conf_SA_layout_floorplacement;
		int conf_SA_layout_packing_iterations;

		// SA: layout-generation handler
		bool generateLayout(CorblivarCore& corb, bool const& perform_alignment);

		// SA: layout operations op-codes
		static constexpr int OP_SWAP_BLOCKS = 1;
		static constexpr int OP_MOVE_TUPLE = 2;
		static constexpr int OP_SWITCH_INSERTION_DIR = 3;
		static constexpr int OP_SWITCH_TUPLE_JUNCTS = 4;
		static constexpr int OP_ROTATE_BLOCK__SHAPE_BLOCK = 5;
		// used only for soft block shaping
		static constexpr int OP_SHAPE_BLOCK__STRETCH_HORIZONTAL = 10;
		static constexpr int OP_SHAPE_BLOCK__STRETCH_VERTICAL = 11;
		static constexpr int OP_SHAPE_BLOCK__SHRINK_HORIZONTAL = 12;
		static constexpr int OP_SHAPE_BLOCK__SHRINK_VERTICAL = 13;
		static constexpr int OP_SHAPE_BLOCK__RANDOM_AR = 14;

		// SA: layout-operation handler variables
		int last_op, last_op_die1, last_op_die2, last_op_tuple1, last_op_tuple2, last_op_juncts;

		// SA: layout-operation handler
		bool performRandomLayoutOp(CorblivarCore& corb, bool const& SA_phase_two = false, bool const& revertLastOp = false);
		// note that die and tuple parameters are return-by-reference; non-const
		// reference for CorblivarCore in order to enable operations on CBL-encode data
		void inline prepareBlockSwappingFailedAlignment(CorblivarCore const& corb, int& die1, int& tuple1, int& die2, int& tuple2);
		bool inline performOpMoveOrSwapBlocks(int const& mode, bool const& revert, bool const& SA_phase_one, CorblivarCore& corb,
				int& die1, int& die2, int& tuple1, int& tuple2) const;
		bool inline performOpSwitchInsertionDirection(bool const& revert, CorblivarCore& corb, int& die1, int& tuple1) const;
		bool inline performOpSwitchTupleJunctions(bool const& revert, CorblivarCore& corb, int& die1, int& tuple1, int& juncts) const;
		bool inline performOpShapeBlock(bool const& revert, CorblivarCore& corb, int& die1, int& tuple1) const;
		bool inline performOpEnhancedHardBlockRotation(CorblivarCore const& corb, Block const* shape_block) const;
		bool inline performOpEnhancedSoftBlockShaping(CorblivarCore const& corb, Block const* shape_block) const;

		// SA: helper for guided layout operations
		//
		// auxilary chip data, tracks major block power density parameters
		struct power_stats {
			double max;
			double min;
			double range;
			double avg;
		} blocks_power_density_stats;

		// SA: ``floorplacement'' parameters, i.e., there should be different
		// processes for floorplanning and placement; here, we use this limit to
		// detect if floorplanning of very large blocks w/ small (/medium) blocks
		// is required
		//
		static constexpr unsigned FP_AREA_RATIO_LIMIT = 50;

		// SA: helper for main handler
		// note that various parameters are return-by-reference
		void initSA(CorblivarCore& corb, vector<double>& cost_samples, int& innerLoopMax, double& init_temp);
		inline void updateTemp(double& cur_temp, int const& iteration, int const& iteration_first_valid_layout) const;

		// thermal analyzer
		ThermalAnalyzer thermalAnalyzer;

		// thermal analyzer parameters: thermal mask parameters
		vector<ThermalAnalyzer::MaskParameters> conf_power_blurring_parameters;

	// constructors, destructors, if any non-implicit
	public:
		FloorPlanner() {
			// memorize start time
			ftime(&(this->start));

			// init random number generator
			srand(time(0));
		}

	// public data, functions
	public:
		friend class IO;

		// logging
		inline bool logMin() const {
			return (this->conf_log >= LOG_MINIMAL);
		};
		inline bool logMed() const {
			return (this->conf_log >= LOG_MEDIUM);
		};
		inline bool logMax() const {
			return (this->conf_log >= LOG_MAXIMUM);
		};

		// ThermalAnalyzer: handler
		inline void initThermalAnalyzer() {

			// init sets of thermal masks
			this->thermalAnalyzer.initThermalMasks(this->conf_layer, this->logMed(), this->conf_power_blurring_parameters);

			// init power maps, i.e. predetermine maps parameters
			this->thermalAnalyzer.initPowerMaps(this->conf_layer, this->getOutline());
		};

		// getter / setter
		inline int const& getLayers() const {
			return this->conf_layer;
		};

		inline Point getOutline() const {
			Point ret;

			ret.x = this->conf_outline_x;
			ret.y = this->conf_outline_y;

			return ret;
		};

		inline bool const& powerAwareBlockHandling() {
			return this->conf_SA_layout_power_aware_block_handling;
		};

		inline string const& getBenchmark() const {
			return this->benchmark;
		};

		inline vector<Block> const& getBlocks() const {
			return this->blocks;
		};

		// additional helper
		//
		inline void resetDieProperties(double const& outline_x, double const& outline_y) {

			// reset outline
			this->conf_outline_x = outline_x;
			this->conf_outline_y = outline_y;

			// this also requires to reset the power maps setting
			this->thermalAnalyzer.initPowerMaps(this->conf_layer, this->getOutline());

			// reset related die properties
			this->die_AR = this->conf_outline_x / this->conf_outline_y;
			this->die_area = this->conf_outline_x * this->conf_outline_y;
			this->stack_area = this->conf_layer * this->die_area;
			this->stack_deadspace = this->stack_area - this->blocks_area;

			// rescale terminal pins' locations
			this->scaleTerminalPins();
		}

		inline void scaleTerminalPins() {
			double pins_scale_x, pins_scale_y;

			// scale terminal pins; first determine original pins outline
			pins_scale_x = pins_scale_y = 0.0;
			for (Block const& pin : this->terminals) {
				pins_scale_x = max(pins_scale_x, pin.bb.ll.x);
				pins_scale_y = max(pins_scale_y, pin.bb.ll.y);
			}
			// scale terminal pins; scale pin coordinates according to die outline
			pins_scale_x = this->conf_outline_x / pins_scale_x;
			pins_scale_y = this->conf_outline_y / pins_scale_y;
			for (Block& pin : this->terminals) {
				pin.bb.ll.x *= pins_scale_x;
				pin.bb.ll.y *= pins_scale_y;
				// also set upper right to same coordinates, thus pins are ``point''
				// blocks w/ zero area
				pin.bb.ur.x = pin.bb.ll.x;
				pin.bb.ur.y = pin.bb.ll.y;
			}
		}

		inline bool inputSolutionFileOpen() const {
			return this->solution_in.is_open();
		};

		// SA: handler
		bool performSA(CorblivarCore& corb);
		void finalize(CorblivarCore& corb, bool const& determ_overall_cost = true, bool const& handle_corblivar = true);
};

#endif
