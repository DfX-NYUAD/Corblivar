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
#include "Net.hpp"
#include "LayoutOperations.hpp"
#include "ThermalAnalyzer.hpp"
#include "Clustering.hpp"
#include "RoutingUtilization.hpp"
// forward declarations, if any
class Block;
class CorblivarCore;
class CorblivarAlignmentReq;

class FloorPlanner {
	// debugging code switch (private)
	private:
		static constexpr bool DBG_SA = false;
		static constexpr bool DBG_CALLS_SA = false;
		static constexpr bool DBG_LAYOUT = false;
		static constexpr bool DBG_TSVS = false;

	// private data, functions
	private:
		// chip data
		std::vector<Block> blocks;
		std::vector<Pin> terminals;
		std::vector<Net> nets;

		// groups of TSVs, will be defined from nets and vertical buses
		std::vector<TSV_Island> TSVs;

		// dummy reference block, represents lower-left corner of dies
		RBOD const RBOD;

		// 3D IC parameters
		struct IC {

			int layers;

			// general geometrical parameters
			double outline_x, outline_y;
			double blocks_scale;
			bool outline_shrink;
			double die_AR, die_area;
			// note that the two parameters below cover all dies
			double blocks_area;
			double stack_area, stack_deadspace;

			// (TODO) refactor into own struct
			// technology parameters
			//
			double die_thickness;;
			double Si_active_thickness;
			double Si_passive_thickness;
			double BEOL_thickness;
			double bond_thickness;
			double TSV_dimension;
			double TSV_pitch;
			// Cu-Si area ratio for TSV groups
			double TSV_group_Cu_Si_ratio;
			// Cu area fraction for TSV groups
			double TSV_group_Cu_area_ratio;

			// global delay threshold
			double delay_threshold;

		} IC;

		// IO files and parameters
		struct IO_conf {
			std::string blocks_file, alignments_file, pins_file, power_density_file, nets_file, solution_file;
			std::ofstream results, solution_out;
			std::ifstream solution_in;
			// flag whether power density file is available / was handled /
			// thermal analysis should be performed / thermal files should be
			// generated
			bool power_density_file_avail;
			// similar flags for other files
			bool alignments_file_avail;
		} IO_conf;

		// benchmark name
		std::string benchmark;

		// run mode; represents thermal analyser runs where TSV density is given
		// as command-line parameter
		bool thermal_analyser_run;

		// time logging
		struct timeb time_start;

		// logging
		int log;
		static constexpr int LOG_MINIMAL = 1;
		static constexpr int LOG_MEDIUM = 2;
		static constexpr int LOG_MAXIMUM = 3;

		// SA schedule parameters
		struct schedule {
			// SA parameters: loop control
			double loop_factor, loop_limit;

			// SA parameter: scaling factor for initial temp
			double temp_init_factor;

			// SA parameters: temperature-scaling factors
			double temp_factor_phase1, temp_factor_phase1_limit, temp_factor_phase2, temp_factor_phase3;
		} schedule;

		// SA parameters: optimization flags
		struct opt_flags {
			bool thermal, interconnects, alignment, voltage_assignment, timing;
		} opt_flags;

		// SA parameters: cost factors/weights
		struct weights {
			double thermal, WL, TSVs, alignment, routing_util, timing, voltage_assignment;
		} weights;

		// (TODO) refactor into own struct
		// SA cost variables: max cost values
		double max_cost_thermal, max_cost_WL, max_cost_alignments, max_cost_routing_util, max_cost_timing, max_cost_voltage_assignment;
		int max_cost_TSVs;

		// SA cost parameters: global weights, enforce that area and outline
		// violation is constantly considered; related weight should be >= 0.5 in
		// order to enforce guiding into outline during whole optimization run
		// TODO dynamically, should be 1/n for n objectives
		static constexpr double SA_COST_WEIGHT_AREA_OUTLINE = 0.1;
		static constexpr double SA_COST_WEIGHT_OTHERS = 1.0 - SA_COST_WEIGHT_AREA_OUTLINE;

		// SA cost; POD declaration
		struct Cost {
			double total_cost;
			double total_cost_fitting;
			double HPWL;
			double HPWL_actual_value;
			double routing_util;
			double routing_util_actual_value;
			// requires double since it contains normalized values
			double TSVs;
			int TSVs_actual_value;
			double TSVs_area_deadspace_ratio;
			double alignments;
			double alignments_actual_value;
			double thermal;
			double thermal_actual_value;
			double area_outline;
			double area_actual_value;
			double outline_actual_value;
			bool fits_fixed_outline;
			double timing;
			double timing_actual_value;
			double voltage_assignment;
			double voltage_assignment_power_saving;
			double voltage_assignment_corners_avg;
			unsigned voltage_assignment_modules_count;

			// http://www.learncpp.com/cpp-tutorial/93-overloading-the-io-operators/
			friend std::ostream& operator<< (std::ostream& out, Cost const& cost) {
				out << "cost=" << cost.total_cost << ", fits_fixed_outline=" << cost.fits_fixed_outline;
				return out;
			}
		};

		// SA: cost functions, i.e., layout-evaluations
		Cost evaluateLayout(std::vector<CorblivarAlignmentReq> const& alignments,
				double const& fitting_layouts_ratio = 0.0,
				bool const& SA_phase_two = false,
				bool const& set_max_cost = false,
				bool const& finalize = false);
		void evaluateThermalDistr(Cost& cost,
				bool const& set_max_cost = false);
		void evaluateAlignments(Cost& cost,
				std::vector<CorblivarAlignmentReq> const& alignments,
				bool const& derive_TSVs = true,
				bool const& set_max_cost = false,
				bool const& finalize = false);
		double evaluateAlignmentsHPWL(std::vector<CorblivarAlignmentReq> const& alignments);
		void evaluateAreaOutline(Cost& cost,
				double const& fitting_layouts_ratio = 0.0) const;
		void evaluateInterconnects(Cost& cost,
				std::vector<CorblivarAlignmentReq> const& alignments,
				bool const& set_max_cost = false);
		void evaluateTiming(Cost& cost,
				bool const& set_max_cost = false);
		void evaluateVoltageAssignment(Cost& cost,
				double const& fitting_layouts_ratio,
				bool const& set_max_cost = false);

		// SA: parameters for cost functions
		//
		// trivial HPWL refers to one global bounding box for each net;
		// non-trivial (i.e., setting the flag to false) considers the bounding
		// boxes on each layer separately
		static constexpr bool SA_COST_INTERCONNECTS_TRIVIAL_HPWL = false;

		// SA parameter: scaling factor for loops during solution-space sampling
		static constexpr int SA_SAMPLING_LOOP_FACTOR = 1;

		// SA-related temperature step; POD declaration
		struct TempStep {
			int step;
			double temp;
			double avg_cost;
			bool new_best_sol_found;
			double cost_best_sol;
		};

		// SA-related temperature phase; POD declaration
		enum TempPhase : unsigned {PHASE_1 = 1, PHASE_2 = 2, PHASE_3 = 3};

		// SA: temperature-schedule log data
		std::vector<TempStep> tempSchedule;

		// SA: reheating parameters, for SA phase 3
		static constexpr int SA_REHEAT_COST_SAMPLES = 3;
		static constexpr double SA_REHEAT_STD_DEV_COST_LIMIT = 1.0e-3;

		// layout-generation handler
		bool generateLayout(CorblivarCore& corb, bool const& perform_alignment = false);

		// layout-operation handler
		LayoutOperations layoutOp;

		// auxiliary chip data, tracks major block power density parameters
		struct power_stats {
			double max;
			double min;
			double range;
			double avg;
		} power_stats;

		// ``floorplacement'' parameters, i.e., there should be different
		// processes for floorplanning and placement; here, we use this limit to
		// detect if floorplanning of very large blocks w/ small (/medium) blocks
		// is required
		//
		static constexpr unsigned FP_AREA_RATIO_LIMIT = 50;

		// SA: helper for main handler
		// note that various parameters are return-by-reference
		void initSA(CorblivarCore& corb, std::vector<double>& cost_samples, int& innerLoopMax, double& init_temp);
		TempPhase updateTemp(double& cur_temp, int const& iteration, int const& iteration_first_valid_layout) const;

		// thermal analyzer
		ThermalAnalyzer thermalAnalyzer;

		// (TODO) encapsulate in thermalAnalyzer
		// thermal analyzer parameters; thermal mask parameters
		ThermalAnalyzer::MaskParameters power_blurring_parameters;

		// (TODO) encapsulate in thermalAnalyzer
		// thermal analyzer; current results of thermal analysis
		ThermalAnalyzer::ThermalAnalysisResult thermal_analysis;

		// clustering handler
		Clustering clustering;

		// routing-utilization analyzer
		RoutingUtilization routingUtil;

		// multi-voltage-domain handler
		MultipleVoltages voltageAssignment;

		// contiguity-analysis handler
		ContiguityAnalysis contigAnalyser;

	// constructors, destructors, if any non-implicit
	public:
		FloorPlanner() {
			// memorize start time
			ftime(&(this->time_start));

			// init random number generator
			srand(time(0));
		}

	// public data, functions
	public:
		friend class IO;

		// logging
		inline bool logMin() const {
			return (this->log >= LOG_MINIMAL);
		};
		inline bool logMed() const {
			return (this->log >= LOG_MEDIUM);
		};
		inline bool logMax() const {
			return (this->log >= LOG_MAXIMUM);
		};

		// ThermalAnalyzer: handler
		inline void initThermalAnalyzer() {

			// init sets of thermal masks
			this->thermalAnalyzer.initThermalMasks(this->IC.layers, this->logMed(), this->power_blurring_parameters);

			// init power maps, i.e. allocate data structure and determine
			// maps' geometrical parameters
			this->thermalAnalyzer.initPowerMaps(this->IC.layers, this->getOutline());

			// init thermal map, i.e., allocate data structure
			this->thermalAnalyzer.initThermalMap(this->getOutline());
		};

		// RoutingUtilization: handler
		inline void initRoutingUtilAnalyzer() {
			this->routingUtil.initUtilMaps(this->IC.layers, this->getOutline());
		}

		// getter / setter
		inline int const& getLayers() const {
			return this->IC.layers;
		};

		inline Point getOutline() const {
			Point ret;

			ret.x = this->IC.outline_x;
			ret.y = this->IC.outline_y;

			return ret;
		};

		inline bool const& powerAwareBlockHandling() {
			return this->layoutOp.parameters.power_aware_block_handling;
		};

		inline std::string const& getBenchmark() const {
			return this->benchmark;
		};

		inline std::vector<Block> const& getBlocks() const {
			return this->blocks;
		};

		// additional helper
		//
		inline void resetDieProperties(double const& outline_x, double const& outline_y) {

			// reset outline
			this->IC.outline_x = outline_x;
			this->IC.outline_y = outline_y;

			// this also requires to reset the power maps setting
			this->thermalAnalyzer.initPowerMaps(this->IC.layers, this->getOutline());

			// and the routing-estimation maps have to be reset as well
			this->routingUtil.initUtilMaps(this->IC.layers, this->getOutline());

			// reset related die properties
			this->IC.die_AR = this->IC.outline_x / this->IC.outline_y;
			this->IC.die_area = this->IC.outline_x * this->IC.outline_y;
			this->IC.stack_area = this->IC.layers * this->IC.die_area;
			this->IC.stack_deadspace = this->IC.stack_area - this->IC.blocks_area;

			// rescale terminal pins' locations
			this->scaleTerminalPins();
		}

		inline void scaleTerminalPins() {
			double pins_scale_x, pins_scale_y;

			// scale terminal pins; first determine original pins outline
			pins_scale_x = pins_scale_y = 0.0;
			for (Pin const& pin : this->terminals) {
				pins_scale_x = std::max(pins_scale_x, pin.bb.ll.x);
				pins_scale_y = std::max(pins_scale_y, pin.bb.ll.y);
			}
			// scale terminal pins; scale pin coordinates according to die outline
			pins_scale_x = this->IC.outline_x / pins_scale_x;
			pins_scale_y = this->IC.outline_y / pins_scale_y;
			for (Pin& pin : this->terminals) {
				pin.bb.ll.x *= pins_scale_x;
				pin.bb.ll.y *= pins_scale_y;
				// also set upper right to same coordinates, thus pins are ``point''
				// blocks w/ zero area
				pin.bb.ur.x = pin.bb.ll.x;
				pin.bb.ur.y = pin.bb.ll.y;
			}
		}

		inline bool inputSolutionFileOpen() const {
			return this->IO_conf.solution_in.is_open();
		};

		// SA: handler
		bool performSA(CorblivarCore& corb);
		void finalize(CorblivarCore& corb, bool const& determ_overall_cost = true, bool const& handle_corblivar = true);
};

#endif
