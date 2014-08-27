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
		static constexpr bool DBG_ALIGNMENT = false;
		static constexpr bool DBG_TSVS = false;
		static constexpr bool DBG_CLUSTERING = true;

	// private data, functions
	private:
		// chip data
		vector<Block> blocks;
		vector<Pin> terminals;
		vector<Net> nets;

		// groups of TSVs, will be defined from nets and vertical buses
		vector<TSV_Group> TSVs;

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

			// technology parameters
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
		} IC;

		// IO files and parameters
		struct IO_conf {
			string blocks_file, alignments_file, pins_file, power_density_file, nets_file, solution_file;
			ofstream results, solution_out;
			ifstream solution_in;
			// flag whether power density file is available / was handled /
			// thermal analysis should be performed / thermal files should be
			// generated
			bool power_density_file_avail;
			// similar flags for other files
			bool alignments_file_avail;
		} IO_conf;

		// benchmark name
		string benchmark;

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

		// SA parameters
		struct SA_parameters {

			// SA parameters: loop control
			double loopFactor, loopLimit;

			// SA parameters: optimization flags
			bool opt_thermal, opt_interconnects, opt_alignment;

			// SA parameters: cost factors
			double cost_thermal, cost_WL, cost_TSVs, cost_alignment;

			// SA parameter: scaling factor for initial temp
			double temp_init_factor;

			// SA parameters: temperature-scaling factors
			double temp_factor_phase1, temp_factor_phase1_limit, temp_factor_phase2, temp_factor_phase3;

			// SA parameters: layout generation options
			bool layout_enhanced_hard_block_rotation, layout_enhanced_soft_block_shaping;
			bool layout_power_aware_block_handling, layout_floorplacement, layout_signal_TSV_clustering;
			int layout_packing_iterations;
		} SA_parameters;

		// SA cost variables: max cost values
		double max_cost_thermal, max_cost_WL, max_cost_alignments;
		int max_cost_TSVs;

		// SA cost parameters: global weights, enforce that area and outline
		// violation is constantly considered; related weight should be >= 0.5 in
		// order to enforce guiding into outline during whole optimization run
		static constexpr double SA_COST_WEIGHT_AREA_OUTLINE = 0.5;
		static constexpr double SA_COST_WEIGHT_OTHERS = 1.0 - SA_COST_WEIGHT_AREA_OUTLINE;

		// SA cost; POD declaration
		struct Cost {
			double total_cost;
			double total_cost_fitting;
			double HPWL;
			double HPWL_actual_value;
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

			// http://www.learncpp.com/cpp-tutorial/93-overloading-the-io-operators/
			friend ostream& operator<< (ostream& out, Cost const& cost) {
				out << "cost=" << cost.total_cost << ", fits_fixed_outline=" << cost.fits_fixed_outline;
				return out;
			}
		};

		// SA: cost functions, i.e., layout-evaluations
		Cost evaluateLayout(vector<CorblivarAlignmentReq> const& alignments,
				double const& fitting_layouts_ratio = 0.0,
				bool const& SA_phase_two = false,
				bool const& set_max_cost = false,
				bool const& finalize = false);
		void evaluateThermalDistr(Cost& cost,
				bool const& set_max_cost = false);
		void evaluateAlignments(Cost& cost,
				vector<CorblivarAlignmentReq> const& alignments,
				bool const& derive_TSVs = true,
				bool const& set_max_cost = false);
		void evaluateAreaOutline(Cost& cost,
				double const& fitting_layouts_ratio = 0.0) const;
		void evaluateInterconnects(Cost& cost,
				bool const& set_max_cost = false);

		// SA: parameters for cost functions
		//
		// trivial HPWL refers to one global bounding box for each net;
		// non-trivial considers the bounding boxes on each layer separately
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

		// SA: temperature-schedule log data
		vector<TempStep> tempSchedule;

		// SA: reheating parameters, for SA phase 3
		static constexpr int SA_REHEAT_COST_SAMPLES = 3;
		static constexpr double SA_REHEAT_STD_DEV_COST_LIMIT = 1.0e-6;

		// signal-TSV clustering
		//
		// POD wrapping nets' segments per layer
		struct SegmentedNet {
			Net const& net;
			Rect bb;
		};
		// container for sorting nets' segments by their bb's area
		typedef multimap< double, SegmentedNet, greater<double> > nets_segments;
		// clustering handler, works on layer-wise vector of net's segments
		void clusterSignalTSVs(vector<FloorPlanner::nets_segments>& nets_seg);

		// layout-generation handler
		bool generateLayout(CorblivarCore& corb, bool const& perform_alignment);

		// layout operations op-codes
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

		// layout-operation handler variables
		int last_op, last_op_die1, last_op_die2, last_op_tuple1, last_op_tuple2, last_op_juncts;

		// layout-operation handler
		bool performRandomLayoutOp(CorblivarCore& corb, bool const& SA_phase_two = false, bool const& revertLastOp = false);
		// note that die and tuple parameters are return-by-reference; non-const
		// reference for CorblivarCore in order to enable operations on CBL-encode data
		inline bool prepareBlockSwappingFailedAlignment(CorblivarCore const& corb, int& die1, int& tuple1, int& die2, int& tuple2);
		inline bool performOpMoveOrSwapBlocks(int const& mode, bool const& revert, bool const& SA_phase_one, CorblivarCore& corb,
				int& die1, int& die2, int& tuple1, int& tuple2) const;
		inline bool performOpSwitchInsertionDirection(bool const& revert, CorblivarCore& corb, int& die1, int& tuple1) const;
		inline bool performOpSwitchTupleJunctions(bool const& revert, CorblivarCore& corb, int& die1, int& tuple1, int& juncts) const;
		inline bool performOpShapeBlock(bool const& revert, CorblivarCore& corb, int& die1, int& tuple1) const;
		inline bool performOpEnhancedHardBlockRotation(CorblivarCore const& corb, Block const* shape_block) const;
		inline bool performOpEnhancedSoftBlockShaping(CorblivarCore const& corb, Block const* shape_block) const;

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
		void initSA(CorblivarCore& corb, vector<double>& cost_samples, int& innerLoopMax, double& init_temp);
		inline void updateTemp(double& cur_temp, int const& iteration, int const& iteration_first_valid_layout) const;

		// thermal analyzer
		ThermalAnalyzer thermalAnalyzer;

		// thermal analyzer parameters; thermal mask parameters
		ThermalAnalyzer::MaskParameters power_blurring_parameters;

		// helper to memorize thermal-analysis result
		ThermalAnalyzer::ThermalAnalysisResult thermal_analysis;

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

			// init power maps, i.e. predetermine maps parameters
			this->thermalAnalyzer.initPowerMaps(this->IC.layers, this->getOutline());
		};

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
			return this->SA_parameters.layout_power_aware_block_handling;
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
			this->IC.outline_x = outline_x;
			this->IC.outline_y = outline_y;

			// this also requires to reset the power maps setting
			this->thermalAnalyzer.initPowerMaps(this->IC.layers, this->getOutline());

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
				pins_scale_x = max(pins_scale_x, pin.bb.ll.x);
				pins_scale_y = max(pins_scale_y, pin.bb.ll.y);
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
