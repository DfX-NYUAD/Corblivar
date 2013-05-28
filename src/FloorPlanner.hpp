/*
 * =====================================================================================
 *
 *    Description:  Corblivar floorplanner (SA operations and related handler)
 *
 *         Author:  Johann Knechtel, johann.knechtel@ifte.de
 *        Company:  Institute of Electromechanical and Electronic Design, www.ifte.de
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

		// 3D IC die layer parameters
		// 100um thick dies; own value
		static constexpr double THICKNESS_SI = 100.0e-6;
		// 2um active Si layer; [Sridhar10]
		static constexpr double THICKNESS_SI_ACTIVE = 2.0e-06;
		// passive Si layer, results in 98um
		static constexpr double THICKNESS_SI_PASSIVE = THICKNESS_SI - THICKNESS_SI_ACTIVE;
		// 12um BEOL; [Sridhar10]
		static constexpr double THICKNESS_BEOL = 12.0e-06;
		// 20um BCB bond; [Sridhar10]
		static constexpr double THICKNESS_BOND = 20.0e-06;
		// TSV properties; own values
		// 5um dimension
		static constexpr double TSV_DIMENSION = 5.0e-06;
		// 10um pitch
		static constexpr double TSV_PITCH = 10.0e-06;

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
			double TSVs;
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
		};

		// IO
		string benchmark, blocks_file, pins_file, power_density_file, nets_file;
		ofstream results, solution_out;
		ifstream solution_in;
		struct timeb start;
		// flag whether power density file is available / was handled / thermal
		// analysis should be performed / thermal files should be generated
		bool power_density_file_avail;

		// logging
		int conf_log;
		static constexpr int LOG_MINIMAL = 1;
		static constexpr int LOG_MEDIUM = 2;
		static constexpr int LOG_MAXIMUM = 3;

		// SA parameters: loop control
		double conf_SA_loopFactor, conf_SA_loopLimit;

		// SA parameters: optimization flags
		bool conf_SA_opt_thermal, conf_SA_opt_interconnects;

		// SA parameters: cost factors
		double conf_SA_cost_thermal, conf_SA_cost_WL, conf_SA_cost_TSVs;

		// SA cost variables: max cost values
		double max_cost_thermal, max_cost_WL, max_cost_TSVs, max_cost_alignments;

		// SA cost paramters: global weights, enforce that area and outline
		// violation is constantly considered; related weight should be >= 0.5 in
		// order to enforce guiding into outline during whole optimization run
		static constexpr double SA_COST_WEIGHT_AREA_OUTLINE = 0.5;
		static constexpr double SA_COST_WEIGHT_OTHERS = 1.0 - SA_COST_WEIGHT_AREA_OUTLINE;

		// SA: cost functions, i.e., layout-evalutions
		Cost determCost(double const& ratio_feasible_solutions_fixed_outline = 0.0, bool const& SA_phase_two = false, bool const& set_max_cost = false);
		inline double determCostThermalDistr(bool const& set_max_cost = false, bool const& normalize = true, bool const& return_max_temp = false) {
			this->thermalAnalyzer.generatePowerMaps(this->conf_layer, this->blocks, this->getOutline(),
					this->conf_power_blurring_power_density_scaling_padding_zone);
			return this->thermalAnalyzer.performPowerBlurring(this->conf_layer, this->conf_power_blurring_temp_offset,
					this->max_cost_thermal, set_max_cost, normalize, return_max_temp);
		};
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
		bool inline performOpMoveOrSwapBlocks(int const& mode, bool const& revert, bool const& SA_phase_one, CorblivarCore& corb,
				int& die1, int& die2, int& tuple1, int& tuple2) const;
		bool inline performOpSwitchInsertionDirection(bool const& revert, CorblivarCore& corb, int& die1, int& tuple1) const;
		bool inline performOpSwitchTupleJunctions(bool const& revert, CorblivarCore& corb, int& die1, int& tuple1, int& juncts) const;
		bool inline performOpShapeBlock(bool const& revert, CorblivarCore& corb, int& die1, int& tuple1) const;
		bool inline performOpEnhancedHardBlockRotation(Block const* shape_block) const;
		bool inline performOpEnhancedSoftBlockShaping(Block const* shape_block) const;

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

		// thermal analyzer parameters: thermal mask fitting
		double conf_power_blurring_impulse_factor, conf_power_blurring_impulse_factor_scaling_exponent, conf_power_blurring_mask_boundary_value;
		double conf_power_blurring_temp_offset;

		// thermal analyzer parameters: power masks
		double conf_power_blurring_power_density_scaling_padding_zone;

		// IO helper: determines the achievable density of TSVs given their
		// dimensions and pitch, assuming a close packing
		double inline dummyTVSsDensity() const {
			double TSVs;

			// in the range TSV_DIMENSION + TSV_PITCH, we could fit two TSVs
			TSVs = (2.0 * this->conf_outline_x / (FloorPlanner::TSV_DIMENSION + FloorPlanner::TSV_PITCH))
				* (2.0 * this->conf_outline_y /(FloorPlanner::TSV_DIMENSION + FloorPlanner::TSV_PITCH));

			return 
				// area occupied by TSVs
				TSVs * pow(FloorPlanner::TSV_DIMENSION, 2)
				// normalize to overall die area; i.e. TSV density
				/ (this->conf_outline_x * this->conf_outline_y);
		};

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
			// init masks parameters
			ThermalAnalyzer::MaskParameters parameters;
			parameters.impulse_factor = this->conf_power_blurring_impulse_factor;
			parameters.impulse_factor_scaling_exponent = this->conf_power_blurring_impulse_factor_scaling_exponent;
			parameters.mask_boundary_value = this->conf_power_blurring_mask_boundary_value;

			// init thermal masks
			this->thermalAnalyzer.initThermalMasks(this->conf_layer, this->logMed(), parameters);

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

		inline void resetDieProperties(double const& outline_x, double const& outline_y) {
			// set outline
			this->conf_outline_x = outline_x;
			this->conf_outline_y = outline_y;

			// also reset related properties
			this->die_AR = this->conf_outline_x / this->conf_outline_y;
			this->die_area = this->conf_outline_x * this->conf_outline_y;
			this->stack_area = this->conf_layer * this->die_area;
			this->stack_deadspace = this->stack_area - this->blocks_area;
		}

		inline bool inputSolutionFileOpen() const {
			return this->solution_in.is_open();
		};

		// SA: handler
		bool performSA(CorblivarCore& corb);
		void finalize(CorblivarCore& corb, bool const& determ_overall_cost = true, bool const& handle_corblivar = true);
};

#endif
