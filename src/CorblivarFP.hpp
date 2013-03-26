/*
 * =====================================================================================
 *
 *    Description:  Corblivar floorplanning header (SA operations and related handler)
 *
 *         Author:  Johann Knechtel, johann.knechtel@ifte.de
 *        Company:  Institute of Electromechanical and Electronic Design, www.ifte.de
 *
 * =====================================================================================
 */
#ifndef _CORBLIVAR_FP_HPP
#define _CORBLIVAR_FP_HPP

#include "ThermalAnalyzer.hpp"

class FloorPlanner {
	private:
		// debugging code switches
		static constexpr bool DBG_SA = false;
		static constexpr bool DBG_CALLS_SA = false;
		static constexpr bool DBG_LAYOUT = false;

		// PODs
		struct Cost {
			double cost;
			bool fits_fixed_outline;

			// http://www.learncpp.com/cpp-tutorial/93-overloading-the-io-operators/
			friend ostream& operator<< (ostream& out, const Cost& cost) {
				out << "cost=" << cost.cost << ", fits_fixed_outline=" << cost.fits_fixed_outline;
				return out;
			}
		};
		struct CostInterconn {
			double HPWL;
			double TSVs;

			friend ostream& operator<< (ostream& out, const CostInterconn& cost) {
				out << "HPWL=" << cost.HPWL << ", TSVs=" << cost.TSVs;
				return out;
			}
		};
		struct OpsRatios {
			double cur_ratio;
			double boundary_1;
			double boundary_2;
		};

		// IO
		string benchmark, blocks_file, power_file, nets_file;
		ofstream results, solution_out;

		// IO: scaling factor for block dimensions
		static constexpr int BLOCKS_SCALE_UP = 50;

		// thermal analyzer
		ThermalAnalyzer thermalAnalyzer;

		// SA config parameters
		double conf_SA_loopFactor, conf_SA_loopLimit;
		double conf_SA_cost_temp, conf_SA_cost_WL, conf_SA_cost_TSVs, conf_SA_cost_area_outline;

		// SA parameters: temperature-scaling factors
		double conf_SA_temp_factor_phase1, conf_SA_temp_factor_phase2, conf_SA_temp_factor_phase3;
		// SA parameters: temperature-phase-transition factos
		double conf_SA_temp_phase_trans_12_factor, conf_SA_temp_phase_trans_23_factor;

		// SA parameter: scaling factor for loops during solution-space sampling
		static constexpr int SA_SAMPLING_LOOP_FACTOR = 2;

		// SA parameter: scaling factor for initial temp
		static constexpr double SA_INIT_TEMP_FACTOR = 0.01;

		// SA: layout-operation handler variables
		mutable int last_op, last_op_die1, last_op_die2, last_op_tuple1, last_op_tuple2, last_op_juncts;

		// SA: layout-operation handler
		bool performRandomLayoutOp(const CorblivarCore& corb, const bool& revertLastOp = false) const;

		// SA: cost functions, i.e., layout-evalutions
		Cost determCost(const double& ratio_feasible_solutions_fixed_outline = 0.0, const bool& phase_two = false, const bool& set_max_cost = false) const;
		inline double determCostThermalDistr(const bool& set_max_cost = false, const bool& normalize = true) const {
			this->thermalAnalyzer.generatePowerMaps(this->conf_layer, this->blocks);
			return this->thermalAnalyzer.performPowerBlurring(this->conf_layer, this->max_cost_temp, set_max_cost, normalize);
		};
		Cost determCostAreaOutline(const double& ratio_feasible_solutions_fixed_outline = 0.0) const;
		CostInterconn determCostInterconnects(const bool& set_max_cost = false, const bool& normalize = true) const;

		// SA parameters: max cost values
		mutable double max_cost_temp, max_cost_WL, max_cost_TSVs, max_cost_alignments;

		// SA: helper for main handler
		void initSA(const CorblivarCore& corb, vector<double>& cost_samples, int& innerLoopMax, double& init_temp, OpsRatios& ratios);
		inline void updateTemp(double& cur_temp, const OpsRatios& accepted_ops_ratio, const int& iteration, const bool& valid_layout_found);

	public:
		friend class IO;

		// chip data
		map<int, Block*> blocks;
		vector<Net*> nets;

		// IO
		struct timeb start;
		ifstream solution_in;

		// config parameters
		int conf_layer;
		int conf_log;
		double conf_outline_x, conf_outline_y, outline_AR;

		// logging
		static constexpr int LOG_MINIMAL = 1;
		static constexpr int LOG_MEDIUM = 2;
		static constexpr int LOG_MAXIMUM = 3;
		inline bool logMin() const {
			return (this->conf_log >= LOG_MINIMAL);
		};
		inline bool logMed() const {
			return (this->conf_log >= LOG_MEDIUM);
		};
		inline bool logMax() const {
			return (this->conf_log >= LOG_MAXIMUM);
		};

		// SA: handler
		inline void initThermalAnalyzer() {
			// init thermal masks
			this->thermalAnalyzer.initThermalMasks(this->conf_layer, this->logMed());
			// init power maps, i.e. predetermine maps parameters
			this->thermalAnalyzer.initPowerMaps(this->conf_layer, this->conf_outline_x, this->conf_outline_y);
		};
		bool performSA(const CorblivarCore& corb);
		void finalize(const CorblivarCore& corb);
};

#endif
