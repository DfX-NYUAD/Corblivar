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

		// PODs for cost functions
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

		// IO
		string benchmark, blocks_file, power_file, nets_file;
		ofstream results, solution_out;

		// IO: scaling factor for block dimensions
		static const int BLOCKS_SCALE_UP = 50;

		// SA config parameters
		double conf_SA_loopFactor, conf_SA_loopLimit;
		double conf_SA_cost_temp, conf_SA_cost_WL, conf_SA_cost_TSVs, conf_SA_cost_area_outline;

		// SA parameters: temperature-scaling factors
		double conf_SA_temp_factor_phase1, conf_SA_temp_factor_phase2, conf_SA_temp_factor_phase3;
		// SA parameters: temperature-phase-transition factos
		double conf_SA_temp_phase_trans_12_factor, conf_SA_temp_phase_trans_23_factor;

		// SA parameter: scaling factor for loops during solution-space sampling
		static const int SA_SAMPLING_LOOP_FACTOR = 2;

		// SA paramter: scaling factor for initial temp
		static constexpr double SA_INIT_TEMP_FACTOR = 0.01;

		// SA: layout-operation handler variables
		int last_op, last_op_die1, last_op_die2, last_op_tuple1, last_op_tuple2, last_op_juncts;

		// SA: layout-operation handler
		bool performRandomLayoutOp(const CorblivarCore& corb, const bool& revertLastOp = false);

		// SA: cost functions, i.e., layout-evalutions
		Cost determCost(
				const double& ratio_feasible_solutions_fixed_outline = 0.0,
				const bool& phase_two = false,
				const bool& set_max_cost = false
				) const;
		inline double determCostThermalDistr(const bool& set_max_cost = false, const bool& normalize = true) const {
			this->thermalAnalyzer.generatePowerMaps(*this);
			return this->thermalAnalyzer.performPowerBlurring(*this, set_max_cost, normalize);
		};
		Cost determCostAreaOutline(const double& ratio_feasible_solutions_fixed_outline = 0.0) const;
		CostInterconn determCostInterconnects(const bool& set_max_cost = false, const bool& normalize = true) const;

	public:
		friend class IO;

		// chip data
		map<int, Block*> blocks;
		vector<Net*> nets;

		// thermal analyzer
		ThermalAnalyzer thermalAnalyzer;

		// IO
		struct timeb start;
		ifstream solution_in;

		// config parameters
		int conf_layer;
		int conf_log;
		double conf_outline_x, conf_outline_y, outline_AR;

		// logging
		static const int LOG_MINIMAL = 1;
		static const int LOG_MEDIUM = 2;
		static const int LOG_MAXIMUM = 3;
		inline bool logMin() const {
			return (this->conf_log >= LOG_MINIMAL);
		};
		inline bool logMed() const {
			return (this->conf_log >= LOG_MEDIUM);
		};
		inline bool logMax() const {
			return (this->conf_log >= LOG_MAXIMUM);
		};

		// SA parameters: max cost values
		mutable double max_cost_temp, max_cost_WL, max_cost_TSVs, max_cost_alignments;

		// SA: floorplanning handler
		bool performSA(const CorblivarCore& corb);
		void finalize(const CorblivarCore& corb);
};

#endif
