/*
 * =====================================================================================
 *
 *    Description:  Corblivar handler for timing, delay and power analysis
 *
 *    Copyright (C) 2015-2016 Johann Knechtel, johann aett jknechtel dot de
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
#ifndef _CORBLIVAR_TIMING_POWER
#define _CORBLIVAR_TIMING_POWER

// library includes
#include "Corblivar.incl.hpp"
// Corblivar includes, if any
#include "Block.hpp"

/// Corblivar handler for timing, delay and power analysis
class TimingPowerAnalyser {
	public:
		/// debugging code switch
		static constexpr bool DBG = false;
		static constexpr bool DBG_VERBOSE= false;

	// private constants
	private:
		/// TSV/wire resistivity/capacitance values, taken from [Ahmed14] which
		/// models 45nm technology; wires are on M7-M8 layers, TSV are assumed to
		/// have 5um diameter, 10um pitch, and 50um length
		///
		/// R_TSV [Ohm * 1e-3; mOhm]
		static constexpr double R_TSV =	42.8e-03;
		/// C_TSV [F * 1e-15; fF]
		static constexpr double C_TSV = 28.664e-15;
		/// R_wire [Ohm/um; mOhm/um]
		static constexpr double R_WIRE = 52.5e-03;
		/// C_wire [F/um; fF/um]
		static constexpr double C_WIRE = 0.823e-15;

		/// factor for modules' base delay [Lin10], [ns/um]; based on 90nm
		/// technology simulations, thus scaled down by factor 2 to roughly match
		/// 45nm technology; delay = factor times (width + height) for any module
		///
		static constexpr double DELAY_FACTOR_MODULE = (1.0/2000.0) / 2.0;
	
		/// delay factors for TSVs and wires, taken/resulting from [Ahmed14] which
		/// models 45nm technology
		///
		/// TSVs' delay, given in [ns]
		static constexpr double DELAY_FACTOR_TSV = 
			// R_TSV [mOhm] * C_TSV [fF]
			R_TSV * C_TSV
			// scale up to ns
			* 1.0e09;
		/// wire delay, given in [ns/um^2]
		static constexpr double DELAY_FACTOR_WIRE =
			// R_wire [mOhm/um] * C_wire [fF/um]
			R_WIRE * C_WIRE
			// scale up to ns
			* 1.0e09;

		/// activity factor, taken from [Ahmed14]
		static constexpr double ACTIVITY_FACTOR = 0.1;

	// public POD, to be declared early on
	public:

	// private data, functions
	private:

		/// inner class of nodes for DAG, to be declared early on
		class DAG_Node {

			// public data
			public:
				// IDs for special DAG nodes
				static constexpr const char* SOURCE_ID = "DAG_SOURCE";
				static constexpr const char* SINK_ID = "DAG_SINK";

				/// block represented by this node
				Block const* block;

				// parents and children nodes in the DAG; keep track of each child instance only once, i.e., we ignore all the multiples of nets connecting from the
				// same source to the same sink; this is valid for the DAG, as we only require it for timing, where the location of source/sink are evaluated, not
				// how many same-type connections pass between them
				//
				std::unordered_map<std::string, DAG_Node*> parents;
				std::unordered_map<std::string, DAG_Node*> children;

				// index for topological order, from global source to sink
				int index;

				// flag for DAG traversal
				bool visited = false;
				bool recursion = false;

				// timing values; AAT: actual arrival time, RAT: required arrival time
				mutable double AAT = 0;
				mutable double RAT = 0;
				mutable double slack = 0;

			/// default constructor
			DAG_Node(Block const* block, int index = -1) {
				this->block = block;
				this->index = index;
			};
		};

		/// data for DAG (directed acyclic graph) of nets
		/// key is id of blocks/pins represented by node
		std::unordered_map<std::string, DAG_Node> nets_DAG;
		/// wrapper for access of final DAG; sorted by topological indices
		std::vector<DAG_Node const*> nets_DAG_sorted;

		// init dummy blocks for special nodes
		Block dummy_block_DAG_source = Block(DAG_Node::SOURCE_ID);
		Block dummy_block_DAG_sink = Block(DAG_Node::SINK_ID);

	// constructors, destructors, if any non-implicit
	public:

	// public data, functions
	public:
		/// module h and w shall be given in um; returned delay is in ns
		inline static double baseDelay(double const& h, double const& w) {
			return TimingPowerAnalyser::DELAY_FACTOR_MODULE * (h + w);
		}

		/// WL shall be given in um; returned delay is in ns
		inline static double elmoreDelay(double const& WL, unsigned const& TSV) {
			return 0.5 * TimingPowerAnalyser::DELAY_FACTOR_WIRE * std::pow(WL, 2.0) + 0.5 * TimingPowerAnalyser::DELAY_FACTOR_TSV * std::pow(TSV, 2);
		}

		/// WL shall be given in um; returned power is in W
		inline static double powerWire(double const& WL, double const& driver_voltage, double const& frequency, double const& activity_factor = TimingPowerAnalyser::ACTIVITY_FACTOR) {

			// P_wire = a * C_wire * WL * V_driver^2 * f
			// a * [F/um * um * V^2 * Hz] = [W]
			//
			return activity_factor * C_WIRE * WL * std::pow(driver_voltage, 2.0) * frequency;
		}

		inline static double powerTSV(double const& driver_voltage, double const& frequency, double const& activity_factor = TimingPowerAnalyser::ACTIVITY_FACTOR) {
			return activity_factor * C_TSV * std::pow(driver_voltage, 2.0) * frequency;
		}

		/// helper to generate the DAG (directed acyclic graph) for the SL-STA
		void initSLSTA(std::vector<Block> const& blocks, std::vector<Pin> const& terminals, std::vector<Net> const& nets, bool const& log);

		/// determine timing values for DAG; will also update the slack for all blocks
		void updateTiming(double const& global_arrival_time);

	// private helper data, functions
	private:
		void determIndicesDAG(DAG_Node *cur_node);
		bool resolveCyclesDAG(DAG_Node *cur_node, bool const& log);
};

#endif
