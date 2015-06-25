/*
 * =====================================================================================
 *
 *    Description:  Corblivar handler for timing and delay analysis
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
#ifndef _CORBLIVAR_TIMING
#define _CORBLIVAR_TIMING

// library includes
#include "Corblivar.incl.hpp"
// Corblivar includes, if any
// forward declarations, if any

class TimingAnalyser {
	// debugging code switch (private)
	private:
		static constexpr bool DBG = false;

	// private constants
	private:
		// factor for modules' base delay [Lin10], [ns/um]; based on 90nm
		// technology simulations, thus scaled down by factor 10 to roughly match
		// 45nm technology; delay = factor times (width + height) for any module
		//
		static constexpr double DELAY_FACTOR_MODULE = 1.0/2000.0 / 10.0;
	
		// delay factors for TSVs and wires, taken from [Ahmed14]; wire delay
		// applies for 0.14um width and 0.28um thickness, that is for 45nm
		// technology TSV delays applies for 5um diameter, 10um pitch, and 50um
		// length TSVs
		//
		// given in [ns]
		static constexpr double DELAY_FACTOR_TSV = 
			// R_TSV [mOhm] * C_TSV [fF]
			42.8e-03 * 28.664e-15
			// scale up to ns
			* 1.0e09;
		// given in [ns/um^2]
		static constexpr double DELAY_FACTOR_WIRE =
			// R_wire [mOhm/um] * C_wire [fF/um]
			428.0e-03 * 0.171e-15
			// scale up to ns
			* 1.0e09;

	// public POD, to be declared early on
	public:

	// private data, functions
	private:

	// constructors, destructors, if any non-implicit
	public:

	// public data, functions
	public:
		// module h and w shall be given in um; returned delay is in ns; note that
		// dimensions are scaled down according to block scaling factor
		inline static double BaseDelay(double h, double w, double block_scaling_factor) {
			return TimingAnalyser::DELAY_FACTOR_MODULE * (h + w) / block_scaling_factor;
		}

		// WL shall be given in um; returned delay is in ns; note that WL is
		// scaled down according to the block scaling factor
		inline static double ElmoreDelay(double WL, unsigned TSV, double block_scaling_factor) {
			return 0.5 * TimingAnalyser::DELAY_FACTOR_WIRE * std::pow(WL / block_scaling_factor, 2.0) + 0.5 * TimingAnalyser::DELAY_FACTOR_TSV * std::pow(TSV, 2);
		}

	// private helper data, functions
	private:
};

#endif
