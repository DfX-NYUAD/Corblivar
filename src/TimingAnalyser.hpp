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
		// factor for modules' base delay [Lin10], [ns/um], supposedly based on
		// 90nm technology simulations; delay = factor times (width + height) for
		// any module
		//
		static constexpr double DELAY_FACTOR_MODULE = 1.0/2000;
	
		// delay factors for TSVs and wires, taken from [Lin10] and [Ahmed14];
		// [Lin10] is for 90nm technology, with 0.9um metal thickness and 1um
		// metal width; [Ahmed14] TSV delays applies for 5um diameter, 10um pitch,
		// and 50um length TSVs
		//
		// [Ahmed14]; given in [ns]
		static constexpr double DELAY_FACTOR_TSV = 
			// R_TSV [mOhm] * C_TSV [fF]
			42.8e-03 * 28.664e-15
			// scale up to ns
			* 1.0e09;
		// [Lin10]; given in [ns/um], i.e., no squared WL is required
		static constexpr double DELAY_FACTOR_WIRE = 1.27e-08;

	// public POD, to be declared early on
	public:

	// private data, functions
	private:

	// constructors, destructors, if any non-implicit
	public:

	// public data, functions
	public:
		// h and w shall be given in um; returned delay is in ns
		inline static double BaseDelay(double h, double w) {
			return TimingAnalyser::DELAY_FACTOR_MODULE * (h + w);
		}

	// private helper data, functions
	private:
};

#endif
