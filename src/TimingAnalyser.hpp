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
	public:
		// factor beta for base delay [Lin10], [ns/um], supposedly based on 90nm
		// technology simulations; base delay = beta times (width + height) for
		// any module
		//
		static constexpr double beta = 1.0/2000;

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
			return TimingAnalyser::beta * (h + w);
		}

	// private helper data, functions
	private:
};

#endif
