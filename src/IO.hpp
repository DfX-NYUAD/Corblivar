/*
 * =====================================================================================
 *
 *    Description:  Corblivar IO handler
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
#ifndef _CORBLIVAR_IO
#define _CORBLIVAR_IO

// library includes
#include "Corblivar.incl.hpp"
// Corblivar includes, if any
// forward declarations, if any
class FloorPlanner;
class CorblivarCore;
class CorblivarAlignmentReq;

class IO {
	// debugging code switch (private)
	private:
		static constexpr bool DBG = false;

	// private data, functions
	private:
		static constexpr int CONFIG_VERSION = 12;

	// constructors, destructors, if any non-implicit
	// private in order to avoid instances of ``static'' class
	private:
		IO() {
		}

	// public data, functions
	public:
		enum class Mode : unsigned {REGULAR, THERMAL_ANALYSIS};

		static void parseParametersFiles(FloorPlanner& fp, Mode const& mode, int const& argc, char** argv);
		static void parseThermalMasksFile(FloorPlanner& fp);
		static void parseBlocks(FloorPlanner& fp);
		static void parseAlignmentRequests(FloorPlanner& fp, vector<CorblivarAlignmentReq>& alignments);
		static void parseNets(FloorPlanner& fp);
		static void parseCorblivarFile(FloorPlanner& fp, CorblivarCore& corb);
		static void writeFloorplanGP(FloorPlanner const& fp, vector<CorblivarAlignmentReq> const& alignment, string const& file_suffix = "");
		static void writeHotSpotFiles(FloorPlanner const& fp);
		// non-const reference due to map acces via []
		static void writePowerThermalMaps(FloorPlanner& fp);
		static void writeTempSchedule(FloorPlanner const& fp);
};

#endif
