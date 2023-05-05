/**
 * =====================================================================================
 *
 *    Description:  Corblivar IO handler
 *
 *    Copyright (C) 2013-2016 Johann Knechtel, johann aett jknechtel dot de
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
#include <boost/polygon/polygon.hpp>
// Corblivar includes, if any
// forward declarations, if any
class FloorPlanner;
class CorblivarCore;
class CorblivarAlignmentReq;

// boost namespaces
//
namespace bp = boost::polygon;
using namespace bp::operators;
typedef bp::rectangle_data<double> BoostRect;
typedef bp::polygon_90_set_data<double> BoostPolygonSet;

/// Corblivar IO handler
class IO {
	private:
		/// debugging code switch (private)
		static constexpr bool DBG = false;

	// private data, functions
	private:
		static constexpr int CONFIG_VERSION = 24;
		static constexpr int TECHNOLOGY_VERSION = 9;

	// constructors, destructors, if any non-implicit
	private:
		/// empty default constructor; private in order to avoid instances of ``static'' class
		IO() {
		}

	// public data, functions
	public:
		enum MAPS_FLAGS : int {POWER = 0, THERMAL = 1, THERMAL_HOTSPOT = 2, TSV_DENSITY = 3, POWER_ORIG = 4, ROUTING = 5};

		static void parseParametersFiles(FloorPlanner& fp, int const& argc, char** argv);
		static void parseBlocks(FloorPlanner& fp);
		static void parseAlignmentRequests(FloorPlanner& fp, std::vector<CorblivarAlignmentReq>& alignments);
		static void parseNets(FloorPlanner& fp);
		static void parseCorblivarFile(FloorPlanner& fp, CorblivarCore& corb);
		static void writeFloorplanGP(FloorPlanner const& fp, std::vector<CorblivarAlignmentReq> const& alignment, std::string const& benchmark_suffix = "");
		static void writeHotSpotFiles(FloorPlanner const& fp, std::string const& benchmark_suffix = "");
		/// non-const reference due to map acces via []
		static void writeMaps(FloorPlanner& fp, int const& flag_parameter = -1, std::string const& benchmark_suffix = "");
		static void writeTempSchedule(FloorPlanner const& fp);
};

#endif
