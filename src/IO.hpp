/*
 * =====================================================================================
 *
 *    Description:  Corblivar IO handler
 *
 *
 *         Author:  Johann Knechtel, johann.knechtel@ifte.de
 *        Company:  Institute of Electromechanical and Electronic Design, www.ifte.de
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
		static constexpr int CONFIG_VERSION = 10;

	// constructors, destructors, if any non-implicit
	// private in order to avoid instances of ``static'' class
	private:
		IO() {
		}

	// public data, functions
	public:
		static void parseParametersFiles(FloorPlanner& fp, int const& argc, char** argv);
		static void parseBlocks(FloorPlanner& fp);
		static void parseAlignmentRequests(FloorPlanner& fp, vector<CorblivarAlignmentReq>& alignments);
		static void parseNets(FloorPlanner& fp);
		static void parseCorblivarFile(FloorPlanner& fp, CorblivarCore& corb);
		static void writeFloorplanGP(FloorPlanner const& fp, vector<CorblivarAlignmentReq> const& alignment, string const& file_suffix = "");
		static void writeHotSpotFiles(FloorPlanner const& fp);
		static void writePowerThermalMaps(FloorPlanner const& fp);
		static void writeTempSchedule(FloorPlanner const& fp);
};

#endif
