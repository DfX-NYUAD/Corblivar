/*
 * =====================================================================================
 *
 *    Description:  Header for Corblivar IO handler
 *
 *
 *         Author:  Johann Knechtel, johann.knechtel@ifte.de
 *        Company:  Institute of Electromechanical and Electronic Design, www.ifte.de
 *
 * =====================================================================================
 */
#ifndef _CORBLIVAR_IO_HPP
#define _CORBLIVAR_IO_HPP

// debugging code switch
//#define DBG_IO

class IO {
	public:
		static void parseParameterConfig(FloorPlanner& fp, const int& argc, char** argv);
		static void parseBlocks(FloorPlanner& fp);
		static void parseNets(FloorPlanner& fp);
		static void parseCorblivarFile(FloorPlanner& fp, CorblivarCore& corb);
		static void writeFloorplanGP(const FloorPlanner& fp, const string& file_suffix = "");
		static void writeHotSpotFiles(const FloorPlanner& fp);
		static void writePowerThermalMaps(const FloorPlanner& fp);
};

#endif
