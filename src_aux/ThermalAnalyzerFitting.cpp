/*
 * =====================================================================================
 *
 *    Description:  Entry (main) for thermal-analysis parameterization of Corblivar
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

// required Corblivar headers
#include "../src/CorblivarCore.hpp"
#include "../src/FloorPlanner.hpp"
#include "../src/IO.hpp"

int main (int argc, char** argv) {
	FloorPlanner fp;

	cout << endl;
	cout << "Corblivar: Corner Block List for Varied [Block] Alignment Requests" << endl;
	// (TODO) set version to 1.0 after release
	cout << "----- Thermal-analysis parameterization tool v0.1 ----------------" << endl << endl;

	// parse program parameter, config file, and further files
	IO::parseParametersFiles(fp, IO::Mode::THERMAL_ANALYSIS, argc, argv);
	// parse blocks
	IO::parseBlocks(fp);
	// parse nets
	IO::parseNets(fp);

	// init Corblivar core
	CorblivarCore corb = CorblivarCore(fp.getLayers(), fp.getBlocks().size());

	// init thermal analyzer, only reasonable after parsing config file
	fp.initThermalAnalyzer();

	// parse in solution file
	IO::parseCorblivarFile(fp, corb);

	// assume read in data as currently best solution
	corb.storeBestCBLs();

	// overall cost is not determined; cost cannot be determined since no
	// normalization during SA search was performed
	fp.finalize(corb, false);
}
