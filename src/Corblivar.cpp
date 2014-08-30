/*
 * =====================================================================================
 *
 *    Description:  Entry (main) for Corblivar
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
#include "CorblivarCore.hpp"
#include "FloorPlanner.hpp"
#include "IO.hpp"

int main (int argc, char** argv) {
	FloorPlanner fp;
	bool done;

	cout << endl;
	cout << "Corblivar: Corner Block List for Varied [Block] Alignment Requests" << endl;
	cout << "----- 3D floorplanning tool v 1.1.1 ------------------------------" << endl << endl;

	// parse program parameter, config file, and further files
	IO::parseParametersFiles(fp, argc, argv);
	// parse blocks
	IO::parseBlocks(fp);
	// parse nets
	IO::parseNets(fp);

	// init Corblivar core
	CorblivarCore corb = CorblivarCore(fp.getLayers(), fp.getBlocks().size());

	// parse alignment request
	IO::parseAlignmentRequests(fp, corb.editAlignments());

	// init thermal analyzer, only reasonable after parsing config file
	fp.initThermalAnalyzer();

	// non-regular run; read in solution file
	if (fp.inputSolutionFileOpen()) {

		if (fp.logMin()) {
			cout << "Corblivar> ";
			cout << "Handling given solution file ..." << endl << endl;
		}

		// read from file
		IO::parseCorblivarFile(fp, corb);

		// assume read in data as currently best solution
		corb.storeBestCBLs();

		// overall cost is not determined; cost cannot be determined since no
		// normalization during SA search was performed
		fp.finalize(corb, false);
	}
	// regular run; perform floorplanning
	else {
		// generate new, random data set
		corb.initCorblivarRandomly(fp.logMed(), fp.getLayers(), fp.getBlocks(), fp.powerAwareBlockHandling());

		if (fp.logMin()) {
			cout << "Corblivar> ";
			cout << "Performing SA floorplanning optimization ..." << endl << endl;
		}

		// perform SA; main handler
		done = fp.performSA(corb);

		if (fp.logMin()) {
			cout << "Corblivar> ";
			if (done) {
				cout << "Done, floorplanning was successful" << endl << endl;
			}
			else {
				cout << "Done, floorplanning was _not_ successful" << endl << endl;
			}
		}

		// finalize: generate output files, final logging
		fp.finalize(corb);
	}
}
