/*
 * =====================================================================================
 *
 *    Description:  Entry (main) for Corblivar
 *
 *         Author:  Johann Knechtel, johann.knechtel@ifte.de
 *        Company:  Institute of Electromechanical and Electronic Design, www.ifte.de
 *
 * =====================================================================================
 */

// own Corblivar header
#include "CorblivarCore.hpp"
// required Corblivar headers
#include "FloorPlanner.hpp"
#include "IO.hpp"

int main (int argc, char** argv) {
	FloorPlanner fp;
	bool done;

	// memorize start time
	fp.setTimeStart();

	cout << endl;
	cout << "Corblivar: Corner Block List for Varied [Block] Alignment Requests" << endl;
	// (TODO) set version to 1.0 after release
	cout << "----- 3D Floorplanning tool v0.1 ---------------------------------" << endl << endl;

	// init random number gen
	srand(time(0));

	// parse program parameter and config file
	IO::parseParameterConfig(fp, argc, argv);
	// parse blocks
	IO::parseBlocks(fp);
	// parse nets
	IO::parseNets(fp);

	// init Corblivar core
	CorblivarCore corb = CorblivarCore(fp.getLayers(), fp.getBlocks().size());

	/// init Corblivar layout data
	if (fp.inputSolutionFileOpen()) {
		// read from file
		IO::parseCorblivarFile(fp, corb);
		// assume read in data as currently best solution
		corb.storeBestCBLs();
	}
	// generate new, random data set
	else {
		corb.initCorblivarRandomly(fp.logMed(), fp.getLayers(), fp.getBlocks());
	}

	// init thermal analyzer, only reasonable after parsing config file
	fp.initThermalAnalyzer();

	// non-regular run; read in solution file
	// (TODO) adapt if further optimization of read in data is desired
	if (fp.inputSolutionFileOpen()) {
		// overall cost is not determined; cost cannot be determined since no
		// normalization during SA search was performed
		fp.finalize(corb, false);
	}
	// regular run; perform floorplanning
	else {

		if (fp.logMin()) {
			cout << "Corblivar> ";
			cout << "Performing SA floorplanning optimization..." << endl << endl;
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
