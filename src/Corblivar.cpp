/*
 * =====================================================================================
 *
 *    Description:  Main file for Corblivar
 *    			(Corner Block List for Varied [block] Alignment Requests)
 *
 *         Author:  Johann Knechtel, johann.knechtel@ifte.de
 *        Company:  Institute of Electromechanical and Electronic Design, www.ifte.de
 *
 * =====================================================================================
 */
#include "Corblivar.hpp"
#include "CorblivarFP.hpp"
#include "CorblivarCore.hpp"
#include "IO.hpp"

int main (int argc, char** argv) {
	FloorPlanner fp;
	CorblivarCore corb;
	ThermalAnalyzer therm;
	bool done;

	// memorize start time
	ftime(&fp.start);

	cout << endl;
	cout << "Corblivar: Corner Block List for Varied [Block] Alignment Requests" << endl;
	cout << "----- 3D Floorplanning tool v0.1 ---------------------------------" << endl << endl;

	// init random number gen
	srand(time(0));

	// parse program parameter and config file
	fp.conf_log = FloorPlanner::LOG_MINIMAL;
	IO::parseParameterConfig(fp, argc, argv);
	// parse blocks
	IO::parseBlocks(fp);
	// parse nets
	IO::parseNets(fp);

	/// init Corblivar layout representation
	if (fp.solution_in.is_open()) {
		// read from file
		IO::parseCorblivarFile(fp, corb);
		// assume read in data as currently best solution
		corb.storeBestCBLs();
	}
	// generate new, random data set
	else {
		corb.initCorblivarRandomly(fp.logMed(), fp.conf_layer, fp.blocks);
	}

	// init thermal masks
	fp.thermalAnalyzer.initThermalMasks(fp.conf_layer, fp.logMed());
	// init power maps, i.e. predetermine maps parameters
	fp.thermalAnalyzer.initPowerMaps(fp.conf_layer, fp.conf_outline_x, fp.conf_outline_y);

	// (TODO) drop if further optimization of read in data is desired
	if (fp.solution_in.is_open()) {
		fp.finalize(corb);
	}

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
