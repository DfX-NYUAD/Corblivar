/*
 * =====================================================================================
 *
 *    Description:  Main file for Corblivar (corner block list for varied [block] alignment requests)
 *
 *         Author:  Johann Knechtel, johann.knechtel@ifte.de
 *        Company:  Institute of Electromechanical and Electronic Design, www.ifte.de
 *
 * =====================================================================================
 */
#include "Corblivar.hpp"

// definitions for const vars, to allocate memory
// initializations see Corblivar.hpp
const int CorblivarFP::LOG_MINIMAL;
const int CorblivarFP::LOG_MEDIUM;
const int CorblivarFP::LOG_MAXIMUM;
const int Point::UNDEF;

int main (int argc, char** argv) {
	CorblivarFP corb;
	CorblivarLayoutRep chip;
	ThermalAnalyzer therm;
	bool done;

	// memorize start time
	ftime(&corb.start);

	cout << "Corblivar: Corner Block List for Varied [Block] Alignment Requests" << endl;
	cout << "----- 3D Floorplanning tool v0.1 ---------------------------------" << endl << endl;

	// init random number gen
	srand(time(0));

	// parse program parameter and config file
	corb.conf_log = CorblivarFP::LOG_MINIMAL;
	IO::parseParameterConfig(corb, argc, argv);
	// parse blocks
	IO::parseBlocks(corb);
	// parse nets
	IO::parseNets(corb);

	/// init Corblivar layout representation
	if (corb.solution_in.is_open()) {
		// read from file
		IO::parseCorblivarFile(corb, chip);
		// assume read in data as currently best solution
		chip.storeBestCBLs();
	}
	// generate new, random data set
	else {
		chip.initCorblivar(corb);
	}

	// init thermal masks
	corb.thermalAnalyzer.initThermalMasks(corb);

	// (TODO) drop if further optimization of read in data is desired
	if (corb.solution_in.is_open()) {
		corb.finalize(chip);
	}

	if (corb.logMin()) {
		cout << "Corblivar> ";
		cout << "Performing SA floorplanning optimization..." << endl << endl;
	}

	// perform SA; main handler
	done = corb.performSA(chip);

	if (corb.logMin()) {
		cout << "Corblivar> ";
		if (done) {
			cout << "Done, floorplanning was successful" << endl << endl;
		}
		else {
			cout << "Done, floorplanning was _not_ successful" << endl << endl;
		}
	}

	// finalize: generate output files, final logging
	corb.finalize(chip);
}
