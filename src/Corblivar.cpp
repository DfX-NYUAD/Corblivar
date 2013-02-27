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

int main (int argc, char** argv) {
	CorblivarFP corb;
	CorblivarLayoutRep chip;
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

	// init Corblivar layout representation
	chip.initCorblivar(corb);

	// init thermal masks for thermal modelling based on power blurring
	corb.initThermalMasks();

	if (corb.logMin()) {
		cout << "Corblivar> ";
		cout << "Performing SA floorplanning optimization..." << endl << endl;
	}

	// perform SA; main handler
	done = corb.SA(chip);

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
	corb.finalize();
}

// definitions for const vars, to allocate memory
// initializations see Corblivar.hpp
const int CorblivarFP::LOG_MINIMAL;
const int CorblivarFP::LOG_MEDIUM;
const int CorblivarFP::LOG_MAXIMUM;
const int Point::UNDEF;
// material parameters for HotSpot thermal 3D-IC simulation
const double IO::HEAT_CAPACITY_SI = 1750000.0;
const double IO::THERMAL_RESISTIVITY_SI = 0.01;
const double IO::THICKNESS_SI = 0.00005;
const double IO::HEAT_CAPACITY_BEOL = 1750000.0;
const double IO::THERMAL_RESISTIVITY_BEOL = 0.05;
const double IO::THICKNESS_BEOL = 0.00001;
