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
	struct timeb start, end;
	Corblivar_FP corb;

	cout << "Corblivar: Corner Block List for Varied [Block] Alignment Requests" << endl;
	cout << "----- 3D Floorplanning tool v0.1 ---------------------------------" << endl << endl;

	// memorize start time
	ftime(&start);

	// parse program parameter and config file
	corb.conf_log = Corblivar_FP::LOG_MINIMAL;
	IO::parseParameterConfig(corb, argc, argv);
	// open/generate results file
	corb.results.open(corb.results_file.c_str());
//	// parse blocks
//	IO::parseBlocks(corb, corb.blocks_file);
//	// parse nets
//	IO::parseNets(corb, corb.nets_file);

//	// perform test suites
//	if (corb.logMin()) {
//		cout << "Performing test suites..." << endl;
//	}
////	// test for TSV planning
////	corb.PG_TSVsRingTestsuite();
//	// tests for CG and block handling
//	corb.CGBlockTestsuite();
//	if (corb.logMin()) {
//		cout << "Done" << endl << endl;
//	}

	// TODO perform optimizations
	if (corb.logMin()) {
		cout << "Performing SA floorplanning optimization..." << endl << endl;
	}
	// TODO runtime of optimization steps
	// TODO bool parameter, flags for performing opt steps

	if (corb.logMin()) {
		cout << "Done, floorplanning was successful" << endl << endl;
	}

//	// generate final GP plots
//	IO::writeFloorplanGP(corb, "final");

	// determine total runtime
	ftime(&end);
	if (corb.logMin()) {
		cout << "Runtime: " << (1000.0 * (end.time - start.time) + (end.millitm - start.millitm)) / 1000.0 << " s" << endl << endl;
		corb.results << "Runtime: " << (1000.0 * (end.time - start.time) + (end.millitm - start.millitm)) / 1000.0 << " s" << endl << endl;
	}

	// close results file
	corb.results.close();
}

// definitions for const vars, to allocate memory
// initializations see Corblivar.hpp
const int Block::TYPE_BLOCK;
const int Block::TYPE_TSV;
const int Corblivar_FP::LOG_MINIMAL;
const int Corblivar_FP::LOG_MEDIUM;
const int Corblivar_FP::LOG_MAXIMUM;
const int Point::UNDEF;
const int Net::TYPE_INTRALAYER;
const int Net::TYPE_INTERLAYER;
