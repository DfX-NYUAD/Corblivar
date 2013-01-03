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
	CorblivarFP corb;
	CorblivarLayoutRep chip;
	bool done;

	// memorize start time
	ftime(&start);

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

	if (corb.logMin()) {
		cout << "Corblivar> ";
		cout << "Performing SA floorplanning optimization..." << endl << endl;
	}

	// TODO runtime of optimization steps
	// TODO bool parameter, flags for performing opt steps
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

	// generate final GP plots
	IO::writeFloorplanGP(corb);

	// determine total runtime
	ftime(&end);
	if (corb.logMin()) {
		cout << "Corblivar> ";
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
const int CorblivarFP::LOG_MINIMAL;
const int CorblivarFP::LOG_MEDIUM;
const int CorblivarFP::LOG_MAXIMUM;
const int CorblivarFP::SA_INIT_T_FACTOR;
const int Point::UNDEF;
// double initialization w/in class defintion is not standard, thus put here
// var factors must sum up to approx. 1 !
const double CorblivarFP::COST_FACTOR_TEMP = 1/8;
const double CorblivarFP::COST_FACTOR_IR = 1/8;
const double CorblivarFP::COST_FACTOR_WL = 1/8;
const double CorblivarFP::COST_FACTOR_TSVS = 1/8;
const double CorblivarFP::COST_FACTOR_AREA = 1/8;
const double CorblivarFP::COST_FACTOR_OUTLINE_X = 1/8;
const double CorblivarFP::COST_FACTOR_OUTLINE_Y = 1/8;
const double CorblivarFP::COST_FACTOR_ALIGNMENTS = 1/8;
