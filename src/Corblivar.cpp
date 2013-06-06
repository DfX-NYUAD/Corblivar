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

// required Corblivar headers
#include "CorblivarCore.hpp"
#include "FloorPlanner.hpp"
#include "IO.hpp"

int main (int argc, char** argv) {
	FloorPlanner fp;
	bool done;

	cout << endl;
	cout << "Corblivar: Corner Block List for Varied [Block] Alignment Requests" << endl;
	// (TODO) set version to 1.0 after release
	cout << "----- 3D Floorplanning tool v0.1 ---------------------------------" << endl << endl;

	// parse program parameter and config file
	IO::parseParameterConfig(fp, argc, argv);
	// parse blocks
	IO::parseBlocks(fp);
	// parse nets
	IO::parseNets(fp);

	// init Corblivar core
	CorblivarCore corb = CorblivarCore(fp.getLayers(), fp.getBlocks().size());

	// init thermal analyzer, only reasonable after parsing config file
	fp.initThermalAnalyzer();

	// TODO drop; test data for alignment handling
	corb.A.push_back(CorblivarAlignmentReq(0, &fp.getBlocks()[5], &fp.getBlocks()[1],
				CorblivarAlignmentReq::Type::RANGE, 100.0, CorblivarAlignmentReq::Type::RANGE_MAX, 2000.0));
	corb.A.push_back(CorblivarAlignmentReq(1, &fp.getBlocks()[5], &fp.getBlocks()[8],
				CorblivarAlignmentReq::Type::RANGE, 100.0, CorblivarAlignmentReq::Type::RANGE_MAX, 2000.0));
	corb.A.push_back(CorblivarAlignmentReq(2, &fp.getBlocks()[5], &fp.getBlocks()[2],
				CorblivarAlignmentReq::Type::RANGE, 100.0, CorblivarAlignmentReq::Type::RANGE_MAX, 2000.0));
	//corb.A.push_back(CorblivarAlignmentReq(3, &fp.getBlocks()[8], &fp.getBlocks()[2],
	//			CorblivarAlignmentReq::Type::RANGE, 1000.0, CorblivarAlignmentReq::Type::RANGE, 1000.0));
	//corb.A.push_back(CorblivarAlignmentReq(4, &fp.getBlocks()[8], &fp.getBlocks()[3],
	//			CorblivarAlignmentReq::Type::RANGE, 1000.0, CorblivarAlignmentReq::Type::RANGE, 1000.0));
	//corb.A.push_back(CorblivarAlignmentReq(5, &fp.getBlocks()[8], &fp.getBlocks()[4],
	//			CorblivarAlignmentReq::Type::RANGE, 1000.0, CorblivarAlignmentReq::Type::RANGE, 1000.0));
	//corb.A.push_back(CorblivarAlignmentReq(6, &fp.getBlocks()[7], &fp.getBlocks()[1],
	//			CorblivarAlignmentReq::Type::RANGE, 1000.0, CorblivarAlignmentReq::Type::RANGE, 1000.0));

	// non-regular run; read in solution file
	// (TODO) adapt if further optimization of read in data is desired
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
