/*
 * =====================================================================================
 *
 *    Description:  Parser for 3DFP solutions, generates output data like Corblivar does
 *
 *         Author:  Johann Knechtel, johann.knechtel@ifte.de
 *        Company:  Institute of Electromechanical and Electronic Design, www.ifte.de
 *
 * =====================================================================================
 */

// required Corblivar headers
#include "../src/CorblivarCore.hpp"
#include "../src/FloorPlanner.hpp"
#include "../src/IO.hpp"

// forward declaration
void parse3DSTAF(FloorPlanner& fp);

int main (int argc, char** argv) {
	double x = 0.0, y = 0.0;
	FloorPlanner fp;

	cout << endl;
	cout << "3DSTAF Results Parser" << endl;
	cout << endl;

	// parse program parameter and config file
	IO::parseParameterConfig(fp, argc, argv);
	// parse blocks
	IO::parseBlocks(fp);
	// parse nets
	IO::parseNets(fp);

	// init (dummy) Corblivar core
	CorblivarCore corb = CorblivarCore(fp.getLayers(), fp.getBlocks().size());

	// parse layer files from 3DFP
	parse3DSTAF(fp);

	// determine die outline according to parsed layout
	for (Block const& b : fp.getBlocks()) {
		x = max(x, b.bb.ur.x);
		y = max(y, b.bb.ur.y);
	}

	// reset die outline and related stuff
	fp.resetDieProperties(x, y);

	// init thermal analyzer, only reasonable after reseting die outline
	fp.initThermalAnalyzer();

	// generate 3DFP related output data
	fp.finalize(corb, false, false);
}

void parse3DSTAF(FloorPlanner& fp) {
	ifstream file_3DSTAF;
	stringstream name_3DSTAF;
	string tmpstr;
	Block* block;
	vector<Block> blocks;
	int layer;
	int id;

	// 3DSTAF file name
	name_3DSTAF << fp.getBenchmark() << ".log";

	// try file opening
	file_3DSTAF.open(name_3DSTAF.str().c_str());
	if (!file_3DSTAF.good()) {
		cout << "3DSTAF file \"" << name_3DSTAF.str() << "\" missing!" << endl;
		exit(1);
	}

	// copy blocks; required for mapping 3D-STAF subsequent block numbering to real
	// string names in Corblivar
	blocks = fp.getBlocks();

	// parse file
	//
	// drop header lines
	file_3DSTAF >> tmpstr;
	while (tmpstr != "Tem" && !file_3DSTAF.eof())
		file_3DSTAF >> tmpstr;

	// parse blocks, e.g.,
	// 72    174   169   61   61   3.20e+06 0.011907   0.00
	//
	// w/ the following format:
	// No      X     Y    L    W   PD  Power  Tem
	//
	// all dimensions are related to benchmark units, i.e. um

	// layers have to be inverted, since 3DSTAF assumes the heatsink to be below die 1
	layer = fp.getLayers();
	// init w/ first block id
	file_3DSTAF >> tmpstr;

	while (!file_3DSTAF.eof()) {

		// we may have also reached the next layer
		if (tmpstr == "**********************************************************") {
			layer--;
			// skip further lines until blocks can be parsed
			while (tmpstr != "Tem" && !file_3DSTAF.eof())
				file_3DSTAF >> tmpstr;
			file_3DSTAF >> tmpstr;

			// we may have parsed the last layer
			if (file_3DSTAF.eof()) {
				break;
			}
		}

		// sanity check of 3DSTAF block id
		id = stoi(tmpstr);
		if (id < 0 || id >= static_cast<int>(blocks.size())) {
			cout << "Block parsed from 3DSTAF file cannot be interpreted, block number: " << id << endl;
			exit(1);
		}

		// try to find related block
		block = fp.editBlock(blocks[id].id);
		if (block == nullptr) {
			cout << "Block parsed from 3DSTAF file cannot be found, block id: " << tmpstr << endl;
			exit(1);
		}
		else {
			// edit block, parse geometry
			file_3DSTAF >> block->bb.ll.x;
			file_3DSTAF >> block->bb.ll.y;
			file_3DSTAF >> block->bb.w;
			file_3DSTAF >> block->bb.h;

			// determine upper right corner
			block->bb.ur.x = block->bb.ll.x + block->bb.w;
			block->bb.ur.y = block->bb.ll.y + block->bb.h;

			// DBG
			//cout << block->id << ", ";
			//cout << block->bb.w << ", " << block->bb.h << ", " << block->bb.ll.x << ", " << block->bb.ll.y << endl;

			// annotate layer to block
			block->layer = layer - 1;

			// drop PD, Power, Tem
			file_3DSTAF >> tmpstr;
			file_3DSTAF >> tmpstr;
			file_3DSTAF >> tmpstr;

			// parse in next block id
			file_3DSTAF >> tmpstr;
		}
	}

	// sanity check for parsed layers
	if (layer != 0) {
		cout << "Layer mismatch, parsed from 3D-STAF file: " << fp.getLayers() - layer << "; expected from Corblivar config file: " << fp.getLayers() << endl;
		exit(1);
	}

	// close file
	file_3DSTAF.close();
}
