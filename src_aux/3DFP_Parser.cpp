/*
 * =====================================================================================
 *
 *    Description:  Parser for 3DFP solutions, generates output data like Corblivar does
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

// forward declaration
void parse3DFP(FloorPlanner& fp);

int main (int argc, char** argv) {
	double x = 0.0, y = 0.0;
	FloorPlanner fp;

	cout << endl;
	cout << "3DFP Results Parser" << endl;
	cout << endl;

	// parse program parameter, config file, and further files
	IO::parseParametersFiles(fp, argc, argv);
	// parse blocks
	IO::parseBlocks(fp);
	// parse nets
	IO::parseNets(fp);

	// init (dummy) Corblivar core
	CorblivarCore corb = CorblivarCore(fp.getLayers(), fp.getBlocks().size());

	// parse layer files from 3DFP
	parse3DFP(fp);

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

void parse3DFP(FloorPlanner& fp) {
	ifstream layer_3DFP_file;
	string tmpstr;
	Block const* block;
	
	for (int layer = 1; layer <= fp.getLayers(); layer++) {

		// file name
		stringstream layer_3DFP;
		layer_3DFP << "layer" << layer << ".flp";

		// try file opening
		layer_3DFP_file.open(layer_3DFP.str().c_str());
		if (!layer_3DFP_file.good()) {
			cout << "3DFP Layer file \"" << layer_3DFP.str() << "\" missing!" << endl;
			exit(1);
		}

		// parse file
		//
		// drop 5 header lines
		for (unsigned l = 1; l <= 5; l++) {
			getline(layer_3DFP_file, tmpstr);
		}

		// parse blocks, e.g.,
		//bk21	0.00367	0.00120	0.00824	0.00197
		// w/ the following format: <unit-name>	<width>	<height> <left-x> <bottom-y>
		// all dimensions are in meters
		
		while (!layer_3DFP_file.eof()) {
			// block id
			layer_3DFP_file >> tmpstr;

			if (layer_3DFP_file.eof())
				break;

			// try to find related block
			block = Block::findBlock(tmpstr, fp.getBlocks());
			if (block == nullptr) {
				cout << "Block parsed from 3DFP file cannot be found, block id: " << tmpstr << endl;
				exit(1);
			}
			else {
				// edit block, parse geometry
				// coordinats are given in meters, normalize to um
				layer_3DFP_file >> block->bb.w;
				block->bb.w *= 1.0e06;
				layer_3DFP_file >> block->bb.h;
				block->bb.h *= 1.0e06;
				layer_3DFP_file >> block->bb.ll.x;
				block->bb.ll.x *= 1.0e06;
				layer_3DFP_file >> block->bb.ll.y;
				block->bb.ll.y *= 1.0e06;

				// determine upper right corner
				block->bb.ur.x = block->bb.ll.x + block->bb.w;
				block->bb.ur.y = block->bb.ll.y + block->bb.h;

				// DBG
				//cout << block->id << ", ";
				//cout << block->bb.w << ", " << block->bb.h << ", " << block->bb.ll.x << ", " << block->bb.ll.y << endl;

				// annotate layer to block
				block->layer = layer - 1;
			}
		}

		// close file
		layer_3DFP_file.close();
	}
}
