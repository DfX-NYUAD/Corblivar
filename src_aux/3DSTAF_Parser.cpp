/*
 * =====================================================================================
 *
 *    Description:  Parser for 3D-STAF solutions, generates output data like Corblivar does
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
void parse3DSTAF(FloorPlanner& fp);

int main (int argc, char** argv) {
	double x = 0.0, y = 0.0;
	FloorPlanner fp;

	std::cout << std::endl;
	std::cout << "3DSTAF Results Parser" << std::endl;
	std::cout << std::endl;

	// parse program parameter, config file, and further files
	IO::parseParametersFiles(fp, argc, argv);
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
		x = std::max(x, b.bb.ur.x);
		y = std::max(y, b.bb.ur.y);
	}

	// reset die outline and related stuff
	fp.resetDieProperties(x, y);

	// init thermal analyzer, only reasonable after reseting die outline
	fp.initThermalAnalyzer();

	// generate 3DFP related output data
	fp.finalize(corb, false, false);
}

void parse3DSTAF(FloorPlanner& fp) {
	std::ifstream file_3DSTAF;
	std::stringstream name_3DSTAF;
	std::string tmpstr;
	Block const* block;
	std::vector<Block> blocks;
	int layer;
	int id;

	// 3DSTAF file name
	name_3DSTAF << fp.getBenchmark() << ".log";

	// try file opening
	file_3DSTAF.open(name_3DSTAF.str().c_str());
	if (!file_3DSTAF.good()) {
		std::cout << "3DSTAF file \"" << name_3DSTAF.str() << "\" missing!" << std::endl;
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
			std::cout << "Block parsed from 3DSTAF file cannot be interpreted, block number: " << id << std::endl;
			exit(1);
		}

		// try to find related block
		block = Block::findBlock(blocks[id].id, fp.getBlocks());
		if (block == nullptr) {
			std::cout << "Block parsed from 3DSTAF file cannot be found, block id: " << tmpstr << std::endl;
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
			//std::cout << block->id << ", ";
			//std::cout << block->bb.w << ", " << block->bb.h << ", " << block->bb.ll.x << ", " << block->bb.ll.y << std::endl;

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
		std::cout << "Layer mismatch, parsed from 3D-STAF file: " << fp.getLayers() - layer << "; expected from Corblivar config file: " << fp.getLayers() << std::endl;
		exit(1);
	}

	// close file
	file_3DSTAF.close();
}
