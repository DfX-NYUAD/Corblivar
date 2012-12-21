/*
 * =====================================================================================
 *
 *    Description:  IO handler
 *
 *
 *         Author:  Johann Knechtel, johann.knechtel@ifte.de
 *        Company:  Institute of Electromechanical and Electronic Design, www.ifte.de
 *
 * =====================================================================================
 */
#include "Corblivar.hpp"

// parse program parameter and config file
void IO::parseParameterConfig(CorblivarFP &corb, int argc, char** argv) {
	ifstream in;
	string config_file;
	stringstream results_file;
	string tmpstr;

	// program parameter
	if (argc < 4)
	{
		cout << "Usage: " << argv[0] << " benchmark_name config_file benchmarks_dir [results_file]" << endl;
		cout << endl;
		cout << "Expected config_file format: see Corblivar.conf" << endl;
		cout << "Expected benchmarks: GSRC n... sets" << endl;
		exit(1);
	}

	corb.benchmark = argv[1];
	config_file = argv[2];
	stringstream blocks_file;
	blocks_file << argv[3] << corb.benchmark << ".blocks";
	corb.blocks_file = blocks_file.str();
	stringstream power_file;
	power_file << argv[3] << corb.benchmark << ".power";
	corb.power_file = power_file.str();
	stringstream nets_file;
	nets_file << argv[3] << corb.benchmark << ".nets";
	corb.nets_file = nets_file.str();

	// test files
	in.open(config_file.c_str());
	if (!in.good())
	{
		cout << "No such config file: " << config_file<< endl;
		exit(1);
	}
	in.close();

	in.open(corb.blocks_file.c_str());
	if (!in.good())
	{
		cout << "No such blocks file: " << corb.blocks_file << endl;
		exit(1);
	}
	in.close();

	in.open(corb.power_file.c_str());
	if (!in.good())
	{
		cout << "No such power file: " << corb.power_file << endl;
		exit(1);
	}
	in.close();

	in.open(corb.nets_file.c_str());
	if (!in.good())
	{
		cout << "No such nets file: " << corb.nets_file << endl;
		exit(1);
	}
	in.close();

	// config file
	if (corb.logMin()) {
		cout << "Parsing config file..." << endl;
	}

	in.open(config_file.c_str());

	// parse in parameters
	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> corb.conf_log;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> corb.conf_layer;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> corb.conf_outline_x;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> corb.conf_outline_y;

	in.close();

	// init results file
	if (argc > 4) {
		results_file << argv[4];
	}
	else {
		results_file << corb.benchmark << "_" << corb.conf_layer << ".solution";
	}
	corb.results.open(results_file.str().c_str());

	if (corb.logMed()) {
		cout << "Config values:" << endl;
		cout << "Loglevel (1 to 3 for minimal, medium, maximal): " << corb.conf_log << endl;
		cout << "Layers for 3D IC: " << corb.conf_layer << endl;
		cout << "Fixed die outline (width, x-dimension): " << corb.conf_outline_x << endl;
		cout << "Fixed die outline (height, y-dimension): " << corb.conf_outline_y << endl;
	}
	if (corb.logMin()) {
		cout << "Done" << endl << endl;
	}
}

// parse blocks file
void IO::parseBlocks(CorblivarFP &corb) {
	ifstream blocks_in, power_in;
	string tmpstr;
	Block *cur_block;
	int id;

	if (corb.logMed()) {
		cout << "Parsing blocks..." << endl;
	}

	// open files
	blocks_in.open(corb.blocks_file.c_str());
	power_in.open(corb.power_file.c_str());

	// reset blocks
	corb.blocks.clear();

	// drop block files header
	while (tmpstr != "sb0" && !blocks_in.eof())
		blocks_in >> tmpstr;

	// parse blocks
	id = 0;
	while (!blocks_in.eof()) {

		// find line containing block
		while (tmpstr.find("sb") == string::npos && !blocks_in.eof()) {
			blocks_in >> tmpstr;
		}
		if (blocks_in.eof()) {
			break;
		}

		// init block
		cur_block = new Block(id);
		// assign power value
		if (!power_in.eof()) {
			power_in >> cur_block->power;
		}
		else {
			if (corb.logMin()) {
				cout << "Block " << id << " has no power value assigned!" << endl;
			}
		}

		// parse block type
		blocks_in >> tmpstr;
		if (tmpstr == "hardrectilinear") {
			// drop "4"
			blocks_in >> tmpstr;
			// drop "(0,"
			blocks_in >> tmpstr;
			// drop "0)"
			blocks_in >> tmpstr;
			// drop "(0,"
			blocks_in >> tmpstr;
			// drop "y)"
			blocks_in >> tmpstr;
			// parse "(x,"
			blocks_in >> tmpstr;
			cur_block->bb.w = atof(tmpstr.substr(1, tmpstr.size() - 2).c_str());
			// parse "y)"
			blocks_in >> tmpstr;
			cur_block->bb.h = atof(tmpstr.substr(0, tmpstr.size() - 1).c_str());
			// drop "(x,"
			blocks_in >> tmpstr;
			// drop "0)"
			blocks_in >> tmpstr;
		}
		else {
			cout << "Unhandled block type: " << tmpstr << endl;
			exit(1);
		}

		// store block
		corb.blocks.insert( pair<int, Block*>(id, cur_block) );

		id++;

	}

	// close files
	blocks_in.close();
	power_in.close();

	// logging
	if (corb.logMed()) {
		double power = 0.0;
		map<int, Block*>::iterator b;

		for (b = corb.blocks.begin(); b != corb.blocks.end(); ++b) {
			power += (*b).second->power;
		}

		cout << "Done; " << corb.blocks.size() << " blocks read in; " << power << " total power" << endl << endl;
	}
}

// parse nets file
void IO::parseNets(CorblivarFP &corb) {

	ifstream in;
	string tmpstr;
	Net *cur_net;
	int i, net_degree;
	int net_block_id;
	string net_block;
	map<int, Block*>::iterator b;
	int id;

	if (corb.logMed()) {
		cout << "Parsing nets..." << endl;
	}

	// reset nets
	corb.nets.clear();

	// open nets file
	in.open(corb.nets_file.c_str());

	// parse nets file
	id = 0;
	while (!in.eof()) {
		cur_net = new Net(id);

		// parse net degree
		//// NetDegree : 2
		while (tmpstr != "NetDegree" && !in.eof()) {
			in >> tmpstr;
		}
		if (in.eof()) {
			break;
		}
		// drop ":"
		in >> tmpstr;

		// read in blocks of net
		in >> net_degree;
		cur_net->blocks.clear();
		for (i = 0; i < net_degree; i++) {
			in >> net_block;
			// parse block
			//// sb31
			if (net_block.find("sb") != string::npos) {

				// retrieve corresponding block
				net_block_id = atoi(net_block.substr(2).c_str());
				b = corb.blocks.find(net_block_id);
				if (b != corb.blocks.end()) {
					cur_net->blocks.push_back((*b).second);
				}
			}
			// parse terminal pin
			//// p1
			else if (net_block.find("p") != string::npos) {
				// mark net as net w/ external pin
				cur_net->hasExternalPin = true;
			}
			else {
				// ignore unknown block
				if (corb.logMin()) {
					cout << "Drop unknown block \"" << net_block << "\" while parsing net " << id << endl;
				}
			}
			// drop "B"
			in >> tmpstr;
		}

		// store nets connecting two or more blocks
		// ignores nets connecting only to external pins
		// (TODO) consider external pins w/ position
		if (cur_net->blocks.size() > 1) {
			corb.nets.push_back(cur_net);
		}
		else {
			delete(cur_net);
		}

		id++;
	}

	// close nets file
	in.close();

#ifdef DBG_IO
	unsigned n, bl;

	for (n = 0; n < corb.nets.size(); n++) {
		cout << "DBG_IO> ";
		cout << "net " << corb.nets[n]->id << endl;
		for (bl = 0; bl < corb.nets[n]->blocks.size(); bl++) {
			cout << "DBG_IO> ";
			cout << " block " << corb.nets[n]->blocks[bl]->id << endl;
		}
	}
#endif

	if (corb.logMed()) {
		cout << "Done; " << corb.nets.size() << " nets read in" << endl << endl;
	}

}

// generate GP plots of FP
void IO::writeFloorplanGP(CorblivarFP &corb) {
	writeFloorplanGP(corb, "");
}

// generate GP plots of FP
// TODO update
void IO::writeFloorplanGP(CorblivarFP &corb, string file_suffix) {
	//ofstream gp_out;
	//int cur_layer;
	//int object_counter;
	//map<int, Block*>::iterator b;
	//Block *cur_block;
//	//TSV *cur_TSV;
	//Rect cur_TSV_bb;

}