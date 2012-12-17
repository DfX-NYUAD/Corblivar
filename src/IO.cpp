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
void IO::parseParameterConfig(Corblivar_FP &corb, int argc, char** argv) {
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
// TODO update
void IO::parseBlocks(Corblivar_FP &corb, string file) {
	ifstream in;
	string tmpstr;
	Block *cur_block;
	int id;

	if (corb.logMed()) {
		cout << "Parsing blocks..." << endl;
	}

	// open fp file
	in.open(file.c_str());

	// reset blocks
	corb.blocks.clear();

	// skip headers
	while (tmpstr != "sb0" && !in.eof())
		in >> tmpstr;

	// parse blocks
	id = 0;
	while (!in.eof()) {

		// find line containing block
		while (tmpstr.find("sb", 0) == string::npos && !in.eof()) {
			in >> tmpstr;
		}
		if (in.eof()) {
			break;
		}

		// init block
		cur_block = new Block();
		cur_block->id = id;

		// parse block type
		in >> tmpstr;
		if (tmpstr == "hardrectilinear") {
			cout << id << endl;
			//TODO parse block info
		}
		else {
			cout << "Unhandled block type: " << tmpstr << endl;
			exit(1);
		}

		// store block
		corb.blocks.insert( pair<int, Block*>(id, cur_block) );

		id++;

	}
}

// parse nets file
//TODO update
void IO::parseNets(Corblivar_FP &corb, string file) {
	ifstream in;
	string tmpstr;
	Net *cur_net;
	int i, net_degree;
	int net_block_id;
	string net_block;
	int cur_layer;
	map<int, Block*>::iterator b;

	if (corb.logMed()) {
		cout << "Parsing nets..." << endl;
	}

}

// generate GP plots of FP
void IO::writeFloorplanGP(Corblivar_FP &corb) {
	writeFloorplanGP(corb, "");
}

// generate GP plots of FP
// TODO update
void IO::writeFloorplanGP(Corblivar_FP &corb, string file_suffix) {
	ofstream gp_out;
	int cur_layer;
	int object_counter;
	map<int, Block*>::iterator b;
	Block *cur_block;
//	TSV *cur_TSV;
	Rect cur_TSV_bb;

}
