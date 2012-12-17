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

// parse in and verify program parameter
void IO::parseProgramParameter(Corblivar &corb, int argc, char** argv) {
	ifstream in;

	if (argc < 5)
	{
		cout << "Usage: " << argv[0] << " GSRC_benchmark_name config_file blocks_file nets_file" << endl;
		cout << endl;
		cout << "Expected config_file format: see Corblivar.conf" << endl;
		cout << "Expected blocks_file format: TODO" << endl;
		cout << "Expected nets_file format: GSRC (UCLA nets 1.0)" << endl;
		exit(1);
	}
	else {
		corb.benchmark = argv[1];
		corb.config_file = argv[2];
		corb.blocks_file = argv[3];
		corb.nets_file = argv[4];

		// test files
		in.open(corb.config_file.c_str());
		if (!in.good())
		{
			cout << "No such config file: " << corb.config_file<< endl;
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

	}
}

//TODO update
void IO::parseConfig(Corblivar &corb, string file) {
	ifstream in;
	string tmpstr;

	if (corb.logMin()) {
		cout << "Parsing config file..." << endl;
	}

//	in.open(file.c_str());
//
//	// parse in parameters
//	in >> tmpstr;
//	while (tmpstr != "value" && !in.eof())
//		in >> tmpstr;
//	in >> corb.log;
//
//	in.close();
//
//	if (corb.logMed()) {
//		cout << "Loglevel (1 to 3 for minimal, medium, maximal): " << corb.log << endl;
//	}
	if (corb.logMin()) {
		cout << "Done" << endl;
	}

}

// parse blocks file
// TODO update
void IO::parseBlocks(Corblivar &corb, string file) {
	ifstream in;
	string tmpstr;
	int i, blocks_count, j;
	Block *cur_block;

	if (corb.logMed()) {
		cout << "Parsing blocks..." << endl;
	}
}

// parse nets file
//TODO update
void IO::parseNets(Corblivar &corb, string file) {
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
void IO::writeFloorplanGP(Corblivar &corb) {
	writeFloorplanGP(corb, "");
}

// generate GP plots of FP
// TODO update
void IO::writeFloorplanGP(Corblivar &corb, string file_suffix) {
	ofstream gp_out;
	int cur_layer;
	int object_counter;
	map<int, Block*>::iterator b;
	Block *cur_block;
//	TSV *cur_TSV;
	Rect cur_TSV_bb;

}
