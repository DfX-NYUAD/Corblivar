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
		cout << "IO> ";
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
		cout << "IO> ";
		cout << "No such config file: " << config_file<< endl;
		exit(1);
	}
	in.close();

	in.open(corb.blocks_file.c_str());
	if (!in.good())
	{
		cout << "IO> ";
		cout << "No such blocks file: " << corb.blocks_file << endl;
		exit(1);
	}
	in.close();

	in.open(corb.power_file.c_str());
	if (!in.good())
	{
		cout << "IO> ";
		cout << "No such power file: " << corb.power_file << endl;
		exit(1);
	}
	in.close();

	in.open(corb.nets_file.c_str());
	if (!in.good())
	{
		cout << "IO> ";
		cout << "No such nets file: " << corb.nets_file << endl;
		exit(1);
	}
	in.close();

	// config file
	if (corb.logMed()) {
		cout << "IO> ";
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

	// determine outline aspect ratio
	corb.outline_AR = corb.conf_outline_x / corb.conf_outline_y;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> corb.conf_SA_loopFactor;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> corb.conf_SA_loopLimit;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> corb.conf_SA_temp_factor_phase1;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> corb.conf_SA_temp_factor_phase2;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> corb.conf_SA_temp_factor_phase3;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> corb.conf_SA_temp_phase_trans_12_factor;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> corb.conf_SA_temp_phase_trans_23_factor;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> corb.conf_SA_cost_temp;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> corb.conf_SA_cost_IR;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> corb.conf_SA_cost_WL;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> corb.conf_SA_cost_TSVs;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> corb.conf_SA_cost_area_outline;

	in.close();

	// init results file
	if (argc > 4) {
		results_file << argv[4];
	}
	else {
		results_file << corb.benchmark << "_" << corb.conf_layer << ".solution";
	}
	corb.results.open(results_file.str().c_str());

	if (corb.logMin()) {
		cout << "IO> Config values:" << endl;
		cout << "IO>  Loglevel (1 to 3 for minimal, medium, maximal): " << corb.conf_log << endl;
		cout << "IO>  Layers for 3D IC: " << corb.conf_layer << endl;
		cout << "IO>  Fixed die outline (width, x-dimension): " << corb.conf_outline_x << endl;
		cout << "IO>  Fixed die outline (height, y-dimension): " << corb.conf_outline_y << endl;
		cout << "IO>  SA -- Inner-loop operation-count a (iterations = a * N^(4/3) for N blocks): " << corb.conf_SA_loopFactor << endl;
		cout << "IO>  SA -- Outer-loop upper limit: " << corb.conf_SA_loopLimit << endl;
		cout << "IO>  SA -- Temperature-scaling factor for phase 1 (fast cooling): " << corb.conf_SA_temp_factor_phase1 << endl;
		cout << "IO>  SA -- Temperature-scaling factor for phase 2 (slow cooling): " << corb.conf_SA_temp_factor_phase2 << endl;
		cout << "IO>  SA -- Temperature-scaling factor for phase 3 (reheating): " << corb.conf_SA_temp_factor_phase3 << endl;
		cout << "IO>  SA -- Boundary factor for transitions b/w phases 1, 2: " << corb.conf_SA_temp_phase_trans_12_factor << endl;
		cout << "IO>  SA -- Boundary factor for transitions b/w phases 2, 3: " << corb.conf_SA_temp_phase_trans_23_factor << endl;
		cout << "IO>  SA -- Cost factor for temperature: " << corb.conf_SA_cost_temp << endl;
		cout << "IO>  SA -- Cost factor for IR-drop: " << corb.conf_SA_cost_IR << endl;
		cout << "IO>  SA -- Cost factor for wirelength: " << corb.conf_SA_cost_WL << endl;
		cout << "IO>  SA -- Cost factor for TSVs: " << corb.conf_SA_cost_TSVs << endl;
		cout << "IO>  SA -- Cost factor for area and outline violation: " << corb.conf_SA_cost_area_outline << endl;
		cout << endl;
	}
}

// parse blocks file
void IO::parseBlocks(CorblivarFP &corb) {
	ifstream blocks_in, power_in;
	string tmpstr;
	Block *cur_block;
	int id;

	if (corb.logMed()) {
		cout << "IO> ";
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
			// normalize to watts
			cur_block->power /= 1000.0;
			// scale power values according to block scaling
			// note that correct scaling (maintain power/area ratio)
			// would imply using pow(IO::BLOCKS_SCALE_UP, 2.0); which is
			// rejected in order to limit overall power values
			cur_block->power *= pow(IO::BLOCKS_SCALE_UP, 3.0/2.0);
		}
		else {
			if (corb.logMin()) {
				cout << "IO> ";
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
			cout << "IO> ";
			cout << "Unhandled block type: " << tmpstr << endl;
			exit(1);
		}

		// scale up dimensions
		cur_block->bb.w *= IO::BLOCKS_SCALE_UP;
		cur_block->bb.h *= IO::BLOCKS_SCALE_UP;

		// calculate block area
		cur_block->bb.area = cur_block->bb.w * cur_block->bb.h;
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
		double area = 0.0;
		map<int, Block*>::iterator b;

		for (b = corb.blocks.begin(); b != corb.blocks.end(); ++b) {
			power += (*b).second->power;
			area += (*b).second->bb.area;
		}

		cout << "IO> ";
		cout << "Done; " << corb.blocks.size() << " blocks read in" << endl;
		cout << "IO>  (blocks power: " << power << "; blocks area: " << area;
		cout << "; blocks area / total area: " << area / (corb.conf_layer * corb.conf_outline_x * corb.conf_outline_y) << ")" << endl;
		cout << endl;
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
		cout << "IO> ";
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
					cout << "IO> ";
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
		cout << "IO> ";
		cout << "Done; " << corb.nets.size() << " nets read in" << endl << endl;
	}

}

void IO::writePowerThermalMaps(CorblivarFP &corb) {
	ofstream gp_out;
	ofstream data_out;
	int cur_layer;
	int layer_limit;
	unsigned x, y;
	unsigned x_limit, y_limit;
	int flag;

	if (corb.logMed()) {
		cout << "IO> ";
		cout << "Generating power maps and thermal profiles ..." << endl;
	}

	// flag=0: generate power maps
	// flag=1: generate thermal map
	for (flag = 0; flag <= 1; flag++) {

		// power maps for all layers
		if (flag == 0) {
			layer_limit = corb.conf_layer;
		}
		// thermal map only for layer 0
		else {
			layer_limit = 1;
		}

		for (cur_layer = 0; cur_layer < layer_limit; cur_layer++) {
			// build up file names
			stringstream gp_out_name;
			stringstream data_out_name;
			if (flag == 0) {
				gp_out_name << corb.benchmark << "_" << cur_layer << "_power.gp";
				data_out_name << corb.benchmark << "_" << cur_layer << "_power.data";
			}
			else {
				gp_out_name << corb.benchmark << "_" << cur_layer << "_thermal.gp";
				data_out_name << corb.benchmark << "_" << cur_layer << "_thermal.data";
			}

			// init file stream for gnuplot script
			gp_out.open(gp_out_name.str().c_str());
			// init file stream for data file
			data_out.open(data_out_name.str().c_str());

			// file header for gnuplot script
			if (flag == 0) {
				gp_out << "set title \"" << corb.benchmark << " - Power Map Layer " << cur_layer + 1 << "\"" << endl;
			}
			else {
				gp_out << "set title \"" << corb.benchmark << " - Thermal Map Layer " << cur_layer + 1 << "\"" << endl;
			}
			gp_out << "set terminal postscript color enhanced \"Times\" 20" << endl;
			gp_out << "set output \"" << gp_out_name.str() << ".eps\"" << endl;
			gp_out << "set size square" << endl;
			if (flag == 0) {
				gp_out << "set xrange [0:" << corb.power_maps[cur_layer].size() - 1 << "]" << endl;
				gp_out << "set yrange [0:" << corb.power_maps[cur_layer][0].size() - 1 << "]" << endl;
			}
			else {
				gp_out << "set xrange [0:" << corb.thermal_map.size() - 1 << "]" << endl;
				gp_out << "set yrange [0:" << corb.thermal_map[0].size() - 1 << "]" << endl;
			}
			gp_out << "set tics front" << endl;
			gp_out << "set grid xtics ytics ztics" << endl;
			gp_out << "set pm3d map" << endl;
			// color printable as gray
			gp_out << "set palette rgbformulae 30,31,32" << endl;
			gp_out << "splot \"" << data_out_name.str() << "\" using 1:2:3 notitle" << endl;

			// close file stream for gnuplot script
			gp_out.close();

			// file header for data file
			data_out << "# X Y Power" << endl;

			// determine grid boundaries
			if (flag == 0) {
				x_limit = corb.power_maps[cur_layer].size();
				y_limit = corb.power_maps[cur_layer][0].size();
			}
			else {
				x_limit = corb.thermal_map.size();
				y_limit = corb.thermal_map[0].size();
			}

			// output grid values
			for (x = 0; x < x_limit; x++) {
				for (y = 0; y < y_limit; y++) {
					if (flag == 0) {
						data_out << x << "	" << y << "	" << corb.power_maps[cur_layer][x][y] << endl;
					}
					else {
						data_out << x << "	" << y << "	" << corb.thermal_map[x][y] << endl;
					}
				}
				data_out << endl;
			}

			// close file stream for data file
			data_out.close();
		}

	}

	if (corb.logMed()) {
		cout << "IO> ";
		cout << "Done" << endl << endl;
	}
}

// generate GP plots of FP
void IO::writeFloorplanGP(CorblivarFP &corb, string file_suffix) {
	ofstream gp_out;
	int cur_layer;
	int object_counter;
	map<int, Block*>::iterator b;
	Block *cur_block;

	if (corb.logMed()) {
		cout << "IO> ";
		if (file_suffix != "")
			cout << "Generating GP scripts for floorplan (suffix \"" << file_suffix << "\")..." << endl;
		else
			cout << "Generating GP scripts for floorplan ..." << endl;
	}

	for (cur_layer = 0; cur_layer < corb.conf_layer; cur_layer++) {
		// build up file name
		stringstream out_name;
		out_name << corb.benchmark << "_" << cur_layer;
		if (file_suffix != "")
			out_name << "_" << file_suffix;
		out_name << ".gp";

		// init file stream
		gp_out.open(out_name.str().c_str());

		// file header
		gp_out << "set title \"" << corb.benchmark << " - Layer " << cur_layer + 1 << "\"" << endl;
		gp_out << "set terminal postscript color enhanced \"Times\" 20" << endl;
		gp_out << "set output \"" << out_name.str() << ".eps\"" << endl;
		gp_out << "set size square" << endl;
		gp_out << "set xrange [0:" << corb.conf_outline_x << "]" << endl;
		gp_out << "set yrange [0:" << corb.conf_outline_y << "]" << endl;
		gp_out << "set xtics " << corb.conf_outline_x / 5 << endl;
		gp_out << "set ytics " << corb.conf_outline_x / 5 << endl;
		gp_out << "set mxtics 4" << endl;
		gp_out << "set mytics 4" << endl;
		gp_out << "set tics front" << endl;
		gp_out << "set grid xtics ytics mxtics mytics" << endl;

		// init obj counter
		object_counter = 1;

		// output blocks
		for (b = corb.blocks.begin(); b != corb.blocks.end(); ++b) {
			cur_block = (*b).second;

			if (cur_block->layer != cur_layer) {
				continue;
			}

			gp_out << "set obj " << object_counter;
			object_counter++;

			// blocks
			gp_out << " rect";
			gp_out << " from " << cur_block->bb.ll.x << "," << cur_block->bb.ll.y;
			gp_out << " to " << cur_block->bb.ur.x << "," << cur_block->bb.ur.y;
			gp_out << " fillcolor rgb \"#ac9d93\" fillstyle solid";
			gp_out << endl;

			// label
			gp_out << "set label \"b" << cur_block->id << "\"";
			gp_out << " at " << cur_block->bb.ll.x + 2.0 * IO::BLOCKS_SCALE_UP;
			gp_out << "," << cur_block->bb.ll.y + 5.0 * IO::BLOCKS_SCALE_UP;
			gp_out << " font \"Times,6\"" << endl;
		}

		// file footer
		gp_out << "plot NaN notitle" << endl;

		// close file stream
		gp_out.close();
	}

	if (corb.logMed()) {
		cout << "IO> ";
		cout << "Done" << endl << endl;
	}
}

// generate files for HotSpot steady-state thermal simulation
void IO::writeHotSpotFiles(CorblivarFP &corb) {
	ofstream file;
	map<int, Block*>::iterator b;
	Block *cur_block;
	int cur_layer;
	// factor to scale um downto m;
	static const double SCALE_DOWN = 0.000001;

	if (corb.logMed()) {
		cout << "IO> Generating files for HotSpot 3D-thermal simulation..." << endl;
	}

	/// generate floorplan files
	for (cur_layer = 0; cur_layer < corb.conf_layer; cur_layer++) {
		// build up file name
		stringstream fp_file;
		fp_file << corb.benchmark << "_HotSpot_" << cur_layer << ".flp";

		// init file stream
		file.open(fp_file.str().c_str());

		// file header
		file << "# Line Format: <unit-name>\\t<width>\\t<height>\\t<left-x>\\t<bottom-y>\\t<specific-heat>\\t<resistivity>" << endl;
		file << "# all dimensions are in meters" << endl;
		file << "# comment lines begin with a '#'" << endl;
		file << "# comments and empty lines are ignored" << endl;
		file << endl;

		// output blocks
		for (b = corb.blocks.begin(); b != corb.blocks.end(); ++b) {
			cur_block = (*b).second;

			if (cur_block->layer != cur_layer) {
				continue;
			}

			file << "b" << cur_block->id;
			file << "	" << cur_block->bb.w * SCALE_DOWN;
			file << "	" << cur_block->bb.h * SCALE_DOWN;
			file << "	" << cur_block->bb.ll.x * SCALE_DOWN;
			file << "	" << cur_block->bb.ll.y * SCALE_DOWN;
			file << "	" << IO::HEAT_CAPACITY_SI;
			file << "	" << IO::THERMAL_RESISTIVITY_SI;
			file << endl;
		}

		// dummy block to describe layer outline
		file << "outline" << cur_layer;
		file << "	" << corb.conf_outline_x * SCALE_DOWN;
		file << "	" << corb.conf_outline_y * SCALE_DOWN;
		file << "	0.0";
		file << "	0.0";
		file << "	0.0";
		file << "	0.0";
		file << endl;

		// close file stream
		file.close();
	}

	/// generate dummy floorplan for BEOL layer
	//
	// build up file name
	stringstream BEOL_fp_file;
	BEOL_fp_file << corb.benchmark << "_HotSpot_BEOL.flp";

	// init file stream
	file.open(BEOL_fp_file.str().c_str());

	// file header
	file << "# Line Format: <unit-name>\\t<width>\\t<height>\\t<left-x>\\t<bottom-y>\\t<specific-heat>\\t<resistivity>" << endl;
	file << "# all dimensions are in meters" << endl;
	file << "# comment lines begin with a '#'" << endl;
	file << "# comments and empty lines are ignored" << endl;

	// BEOL ``block''
	file << "BEOL";
	file << "	" << corb.conf_outline_x * SCALE_DOWN;
	file << "	" << corb.conf_outline_y * SCALE_DOWN;
	file << "	0.0";
	file << "	0.0";
	file << "	" << IO::HEAT_CAPACITY_BEOL;
	file << "	" << IO::THERMAL_RESISTIVITY_BEOL;
	file << endl;

	// close file stream
	file.close();

	/// generate power-trace file
	//
	// build up file name
	stringstream power_file;
	power_file << corb.benchmark << "_HotSpot.ptrace";

	// init file stream
	file.open(power_file.str().c_str());

	// block sequence in trace file has to follow layer files, thus build up file
	// according to layer structure
	//
	// output block labels in first line
	for (cur_layer = 0; cur_layer < corb.conf_layer; cur_layer++) {

		for (b = corb.blocks.begin(); b != corb.blocks.end(); ++b) {
			cur_block = (*b).second;

			if (cur_block->layer != cur_layer) {
				continue;
			}

			file << "b" << cur_block->id << " ";
		}

		// dummy outline block
		file << "outline" << cur_layer << " ";
	}
	file << endl;

	// output block power in second line
	for (cur_layer = 0; cur_layer < corb.conf_layer; cur_layer++) {

		for (b = corb.blocks.begin(); b != corb.blocks.end(); ++b) {
			cur_block = (*b).second;

			if (cur_block->layer != cur_layer) {
				continue;
			}

			file << cur_block->power << " ";
		}

		// dummy outline block
		file << "0.0 ";
	}
	file << endl;

	// close file stream
	file.close();

	/// generate 3D-IC description file
	//
	// build up file name
	stringstream stack_file;
	stack_file << corb.benchmark << "_HotSpot.lcf";

	// init file stream
	file.open(stack_file.str().c_str());

	// file header
	file << "#Lines starting with # are used for commenting" << endl;
	file << "#Blank lines are also ignored" << endl;
	file << endl;
	file << "#File Format:" << endl;
	file << "#<Layer Number>" << endl;
	file << "#<Lateral heat flow Y/N?>" << endl;
	file << "#<Power Dissipation Y/N?>" << endl;
	file << "#<Specific heat capacity in J/(m^3K)>" << endl;
	file << "#<Resistivity in (m-K)/W>" << endl;
	file << "#<Thickness in m>" << endl;
	file << "#<floorplan file>" << endl;
	file << endl;

	for (cur_layer = 0; cur_layer < corb.conf_layer; cur_layer++) {

		file << "# Active Si layer; design layer " << cur_layer << endl;
		file << 2 * cur_layer << endl;
		file << "Y" << endl;
		file << "Y" << endl;
		file << IO::HEAT_CAPACITY_SI << endl;
		file << IO::THERMAL_RESISTIVITY_SI << endl;
		file << IO::THICKNESS_SI << endl;
		file << corb.benchmark << "_HotSpot_" << cur_layer << ".flp" << endl;
		file << endl;

		file << "# BEOL layer for active layer " << cur_layer << endl;
		file << 2 * cur_layer + 1 << endl;
		file << "Y" << endl;
		file << "N" << endl;
		file << IO::HEAT_CAPACITY_BEOL << endl;
		file << IO::THERMAL_RESISTIVITY_BEOL << endl;
		file << IO::THICKNESS_BEOL << endl;
		file << corb.benchmark << "_HotSpot_BEOL.flp" << endl;
		file << endl;
	}

	// close file stream
	file.close();

	if (corb.logMed()) {
		cout << "IO> Done" << endl << endl;
	}
}
