/*
 * =====================================================================================
 *
 *    Description:  Corblivar IO handler
 *
 *
 *         Author:  Johann Knechtel, johann.knechtel@ifte.de
 *        Company:  Institute of Electromechanical and Electronic Design, www.ifte.de
 *
 * =====================================================================================
 */
#include "Corblivar.hpp"
#include "CorblivarFP.hpp"
#include "CorblivarCore.hpp"
#include "IO.hpp"

// parse program parameter and config file
void IO::parseParameterConfig(FloorPlanner& fp, int const& argc, char** argv, bool const& log) {
	ifstream in;
	string config_file;
	stringstream results_file, solution_file;
	stringstream blocks_file;
	stringstream power_density_file;
	stringstream nets_file;
	string tmpstr;

	// program parameter
	if (argc < 4)
	{
		cout << "IO> ";
		cout << "Usage: " << argv[0] << " benchmark_name config_file benchmarks_dir [solution_file]" << endl;
		cout << endl;
		cout << "Expected config_file format: see Corblivar.conf" << endl;
		cout << "Expected benchmarks: GSRC n... sets" << endl;
		cout << "Note: solution_file can be used to start tool w/ given Corblivar data" << endl;

		exit(1);
	}

	fp.benchmark = argv[1];
	config_file = argv[2];

	blocks_file << argv[3] << fp.benchmark << ".blocks";
	fp.blocks_file = blocks_file.str();

	power_density_file << argv[3] << fp.benchmark << ".power";
	fp.power_density_file = power_density_file.str();

	nets_file << argv[3] << fp.benchmark << ".nets";
	fp.nets_file = nets_file.str();

	results_file << fp.benchmark << ".results";
	fp.results.open(results_file.str().c_str());

	// test files
	in.open(config_file.c_str());
	if (!in.good())
	{
		cout << "IO> ";
		cout << "No such config file: " << config_file<< endl;
		exit(1);
	}
	in.close();

	in.open(fp.blocks_file.c_str());
	if (!in.good())
	{
		cout << "IO> ";
		cout << "No such blocks file: " << fp.blocks_file << endl;
		exit(1);
	}
	in.close();

	in.open(fp.power_density_file.c_str());
	if (!in.good())
	{
		cout << "IO> ";
		cout << "No such power density file: " << fp.power_density_file << endl;
		exit(1);
	}
	in.close();

	in.open(fp.nets_file.c_str());
	if (!in.good())
	{
		cout << "IO> ";
		cout << "No such nets file: " << fp.nets_file << endl;
		exit(1);
	}
	in.close();

	// additional parameter for solution file given; consider file for readin
	if (argc == 5) {

		solution_file << argv[4];
		// open file if possible
		fp.solution_in.open(solution_file.str().c_str());
		if (!fp.solution_in.good())
		{
			cout << "IO> ";
			cout << "No such solution file: " << solution_file.str() << endl;
			exit(1);
		}
	}
	// open new solution file
	else {
		solution_file << fp.benchmark << ".solution";
		fp.solution_out.open(solution_file.str().c_str());
	}

	// handle config file
	if (log) {
		cout << "IO> ";
		cout << "Parsing config file..." << endl;
	}

	in.open(config_file.c_str());

	// parse in parameters
	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.conf_log;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.conf_layer;

	// sanity check for >= 2 layers
	if (fp.conf_layer < 2) {
		cout << "Corblivar only supports floorplanning on >= 2 layers!" << endl;
		exit(1);
	}

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.conf_outline_x;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.conf_outline_y;

	// sanity check for positive, non-zero dimensions
	if (fp.conf_outline_x <= 0.0 || fp.conf_outline_y <= 0.0) {
		cout << "Provde positive, non-zero outline dimensions!" << endl;
		exit(1);
	}

	// determine outline aspect ratio
	fp.outline_AR = fp.conf_outline_x / fp.conf_outline_y;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.conf_SA_loopFactor;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.conf_SA_loopLimit;

	// sanity check for positive, non-zero parameters
	if (fp.conf_SA_loopFactor <= 0.0 || fp.conf_SA_loopLimit <= 0.0) {
		cout << "Provde positive, non-zero SA loop parameters!" << endl;
		exit(1);
	}

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.conf_SA_temp_factor_phase1;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.conf_SA_temp_factor_phase2;

	// sanity check for positive, non-zero parameters
	if (fp.conf_SA_temp_factor_phase1 <= 0.0 || fp.conf_SA_temp_factor_phase2 <= 0.0) {
		cout << "Provde positive, non-zero SA cooling parameters!" << endl;
		exit(1);
	}

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.conf_SA_cost_temp;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.conf_SA_cost_WL;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.conf_SA_cost_TSVs;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.conf_SA_cost_area_outline;

	// sanity check for mandatory area, outline cost
	if (fp.conf_SA_cost_area_outline == 0.0) {
		cout << "A cost factor > 0 is required for area and outline optimization!" << endl;
		exit(1);
	}

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.conf_power_blurring_impulse_factor;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.conf_power_blurring_impulse_factor_scaling_exponent;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.conf_power_blurring_mask_boundary_value;

	// sanity check for positive, non-zero parameters
	if (fp.conf_power_blurring_impulse_factor <= 0.0 || fp.conf_power_blurring_impulse_factor_scaling_exponent <= 0.0 || fp.conf_power_blurring_mask_boundary_value <= 0.0) {
		cout << "Provde positive, non-zero power blurring parameters!" << endl;
		exit(1);
	}

	in.close();

	if (log) {
		cout << "IO> Config values:" << endl;
		cout << "IO>  Loglevel (1 to 3 for minimal, medium, maximal): " << fp.conf_log << endl;
		cout << "IO>  Chip -- Layers for 3D IC: " << fp.conf_layer << endl;
		cout << "IO>  Chip -- Fixed die outline (width, x-dimension): " << fp.conf_outline_x << endl;
		cout << "IO>  Chip -- Fixed die outline (height, y-dimension): " << fp.conf_outline_y << endl;
		cout << "IO>  SA -- Inner-loop operation-count a (iterations = a * N^(4/3) for N blocks): " << fp.conf_SA_loopFactor << endl;
		cout << "IO>  SA -- Outer-loop upper limit: " << fp.conf_SA_loopLimit << endl;
		cout << "IO>  SA -- Temperature-scaling factor for phase 1 (adaptive cooling): " << fp.conf_SA_temp_factor_phase1 << endl;
		cout << "IO>  SA -- Temperature-scaling factor for phase 2 (reheating and converging): " << fp.conf_SA_temp_factor_phase2 << endl;
		cout << "IO>  SA -- Cost factor for temperature: " << fp.conf_SA_cost_temp << endl;
		cout << "IO>  SA -- Cost factor for wirelength: " << fp.conf_SA_cost_WL << endl;
		cout << "IO>  SA -- Cost factor for TSVs: " << fp.conf_SA_cost_TSVs << endl;
		cout << "IO>  SA -- Cost factor for area and outline violation: " << fp.conf_SA_cost_area_outline << endl;
		cout << "IO>  Power blurring -- Impulse factor: " << fp.conf_power_blurring_impulse_factor << endl;
		cout << "IO>  Power blurring -- Impulse factor down-scaling exponent: " << fp.conf_power_blurring_impulse_factor_scaling_exponent << endl;
		cout << "IO>  Power blurring -- Mask-boundary value: " << fp.conf_power_blurring_mask_boundary_value << endl;
	}
}

void IO::parseCorblivarFile(FloorPlanner& fp, CorblivarCore& corb) {
	string tmpstr;
	map<int, Block*>::iterator b;
	Block* cur_block;
	int cur_layer;
	int block_id;
	unsigned dir;
	int juncts;
	double w, h;

	if (fp.logMed()) {
		cout << "Layout> ";
		cout << "Initializing Corblivar data from solution file ..." << endl;
	}

	// init dies data
	corb.initCorblivarDies(fp.conf_layer, fp.blocks.size());

	// drop solution file header
	while (tmpstr != "data_start" && !fp.solution_in.eof()) {
		fp.solution_in >> tmpstr;
	}

	while (!fp.solution_in.eof()) {
		fp.solution_in >> tmpstr;

		// new die; new CBL
		if (tmpstr == "CBL") {
			// drop "["
			fp.solution_in >> tmpstr;

			// layer id
			fp.solution_in >> cur_layer;

			// drop "]"
			fp.solution_in >> tmpstr;
		}
		// new CBL tuple; new block
		else if (tmpstr == "(") {
			// block id
			fp.solution_in >> block_id;
			// find related block
			b = fp.blocks.find(block_id);
			if (b != fp.blocks.end()) {
				cur_block = (*b).second;
			}
			else {
				cout << "Block " << block_id << " cannot be retrieved; ensure solution file and benchmark file match!" << endl;
				cur_block = nullptr;
			}
			// store block into S sequence
			corb.getDie(cur_layer)->getCBL().S_push_back(cur_block);

			// direction L
			fp.solution_in >> dir;
			// store direction into L sequence
			if (dir == 0) {
				corb.getDie(cur_layer)->getCBL().L_push_back(Direction::VERTICAL);
			}
			else {
				corb.getDie(cur_layer)->getCBL().L_push_back(Direction::HORIZONTAL);
			}

			// T-junctions
			fp.solution_in >> juncts;
			// store junctions into T sequence
			corb.getDie(cur_layer)->getCBL().T_push_back(juncts);

			// block width
			fp.solution_in >> w;
			// store width in block
			cur_block->bb.w = w;

			// block height
			fp.solution_in >> h;
			// store height in block
			cur_block->bb.h = h;

			// drop ")"
			fp.solution_in >> tmpstr;
			// drop ","
			fp.solution_in >> tmpstr;
		}
	}

	if (fp.logMed()) {
		cout << "Layout> ";
		cout << "Done" << endl << endl;
	}
}

// parse blocks file
void IO::parseBlocks(FloorPlanner& fp) {
	ifstream blocks_in, power_in;
	string tmpstr;
	Block* cur_block;
	double power = 0.0;
	double area = 0.0;
	int id;

	if (fp.logMed()) {
		cout << "IO> ";
		cout << "Parsing blocks..." << endl;
	}

	// open files
	blocks_in.open(fp.blocks_file.c_str());
	power_in.open(fp.power_density_file.c_str());

	// reset blocks
	fp.blocks.clear();

	// drop block files header
	while (tmpstr != "sb0" && !blocks_in.eof())
		blocks_in >> tmpstr;

	// parse blocks
	id = -1;
	while (!blocks_in.eof()) {

		// find line containing block
		while (tmpstr.find("sb") == string::npos && !blocks_in.eof()) {
			blocks_in >> tmpstr;
		}
		if (blocks_in.eof()) {
			break;
		}

		// init block
		id++;
		cur_block = new Block(id);

		// determine power density
		if (!power_in.eof()) {
			power_in >> cur_block->power_density;
			// GSRC benchmarks provide power density in 10^5 W/(m^2);
			// normalize to W/(um^2)
			cur_block->power_density *= 1.0e-7;
		}
		else {
			if (fp.logMin()) {
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
		cur_block->bb.w *= FloorPlanner::BLOCKS_SCALE_UP;
		cur_block->bb.h *= FloorPlanner::BLOCKS_SCALE_UP;

		// calculate block area
		cur_block->bb.area = cur_block->bb.w * cur_block->bb.h;
		area += cur_block->bb.area;

		// memorize total block power
		power += cur_block->power();

		// store block
		fp.blocks.insert( pair<int, Block*>(cur_block->id, cur_block) );
	}

	// close files
	blocks_in.close();
	power_in.close();

	// logging
	if (fp.logMed()) {
		cout << "IO> ";
		cout << "Done; " << fp.blocks.size() << " blocks read in" << endl;
		cout << "IO>  (blocks power: " << power << "; blocks area: " << area;
		cout << "; blocks area / total area: " << area / (fp.conf_layer * fp.conf_outline_x * fp.conf_outline_y) << ")" << endl;
		cout << endl;
	}

	// sanity check of fixed outline
	if (area / (fp.conf_layer * fp.conf_outline_x * fp.conf_outline_y) > 1.0) {
		cout << "IO> Outline too small; consider fixing the config file" << endl;
		exit(1);
	}
}

// parse nets file
void IO::parseNets(FloorPlanner& fp) {
	ifstream in;
	string tmpstr;
	Net* cur_net;
	int i, net_degree;
	int net_block_id;
	string net_block;
	map<int, Block*>::iterator b;
	int id;

	if (fp.logMed()) {
		cout << "IO> ";
		cout << "Parsing nets..." << endl;
	}

	// reset nets
	fp.nets.clear();

	// open nets file
	in.open(fp.nets_file.c_str());

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
				b = fp.blocks.find(net_block_id);
				if (b != fp.blocks.end()) {
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
				if (fp.logMin()) {
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
			fp.nets.push_back(cur_net);
		}
		else {
			delete(cur_net);
		}

		id++;
	}

	// close nets file
	in.close();

	if (IO::DBG) {
		for (Net* const& n : fp.nets) {
			cout << "DBG_IO> ";
			cout << "net " << n->id << endl;

			for (Block* const& b : n->blocks) {
				cout << "DBG_IO> ";
				cout << " block " << b->id << endl;
			}
		}
	}

	if (fp.logMed()) {
		cout << "IO> ";
		cout << "Done; " << fp.nets.size() << " nets read in" << endl << endl;
	}

}

void IO::writePowerThermalMaps(FloorPlanner const& fp) {
	ofstream gp_out;
	ofstream data_out;
	int cur_layer;
	int layer_limit;
	unsigned x, y;
	unsigned x_limit, y_limit;
	int flag;

	// sanity check
	if (fp.thermalAnalyzer.power_maps.empty() || fp.thermalAnalyzer.thermal_map.empty()) {
		return;
	}

	if (fp.logMed()) {
		cout << "IO> ";
		cout << "Generating power maps and thermal profiles ..." << endl;
	}

	// flag=0: generate power maps
	// flag=1: generate thermal map
	for (flag = 0; flag <= 1; flag++) {

		// power maps for all layers
		if (flag == 0) {
			layer_limit = fp.conf_layer;
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
				gp_out_name << fp.benchmark << "_" << cur_layer << "_power.gp";
				data_out_name << fp.benchmark << "_" << cur_layer << "_power.data";
			}
			else {
				gp_out_name << fp.benchmark << "_" << cur_layer << "_thermal.gp";
				data_out_name << fp.benchmark << "_" << cur_layer << "_thermal.data";
			}

			// init file stream for gnuplot script
			gp_out.open(gp_out_name.str().c_str());
			// init file stream for data file
			data_out.open(data_out_name.str().c_str());

			// file header for gnuplot script
			if (flag == 0) {
				gp_out << "set title \"" << fp.benchmark << " - Padded Power Map Layer " << cur_layer + 1 << "\"" << endl;
			}
			else {
				gp_out << "set title \"" << fp.benchmark << " - Thermal Map Layer " << cur_layer + 1 << "\"" << endl;
			}
			gp_out << "set terminal postscript color enhanced \"Times\" 20" << endl;
			gp_out << "set output \"" << gp_out_name.str() << ".eps\"" << endl;
			gp_out << "set size square" << endl;
			if (flag == 0) {
				gp_out << "set xrange [0:" << fp.thermalAnalyzer.power_maps[cur_layer].size() - 1 << "]" << endl;
				gp_out << "set yrange [0:" << fp.thermalAnalyzer.power_maps[cur_layer][0].size() - 1 << "]" << endl;
			}
			else {
				gp_out << "set xrange [0:" << fp.thermalAnalyzer.thermal_map.size() - 1 << "]" << endl;
				gp_out << "set yrange [0:" << fp.thermalAnalyzer.thermal_map[0].size() - 1 << "]" << endl;
			}
			gp_out << "set tics front" << endl;
			gp_out << "set grid xtics ytics ztics" << endl;
			// pm3d algorithm determines an average value for each pixel,
			// considering sourrounding pixels
			// skip this behaviour w/ ``corners2color''; c1 means to select
			// the lower-left value, practically loosing one row and column in
			// the overall plot
			// see also http://gnuplot.sourceforge.net/demo/pm3d.html
			gp_out << "set pm3d map corners2color c1" << endl;
			// color printable as gray
			gp_out << "set palette rgbformulae 30,31,32" << endl;
			gp_out << "splot \"" << data_out_name.str() << "\" using 1:2:3 notitle" << endl;

			// close file stream for gnuplot script
			gp_out.close();

			// file header for data file
			if (flag == 0) {
				data_out << "# X Y power" << endl;
			}
			else {
				data_out << "# X Y thermal" << endl;
			}

			// determine grid boundaries
			if (flag == 0) {
				x_limit = fp.thermalAnalyzer.power_maps[cur_layer].size();
				y_limit = fp.thermalAnalyzer.power_maps[cur_layer][0].size();
			}
			else {
				x_limit = fp.thermalAnalyzer.thermal_map.size();
				y_limit = fp.thermalAnalyzer.thermal_map[0].size();
			}

			// output grid values
			for (x = 0; x < x_limit; x++) {
				for (y = 0; y < y_limit; y++) {
					if (flag == 0) {
						data_out << x << "	" << y << "	" << fp.thermalAnalyzer.power_maps[cur_layer][x][y] << endl;
					}
					else {
						data_out << x << "	" << y << "	" << fp.thermalAnalyzer.thermal_map[x][y] << endl;
					}
				}
				data_out << endl;
			}

			// close file stream for data file
			data_out.close();
		}

	}

	if (fp.logMed()) {
		cout << "IO> ";
		cout << "Done" << endl << endl;
	}
}

void IO::writeTempSchedule(FloorPlanner const& fp) {
	ofstream gp_out;
	ofstream data_out;

	// sanity check
	if (fp.tempSchedule.empty()) {
		return;
	}

	if (fp.logMed()) {
		cout << "IO> ";
		cout << "Generating GP scripts for SA temperature-schedule ..." << endl;
	}

	// build up file names
	stringstream gp_out_name;
	stringstream data_out_name;
	gp_out_name << fp.benchmark << "_TempSchedule.gp";
	data_out_name << fp.benchmark << "_TempSchedule.data";

	// init file stream for gnuplot script
	gp_out.open(gp_out_name.str().c_str());
	// init file stream for data file
	data_out.open(data_out_name.str().c_str());

	// gp header
	gp_out << "set title \"" << fp.benchmark << " - SA Temperature Schedule \"" << endl;
	gp_out << "set output \"" << gp_out_name.str() << ".eps\"" << endl;

	// general settings for more attractive plots, extracted from
	// http://youinfinitesnake.blogspot.de/2011/02/attractive-scientific-plots-with.html
	gp_out << "set terminal pdfcairo font \"Gill Sans, 12\" linewidth 4 rounded" << endl;
	gp_out << "# Line style for axes" << endl;
	gp_out << "set style line 80 lt rgb \"#808080\"" << endl;
	gp_out << "# Line style for grid" << endl;
	gp_out << "set style line 81 lt 0  # dashed" << endl;
	gp_out << "set style line 81 lt rgb \"#808080\"  # grey" << endl;
	gp_out << "set grid back linestyle 81" << endl;
	gp_out << "# Remove border on top and right." << endl;
	gp_out << "# Also, put it in grey; no need for so much emphasis on a border." << endl;
	gp_out << "set border 3 back linestyle 80" << endl;
	gp_out << "set xtics nomirror" << endl;
	gp_out << "set ytics nomirror" << endl;
	gp_out << "# Line styles: try to pick pleasing colors, rather" << endl;
	gp_out << "# than strictly primary colors or hard-to-see colors" << endl;
	gp_out << "# like gnuplot's default yellow. Make the lines thick" << endl;
	gp_out << "# so they're easy to see in small plots in papers." << endl;
	gp_out << "set style line 1 lt rgb \"#A00000\" lw 2 pt 1" << endl;
	gp_out << "set style line 2 lt rgb \"#00A000\" lw 3 pt 6" << endl;
	gp_out << "#set style line 2 lt rgb \"#00A000\" lw 2 pt 6" << endl;
	gp_out << "set style line 3 lt rgb \"#5060D0\" lw 2 pt 2" << endl;
	gp_out << "set style line 4 lt rgb \"#F25900\" lw 2 pt 9" << endl;

	// specific settings: labels
	gp_out << "set xlabel \"SA Step\"" << endl;
	gp_out << "set ylabel \"SA Temperature\"" << endl;
	// specific settings: log scale
	gp_out << "set log y" << endl;
	gp_out << "set mytics 10" << endl;

	// gp data plot command
	gp_out << "plot \"" << data_out_name.str() << "\" index 0 using 1:2 notitle with linespoints linestyle 1, \\" << endl;
	gp_out << "\"" << data_out_name.str() << "\" index 1 using 1:2 notitle with points linestyle 2" << endl;

	// close file stream
	gp_out.close();

	// output data: SA step and SA temp
	data_out << "# Step Temperature (index 0)" << endl;
	for (FloorPlanner::TempStep step : fp.tempSchedule) {
		data_out << step.step << " " << step.temp << endl;
	}
	// two blank lines trigger gnuplot to interpret data file as separate data sets
	data_out << endl;
	data_out << endl;
	// output data: markers for best-solution steps
	data_out << "# Step Temperature (only steps w/ new best solutions, index 1)" << endl;
	for (FloorPlanner::TempStep step : fp.tempSchedule) {
		if (step.new_best_sol_found) {
			data_out << step.step << " " << step.temp << endl;
		}
	}

	// close file stream
	data_out.close();

	if (fp.logMed()) {
		cout << "IO> ";
		cout << "Done" << endl << endl;
	}
}

// generate GP plots of FP
void IO::writeFloorplanGP(FloorPlanner const& fp, string const& file_suffix) {
	ofstream gp_out;
	int cur_layer;
	int object_counter;
	map<int, Block*>::iterator b;
	Block const* cur_block;
	double ratio_inv;
	int tics;

	if (fp.logMed()) {
		cout << "IO> ";
		if (file_suffix != "")
			cout << "Generating GP scripts for floorplan (suffix \"" << file_suffix << "\")..." << endl;
		else
			cout << "Generating GP scripts for floorplan ..." << endl;
	}

	ratio_inv = fp.conf_outline_y / fp.conf_outline_x;
	tics = max(fp.conf_outline_x, fp.conf_outline_y) / 5;

	for (cur_layer = 0; cur_layer < fp.conf_layer; cur_layer++) {
		// build up file name
		stringstream out_name;
		out_name << fp.benchmark << "_" << cur_layer;
		if (file_suffix != "")
			out_name << "_" << file_suffix;
		out_name << ".gp";

		// init file stream
		gp_out.open(out_name.str().c_str());

		// file header
		gp_out << "set title \"" << fp.benchmark << " - Layer " << cur_layer + 1 << "\"" << endl;
		gp_out << "set terminal postscript color enhanced \"Times\" 20" << endl;
		gp_out << "set output \"" << out_name.str() << ".eps\"" << endl;
		gp_out << "set size ratio " << ratio_inv << endl;
		gp_out << "set xrange [0:" << fp.conf_outline_x << "]" << endl;
		gp_out << "set yrange [0:" << fp.conf_outline_y << "]" << endl;
		gp_out << "set xtics " << tics << endl;
		gp_out << "set ytics " << tics << endl;
		gp_out << "set mxtics 4" << endl;
		gp_out << "set mytics 4" << endl;
		gp_out << "set tics front" << endl;
		gp_out << "set grid xtics ytics mxtics mytics" << endl;

		// init obj counter
		object_counter = 1;

		// output blocks
		for (auto& b : fp.blocks) {
			cur_block = b.second;

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
			gp_out << " at " << cur_block->bb.ll.x + 2.0 * FloorPlanner::BLOCKS_SCALE_UP;
			gp_out << "," << cur_block->bb.ll.y + 5.0 * FloorPlanner::BLOCKS_SCALE_UP;
			gp_out << " font \"Times,6\"" << endl;
		}

		// file footer
		gp_out << "plot NaN notitle" << endl;

		// close file stream
		gp_out.close();
	}

	if (fp.logMed()) {
		cout << "IO> ";
		cout << "Done" << endl << endl;
	}
}

// generate files for HotSpot steady-state thermal simulation
void IO::writeHotSpotFiles(FloorPlanner const& fp) {
	ofstream file;
	map<int, Block*>::iterator b;
	Block const* cur_block;
	int cur_layer;
	// factor to scale um downto m;
	static constexpr double SCALE_UM_M = 0.000001;

	if (fp.logMed()) {
		cout << "IO> Generating files for HotSpot 3D-thermal simulation..." << endl;
	}

	/// generate floorplan files
	for (cur_layer = 0; cur_layer < fp.conf_layer; cur_layer++) {
		// build up file name
		stringstream fp_file;
		fp_file << fp.benchmark << "_HotSpot_" << cur_layer << ".flp";

		// init file stream
		file.open(fp_file.str().c_str());

		// file header
		file << "# Line Format: <unit-name>\\t<width>\\t<height>\\t<left-x>\\t<bottom-y>\\t<specific-heat>\\t<resistivity>" << endl;
		file << "# all dimensions are in meters" << endl;
		file << "# comment lines begin with a '#'" << endl;
		file << "# comments and empty lines are ignored" << endl;
		file << endl;

		// output blocks
		for (auto& b : fp.blocks) {
			cur_block = b.second;

			if (cur_block->layer != cur_layer) {
				continue;
			}

			file << "b" << cur_block->id;
			file << "	" << cur_block->bb.w * SCALE_UM_M;
			file << "	" << cur_block->bb.h * SCALE_UM_M;
			file << "	" << cur_block->bb.ll.x * SCALE_UM_M;
			file << "	" << cur_block->bb.ll.y * SCALE_UM_M;
			file << "	" << ThermalAnalyzer::HEAT_CAPACITY_SI;
			file << "	" << ThermalAnalyzer::THERMAL_RESISTIVITY_SI;
			file << endl;
		}

		// dummy block to describe layer outline
		file << "outline" << cur_layer;
		file << "	" << fp.conf_outline_x * SCALE_UM_M;
		file << "	" << fp.conf_outline_y * SCALE_UM_M;
		file << "	0.0";
		file << "	0.0";
		file << "	" << ThermalAnalyzer::HEAT_CAPACITY_SI;
		file << "	" << ThermalAnalyzer::THERMAL_RESISTIVITY_SI;
		file << endl;

		// close file stream
		file.close();
	}

	/// generate dummy floorplan for inactive Si layer
	//
	// build up file name
	stringstream Si_fp_file;
	Si_fp_file << fp.benchmark << "_HotSpot_Si.flp";

	// init file stream
	file.open(Si_fp_file.str().c_str());

	// file header
	file << "# Line Format: <unit-name>\\t<width>\\t<height>\\t<left-x>\\t<bottom-y>\\t<specific-heat>\\t<resistivity>" << endl;
	file << "# all dimensions are in meters" << endl;
	file << "# comment lines begin with a '#'" << endl;
	file << "# comments and empty lines are ignored" << endl;

	// inactive Si ``block''
	file << "Si";
	file << "	" << fp.conf_outline_x * SCALE_UM_M;
	file << "	" << fp.conf_outline_y * SCALE_UM_M;
	file << "	0.0";
	file << "	0.0";
	file << "	" << ThermalAnalyzer::HEAT_CAPACITY_SI;
	file << "	" << ThermalAnalyzer::THERMAL_RESISTIVITY_SI;
	file << endl;

	// close file stream
	file.close();

	/// generate dummy floorplan for BEOL layer
	//
	// build up file name
	stringstream BEOL_fp_file;
	BEOL_fp_file << fp.benchmark << "_HotSpot_BEOL.flp";

	// init file stream
	file.open(BEOL_fp_file.str().c_str());

	// file header
	file << "# Line Format: <unit-name>\\t<width>\\t<height>\\t<left-x>\\t<bottom-y>\\t<specific-heat>\\t<resistivity>" << endl;
	file << "# all dimensions are in meters" << endl;
	file << "# comment lines begin with a '#'" << endl;
	file << "# comments and empty lines are ignored" << endl;

	// BEOL ``block''
	file << "BEOL";
	file << "	" << fp.conf_outline_x * SCALE_UM_M;
	file << "	" << fp.conf_outline_y * SCALE_UM_M;
	file << "	0.0";
	file << "	0.0";
	file << "	" << ThermalAnalyzer::HEAT_CAPACITY_BEOL;
	file << "	" << ThermalAnalyzer::THERMAL_RESISTIVITY_BEOL;
	file << endl;

	// close file stream
	file.close();

	/// generate dummy floorplan for Bond layer
	//
	// build up file name
	stringstream Bond_fp_file;
	Bond_fp_file << fp.benchmark << "_HotSpot_Bond.flp";

	// init file stream
	file.open(Bond_fp_file.str().c_str());

	// file header
	file << "# Line Format: <unit-name>\\t<width>\\t<height>\\t<left-x>\\t<bottom-y>\\t<specific-heat>\\t<resistivity>" << endl;
	file << "# all dimensions are in meters" << endl;
	file << "# comment lines begin with a '#'" << endl;
	file << "# comments and empty lines are ignored" << endl;

	// Bond ``block''
	file << "Bond";
	file << "	" << fp.conf_outline_x * SCALE_UM_M;
	file << "	" << fp.conf_outline_y * SCALE_UM_M;
	file << "	0.0";
	file << "	0.0";
	file << "	" << ThermalAnalyzer::HEAT_CAPACITY_BOND;
	file << "	" << ThermalAnalyzer::THERMAL_RESISTIVITY_BOND;
	file << endl;

	// close file stream
	file.close();

	/// generate power-trace file
	//
	// build up file name
	stringstream power_file;
	power_file << fp.benchmark << "_HotSpot.ptrace";

	// init file stream
	file.open(power_file.str().c_str());

	// block sequence in trace file has to follow layer files, thus build up file
	// according to layer structure
	//
	// output block labels in first line
	for (cur_layer = 0; cur_layer < fp.conf_layer; cur_layer++) {

		for (auto& b : fp.blocks) {
			cur_block = b.second;

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
	for (cur_layer = 0; cur_layer < fp.conf_layer; cur_layer++) {

		for (auto& b : fp.blocks) {
			cur_block = b.second;

			if (cur_block->layer != cur_layer) {
				continue;
			}

			file << cur_block->power() << " ";
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
	stack_file << fp.benchmark << "_HotSpot.lcf";

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

	for (cur_layer = 0; cur_layer < fp.conf_layer; cur_layer++) {

		file << "# BEOL (interconnects) layer " << cur_layer << endl;
		file << 4 * cur_layer << endl;
		file << "Y" << endl;
		file << "N" << endl;
		file << ThermalAnalyzer::HEAT_CAPACITY_BEOL << endl;
		file << ThermalAnalyzer::THERMAL_RESISTIVITY_BEOL << endl;
		file << ThermalAnalyzer::THICKNESS_BEOL << endl;
		file << fp.benchmark << "_HotSpot_BEOL.flp" << endl;
		file << endl;

		file << "# Active Si layer; design layer " << cur_layer << endl;
		file << 4 * cur_layer + 1 << endl;
		file << "Y" << endl;
		file << "Y" << endl;
		file << ThermalAnalyzer::HEAT_CAPACITY_SI << endl;
		file << ThermalAnalyzer::THERMAL_RESISTIVITY_SI << endl;
		file << ThermalAnalyzer::THICKNESS_SI_ACTIVE << endl;
		file << fp.benchmark << "_HotSpot_" << cur_layer << ".flp" << endl;
		file << endl;

		file << "# Inactive Si layer " << cur_layer << endl;
		file << 4 * cur_layer + 2 << endl;
		file << "Y" << endl;
		file << "N" << endl;
		file << ThermalAnalyzer::HEAT_CAPACITY_SI << endl;
		file << ThermalAnalyzer::THERMAL_RESISTIVITY_SI << endl;
		file << ThermalAnalyzer::THICKNESS_SI << endl;
		file << fp.benchmark << "_HotSpot_Si.flp" << endl;
		file << endl;

		if (cur_layer < (fp.conf_layer - 1)) {
			file << "# Bond layer " << cur_layer << "; for F2B bonding to next die " << cur_layer + 1 << endl;
			file << 4 * cur_layer + 3 << endl;
			file << "Y" << endl;
			file << "N" << endl;
			file << ThermalAnalyzer::HEAT_CAPACITY_BOND << endl;
			file << ThermalAnalyzer::THERMAL_RESISTIVITY_BOND << endl;
			file << ThermalAnalyzer::THICKNESS_BOND << endl;
			file << fp.benchmark << "_HotSpot_Bond.flp" << endl;
			file << endl;
		}
	}

	// close file stream
	file.close();

	if (fp.logMed()) {
		cout << "IO> Done" << endl << endl;
	}
}
