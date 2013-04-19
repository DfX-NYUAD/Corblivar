/*
 * =====================================================================================
 *
 *    Description:  Corblivar IO handler
 *
 *         Author:  Johann Knechtel, johann.knechtel@ifte.de
 *        Company:  Institute of Electromechanical and Electronic Design, www.ifte.de
 *
 * =====================================================================================
 */

// own Corblivar header
#include "IO.hpp"
// required Corblivar headers
#include "FloorPlanner.hpp"
#include "ThermalAnalyzer.hpp"
#include "CornerBlockList.hpp"
#include "CorblivarCore.hpp"
#include "Net.hpp"
#include "Math.hpp"

// parse program parameter and config file
void IO::parseParameterConfig(FloorPlanner& fp, int const& argc, char** argv) {
	ifstream in;
	string config_file;
	stringstream results_file, solution_file;
	stringstream blocks_file;
	stringstream pins_file;
	stringstream power_density_file;
	stringstream nets_file;
	string tmpstr;

	// program parameter
	if (argc < 4) {
		cout << "IO> Usage: " << argv[0] << " benchmark_name config_file benchmarks_dir [solution_file]" << endl;
		cout << "IO> " << endl;
		cout << "IO> Expected config_file format: see provided Corblivar.conf" << endl;
		cout << "IO> Expected benchmarks: any in GSRC Bookshelf format" << endl;
		cout << "IO> Note: solution_file can be used to start tool w/ given Corblivar data" << endl;

		exit(1);
	}

	fp.benchmark = argv[1];
	config_file = argv[2];

	blocks_file << argv[3] << fp.benchmark << ".blocks";
	fp.blocks_file = blocks_file.str();

	pins_file << argv[3] << fp.benchmark << ".pl";
	fp.pins_file = pins_file.str();

	power_density_file << argv[3] << fp.benchmark << ".power";
	fp.power_density_file = power_density_file.str();

	nets_file << argv[3] << fp.benchmark << ".nets";
	fp.nets_file = nets_file.str();

	results_file << fp.benchmark << ".results";
	fp.results.open(results_file.str().c_str());

	// test files
	in.open(config_file.c_str());
	if (!in.good()) {
		cout << "IO> ";
		cout << "No such config file: " << config_file<< endl;
		exit(1);
	}
	in.close();

	in.open(fp.blocks_file.c_str());
	if (!in.good()) {
		cout << "IO> ";
		cout << "Blocks file missing: " << fp.blocks_file << endl;
		exit(1);
	}
	in.close();

	in.open(fp.pins_file.c_str());
	if (!in.good()) {
		cout << "IO> ";
		cout << "Pins file missing: " << fp.pins_file << endl;
		exit(1);
	}
	in.close();

	in.open(fp.power_density_file.c_str());
	// memorize file availability
	fp.power_density_file_avail = in.good();
	if (!in.good()) {
		cout << "IO> ";
		cout << "Note: power density file missing : " << fp.power_density_file << endl;
		cout << "IO> Thermal optimization cannot be performed; is deactivated." << endl;
		cout << endl;
	}
	in.close();

	in.open(fp.nets_file.c_str());
	if (!in.good()) {
		cout << "IO> ";
		cout << "Nets file missing: " << fp.nets_file << endl;
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

	// open config file
	in.open(config_file.c_str());

	// sanity check for file version
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;

	int file_version;
	in >> file_version;

	if (file_version != IO::CONFIG_VERSION) {
		cout << "IO> Wrong version of config file; required version is \"" << IO::CONFIG_VERSION << "\"; consider using matching config file!" << endl;
		exit(1);
	}

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
		cout << "IO> Corblivar only supports floorplanning on >= 2 layers!" << endl;
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
		cout << "IO> Provide positive, non-zero outline dimensions!" << endl;
		exit(1);
	}

	// determine aspect ratio and area
	fp.die_AR = fp.conf_outline_x / fp.conf_outline_y;
	fp.die_area = fp.conf_outline_x * fp.conf_outline_y;
	fp.stack_area = fp.die_area * fp.conf_layer;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.conf_blocks_scale;

	// sanity check for block scaling factor
	if (fp.conf_blocks_scale <= 0.0) {
		cout << "IO> Provide a positive, non-zero block scaling factor!" << endl;
		exit(1);
	}

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.conf_SA_layout_enhanced_hard_block_rotation;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.conf_SA_layout_packing;

	// sanity check for packing and block rotation
	if (fp.conf_SA_layout_enhanced_hard_block_rotation && fp.conf_SA_layout_packing) {
		cout << "IO> Activate only guided hard block rotation OR layout packing; both cannot be performed!" << endl;
		exit(1);
	}

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.conf_SA_layout_power_guided_block_swapping;

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
		cout << "IO> Provide positive, non-zero SA loop parameters!" << endl;
		exit(1);
	}

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.conf_SA_temp_factor_phase1;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.conf_SA_temp_factor_phase1_limit;

	// sanity check for dependent temperature-scaling factors
	if (fp.conf_SA_temp_factor_phase1 >= fp.conf_SA_temp_factor_phase1_limit) {
		cout << "IO> Initial cooling factor for SA phase 1 should be smaller than the related final factor!" << endl;
		exit(1);
	}

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.conf_SA_temp_factor_phase2;

	// sanity check for positive, non-zero parameters
	if (fp.conf_SA_temp_factor_phase1 <= 0.0 || fp.conf_SA_temp_factor_phase2 <= 0.0) {
		cout << "IO> Provide positive, non-zero SA cooling factors for phases 1 and 2!" << endl;
		exit(1);
	}

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.conf_SA_temp_factor_phase3;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.conf_SA_cost_thermal;

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
		cout << "IO> A cost factor > 0 is required for area and outline optimization!" << endl;
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
	if (fp.conf_power_blurring_impulse_factor <= 0.0) {
		cout << "IO> Provide a positive, non-zero power blurring impulse factor!" << endl;
		exit(1);
	}

	in.close();

	if (fp.logMin()) {
		cout << "IO> Config values:" << endl;

		// log
		cout << "IO>  Loglevel (1 to 3 for minimal, medium, maximal): " << fp.conf_log << endl;

		// 3D IC setup
		cout << "IO>  Chip -- Layers for 3D IC: " << fp.conf_layer << endl;
		cout << "IO>  Chip -- Fixed die outline (width, x-dimension) [um]: " << fp.conf_outline_x << endl;
		cout << "IO>  Chip -- Fixed die outline (height, y-dimension) [um]: " << fp.conf_outline_y << endl;
		cout << "IO>  Chip -- Block scaling factor: " << fp.conf_blocks_scale << endl;

		// layout generation options
		cout << "IO>  SA -- Layout generation; guided hard block rotation: " << fp.conf_SA_layout_enhanced_hard_block_rotation << endl;
		cout << "IO>  SA -- Layout generation; packing: " << fp.conf_SA_layout_packing << endl;
		// consider power-guided block swapping only if thermal optimization is on
		if (!fp.power_density_file_avail || fp.conf_SA_cost_thermal == 0.0) {
			fp.conf_SA_layout_power_guided_block_swapping = false;
		}
		cout << "IO>  SA -- Layout generation; power-guided block swapping: " << fp.conf_SA_layout_power_guided_block_swapping << endl;
		if (!fp.power_density_file_avail || fp.conf_SA_cost_thermal == 0.0) {
			cout << "IO>     Note: power-guided block swapping is ignored since thermal optimization is disabled" << endl;
		}

		// SA loop setup
		cout << "IO>  SA -- Inner-loop operation-count a (iterations = a * N^(4/3) for N blocks): " << fp.conf_SA_loopFactor << endl;
		cout << "IO>  SA -- Outer-loop upper limit: " << fp.conf_SA_loopLimit << endl;

		// SA cooling schedule
		cout << "IO>  SA -- Initial temperature-scaling factor for phase 1 (adaptive cooling): " << fp.conf_SA_temp_factor_phase1 << endl;
		cout << "IO>  SA -- Final temperature-scaling factor for phase 1 (adaptive cooling): " << fp.conf_SA_temp_factor_phase1_limit << endl;
		cout << "IO>  SA -- Temperature-scaling factor for phase 2 (reheating and freezing): " << fp.conf_SA_temp_factor_phase2 << endl;
		cout << "IO>  SA -- Temperature-scaling factor for phase 3 (brief reheating, escaping local minima) : " << fp.conf_SA_temp_factor_phase3 << endl;

		// SA cost factors
		if (fp.power_density_file_avail) {
			cout << "IO>  SA -- Cost factor for thermal distribution: " << fp.conf_SA_cost_thermal << endl;
		}
		cout << "IO>  SA -- Cost factor for wirelength: " << fp.conf_SA_cost_WL << endl;
		cout << "IO>  SA -- Cost factor for TSVs: " << fp.conf_SA_cost_TSVs << endl;
		cout << "IO>  SA -- Cost factor for area and outline violation: " << fp.conf_SA_cost_area_outline << endl;

		// power blurring parameters; for thermal analysis
		cout << "IO>  Power blurring -- Impulse factor: " << fp.conf_power_blurring_impulse_factor << endl;
		cout << "IO>  Power blurring -- Impulse factor down-scaling exponent: " << fp.conf_power_blurring_impulse_factor_scaling_exponent << endl;
		cout << "IO>  Power blurring -- Mask-boundary value: " << fp.conf_power_blurring_mask_boundary_value << endl;

		cout << endl;
	}
}

void IO::parseCorblivarFile(FloorPlanner& fp, CorblivarCore& corb) {
	string tmpstr;
	CornerBlockList::Tuple tuple;
	unsigned tuples;
	int cur_layer;
	string block_id;
	unsigned dir;

	if (fp.logMed()) {
		cout << "Layout> ";
		cout << "Initializing Corblivar data from solution file ..." << endl;
	}

	// drop solution file header
	while (tmpstr != "data_start" && !fp.solution_in.eof()) {
		fp.solution_in >> tmpstr;
	}

	tuples = 0;
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
		else if (tmpstr == "tuple") {
			// drop tuple id
			fp.solution_in >> tmpstr;
			// drop ":"
			fp.solution_in >> tmpstr;
			// drop "("
			fp.solution_in >> tmpstr;

			// block id
			fp.solution_in >> block_id;
			// find related block
			tuple.S = Block::findBlock(block_id, fp.blocks);
			if (tuple.S == nullptr) {
				cout << "IO> Block " << block_id << " cannot be retrieved; ensure solution file and benchmark file match!" << endl;
				exit(1);
			}

			// direction L
			fp.solution_in >> dir;
			// parse direction; unsigned 
			if (dir == static_cast<unsigned>(Direction::VERTICAL)) {
				tuple.L = Direction::VERTICAL;
			}
			else {
				tuple.L = Direction::HORIZONTAL;
			}

			// T-junctions
			fp.solution_in >> tuple.T;

			// block width
			fp.solution_in >> tuple.S->bb.w;

			// block height
			fp.solution_in >> tuple.S->bb.h;

			// drop ");"
			fp.solution_in >> tmpstr;

			// store successfully parsed tuple into CBL
			corb.editDie(cur_layer).editCBL().insert(move(tuple));
			tuples++;
		}
	}

	if (fp.logMed()) {
		cout << "Layout> ";
		cout << "Done; parsed " << tuples << " tuples" << endl << endl;
	}
}

// parse blocks file
void IO::parseBlocks(FloorPlanner& fp) {
	ifstream blocks_in, pins_in, power_in;
	string tmpstr;
	double power = 0.0;
	double max_area = 0.0;
	int soft_blocks = 0;
	double blocks_outline_ratio;
	string id;
	double pins_scale_x, pins_scale_y;

	if (fp.logMed()) {
		cout << "IO> ";
		cout << "Parsing blocks..." << endl;
	}

	// open files
	blocks_in.open(fp.blocks_file.c_str());
	pins_in.open(fp.pins_file.c_str());
	power_in.open(fp.power_density_file.c_str());

	// drop power density file header line
	if (fp.power_density_file_avail) {
		while (tmpstr != "end" && !power_in.eof())
			power_in >> tmpstr;
		// if we reached eof, there was no header line; reset the input stream
		if (power_in.eof()) {
			power_in.clear() ;
			power_in.seekg(0, power_in.beg);
		}
	}

	// reset blocks
	fp.blocks_area = 0.0;
	fp.blocks.clear();
	// reset terminals
	fp.terminals.clear();

	// reset blocks power statistics
	fp.blocks_power_density_stats.max = fp.blocks_power_density_stats.range = fp.blocks_power_density_stats.avg = 0.0;
	fp.blocks_power_density_stats.min = -1;

	// drop block files header
	while (tmpstr != "NumTerminals" && !blocks_in.eof())
		blocks_in >> tmpstr;
	// drop ":"
	blocks_in >> tmpstr;
	// drop terminals count
	blocks_in >> tmpstr;

	// parse blocks and pins
	while (!blocks_in.eof()) {

		// each line contains a block, two examples are below
		// bk1 hardrectilinear 4 (0, 0) (0, 133) (336, 133) (336, 0)
		// BLOCK_7 softrectangular 2464 0.33 3.0
		// VSS terminal

		// parse block identifier
		blocks_in >> id;

		// init block
		Block new_block = Block(id);

		// parse block type
		blocks_in >> tmpstr;

		// due to some empty lines at the end, we may have reached eof just now
		if (blocks_in.eof()) {
			break;
		}

		// terminal pins: store separately (as dummy blocks)
		if (tmpstr == "terminal") {

			// parse pins file for related coordinates
			while (tmpstr != id && !pins_in.eof()) {
				pins_in >> tmpstr;
			}

			// pin cannot be found; log
			if (pins_in.eof() && fp.logMin()) {
				cout << "IO>  Coordinates for pin \"" << id << "\" cannot be retrieved, consider checking the pins file!" << endl;
			}
			// initially, parse coordinates of found pin; they will be scaled
			// after parsing whole blocks file
			else {
				pins_in >> new_block.bb.ll.x;
				pins_in >> new_block.bb.ll.y;
			}

			// store pin (block)
			fp.terminals.push_back(new_block);

			// reset pins file stream for next search
			pins_in.clear() ;
			pins_in.seekg(0, pins_in.beg);

			// skip further block related handling
			continue;
		}
		// hard blocks: parse dimensions
		else if (tmpstr == "hardrectilinear") {

			// drop "4"
			blocks_in >> tmpstr;
			// drop "(0,"
			blocks_in >> tmpstr;
			// drop "0)"
			blocks_in >> tmpstr;
			// drop "(0,"
			blocks_in >> tmpstr;
			// drop "Y)"
			blocks_in >> tmpstr;
			// parse "(X,"
			blocks_in >> tmpstr;
			new_block.bb.w = atof(tmpstr.substr(1, tmpstr.size() - 2).c_str());
			// parse "Y)"
			blocks_in >> tmpstr;
			new_block.bb.h = atof(tmpstr.substr(0, tmpstr.size() - 1).c_str());
			// drop "(X,"
			blocks_in >> tmpstr;
			// drop "0)"
			blocks_in >> tmpstr;

			// scale up dimensions
			new_block.bb.w *= fp.conf_blocks_scale;
			new_block.bb.h *= fp.conf_blocks_scale;

			// calculate block area
			new_block.bb.area = new_block.bb.w * new_block.bb.h;
		}
		// soft blocks: parse area and AR range
		else if (tmpstr == "softrectangular") {

			// parse area, min AR, max AR
			blocks_in >> new_block.bb.area;
			blocks_in >> new_block.AR.min;
			blocks_in >> new_block.AR.max;

			// scale up blocks area
			new_block.bb.area *= pow(fp.conf_blocks_scale, 2);

			// init block dimensions randomly w/in AR range; note that
			// x^2 = AR * A
			new_block.bb.w = sqrt(Math::randF(new_block.AR.min, new_block.AR.max) * new_block.bb.area);
			new_block.bb.h = new_block.bb.area / new_block.bb.w;

			// mark block as soft
			new_block.soft = true;
			// memorize soft blocks count
			soft_blocks++;
		}
		// unknown block type
		else {
			cout << "IO>  Unknown block type: " << tmpstr << endl;
			cout << "IO>  Consider checking the benchmark format, should comply w/ GSRC Bookshelf" << endl;
			exit(1);
		}

		// determine power density
		if (fp.power_density_file_avail) {
			if (!power_in.eof()) {
				power_in >> new_block.power_density;
				// GSRC benchmarks provide power density in 10^5 W/m^2
				// (which equals 10^-1 uW/um^2); reduce by factor 10 in
				// order to limit power consumption reasonably
				new_block.power_density *= 1.0e-1;
			}
			else {
				if (fp.logMin()) {
					cout << "IO>  Some blocks have no power value assigned, consider checking the power density file!" << endl;
				}
			}
		}

		// track block power statistics
		power += new_block.power();
		fp.blocks_power_density_stats.max = max(fp.blocks_power_density_stats.max, new_block.power_density);
		if (fp.blocks_power_density_stats.min == -1) {
			fp.blocks_power_density_stats.min = new_block.power_density;
		}
		fp.blocks_power_density_stats.min = min(fp.blocks_power_density_stats.min, new_block.power_density);
		fp.blocks_power_density_stats.avg += new_block.power_density;

		// memorize summed blocks area and largest block, needs to fit into die
		fp.blocks_area += new_block.bb.area;
		max_area = max(max_area, new_block.bb.area);

		// store block
		fp.blocks.push_back(move(new_block));
	}

	// close files
	blocks_in.close();
	power_in.close();
	pins_in.close();

	// determine deadspace amount for whole stack, now that the occupied blocks area
	// is known
	fp.stack_deadspace = fp.stack_area - fp.blocks_area;

	// determine block power statistics
	fp.blocks_power_density_stats.avg /= fp.blocks.size();
	fp.blocks_power_density_stats.range = fp.blocks_power_density_stats.max - fp.blocks_power_density_stats.min;

	// scale terminal pins; first determine original pins outline
	pins_scale_x = pins_scale_y = 0.0;
	for (Block const& pin : fp.terminals) {
		pins_scale_x = max(pins_scale_x, pin.bb.ll.x);
		pins_scale_y = max(pins_scale_y, pin.bb.ll.y);
	}
	// scale terminal pins; scale pin coordinates according to die outline
	pins_scale_x = fp.conf_outline_x / pins_scale_x;
	pins_scale_y = fp.conf_outline_y / pins_scale_y;
	for (Block& pin : fp.terminals) {
		pin.bb.ll.x *= pins_scale_x;
		pin.bb.ll.y *= pins_scale_y;
		// also set upper right to same coordinates, thus pins are ``point''
		// blocks w/ zero area
		pin.bb.ur.x = pin.bb.ll.x;
		pin.bb.ur.y = pin.bb.ll.y;
	}

	// sanity check of fixed outline
	blocks_outline_ratio = fp.blocks_area / fp.stack_area;
	if (blocks_outline_ratio > 1.0) {
		cout << "IO>  Chip too small; consider increasing the die outline or layers count" << endl;
		cout << "IO>  Summed Blocks/dies area ratio: " << blocks_outline_ratio << endl;
		exit(1);
	}
	// sanity check for largest block
	if (max_area > fp.die_area) {
		cout << "IO>  Die outline too small; consider increasing it" << endl;
		cout << "IO>  Largest-block/die area ratio: " << max_area / fp.die_area << endl;
		exit(1);
	}

	// sanity check for parsed blocks
	if (fp.blocks.empty()) {
		cout << "IO>  No blocks parsed; consider checking the benchmark format, should comply w/ GSRC Bookshelf" << endl;
		exit(1);
	}

	// logging
	if (fp.logMed()) {
		cout << "IO> ";
		cout << "Done; " << fp.blocks.size() << " blocks read in, " << fp.terminals.size() << " terminal pins read in" << endl;
		cout << "IO>  Soft blocks: " << soft_blocks << ", hard blocks: " << fp.blocks.size() - soft_blocks << endl;
		cout << "IO>  Summed blocks power [W]: " << power;
		if (power != 0.0) {
			cout << "; min power: " << fp.blocks_power_density_stats.min << ", max power: " << fp.blocks_power_density_stats.max;
			cout << ", avg power: " << fp.blocks_power_density_stats.avg << endl;
		}
		else {
			cout << endl;
		}
		cout << "IO>  Summed blocks area [cm^2]: " << fp.blocks_area * 1.0e-8;
		cout << "; summed blocks area / summed dies area: " << blocks_outline_ratio << endl;
		cout << endl;
	}
}

// parse nets file
void IO::parseNets(FloorPlanner& fp) {
	ifstream in;
	string tmpstr;
	int i, net_degree;
	string net_block;
	Block const* b;
	int id;
	bool pin_not_found_block, pin_not_found_terminal;

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
		Net new_net = Net(id);

		// parse net degree
		//// NetDegree : 2
		while (tmpstr != "NetDegree" && !in.eof()) {
			in >> tmpstr;
		}

		// drop ":"
		in >> tmpstr;
		// parse net degree
		in >> net_degree;

		// due to some empty lines at the end, we may have reached eof just now
		if (in.eof()) {
			break;
		}

		// read in blocks and terminals of net
		new_net.blocks.clear();
		new_net.terminals.clear();
		for (i = 0; i < net_degree; i++) {

			// parse block id
			in >> net_block;

			// try to parse pin as terminal pin
			pin_not_found_terminal = false;
			b = Block::findBlock(net_block, fp.terminals);
			if (b != nullptr) {
				// mark net as net w/ external pin
				new_net.hasExternalPin = true;
				// store terminal
				new_net.terminals.push_back(move(b));
			}
			else {
				pin_not_found_terminal = true;
			}

			// try to parse pin as regular block pin
			pin_not_found_block = false;
			if (b == nullptr) {
				b = Block::findBlock(net_block, fp.blocks);
				if (b != nullptr) {
					// store block
					new_net.blocks.push_back(move(b));
				}
				else {
					pin_not_found_block = true;
				}
			}

			// drop "B"
			in >> tmpstr;

			// log pin parsing failure
			if (fp.logMin()) {
				if (pin_not_found_block && !pin_not_found_terminal) {
					cout << "IO>  Net " << id << "'s block pin \"" << net_block << "\"";
					cout << " cannot be retrieved; consider checking net / blocks file" << endl;
				}
				else if (pin_not_found_block && pin_not_found_terminal) {
					cout << "IO>  Net " << id << "'s terminal pin \"" << net_block << "\"";
					cout << " cannot be retrieved; consider checking net / blocks file" << endl;
				}
			}
		}

		// sanity check; store only nets connecting two or more blocks
		if (new_net.blocks.size() > 1) {
			fp.nets.push_back(move(new_net));
		}

		id++;
	}

	// close nets file
	in.close();

	if (IO::DBG) {
		for (Net const& n : fp.nets) {
			cout << "DBG_IO> ";
			cout << "net " << n.id << endl;

			for (Block const* b : n.blocks) {
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
	double max_power_density;

	// sanity check
	if (fp.thermalAnalyzer.power_maps.empty() || fp.thermalAnalyzer.thermal_map.empty()) {
		return;
	}

	if (fp.logMed()) {
		cout << "IO> ";
		cout << "Generating power maps and thermal profiles ..." << endl;
	}

	// for power maps: fixed scale for all layers to ease comparision, i.e. requires
	// to determine max blocks' power
	max_power_density = 0.0;
	for (Block const& block : fp.blocks) {
		max_power_density = max(max_power_density, block.power_density);
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
				gp_out_name << fp.benchmark << "_" << cur_layer + 1 << "_power.gp";
				data_out_name << fp.benchmark << "_" << cur_layer + 1 << "_power.data";
			}
			else {
				gp_out_name << fp.benchmark << "_" << cur_layer + 1 << "_thermal.gp";
				data_out_name << fp.benchmark << "_" << cur_layer + 1 << "_thermal.data";
			}

			// init file stream for gnuplot script
			gp_out.open(gp_out_name.str().c_str());
			// init file stream for data file
			data_out.open(data_out_name.str().c_str());

			// file header for gnuplot script
			if (flag == 0) {
				gp_out << "set title \"Padded Power Map - " << fp.benchmark << ", Layer " << cur_layer + 1 << "\"" << endl;
			}
			else {
				gp_out << "set title \"Thermal Map - " << fp.benchmark << ", Layer " << cur_layer + 1 << "\"" << endl;
			}

			gp_out << "set terminal pdfcairo enhanced font \"Gill Sans, 12\"" << endl;
			gp_out << "set output \"" << gp_out_name.str() << ".pdf\"" << endl;
			gp_out << "set size square" << endl;

			// different 2D ranges for power map and thermal map
			if (flag == 0) {
				gp_out << "set xrange [0:" << ThermalAnalyzer::power_maps_dim - 1 << "]" << endl;
				gp_out << "set yrange [0:" << ThermalAnalyzer::power_maps_dim - 1 << "]" << endl;
			}
			else {
				gp_out << "set xrange [0:" << ThermalAnalyzer::thermal_map_dim - 1 << "]" << endl;
				gp_out << "set yrange [0:" << ThermalAnalyzer::thermal_map_dim - 1 << "]" << endl;
			}

			// power maps: scale, label for cbrange
			if (flag == 0) {
				// fixed scale for all layers to ease comparision
				gp_out << "set cbrange [0:" << max_power_density << "]" << endl;
				// label for power density
				gp_out << "set cblabel \"Power Density [10^{-2} {/Symbol m}W/{/Symbol m}m^2]\"" << endl;
			}
			// thermal maps: label for cbrange
			else {
				// thermal estimation, correlates w/ power density
				gp_out << "set cblabel \"Thermal Estimate [{/Symbol a}K]\"" << endl;
			}

			// tics
			gp_out << "set tics front" << endl;
			gp_out << "set grid xtics ytics ztics" << endl;
			// pm3d algorithm determines an average value for each pixel,
			// considering sourrounding pixels
			// skip this behaviour w/ ``corners2color''; c1 means to select
			// the lower-left value, practically loosing one row and column in
			// the overall plot
			// see also http://gnuplot.sourceforge.net/demo/pm3d.html
			gp_out << "set pm3d map corners2color c1" << endl;
			//// color printable as gray
			//gp_out << "set palette rgbformulae 30,31,32" << endl;
			// mathlab color palette; see
			// http://www.gnuplotting.org/matlab-colorbar-with-gnuplot/
			gp_out << "set palette defined ( 0 \"#000090\",\\" << endl;
			gp_out << "1 \"#000fff\",\\" << endl;
			gp_out << "2 \"#0090ff\",\\" << endl;
			gp_out << "3 \"#0fffee\",\\" << endl;
			gp_out << "4 \"#90ff70\",\\" << endl;
			gp_out << "5 \"#ffee00\",\\" << endl;
			gp_out << "6 \"#ff7000\",\\" << endl;
			gp_out << "7 \"#ee0000\",\\" << endl;
			gp_out << "8 \"#7f0000\")" << endl;

			// for padded power maps: draw rectangle for unpadded core
			if (flag == 0 && ThermalAnalyzer::mask_dim_half > 0) {
				gp_out << "set obj 1 rect from ";
				gp_out << ThermalAnalyzer::mask_dim_half - 1 << ", " << ThermalAnalyzer::mask_dim_half - 1 << " to ";
				gp_out << ThermalAnalyzer::power_maps_dim - ThermalAnalyzer::mask_dim_half << ", ";
				gp_out << ThermalAnalyzer::power_maps_dim - ThermalAnalyzer::mask_dim_half << " ";
				gp_out << "front fillstyle empty border rgb \"white\" linewidth 3" << endl;
			}

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
				x_limit = ThermalAnalyzer::power_maps_dim;
				y_limit = ThermalAnalyzer::power_maps_dim;
			}
			else {
				x_limit = ThermalAnalyzer::thermal_map_dim;
				y_limit = ThermalAnalyzer::thermal_map_dim;
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
	bool valid_solutions = false;
	bool first_valid_sol = false;

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

	// output data: SA step and SA temp
	data_out << "# Step Temperature (index 0)" << endl;
	for (FloorPlanner::TempStep step : fp.tempSchedule) {
		data_out << step.step << " " << step.temp << endl;
		// memorize if valid solutions are given at all
		if (step.new_best_sol_found) {
			valid_solutions = true;
		}
	}

	// two blank lines trigger gnuplot to interpret data file as separate data sets
	data_out << endl;
	data_out << endl;

	// output data: markers for best-solution steps
	if (valid_solutions) {
		data_out << "# Step Temperature (only steps w/ new best solutions, index 1)" << endl;
		for (FloorPlanner::TempStep step : fp.tempSchedule) {
			if (step.new_best_sol_found) {
				data_out << step.step << " " << step.temp << endl;
			}
		}
	}

	// two blank lines trigger gnuplot to interpret data file as separate data sets
	data_out << endl;
	data_out << endl;

	// output data: SA step and avg costs phase 1; if no valid solutions are
	// available, this data is represented by index 1
	if (!valid_solutions) {
		data_out << "# Step Avg_Cost_Phase_1 (index 1)" << endl;
	}
	else {
		data_out << "# Step Avg_Cost_Phase_1 (index 2)" << endl;
	}
	// avg costs for SA phase 1
	for (FloorPlanner::TempStep step : fp.tempSchedule) {

		data_out << step.step << " " << step.avg_cost << endl;

		// output data until first valid solution is found
		if (step.new_best_sol_found) {
			break;
		}
	}

	// two blank lines trigger gnuplot to interpret data file as separate data sets
	data_out << endl;
	data_out << endl;

	// output data: SA step and avg costs phase 2; only available if some valid
	// solutions are given
	if (valid_solutions) {
		data_out << "# Step Avg_Cost_Phase_2 (index 3)" << endl;
		// avg costs for SA phase 2
		for (FloorPlanner::TempStep step : fp.tempSchedule) {

			// output data only after first solution is found
			if (first_valid_sol) {
				data_out << step.step << " " << step.avg_cost << endl;
			}

			if (step.new_best_sol_found) {
				first_valid_sol = true;
			}
		}
	}

	// close file stream
	data_out.close();

	// gp header
	gp_out << "set title \"Temperature and Cost Schedule - " << fp.benchmark << "\"" << endl;
	gp_out << "set output \"" << gp_out_name.str() << ".pdf\"" << endl;

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
	gp_out << "set style line 2 lt rgb \"#00A000\" lw 2 pt 6" << endl;
	gp_out << "set style line 3 lt rgb \"#5060D0\" lw 2 pt 2" << endl;
	gp_out << "set style line 4 lt rgb \"#F25900\" lw 2 pt 9" << endl;

	// specific settings: labels
	gp_out << "set xlabel \"SA Step\"" << endl;
	gp_out << "set ylabel \"SA Temperature\"" << endl;
	gp_out << "set y2label \"Avg Solution Cost\"" << endl;
	// specific settings: key, labels box
	gp_out << "set key box lt rgb \"#808080\" out bottom center" << endl;
	// specific settings: log scale
	gp_out << "set log y" << endl;
	gp_out << "set mytics 10" << endl;
	// second, indepentend log scale for cost values
	gp_out << "set y2tics nomirror" << endl;
	gp_out << "set log y2" << endl;
	gp_out << "set mytics 10" << endl;

	// gp data plot command
	gp_out << "plot \"" << data_out_name.str() << "\" index 0 using 1:2 title \"SA Temperature\" with lines linestyle 2, \\" << endl;
	// there may be no valid solutions, then only the costs for phase 1 are plotted
	// besides the temperature schedule
	if (!valid_solutions) {
		gp_out << "\"" << data_out_name.str() << "\" index 1";
		gp_out << " using 1:2 title \"Avg Cost\" with lines linestyle 3 axes x1y2" << endl;
	}
	// otherwise, we consider both cost and the best solutions data sets
	else {
		gp_out << "\"" << data_out_name.str() << "\" index 1 using 1:2 title \"New Best Solution\" with points linestyle 1, \\" << endl;
		gp_out << "\"" << data_out_name.str() << "\" index 2";
		gp_out << " using 1:2 title \"Avg Cost for SA Phase 1\" with lines linestyle 3 axes x1y2, \\" << endl;
		gp_out << "\"" << data_out_name.str() << "\" index 3";
		gp_out << " using 1:2 title \"Avg Cost for SA Phase 2\" with lines linestyle 4 axes x1y2" << endl;
	}

	// close file stream
	gp_out.close();

	if (fp.logMed()) {
		cout << "IO> ";
		cout << "Done" << endl << endl;
	}
}

// generate GP plots of FP
void IO::writeFloorplanGP(FloorPlanner const& fp, string const& file_suffix) {
	ofstream gp_out;
	int cur_layer;
	int block_id;
	double ratio_inv;
	int tics;

	if (fp.logMed()) {
		cout << "IO> ";
		if (file_suffix != "")
			cout << "Generating GP scripts for floorplan (suffix \"" << file_suffix << "\")..." << endl;
		else
			cout << "Generating GP scripts for floorplan ..." << endl;
	}

	ratio_inv = 1.0 / fp.die_AR;
	tics = max(fp.conf_outline_x, fp.conf_outline_y) / 5;
	block_id = 1;

	for (cur_layer = 0; cur_layer < fp.conf_layer; cur_layer++) {
		// build up file name
		stringstream out_name;
		out_name << fp.benchmark << "_" << cur_layer + 1;
		if (file_suffix != "")
			out_name << "_" << file_suffix;
		out_name << ".gp";

		// init file stream
		gp_out.open(out_name.str().c_str());

		// file header
		gp_out << "set title \"Floorplan - " << fp.benchmark << ", Layer " << cur_layer + 1 << "\"" << endl;
		gp_out << "set terminal pdfcairo enhanced font \"Gill Sans, 12\"" << endl;
		gp_out << "set output \"" << out_name.str() << ".pdf\"" << endl;
		gp_out << "set size ratio " << ratio_inv << endl;
		gp_out << "set xrange [0:" << fp.conf_outline_x << "]" << endl;
		gp_out << "set yrange [0:" << fp.conf_outline_y << "]" << endl;
		gp_out << "set xlabel \"Width [{/Symbol m}m]\"" << endl;
		gp_out << "set ylabel \"Height [{/Symbol m}m]\"" << endl;
		gp_out << "set xtics " << tics << endl;
		gp_out << "set ytics " << tics << endl;
		gp_out << "set mxtics 4" << endl;
		gp_out << "set mytics 4" << endl;
		gp_out << "set tics front" << endl;
		gp_out << "set grid xtics ytics mxtics mytics" << endl;

		// output blocks
		for (Block const& cur_block : fp.blocks) {

			if (cur_block.layer != cur_layer) {
				continue;
			}

			// GP requires numerical ids starting w/ q
			gp_out << "set obj " << block_id;
			block_id++;

			// block rectangles
			gp_out << " rect";
			gp_out << " from " << cur_block.bb.ll.x << "," << cur_block.bb.ll.y;
			gp_out << " to " << cur_block.bb.ur.x << "," << cur_block.bb.ur.y;
			gp_out << " fillcolor rgb \"#ac9d93\" fillstyle solid";
			gp_out << endl;

			// label
			gp_out << "set label \"" << cur_block.id << "\"";
			gp_out << " at " << cur_block.bb.ll.x + 0.01 * fp.conf_outline_x;
			gp_out << "," << cur_block.bb.ll.y + 0.01 * fp.conf_outline_y;
			gp_out << " font \"Gill Sans,4\"" << endl;
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
		fp_file << fp.benchmark << "_HotSpot_" << cur_layer + 1 << ".flp";

		// init file stream
		file.open(fp_file.str().c_str());

		// file header
		file << "# Line Format: <unit-name>\\t<width>\\t<height>\\t<left-x>\\t<bottom-y>\\t<specific-heat>\\t<resistivity>" << endl;
		file << "# all dimensions are in meters" << endl;
		file << "# comment lines begin with a '#'" << endl;
		file << "# comments and empty lines are ignored" << endl;
		file << endl;

		// output blocks
		for (Block const& cur_block : fp.blocks) {

			if (cur_block.layer != cur_layer) {
				continue;
			}

			file << cur_block.id;
			file << "	" << cur_block.bb.w * SCALE_UM_M;
			file << "	" << cur_block.bb.h * SCALE_UM_M;
			file << "	" << cur_block.bb.ll.x * SCALE_UM_M;
			file << "	" << cur_block.bb.ll.y * SCALE_UM_M;
			file << "	" << ThermalAnalyzer::HEAT_CAPACITY_SI;
			file << "	" << ThermalAnalyzer::THERMAL_RESISTIVITY_SI;
			file << endl;
		}

		// dummy block to describe layer outline
		file << "outline" << cur_layer + 1;
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

	/// generate dummy floorplan for passive Si layer
	//
	// build up file name
	stringstream Si_fp_file;
	Si_fp_file << fp.benchmark << "_HotSpot_Si_passive.flp";

	// init file stream
	file.open(Si_fp_file.str().c_str());

	// file header
	file << "# Line Format: <unit-name>\\t<width>\\t<height>\\t<left-x>\\t<bottom-y>\\t<specific-heat>\\t<resistivity>" << endl;
	file << "# all dimensions are in meters" << endl;
	file << "# comment lines begin with a '#'" << endl;
	file << "# comments and empty lines are ignored" << endl;

	// passive Si ``block''
	file << "Si_passive";
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

		for (Block const& cur_block : fp.blocks) {

			if (cur_block.layer != cur_layer) {
				continue;
			}

			file << cur_block.id << " ";
		}

		// dummy outline block
		file << "outline" << cur_layer + 1 << " ";
	}
	file << endl;

	// output block power in second line
	for (cur_layer = 0; cur_layer < fp.conf_layer; cur_layer++) {

		for (Block const& cur_block : fp.blocks) {

			if (cur_block.layer != cur_layer) {
				continue;
			}

			file << cur_block.power() << " ";
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
		file << FloorPlanner::THICKNESS_BEOL << endl;
		file << fp.benchmark << "_HotSpot_BEOL.flp" << endl;
		file << endl;

		file << "# Active Si layer; design layer " << cur_layer << endl;
		file << 4 * cur_layer + 1 << endl;
		file << "Y" << endl;
		file << "Y" << endl;
		file << ThermalAnalyzer::HEAT_CAPACITY_SI << endl;
		file << ThermalAnalyzer::THERMAL_RESISTIVITY_SI << endl;
		file << FloorPlanner::THICKNESS_SI_ACTIVE << endl;
		file << fp.benchmark << "_HotSpot_" << cur_layer + 1 << ".flp" << endl;
		file << endl;

		file << "# Passive Si layer " << cur_layer << endl;
		file << 4 * cur_layer + 2 << endl;
		file << "Y" << endl;
		file << "N" << endl;
		file << ThermalAnalyzer::HEAT_CAPACITY_SI << endl;
		file << ThermalAnalyzer::THERMAL_RESISTIVITY_SI << endl;
		file << FloorPlanner::THICKNESS_SI_PASSIVE << endl;
		file << fp.benchmark << "_HotSpot_Si_passive.flp" << endl;
		file << endl;

		if (cur_layer < (fp.conf_layer - 1)) {
			file << "# Bond layer " << cur_layer << "; for F2B bonding to next die " << cur_layer + 1 << endl;
			file << 4 * cur_layer + 3 << endl;
			file << "Y" << endl;
			file << "N" << endl;
			file << ThermalAnalyzer::HEAT_CAPACITY_BOND << endl;
			file << ThermalAnalyzer::THERMAL_RESISTIVITY_BOND << endl;
			file << FloorPlanner::THICKNESS_BOND << endl;
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
