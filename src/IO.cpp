/*
 * =====================================================================================
 *
 *    Description:  Corblivar IO handler
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

// own Corblivar header
#include "IO.hpp"
// required Corblivar headers
#include "FloorPlanner.hpp"
#include "Chip.hpp"
#include "ThermalAnalyzer.hpp"
#include "CornerBlockList.hpp"
#include "CorblivarCore.hpp"
#include "CorblivarAlignmentReq.hpp"
#include "Net.hpp"
#include "Math.hpp"

// memory allocation
IO::Mode IO::mode;

// parse program parameter, config file, and further files
void IO::parseParametersFiles(FloorPlanner& fp, int const& argc, char** argv) {
	ifstream in;
	string config_file;
	stringstream results_file, solution_file;
	stringstream blocks_file;
	stringstream alignments_file;
	stringstream pins_file;
	stringstream power_density_file;
	stringstream nets_file;
	string tmpstr;
	ThermalAnalyzer::MaskParameters mask_parameters;

	// program parameters; two modes, one for regular Corblivar runs, one for for
	// thermal-analysis parameterization runs
	if (IO::mode == IO::Mode::REGULAR) {
		if (argc < 4) {
			cout << "IO> Usage: " << argv[0] << " benchmark_name config_file benchmarks_dir [solution_file]" << endl;
			cout << "IO> " << endl;
			cout << "IO> Expected config_file format: see provided Corblivar.conf" << endl;
			cout << "IO> Expected benchmarks: any in GSRC Bookshelf format" << endl;
			cout << "IO> Note: solution_file can be used to start tool w/ given Corblivar data" << endl;

			exit(1);
		}
	}
	else if (IO::mode == IO::Mode::THERMAL_ANALYSIS) {
		if (argc < 6) {
			cout << "IO> Usage: " << argv[0] << " benchmark_name config_file benchmarks_dir solution_file TSV_density" << endl;
			cout << "IO> " << endl;
			cout << "IO> Expected config_file format: see provided Corblivar.conf" << endl;
			cout << "IO> Expected benchmarks: any in GSRC Bookshelf format" << endl;
			cout << "IO> Expected solution_file: any in Corblivar format" << endl;
			cout << "IO> Expected TSV density: average TSV density for whole chip, to be given in \%" << endl;

			exit(1);
		}
	}

	fp.benchmark = argv[1];
	config_file = argv[2];

	blocks_file << argv[3] << fp.benchmark << ".blocks";
	fp.blocks_file = blocks_file.str();

	alignments_file << argv[3] << fp.benchmark << ".alr";
	fp.alignments_file = alignments_file.str();

	pins_file << argv[3] << fp.benchmark << ".pl";
	fp.pins_file = pins_file.str();

	power_density_file << argv[3] << fp.benchmark << ".power";
	fp.power_density_file = power_density_file.str();

	nets_file << argv[3] << fp.benchmark << ".nets";
	fp.nets_file = nets_file.str();

	results_file << fp.benchmark << ".results";
	fp.results.open(results_file.str().c_str());

	// assume minimal log level; actual level to be parsed later on
	fp.conf_log = FloorPlanner::LOG_MINIMAL;

	// test files
	//
	// config file
	in.open(config_file.c_str());
	if (!in.good()) {
		cout << "IO> ";
		cout << "No such config file: " << config_file << endl;
		exit(1);
	}
	in.close();

	// blocks file
	in.open(fp.blocks_file.c_str());
	if (!in.good()) {
		cout << "IO> ";
		cout << "Blocks file missing: " << fp.blocks_file << endl;
		exit(1);
	}
	in.close();

	// alignments file; only considered for regular runs
	if (IO::mode == IO::Mode::REGULAR) {

		in.open(fp.alignments_file.c_str());
		// memorize file availability
		fp.alignments_file_avail = in.good();

		if (!in.good() && fp.logMin()) {
			cout << "IO> ";
			cout << "Note: alignment-requests file missing : " << fp.alignments_file<< endl;
			cout << "IO> Block alignment cannot be performed; is deactivated." << endl;
			cout << endl;
		}

		in.close();
	}
	else {
		fp.alignments_file_avail = false;
	}

	// pins file
	in.open(fp.pins_file.c_str());
	if (!in.good()) {
		cout << "IO> ";
		cout << "Pins file missing: " << fp.pins_file << endl;
		exit(1);
	}
	in.close();

	// power file
	in.open(fp.power_density_file.c_str());
	// memorize file availability
	fp.power_density_file_avail = in.good();
	if (!in.good()) {
		cout << "IO> ";
		cout << "Note: power density file missing : " << fp.power_density_file << endl;

		// for thermal analysis, the power file is required
		if (IO::mode == IO::Mode::THERMAL_ANALYSIS) {
			exit(1);
		}
		else if (IO::mode == IO::Mode::REGULAR) {
			cout << "IO> Thermal optimization cannot be performed; is deactivated." << endl;
			cout << endl;
		}
	}
	in.close();

	// nets file
	in.open(fp.nets_file.c_str());
	if (!in.good()) {
		cout << "IO> ";
		cout << "Nets file missing: " << fp.nets_file << endl;
		exit(1);
	}
	in.close();

	// additional command-line parameters
	//
	// additional parameter for solution file given; consider file for readin
	if (argc > 4) {

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

	// for thermal-analysis parameterization runs; additional parameter for TSV
	// density, in percent, should be given (parameter count checked above)
	if (IO::mode == IO::Mode::THERMAL_ANALYSIS) {

		mask_parameters.TSV_density = atof(argv[5]);
	}
	// for non-thermal-analysis runs, assume that parameters refer to setup w/o TSVs
	else {
		mask_parameters.TSV_density = 0.0;
	}

	// config file parsing
	//
	in.open(config_file.c_str());

	if (fp.logMin()) {
		cout << "IO> Parsing config file ..." << endl;
	}

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

	// sanity check for positive, non-zero layer
	if (fp.conf_layer <= 0) {
		cout << "IO> Provide positive, non-zero layer count!" << endl;
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
	in >> fp.conf_outline_shrink;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.conf_SA_layout_enhanced_hard_block_rotation;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.conf_SA_layout_enhanced_soft_block_shaping;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.conf_SA_layout_packing_iterations;

	// sanity check for packing iterations
	if (fp.conf_SA_layout_packing_iterations < 0) {
		cout << "IO> Provide a positive packing iterations count or set 0 to disable!" << endl;
		exit(1);
	}

	// sanity check for packing and block rotation
	if (fp.conf_SA_layout_enhanced_hard_block_rotation && (fp.conf_SA_layout_packing_iterations > 0)) {
		cout << "IO> Activate only guided hard block rotation OR layout packing; both cannot be performed!" << endl;
		exit(1);
	}

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.conf_SA_layout_power_aware_block_handling;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.conf_SA_layout_floorplacement;

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
	in >> fp.conf_SA_temp_init_factor;

	// sanity check for positive, non-zero factor
	if (fp.conf_SA_temp_init_factor <= 0.0) {
		cout << "IO> Provide positive, non-zero SA start temperature scaling factor!" << endl;
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

	// memorize if thermal optimization should be performed
	fp.conf_SA_opt_thermal = (fp.conf_SA_cost_thermal > 0.0 && fp.power_density_file_avail);

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.conf_SA_cost_WL;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.conf_SA_cost_TSVs;

	// memorize if interconnects optimization should be performed
	fp.conf_SA_opt_interconnects = (fp.conf_SA_cost_WL > 0.0 || fp.conf_SA_cost_TSVs > 0.0);

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.conf_SA_cost_alignment;

	// memorize if alignment optimization should be performed
	fp.conf_SA_opt_alignment = (fp.conf_SA_cost_alignment > 0.0 && fp.alignments_file_avail);

	// sanity check for positive cost factors
	if (fp.conf_SA_cost_thermal < 0.0 || fp.conf_SA_cost_WL < 0.0 || fp.conf_SA_cost_TSVs < 0.0 || fp.conf_SA_cost_alignment < 0.0) {
		cout << "IO> Provide positive cost factors!" << endl;
		exit(1);
	}

	// sanity check for sum of cost factors
	if (abs(fp.conf_SA_cost_thermal + fp.conf_SA_cost_WL + fp.conf_SA_cost_TSVs + fp.conf_SA_cost_alignment - 1.0) > 0.1) {
		cout << "IO> Cost factors should sum up to approx. 1!" << endl;
		exit(1);
	}

	// thermal-analysis parameters
	//
	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> mask_parameters.impulse_factor;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> mask_parameters.impulse_factor_scaling_exponent;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> mask_parameters.mask_boundary_value;

	// sanity check for positive, non-zero parameters
	if (mask_parameters.impulse_factor <= 0.0) {
		cout << "IO> Provide a positive, non-zero power blurring impulse factor!" << endl;
		exit(1);
	}
	if (mask_parameters.mask_boundary_value <= 0.0) {
		cout << "IO> Provide a positive, non-zero power blurring mask boundary value!" << endl;
		exit(1);
	}

	// sanity check for reasonable mask parameters
	if (mask_parameters.impulse_factor <= mask_parameters.mask_boundary_value) {
		cout << "IO> Provide a power blurring impulse factor larger than the power blurring mask boundary value!" << endl;
		exit(1);
	}

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> mask_parameters.power_density_scaling_padding_zone;

	// sanity check for positive parameter
	if (mask_parameters.power_density_scaling_padding_zone < 1.0) {
		cout << "IO> Provide a positive (greater or equal 1.0) power-density scaling factor!" << endl;
		exit(1);
	}

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> mask_parameters.power_density_scaling_TSV_region;

	// sanity check for parameter range
	if (mask_parameters.power_density_scaling_TSV_region > 1.0 || mask_parameters.power_density_scaling_TSV_region < 0.0) {
		cout << "IO> Provide a power-density down-scaling factor for TSV regions between 0.0 and 1.0!" << endl;
		exit(1);
	}

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> mask_parameters.temp_offset;

	// sanity check for positive parameter
	if (mask_parameters.temp_offset < 0.0) {
		cout << "IO> Provide a positive temperature offset!" << endl;
		exit(1);
	}

	// store power-blurring parameters
	fp.conf_power_blurring_parameters = mask_parameters;

	in.close();

	if (fp.logMin()) {
		cout << "IO> Done; config values:" << endl;

		// log
		cout << "IO>  Loglevel (1 to 3 for minimal, medium, maximal): " << fp.conf_log << endl;

		// 3D IC setup
		cout << "IO>  Chip -- Layers for 3D IC: " << fp.conf_layer << endl;
		cout << "IO>  Chip -- Fixed die outline (width, x-dimension) [um]: " << fp.conf_outline_x << endl;
		cout << "IO>  Chip -- Fixed die outline (height, y-dimension) [um]: " << fp.conf_outline_y << endl;
		cout << "IO>  Chip -- Block scaling factor: " << fp.conf_blocks_scale << endl;
		cout << "IO>  Chip -- Final die outline shrink: " << fp.conf_outline_shrink << endl;

		// layout generation options
		cout << "IO>  SA -- Layout generation; guided hard block rotation: " << fp.conf_SA_layout_enhanced_hard_block_rotation << endl;
		cout << "IO>  SA -- Layout generation; guided soft block shaping: " << fp.conf_SA_layout_enhanced_soft_block_shaping << endl;
		cout << "IO>  SA -- Layout generation; packing iterations: " << fp.conf_SA_layout_packing_iterations << endl;
		cout << "IO>  SA -- Layout generation; power-aware block handling: " << fp.conf_SA_layout_power_aware_block_handling << endl;
		cout << "IO>  SA -- Layout generation; floorplacement handling: " << fp.conf_SA_layout_floorplacement << endl;

		// SA loop setup
		cout << "IO>  SA -- Inner-loop operation-factor a (ops = N^a for N blocks): " << fp.conf_SA_loopFactor << endl;
		cout << "IO>  SA -- Outer-loop upper limit: " << fp.conf_SA_loopLimit << endl;

		// SA cooling schedule
		cout << "IO>  SA -- Start temperature scaling factor: " << fp.conf_SA_temp_init_factor << endl;
		cout << "IO>  SA -- Initial temperature-scaling factor for phase 1 (adaptive cooling): " << fp.conf_SA_temp_factor_phase1 << endl;
		cout << "IO>  SA -- Final temperature-scaling factor for phase 1 (adaptive cooling): " << fp.conf_SA_temp_factor_phase1_limit << endl;
		cout << "IO>  SA -- Temperature-scaling factor for phase 2 (reheating and freezing): " << fp.conf_SA_temp_factor_phase2 << endl;
		cout << "IO>  SA -- Temperature-scaling factor for phase 3 (brief reheating, escaping local minima) : " << fp.conf_SA_temp_factor_phase3 << endl;

		// SA cost factors
		cout << "IO>  SA -- Cost factor for thermal distribution: " << fp.conf_SA_cost_thermal << endl;
		if (!fp.power_density_file_avail) {
			cout << "IO>     Note: thermal optimization is disabled since no power density file is available" << endl;
		}
		cout << "IO>  SA -- Cost factor for wirelength: " << fp.conf_SA_cost_WL << endl;
		cout << "IO>  SA -- Cost factor for TSVs: " << fp.conf_SA_cost_TSVs << endl;
		cout << "IO>  SA -- Cost factor for block alignment: " << fp.conf_SA_cost_alignment << endl;
		if (!fp.alignments_file_avail) {
			cout << "IO>     Note: block alignment is disabled since no alignment-requests file is available" << endl;
		}

		// power blurring mask parameters
		cout << "IO>  Power-blurring mask parameterization -- TSV density: " << mask_parameters.TSV_density << endl;
		cout << "IO>  Power-blurring mask parameterization -- Impulse factor: " << mask_parameters.impulse_factor << endl;
		cout << "IO>  Power-blurring mask parameterization -- Impulse scaling-factor: " << mask_parameters.impulse_factor_scaling_exponent << endl;
		cout << "IO>  Power-blurring mask parameterization -- Mask-boundary value: " << mask_parameters.mask_boundary_value << endl;
		cout << "IO>  Power-blurring mask parameterization -- Power-density scaling factor (padding zone): " << mask_parameters.power_density_scaling_padding_zone << endl;
		cout << "IO>  Power-blurring mask parameterization -- Power-density down-scaling factor (TSV regions): " << mask_parameters.power_density_scaling_TSV_region << endl;
		cout << "IO>  Power-blurring mask parameterization -- Temperature offset: " << mask_parameters.temp_offset << endl;

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
		cout << "IO> ";
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

			// memorize layer in block itself
			tuple.S->layer = cur_layer;

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
		cout << "IO> ";
		cout << "Done; parsed " << tuples << " tuples" << endl << endl;
	}
}

// parse alignment-requests file
void IO::parseAlignmentRequests(FloorPlanner& fp, vector<CorblivarAlignmentReq>& alignments) {
	ifstream al_in;
	string tmpstr;
	int tuple;
	string block_id;
	Block const* b1;
	Block const* b2;
	string type;
	CorblivarAlignmentReq::Type type_x;
	CorblivarAlignmentReq::Type type_y;
	double offset_range_x;
	double offset_range_y;

	// sanity check for unavailable file
	if (!fp.alignments_file_avail) {
		return;
	}

	if (fp.logMed()) {
		cout << "IO> ";
		cout << "Parsing alignment requests..." << endl;
	}

	// open file
	al_in.open(fp.alignments_file.c_str());

	// reset alignments
	alignments.clear();

	// drop file header
	while (tmpstr != "data_start" && !al_in.eof()) {
		al_in >> tmpstr;
	}

	// parse alignment tuples
	// e.g.
	// ( sb1 sb5 RANGE 10.0 RANGE_MAX 1000.0 )
	tuple = 0;
	while (!al_in.eof()) {

		// drop "("
		al_in >> tmpstr;

		// due to some empty lines at the end, we may have reached eof just now
		if (al_in.eof()) {
			break;
		}

		// block 1 id
		al_in >> block_id;

		// find related block
		b1 = Block::findBlock(block_id, fp.blocks);
		// no parsed block found
		if (b1 == nullptr) {

			// check for dummy reference block
			if (block_id == "RBOD") {
				// link dummy block to alignment request
				b1 = &fp.RBOD;
			}
			// otherwise, we triggered some parsing error
			else {
				cout << "IO> Block " << block_id << " cannot be retrieved; ensure alignment-requests file and benchmark file match!" << endl;
				exit(1);
			}
		}

		// block 2 id
		al_in >> block_id;

		// find related block
		b2 = Block::findBlock(block_id, fp.blocks);
		// no parsed block found
		if (b2 == nullptr) {

			// check for dummy reference block
			if (block_id == "RBOD") {
				// link dummy block to alignment request
				b2 = &fp.RBOD;
			}
			// otherwise, we triggered some parsing error
			else {
				cout << "IO> Block " << block_id << " cannot be retrieved; ensure alignment-requests file and benchmark file match!" << endl;
				exit(1);
			}
		}

		// alignment type for x-dimension
		al_in >> type;

		if (type == "RANGE") {
			type_x = CorblivarAlignmentReq::Type::RANGE;
		}
		else if (type == "RANGE_MAX") {
			type_x = CorblivarAlignmentReq::Type::RANGE_MAX;
		}
		else if (type == "OFFSET") {
			type_x = CorblivarAlignmentReq::Type::OFFSET;
		}
		else if (type == "UNDEF") {
			type_x = CorblivarAlignmentReq::Type::UNDEF;
		}
		else {
			cout << "IO> Unknown alignment-request type: " << type << "; ensure alignment-requests file has correct format!" << endl;
			exit(1);
		}

		// alignment range/offset for x-dimension
		al_in >> offset_range_x;

		// alignment type for y-dimension
		al_in >> type;

		if (type == "RANGE") {
			type_y = CorblivarAlignmentReq::Type::RANGE;
		}
		else if (type == "RANGE_MAX") {
			type_y = CorblivarAlignmentReq::Type::RANGE_MAX;
		}
		else if (type == "OFFSET") {
			type_y = CorblivarAlignmentReq::Type::OFFSET;
		}
		else if (type == "UNDEF") {
			type_y = CorblivarAlignmentReq::Type::UNDEF;
		}
		else {
			cout << "IO> Unknown alignment-request type: " << type << "; ensure alignment-requests file has correct format!" << endl;
			exit(1);
		}

		// alignment range/offset for y-dimension
		al_in >> offset_range_y;

		// drop ");"
		al_in >> tmpstr;

		// generate and store successfully parsed request
		alignments.push_back(CorblivarAlignmentReq(tuple, b1, b2, type_x, offset_range_x, type_y, offset_range_y));

		tuple++;
	}

	if (IO::DBG) {
		for (CorblivarAlignmentReq const& req : alignments) {
			cout << "DBG_IO> " << req.tupleString() << endl;
		}
	}

	if (fp.logMed()) {
		cout << "IO> ";
		cout << "Done; parsed " << tuple << " alignment requests" << endl << endl;
	}
}

// parse blocks file
void IO::parseBlocks(FloorPlanner& fp) {
	ifstream blocks_in, pins_in, power_in;
	string tmpstr;
	double power = 0.0;
	double blocks_max_area = 0.0, blocks_avg_area = 0.0;
	int soft_blocks = 0;
	double blocks_outline_ratio;
	string id;
	unsigned to_parse_soft_blocks, to_parse_hard_blocks, to_parse_terminals;
	bool floorplacement;

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
	while (tmpstr != "NumSoftRectangularBlocks" && !blocks_in.eof())
		blocks_in >> tmpstr;
	// drop ":"
	blocks_in >> tmpstr;
	// memorize how many soft blocks to be parsed
	blocks_in >> to_parse_soft_blocks;
	// drop "NumHardRectilinearBlocks" and ":"
	blocks_in >> tmpstr;
	blocks_in >> tmpstr;
	// memorize how many hard blocks to be parsed
	blocks_in >> to_parse_hard_blocks;
	// drop "NumTerminals" and ":"
	blocks_in >> tmpstr;
	blocks_in >> tmpstr;
	// memorize how many terminal pins to be parsed
	blocks_in >> to_parse_terminals;

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

			// init block dimensions randomly
			new_block.shapeRandomlyByAR();
			// mark block as soft
			new_block.soft = true;

			// memorize soft blocks count
			soft_blocks++;
		}
		// due to some blank lines at the end, we may have reached eof just now
		else if (blocks_in.eof()) {
			break;
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
				// TODO drop fixed down-scaling; instead modify files
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
		blocks_max_area = max(blocks_max_area, new_block.bb.area);

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

	// determine if floorplacement case, i.e., some very large blocks exist
	blocks_avg_area = fp.blocks_area / fp.blocks.size();
	floorplacement = false;
	for (Block& block : fp.blocks) {

		if (block.bb.area >= FloorPlanner::FP_AREA_RATIO_LIMIT * blocks_avg_area) {

			floorplacement = true;
			// also mark block as floorplacement instance
			block.floorplacement = true;
		}
	}
	// update config parameter, i.e., deactivated floorplacement if not required
	fp.conf_SA_layout_floorplacement &= floorplacement;

	// determine block power statistics
	fp.blocks_power_density_stats.avg /= fp.blocks.size();
	fp.blocks_power_density_stats.range = fp.blocks_power_density_stats.max - fp.blocks_power_density_stats.min;

	// scale terminal pins
	fp.scaleTerminalPins();

	// sanity check of fixed outline
	blocks_outline_ratio = fp.blocks_area / fp.stack_area;
	if (blocks_outline_ratio > 1.0) {
		cout << "IO>  Chip too small; consider increasing the die outline or layers count" << endl;
		cout << "IO>  Summed Blocks/dies area ratio: " << blocks_outline_ratio << endl;
		exit(1);
	}
	// sanity check for largest block
	if (blocks_max_area > fp.die_area) {
		cout << "IO>  Die outline too small; consider increasing it" << endl;
		cout << "IO>  Largest-block/die area ratio: " << blocks_max_area / fp.die_area << endl;
		exit(1);
	}

	// sanity check for parsed blocks
	if (fp.blocks.size() != (to_parse_soft_blocks + to_parse_hard_blocks)) {
		cout << "IO>  Not all given blocks could be parsed; consider checking the benchmark format, should comply w/ GSRC Bookshelf" << endl;
		cout << "IO>   Parsed hard blocks: " << fp.blocks.size() - soft_blocks << ", expected hard blocks count: " << to_parse_hard_blocks << endl;
		exit(1);
	}

	// sanity check for parsed terminals
	if (fp.terminals.size() != to_parse_terminals) {
		cout << "IO>  Not all given terminals could be parsed; consider checking the benchmark format, should comply w/ GSRC Bookshelf" << endl;
		cout << "IO>   Parsed pins: " << fp.terminals.size() << ", expected pins count: " << to_parse_terminals << endl;
		exit(1);
	}

	// logging
	if (fp.logMed()) {
		cout << "IO> ";
		cout << "Done; " << fp.blocks.size() << " blocks read in, " << fp.terminals.size() << " terminal pins read in" << endl;
		cout << "IO>  Soft blocks: " << soft_blocks << ", hard blocks: " << fp.blocks.size() - soft_blocks << endl;

		// floorplacement
		cout << "IO>  Largest-block / Average-block area ratio: " << blocks_max_area / blocks_avg_area << ", to be handled as floorplacement: " << floorplacement << endl;
		if (!fp.conf_SA_layout_floorplacement && floorplacement) {
			cout << "IO>   Note: floorplacement is ignored since it's deactivated" << endl;
		}

		// blocks power
		cout << "IO>  Summed blocks power [W]: " << power;
		if (power != 0.0) {
			cout << "; min power: " << fp.blocks_power_density_stats.min << ", max power: " << fp.blocks_power_density_stats.max;
			cout << ", avg power: " << fp.blocks_power_density_stats.avg << endl;
		}
		else {
			cout << endl;
		}

		// blocks area
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
	unsigned to_parse_nets;

	if (fp.logMed()) {
		cout << "IO> ";
		cout << "Parsing nets..." << endl;
	}

	// reset nets
	fp.nets.clear();

	// open nets file
	in.open(fp.nets_file.c_str());

	// drop nets file header
	while (tmpstr != "NumNets" && !in.eof())
		in >> tmpstr;
	// drop ":"
	in >> tmpstr;
	// memorize how many nets to be parsed
	in >> to_parse_nets;

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

		// store net
		fp.nets.push_back(move(new_net));

		// consider next net id
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

	// sanity check for parsed nets
	if (fp.nets.size() != to_parse_nets) {
		cout << "IO>  Not all given nets could be parsed; consider checking the benchmark format, should comply w/ GSRC Bookshelf" << endl;
		cout << "IO>   Parsed nets: " << fp.nets.size() << ", expected nets count: " << to_parse_nets << endl;
		exit(1);
	}

	if (fp.logMed()) {
		cout << "IO> ";
		cout << "Done; " << fp.nets.size() << " nets read in" << endl << endl;
	}

}

void IO::writePowerThermalTSVMaps(FloorPlanner& fp) {
	ofstream gp_out;
	ofstream data_out;
	int cur_layer;
	int layer_limit;
	unsigned x, y;
	enum FLAGS : int {power = 0, thermal = 1, TSV_density = 2};
	int flag, flag_start, flag_stop;
	double max_temp, min_temp;

	// sanity check
	if (fp.thermalAnalyzer.power_maps.empty() || fp.thermalAnalyzer.thermal_map.empty()) {
		return;
	}

	if (fp.logMed()) {
		cout << "IO> ";

		if (IO::mode == IO::Mode::REGULAR) {
			cout << "Generating power maps, TSV-density maps, and thermal map ..." << endl;
		}
		else if (IO::mode == IO::Mode::THERMAL_ANALYSIS) {
			cout << "Generating thermal map ..." << endl;
		}
	}

	// generate set of maps; integer encoding
	//
	// flag=0: generate power maps
	// flag=1: generate thermal map
	// flag=2: generate TSV-density map
	//
	// for regular runs, generate all sets; for thermal-analyzer runs, only generate
	// the required thermal map
	flag_start = flag_stop = -1;
	if (IO::mode == IO::Mode::REGULAR) {
		flag_start = FLAGS::power;
		flag_stop = FLAGS::TSV_density;
	}
	else if (IO::mode == IO::Mode::THERMAL_ANALYSIS) {
		flag_start = flag_stop = FLAGS::thermal;
	}
	//
	// actual map generation	
	for (flag = flag_start; flag <= flag_stop; flag++) {

		// power and TSV-density maps for all layers
		if (flag == FLAGS::power || flag == FLAGS::TSV_density) {
			layer_limit = fp.conf_layer;
		}
		// thermal map only for layer 0
		else if (flag == FLAGS::thermal) {
			layer_limit = 1;
		}

		for (cur_layer = 0; cur_layer < layer_limit; cur_layer++) {
			// build up file names
			stringstream gp_out_name;
			stringstream data_out_name;
			if (flag == FLAGS::power) {
				gp_out_name << fp.benchmark << "_" << cur_layer + 1 << "_power.gp";
				data_out_name << fp.benchmark << "_" << cur_layer + 1 << "_power.data";
			}
			else if (flag == FLAGS::thermal) {
				gp_out_name << fp.benchmark << "_" << cur_layer + 1 << "_thermal.gp";
				data_out_name << fp.benchmark << "_" << cur_layer + 1 << "_thermal.data";
			}
			else if (flag == FLAGS::TSV_density) {
				gp_out_name << fp.benchmark << "_" << cur_layer + 1 << "_TSV_density.gp";
				data_out_name << fp.benchmark << "_" << cur_layer + 1 << "_TSV_density.data";
			}

			// init file stream for gnuplot script
			gp_out.open(gp_out_name.str().c_str());
			// init file stream for data file
			data_out.open(data_out_name.str().c_str());

			// file header for data file
			if (flag == FLAGS::power) {
				data_out << "# X Y power" << endl;
			}
			else if (flag == FLAGS::thermal) {
				data_out << "# X Y thermal" << endl;
			}
			else if (flag == FLAGS::TSV_density) {
				data_out << "# X Y TSV_density" << endl;
			}

			// output grid values for power maps
			if (flag == FLAGS::power) {

				for (x = 0; x < ThermalAnalyzer::POWER_MAPS_DIM; x++) {
					for (y = 0; y < ThermalAnalyzer::POWER_MAPS_DIM; y++) {
						data_out << x << "	" << y << "	" << fp.thermalAnalyzer.power_maps[cur_layer][x][y].power_density << endl;
					}

					// add dummy data point, required since gnuplot option corners2color cuts last row and column of dataset
					data_out << x << "	" << ThermalAnalyzer::POWER_MAPS_DIM << "	" << "0.0" << endl;

					// blank line marks new row for gnuplot
					data_out << endl;
				}

				// add dummy data row, required since gnuplot option corners2color cuts last row and column of dataset
				for (y = 0; y <= ThermalAnalyzer::POWER_MAPS_DIM; y++) {
					data_out << ThermalAnalyzer::POWER_MAPS_DIM << "	" << y << "	" << "0.0" << endl;
				}

			}
			// output grid values for thermal maps
			else if (flag == FLAGS::thermal) {
				max_temp = 0.0;
				min_temp = 1.0e6;

				for (x = 0; x < ThermalAnalyzer::THERMAL_MAP_DIM; x++) {
					for (y = 0; y < ThermalAnalyzer::THERMAL_MAP_DIM; y++) {
						data_out << x << "	" << y << "	" << fp.thermalAnalyzer.thermal_map[x][y] << endl;
						// also track max and min temp
						max_temp = max(max_temp, fp.thermalAnalyzer.thermal_map[x][y]);
						min_temp = min(min_temp, fp.thermalAnalyzer.thermal_map[x][y]);
					}

					// add dummy data point, required since gnuplot option corners2color cuts last row and column of dataset
					data_out << x << "	" << ThermalAnalyzer::THERMAL_MAP_DIM << "	" << "0.0" << endl;

					// blank line marks new row for gnuplot
					data_out << endl;
				}

				// add dummy data row, required since gnuplot option corners2color cuts last row and column of dataset
				for (y = 0; y <= ThermalAnalyzer::THERMAL_MAP_DIM; y++) {
					data_out << ThermalAnalyzer::THERMAL_MAP_DIM << "	" << y << "	" << "0.0" << endl;
				}
			}
			// output grid values for TSV-density maps; consider only bin bins
			// w/in die outline, not in padded zone
			else if (flag == FLAGS::TSV_density) {

				for (x = 0; x < ThermalAnalyzer::THERMAL_MAP_DIM; x++) {
					for (y = 0; y < ThermalAnalyzer::THERMAL_MAP_DIM; y++) {
						// access map bins w/ offset related to
						// padding zone
						data_out << x << "	" << y << "	" << fp.thermalAnalyzer.power_maps[cur_layer][x + ThermalAnalyzer::POWER_MAPS_PADDED_BINS][y + ThermalAnalyzer::POWER_MAPS_PADDED_BINS].TSV_density << endl;
					}

					// add dummy data point, required since gnuplot option corners2color cuts last row and column of dataset
					data_out << x << "	" << ThermalAnalyzer::THERMAL_MAP_DIM << "	" << "0.0" << endl;

					// blank line marks new row for gnuplot
					data_out << endl;
				}

				// add dummy data row, required since gnuplot option corners2color cuts last row and column of dataset
				for (y = 0; y <= ThermalAnalyzer::THERMAL_MAP_DIM; y++) {
					data_out << ThermalAnalyzer::THERMAL_MAP_DIM << "	" << y << "	" << "0.0" << endl;
				}
			}

			// close file stream for data file
			data_out.close();

			// file header for gnuplot script
			if (flag == FLAGS::power) {
				gp_out << "set title \"Padded and Scaled Power Map - " << fp.benchmark << ", Layer " << cur_layer + 1 << "\"" << endl;
			}
			else if (flag == FLAGS::thermal) {
				gp_out << "set title \"Thermal Map - " << fp.benchmark << ", Layer " << cur_layer + 1 << "\"" << endl;
			}
			else if (flag == FLAGS::TSV_density) {
				gp_out << "set title \"TSV-Density Map - " << fp.benchmark << ", Layer " << cur_layer + 1 << "\"" << endl;
			}

			gp_out << "set terminal pdfcairo enhanced font \"Gill Sans, 12\"" << endl;
			gp_out << "set output \"" << gp_out_name.str() << ".pdf\"" << endl;
			gp_out << "set size square" << endl;

			// different 2D ranges for maps; consider dummy data row and
			// column, since gnuplot option corners2color cuts off last row
			// and column
			if (flag == FLAGS::power) {
				gp_out << "set xrange [0:" << ThermalAnalyzer::POWER_MAPS_DIM << "]" << endl;
				gp_out << "set yrange [0:" << ThermalAnalyzer::POWER_MAPS_DIM << "]" << endl;
			}
			else if (flag == FLAGS::thermal	|| flag == FLAGS::TSV_density) {
				gp_out << "set xrange [0:" << ThermalAnalyzer::THERMAL_MAP_DIM << "]" << endl;
				gp_out << "set yrange [0:" << ThermalAnalyzer::THERMAL_MAP_DIM << "]" << endl;
			}

			// power maps: scale, label for cbrange
			if (flag == FLAGS::power) {
				// label for power density
				gp_out << "set cblabel \"Power Density [10^{-2} {/Symbol m}W/{/Symbol m}m^2]\"" << endl;
			}
			// thermal maps: scale, label for cbrange
			else if (flag == FLAGS::thermal) {
				// fixed scale to avoid remapping to extended range
				gp_out << "set cbrange [" << min_temp << ":" << max_temp << "]" << endl;
				// thermal estimation, correlates w/ power density
				gp_out << "set cblabel \"Estimated Temperature [K]\"" << endl;
			}
			// TSV-density maps: scale, label for cbrange
			else if (flag == FLAGS::TSV_density) {
				// fixed log scale to emphasize both low densities (single
				// TSVs) as well as large densities (TSV groups, vertical
				// buses)
				gp_out << "set log cb" << endl;
				gp_out << "set cbrange [0.1:100]" << endl;
				// label for power density
				gp_out << "set cblabel \"Probabilistic TSV-Density [%]\"" << endl;
			}

			// tics
			gp_out << "set tics front" << endl;
			gp_out << "set grid xtics ytics ztics" << endl;
			// pm3d algorithm determines an average value for each pixel,
			// considering sourrounding pixels;
			// skip this behaviour w/ ``corners2color''; c1 means to select
			// the lower-left value, practically loosing one row and column in
			// the overall plot (compensated for by dummy data; see also
			// http://gnuplot.sourceforge.net/demo/pm3d.html
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
			if (flag == FLAGS::power && ThermalAnalyzer::POWER_MAPS_PADDED_BINS > 0) {
				gp_out << "set obj 1 rect from ";
				gp_out << ThermalAnalyzer::POWER_MAPS_PADDED_BINS << ", " << ThermalAnalyzer::POWER_MAPS_PADDED_BINS << " to ";
				gp_out << ThermalAnalyzer::POWER_MAPS_DIM - ThermalAnalyzer::POWER_MAPS_PADDED_BINS << ", ";
				gp_out << ThermalAnalyzer::POWER_MAPS_DIM - ThermalAnalyzer::POWER_MAPS_PADDED_BINS << " ";
				gp_out << "front fillstyle empty border rgb \"white\" linewidth 3" << endl;
			}

			gp_out << "splot \"" << data_out_name.str() << "\" using 1:2:3 notitle" << endl;

			// close file stream for gnuplot script
			gp_out.close();
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
	bool valid_solutions, first_valid_sol;

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
	}

	// two blank lines trigger gnuplot to interpret data file as separate data sets
	data_out << endl;
	data_out << endl;

	// output data: SA step and avg costs phase 1
	data_out << "# Step Avg_Cost_Phase_1 (index 1)" << endl;

	// memorize if valid solutions are given at all
	valid_solutions = false;

	for (FloorPlanner::TempStep step : fp.tempSchedule) {

		data_out << step.step << " " << step.avg_cost << endl;

		// output data until first valid solution is found
		if (step.new_best_sol_found) {

			valid_solutions = true;

			break;
		}
	}

	if (valid_solutions) {

		// two blank lines trigger gnuplot to interpret data file as separate data sets
		data_out << endl;
		data_out << endl;

		// output data: markers for best-solution steps
		data_out << "# Step Temperature (only steps w/ new best solutions, index 2)" << endl;

		for (FloorPlanner::TempStep step : fp.tempSchedule) {

			if (step.new_best_sol_found) {
				data_out << step.step << " " << step.temp << endl;
			}
		}

		// two blank lines trigger gnuplot to interpret data file as separate data sets
		data_out << endl;
		data_out << endl;

		// output data: SA step and avg costs phase 2
		data_out << "# Step Avg_Cost_Phase_2 (index 3)" << endl;

		first_valid_sol = false;
		for (FloorPlanner::TempStep step : fp.tempSchedule) {

			// output data only after first solution is found
			if (first_valid_sol) {
				data_out << step.step << " " << step.avg_cost << endl;
			}

			if (step.new_best_sol_found) {
				first_valid_sol = true;
			}
		}

		// two blank lines trigger gnuplot to interpret data file as separate data sets
		data_out << endl;
		data_out << endl;

		// output data: SA step and best costs phase 2
		data_out << "# Step Best_Cost_Phase_2 (index 4)" << endl;

		first_valid_sol = false;
		for (FloorPlanner::TempStep step : fp.tempSchedule) {

			// output data only after first solution is found
			if (first_valid_sol) {
				data_out << step.step << " " << step.cost_best_sol << endl;
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
	gp_out << "set y2label \"Normalized Avg Solution Cost\"" << endl;
	// specific settings: key, labels box
	gp_out << "set key box lt rgb \"#808080\" out bottom center" << endl;
	// specific settings: log scale (for SA temp)
	gp_out << "set log y" << endl;
	gp_out << "set mytics 10" << endl;
	// second, indepentend scale for cost values
	gp_out << "set y2tics nomirror" << endl;
	gp_out << "set mytics 10" << endl;
	// cut cost above 1 in order to emphasize cost trend
	gp_out << "set y2range [:1]" << endl;

	// gp data plot command
	gp_out << "plot \"" << data_out_name.str() << "\" index 0 using 1:2 title \"SA Temperature\" with lines linestyle 2, \\" << endl;
	// there may be no valid solutions, then only the costs for phase 1 are plotted
	// besides the temperature schedule
	if (!valid_solutions) {
		gp_out << "\"" << data_out_name.str() << "\" index 1";
		gp_out << " using 1:2 title \"Avg Cost (Accept. Sol.)\" with lines linestyle 3 axes x1y2" << endl;
	}
	// otherwise, we consider both cost and the best solutions data sets
	else {
		gp_out << "\"" << data_out_name.str() << "\" index 2 using 1:2 title \"New Best Solution\" with points linestyle 1, \\" << endl;
		gp_out << "\"" << data_out_name.str() << "\" index 1";
		gp_out << " using 1:2 title \"Avg Cost (Accepted Sol.) - SA Phase 1\" with lines linestyle 3 axes x1y2, \\" << endl;
		//gp_out << "\"" << data_out_name.str() << "\" index 3";
		//gp_out << " using 1:2 title \"Avg Cost (Accepted Sol.) - SA Phase 2\" with lines linestyle 4 axes x1y2" << endl;
		gp_out << "\"" << data_out_name.str() << "\" index 4";
		gp_out << " using 1:2 title \"Best Cost - SA Phase 2\" with lines linestyle 4 axes x1y2" << endl;
	}

	// close file stream
	gp_out.close();

	if (fp.logMed()) {
		cout << "IO> ";
		cout << "Done" << endl << endl;
	}
}

// generate GP plots of FP
void IO::writeFloorplanGP(FloorPlanner const& fp, vector<CorblivarAlignmentReq> const& alignment, string const& file_suffix) {
	ofstream gp_out;
	int cur_layer;
	double ratio_inv;
	int tics;
	Rect alignment_rect, alignment_rect_tmp;
	int req_x_fulfilled, req_y_fulfilled;
	string alignment_color_fulfilled;
	string alignment_color_failed;
	string alignment_color_undefined;

	// sanity check, not for thermal-analysis runs
	if (IO::mode == IO::Mode::THERMAL_ANALYSIS) {
		return;
	}

	if (fp.logMed()) {
		cout << "IO> ";
		if (file_suffix != "")
			cout << "Generating GP scripts for floorplan (suffix \"" << file_suffix << "\")..." << endl;
		else
			cout << "Generating GP scripts for floorplan ..." << endl;
	}

	// GP parameters
	ratio_inv = 1.0 / fp.die_AR;
	tics = max(fp.conf_outline_x, fp.conf_outline_y) / 5;

	// color for alignment rects; for fullfiled alignment, green-ish color
	alignment_color_fulfilled = "#00A000";
	// color for alignment rects; for failed alignment, red-ish color
	alignment_color_failed = "#A00000";
	// color for alignment rects; for undefined alignment, blue-ish color
	alignment_color_undefined = "#0000A0";

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

			// block rectangles
			gp_out << "set obj rect";
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

		// check alignment fullfillment
		for (Block const& cur_block : fp.blocks) {

			if (cur_block.layer != cur_layer) {
				continue;
			}

			// for each alignment request defined for the block; draw the
			// related intersection/offset to illustrate block alignment
			for (CorblivarAlignmentReq const& req :  alignment) {

				if (req.s_i->id == cur_block.id || req.s_j->id == cur_block.id) {

					// init alignment flags; -1 equals undefined
					req_x_fulfilled = req_y_fulfilled = -1;

					// check partial request, horizontal aligment
					//
					// alignment range
					if (req.range_x()) {

						// determine the blocks' intersection in
						// x-dimensions; equals the partial
						// alignment rect
						alignment_rect_tmp = Rect::determineIntersection(req.s_i->bb, req.s_j->bb);

						alignment_rect.ll.x = alignment_rect_tmp.ll.x;
						alignment_rect.ur.x = alignment_rect_tmp.ur.x;
						alignment_rect.w = alignment_rect_tmp.w;

						// overlap and thus alignment fulfilled
						if (alignment_rect.w >= req.offset_range_x) {

							req_x_fulfilled = 1;
						}
						// overlap and thus alignment failed
						else {

							req_x_fulfilled = 0;

							// extend the intersection such
							// that inner block fronts are
							// covered w.r.t. the failed
							// dimension
							if (Rect::rectA_leftOf_rectB(req.s_i->bb, req.s_j->bb, false)) {
								alignment_rect.ll.x = req.s_i->bb.ur.x;
								alignment_rect.ur.x = req.s_j->bb.ll.x;
							}
							else {
								alignment_rect.ll.x = req.s_j->bb.ur.x;
								alignment_rect.ur.x = req.s_i->bb.ll.x;
							}
						}
					}
					// max distance range
					else if (req.range_max_x()) {

						// determine the blocks' bounding box in
						// x-dimensions; equals the partial
						// alignment rect; consider the blocks'
						// center points
						alignment_rect_tmp = Rect::determBoundingBox(req.s_i->bb, req.s_j->bb, true);

						alignment_rect.ll.x = alignment_rect_tmp.ll.x;
						alignment_rect.ur.x = alignment_rect_tmp.ur.x;
						alignment_rect.w = alignment_rect_tmp.w;

						// distance and thus alignment fulfilled
						if (alignment_rect.w <= req.offset_range_x) {

							req_x_fulfilled = 1;
						}
						// distance and thus alignment failed
						else {

							req_x_fulfilled = 0;
						}
					}
					// alignment offset
					else if (req.offset_x()) {

						// for an alignment offset, the related
						// blocks' lower-left corners are relevant
						alignment_rect.ll.x = req.s_i->bb.ll.x;
						alignment_rect.ur.x = req.s_j->bb.ll.x;
						alignment_rect.w = alignment_rect.ur.x - alignment_rect.ll.x;

						// offset and thus alignment fulfilled
						if (Math::doubleComp(alignment_rect.w,  req.offset_range_x)) {

							req_x_fulfilled = 1;
						}
						// offset and thus alignment failed
						else {

							req_x_fulfilled = 0;
						}
					}
					// undefined request
					else {
						// define the rect as a bounding box
						// w.r.t. the undefined dimension
						alignment_rect.ll.x = min(req.s_i->bb.ll.x, req.s_j->bb.ll.x);
						alignment_rect.ur.x = max(req.s_i->bb.ur.x, req.s_j->bb.ur.x);
					}

					// check partial request, vertical aligment
					//
					// alignment range
					if (req.range_y()) {

						// determine the blocks' intersection in
						// y-dimensions; equals the partial
						// alignment rect
						alignment_rect_tmp = Rect::determineIntersection(req.s_i->bb, req.s_j->bb);

						alignment_rect.ll.y = alignment_rect_tmp.ll.y;
						alignment_rect.ur.y = alignment_rect_tmp.ur.y;
						alignment_rect.h = alignment_rect_tmp.h;

						// overlap and thus alignment fulfilled
						if (alignment_rect.h >= req.offset_range_y) {

							req_y_fulfilled = 1;
						}
						// overlap and thus alignment failed
						else {

							req_y_fulfilled = 0;

							// extend the intersection such
							// that inner block fronts are
							// covered w.r.t. the failed
							// dimension
							if (Rect::rectA_below_rectB(req.s_i->bb, req.s_j->bb, false)) {
								alignment_rect.ll.y = req.s_i->bb.ur.y;
								alignment_rect.ur.y = req.s_j->bb.ll.y;
							}
							else {
								alignment_rect.ll.y = req.s_j->bb.ur.y;
								alignment_rect.ur.y = req.s_i->bb.ll.y;
							}
						}
					}
					// max distance range
					else if (req.range_max_y()) {

						// determine the blocks' bounding box in
						// y-dimensions; equals the partial
						// alignment rect; consider the blocks'
						// center points
						alignment_rect_tmp = Rect::determBoundingBox(req.s_i->bb, req.s_j->bb, true);

						alignment_rect.ll.y = alignment_rect_tmp.ll.y;
						alignment_rect.ur.y = alignment_rect_tmp.ur.y;
						alignment_rect.h = alignment_rect_tmp.h;

						// distance and thus alignment fulfilled
						if (alignment_rect.h <= req.offset_range_y) {

							req_y_fulfilled = 1;
						}
						// distance and thus alignment failed
						else {

							req_y_fulfilled = 0;
						}
					}
					// alignment offset
					else if (req.offset_y()) {

						// for an alignment offset, the related
						// blocks' lower-left corners are relevant
						alignment_rect.ll.y = req.s_i->bb.ll.y;
						alignment_rect.ur.y = req.s_j->bb.ll.y;
						alignment_rect.h = alignment_rect.ur.y - alignment_rect.ll.y;

						// offset and thus alignment fulfilled
						if (Math::doubleComp(alignment_rect.h,  req.offset_range_y)) {

							req_y_fulfilled = 1;
						}
						// offset and thus alignment failed
						else {

							req_y_fulfilled = 0;
						}
					}
					// undefined request
					else {
						// define the rect as a bounding box
						// w.r.t. the undefined dimension
						alignment_rect.ll.y = min(req.s_i->bb.ll.y, req.s_j->bb.ll.y);
						alignment_rect.ur.y = max(req.s_i->bb.ur.y, req.s_j->bb.ur.y);
					}

					// construct the alignment rectangle w/ separate,
					// possibly different color-coded lines, in order
					// to represent particular failed/fulfilled
					// alignment requests

					// fixed offset alignments
					if (req.offset_x()) {

						// zero offset, i.e., alignment in
						// x-coordinate; mark w/ small rect
						if (alignment_rect.w == 0.0) {

							// rect as marker
							gp_out << "set obj rect ";
							gp_out << " from " << alignment_rect.ll.x << "," << alignment_rect.ll.y;
							gp_out << " to " << alignment_rect.ll.x + 0.01 * fp.conf_outline_x << "," << alignment_rect.ll.y + 0.01 * fp.conf_outline_y;
							// box colors
							if (req_x_fulfilled == 1) {
								gp_out << " fc rgb \"" << alignment_color_fulfilled << "\"";
							}
							else {
								gp_out << " fc rgb \"" << alignment_color_failed << "\"";
							}
							gp_out << " fs solid";
							gp_out << endl;
						}
						// non-zero offset, i.e., offset range;
						// mark w/ arrows
						else {

							// horizontal arrow
							gp_out << "set arrow";
							gp_out << " from " << alignment_rect.ll.x << "," << alignment_rect.ll.y;
							gp_out << " to " << alignment_rect.ur.x << "," << alignment_rect.ll.y;
							gp_out << " head";
							// line colors
							if (req_x_fulfilled == 1) {
								gp_out << " lc rgb \"" << alignment_color_fulfilled << "\"";
							}
							else {
								gp_out << " lc rgb \"" << alignment_color_failed << "\"";
							}
							gp_out << " lw 3";
							gp_out << endl;
						}
					}
					// range alignments
					else {

						// lower horizontal line
						gp_out << "set arrow";
						gp_out << " from " << alignment_rect.ll.x << "," << alignment_rect.ll.y;
						gp_out << " to " << alignment_rect.ur.x << "," << alignment_rect.ll.y;
						gp_out << " nohead";
						// line colors
						if (req_x_fulfilled == 1) {
							gp_out << " lc rgb \"" << alignment_color_fulfilled << "\"";
						}
						else if (req_x_fulfilled == -1) {
							gp_out << " lc rgb \"" << alignment_color_undefined << "\"";
						}
						else {
							gp_out << " lc rgb \"" << alignment_color_failed << "\"";
						}
						gp_out << " lw 3";
						gp_out << endl;

						// upper horizontal line
						gp_out << "set arrow";
						gp_out << " from " << alignment_rect.ll.x << "," << alignment_rect.ur.y;
						gp_out << " to " << alignment_rect.ur.x << "," << alignment_rect.ur.y;
						gp_out << " nohead";
						// line colors
						if (req_x_fulfilled == 1) {
							gp_out << " lc rgb \"" << alignment_color_fulfilled << "\"";
						}
						else if (req_x_fulfilled == -1) {
							gp_out << " lc rgb \"" << alignment_color_undefined << "\"";
						}
						else {
							gp_out << " lc rgb \"" << alignment_color_failed << "\"";
						}
						gp_out << " lw 3";
						gp_out << endl;
					}

					// fixed offset alignments
					if (req.offset_y()) {

						// zero offset, i.e., alignment in
						// y-coordinate; mark w/ small rect
						if (alignment_rect.h == 0.0) {

							// rect as marker
							gp_out << "set obj rect ";
							gp_out << " from " << alignment_rect.ll.x << "," << alignment_rect.ll.y;
							gp_out << " to " << alignment_rect.ll.x + 0.01 * fp.conf_outline_x << "," << alignment_rect.ll.y + 0.01 * fp.conf_outline_y;
							// box colors
							if (req_y_fulfilled == 1) {
								gp_out << " fc rgb \"" << alignment_color_fulfilled << "\"";
							}
							else {
								gp_out << " fc rgb \"" << alignment_color_failed << "\"";
							}
							gp_out << " fs solid";
							gp_out << endl;
						}
						// non-zero offset, i.e., offset range;
						// mark w/ arrows
						else {

							// vertical arrow
							gp_out << "set arrow";
							gp_out << " from " << alignment_rect.ll.x << "," << alignment_rect.ll.y;
							gp_out << " to " << alignment_rect.ll.x << "," << alignment_rect.ur.y;
							gp_out << " head";
							// line colors
							if (req_y_fulfilled == 1) {
								gp_out << " lc rgb \"" << alignment_color_fulfilled << "\"";
							}
							else {
								gp_out << " lc rgb \"" << alignment_color_failed << "\"";
							}
							gp_out << " lw 3";
							gp_out << endl;
						}
					}
					// range alignments
					else {

						// left vertical line
						gp_out << "set arrow";
						gp_out << " from " << alignment_rect.ll.x << "," << alignment_rect.ll.y;
						gp_out << " to " << alignment_rect.ll.x << "," << alignment_rect.ur.y;
						gp_out << " nohead";
						// line colors
						if (req_y_fulfilled == 1) {
							gp_out << " lc rgb \"" << alignment_color_fulfilled << "\"";
						}
						else if (req_y_fulfilled == -1) {
							gp_out << " lc rgb \"" << alignment_color_undefined << "\"";
						}
						else {
							gp_out << " lc rgb \"" << alignment_color_failed << "\"";
						}
						gp_out << " lw 3";
						gp_out << endl;

						// right vertical line
						gp_out << "set arrow";
						gp_out << " from " << alignment_rect.ur.x << "," << alignment_rect.ll.y;
						gp_out << " to " << alignment_rect.ur.x << "," << alignment_rect.ur.y;
						gp_out << " nohead";
						// line colors
						if (req_y_fulfilled == 1) {
							gp_out << " lc rgb \"" << alignment_color_fulfilled << "\"";
						}
						else if (req_y_fulfilled == -1) {
							gp_out << " lc rgb \"" << alignment_color_undefined << "\"";
						}
						else {
							gp_out << " lc rgb \"" << alignment_color_failed << "\"";
						}
						gp_out << " lw 3";
						gp_out << endl;
					}
				}
			}
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
	ofstream file, file_bond;
	int cur_layer;
	int x, y;
	int map_x, map_y;

	if (fp.logMed()) {
		cout << "IO> Generating files for HotSpot 3D-thermal simulation..." << endl;
	}

	/// generate floorplan files
	for (cur_layer = 0; cur_layer < fp.conf_layer; cur_layer++) {

		// build up file name
		stringstream fp_file;
		fp_file << fp.benchmark << "_HotSpot_Si_active_" << cur_layer + 1 << ".flp";

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
			file << "	" << cur_block.bb.w * IO::SCALE_UM_M;
			file << "	" << cur_block.bb.h * IO::SCALE_UM_M;
			file << "	" << cur_block.bb.ll.x * IO::SCALE_UM_M;
			file << "	" << cur_block.bb.ll.y * IO::SCALE_UM_M;
			file << "	" << ThermalAnalyzer::HEAT_CAPACITY_SI;
			file << "	" << ThermalAnalyzer::THERMAL_RESISTIVITY_SI;
			file << endl;
		}

		// dummy block to describe layer outline
		file << "outline" << cur_layer + 1;
		file << "	" << fp.conf_outline_x * IO::SCALE_UM_M;
		file << "	" << fp.conf_outline_y * IO::SCALE_UM_M;
		file << "	0.0";
		file << "	0.0";
		file << "	" << ThermalAnalyzer::HEAT_CAPACITY_SI;
		file << "	" << ThermalAnalyzer::THERMAL_RESISTIVITY_SI;
		file << endl;

		// close file stream
		file.close();
	}

	/// generate floorplans for passive Si and bonding layer; considering TSVs (modelled via densities)
	for (cur_layer = 0; cur_layer < fp.conf_layer; cur_layer++) {

		// build up file names
		stringstream Si_fp_file;
		Si_fp_file << fp.benchmark << "_HotSpot_Si_passive_" << cur_layer + 1 << ".flp";
		stringstream bond_fp_file;
		bond_fp_file << fp.benchmark << "_HotSpot_bond_" << cur_layer + 1 << ".flp";

		// init file streams
		file.open(Si_fp_file.str().c_str());
		file_bond.open(bond_fp_file.str().c_str());

		// file headers
		file << "# Line Format: <unit-name>\\t<width>\\t<height>\\t<left-x>\\t<bottom-y>\\t<specific-heat>\\t<resistivity>" << endl;
		file << "# all dimensions are in meters" << endl;
		file << "# comment lines begin with a '#'" << endl;
		file << "# comments and empty lines are ignored" << endl;
		file_bond << "# Line Format: <unit-name>\\t<width>\\t<height>\\t<left-x>\\t<bottom-y>\\t<specific-heat>\\t<resistivity>" << endl;
		file_bond << "# all dimensions are in meters" << endl;
		file_bond << "# comment lines begin with a '#'" << endl;
		file_bond << "# comments and empty lines are ignored" << endl;

		// for regular runs, i.e., Corblivar runs, we have to consider different
		// TSV densities for each grid bin, given in the power_maps
		if (IO::mode == IO::Mode::REGULAR) {

			// walk power-map grid to obtain specific TSV densities of bins
			for (x = ThermalAnalyzer::POWER_MAPS_PADDED_BINS; x < ThermalAnalyzer::THERMAL_MAP_DIM + ThermalAnalyzer::POWER_MAPS_PADDED_BINS; x++) {

				// adapt index for final thermal map according to padding
				map_x = x - ThermalAnalyzer::POWER_MAPS_PADDED_BINS;

				for (y = ThermalAnalyzer::POWER_MAPS_PADDED_BINS; y < ThermalAnalyzer::THERMAL_MAP_DIM + ThermalAnalyzer::POWER_MAPS_PADDED_BINS; y++) {

					// adapt index for final thermal map according to padding
					map_y = y - ThermalAnalyzer::POWER_MAPS_PADDED_BINS;

					// put grid block as floorplan blocks; passive Si layer
					file << "Si_passive_" << cur_layer + 1 << "_" << map_x << ":" << map_y;
					/// bin dimensions
					file << "	" << fp.thermalAnalyzer.power_maps_dim_x * IO::SCALE_UM_M;
					file << "	" << fp.thermalAnalyzer.power_maps_dim_y * IO::SCALE_UM_M;
					/// bin lower-left corner; float precision required in
					//order to avoid grid coordinate mismatches
					file << "	" << static_cast<float>(map_x * fp.thermalAnalyzer.power_maps_dim_x * IO::SCALE_UM_M);
					file << "	" << static_cast<float>(map_y * fp.thermalAnalyzer.power_maps_dim_x * IO::SCALE_UM_M);
					// thermal properties, depending on bin's TSV density
					file << "	" << ThermalAnalyzer::heatCapSi(fp.thermalAnalyzer.power_maps[cur_layer][x][y].TSV_density);
					file << "	" << ThermalAnalyzer::thermResSi(fp.thermalAnalyzer.power_maps[cur_layer][x][y].TSV_density);
					file << endl;

					// put grid block as floorplan blocks; bonding layer
					file_bond << "bond_" << cur_layer + 1 << "_" << map_x << ":" << map_y;
					/// bin dimensions
					file_bond << "	" << fp.thermalAnalyzer.power_maps_dim_x * IO::SCALE_UM_M;
					file_bond << "	" << fp.thermalAnalyzer.power_maps_dim_y * IO::SCALE_UM_M;
					/// bin lower-left corner; float precision required in
					//order to avoid grid coordinate mismatches
					file_bond << "	" << static_cast<float>(map_x * fp.thermalAnalyzer.power_maps_dim_x * IO::SCALE_UM_M);
					file_bond << "	" << static_cast<float>(map_y * fp.thermalAnalyzer.power_maps_dim_x * IO::SCALE_UM_M);
					// thermal properties, depending on bin's TSV density
					file_bond << "	" << ThermalAnalyzer::heatCapBond(fp.thermalAnalyzer.power_maps[cur_layer][x][y].TSV_density);
					file_bond << "	" << ThermalAnalyzer::thermResBond(fp.thermalAnalyzer.power_maps[cur_layer][x][y].TSV_density);
					file_bond << endl;
				}
			}
		}
		// for thermal-analysis fitting runs, we consider one common TSV density
		// for the whole chip outline
		else if (IO::mode == IO::Mode::THERMAL_ANALYSIS) {

			file << "Si_passive_" << cur_layer + 1;
			file << "	" << fp.conf_outline_x * IO::SCALE_UM_M;
			file << "	" << fp.conf_outline_y * IO::SCALE_UM_M;
			file << "	0.0";
			file << "	0.0";
			file << "	" << ThermalAnalyzer::heatCapSi(fp.conf_power_blurring_parameters.TSV_density);
			file << "	" << ThermalAnalyzer::thermResSi(fp.conf_power_blurring_parameters.TSV_density);
			file << endl;

			file_bond << "bond_" << cur_layer + 1;
			file_bond << "	" << fp.conf_outline_x * IO::SCALE_UM_M;
			file_bond << "	" << fp.conf_outline_y * IO::SCALE_UM_M;
			file_bond << "	0.0";
			file_bond << "	0.0";
			file_bond << "	" << ThermalAnalyzer::heatCapBond(fp.conf_power_blurring_parameters.TSV_density);
			file_bond << "	" << ThermalAnalyzer::thermResBond(fp.conf_power_blurring_parameters.TSV_density);
			file_bond << endl;
		}

		// close file streams
		file.close();
		file_bond.close();
	}

	/// generate dummy floorplan for BEOL layer; TSVs are not to be considered
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
	file << "	" << fp.conf_outline_x * IO::SCALE_UM_M;
	file << "	" << fp.conf_outline_y * IO::SCALE_UM_M;
	file << "	0.0";
	file << "	0.0";
	file << "	" << ThermalAnalyzer::HEAT_CAPACITY_BEOL;
	file << "	" << ThermalAnalyzer::THERMAL_RESISTIVITY_BEOL;
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

		file << "# BEOL (interconnects) layer " << cur_layer + 1 << endl;
		file << 4 * cur_layer << endl;
		file << "Y" << endl;
		file << "N" << endl;
		file << ThermalAnalyzer::HEAT_CAPACITY_BEOL << endl;
		file << ThermalAnalyzer::THERMAL_RESISTIVITY_BEOL << endl;
		file << Chip::THICKNESS_BEOL << endl;
		file << fp.benchmark << "_HotSpot_BEOL.flp" << endl;
		file << endl;

		file << "# Active Si layer; design layer " << cur_layer + 1 << endl;
		file << 4 * cur_layer + 1 << endl;
		file << "Y" << endl;
		file << "Y" << endl;
		file << ThermalAnalyzer::HEAT_CAPACITY_SI << endl;
		file << ThermalAnalyzer::THERMAL_RESISTIVITY_SI << endl;
		file << Chip::THICKNESS_SI_ACTIVE << endl;
		file << fp.benchmark << "_HotSpot_Si_active_" << cur_layer + 1 << ".flp" << endl;
		file << endl;

		file << "# Passive Si layer " << cur_layer + 1 << endl;
		file << 4 * cur_layer + 2 << endl;
		file << "Y" << endl;
		file << "N" << endl;
		// dummy values, proper values (depending on TSV densities) are in the
		// actual floorplan file
		file << ThermalAnalyzer::HEAT_CAPACITY_SI << endl;
		file << ThermalAnalyzer::THERMAL_RESISTIVITY_SI << endl;
		file << Chip::THICKNESS_SI_PASSIVE << endl;
		file << fp.benchmark << "_HotSpot_Si_passive_" << cur_layer + 1 << ".flp" << endl;
		file << endl;

		if (cur_layer < (fp.conf_layer - 1)) {
			file << "# bond layer " << cur_layer + 1 << "; for F2B bonding to next die " << cur_layer + 2 << endl;
			file << 4 * cur_layer + 3 << endl;
			file << "Y" << endl;
			file << "N" << endl;
			// dummy values, proper values (depending on TSV densities) are in
			// the actual floorplan file
			file << ThermalAnalyzer::HEAT_CAPACITY_BOND << endl;
			file << ThermalAnalyzer::THERMAL_RESISTIVITY_BOND << endl;
			file << Chip::THICKNESS_BOND << endl;
			file << fp.benchmark << "_HotSpot_bond_" << cur_layer + 1 << ".flp" << endl;
			file << endl;
		}
	}

	// close file stream
	file.close();

	if (fp.logMed()) {
		cout << "IO> Done" << endl << endl;
	}
}
