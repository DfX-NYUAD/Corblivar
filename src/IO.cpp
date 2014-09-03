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
#include "ThermalAnalyzer.hpp"
#include "CornerBlockList.hpp"
#include "CorblivarCore.hpp"
#include "CorblivarAlignmentReq.hpp"
#include "Net.hpp"
#include "Math.hpp"
#include "Clustering.hpp"

// parse program parameter, config file, and further files
void IO::parseParametersFiles(FloorPlanner& fp, int const& argc, char** argv) {
	int file_version;
	size_t last_slash;
	ifstream in;
	string config_file, technology_file;
	stringstream results_file;
	stringstream blocks_file;
	stringstream alignments_file;
	stringstream pins_file;
	stringstream power_density_file;
	stringstream nets_file;
	string tmpstr;
	ThermalAnalyzer::MaskParameters mask_parameters;

	// print command-line parameters
	if (argc < 4) {
		cout << "IO> Usage: " << argv[0] << " benchmark_name config_file benchmarks_dir [solution_file] [TSV_density]" << endl;
		cout << "IO> " << endl;
		cout << "IO> Mandatory parameter ``benchmark_name'': any name, should refer to GSRC-Bookshelf benchmark" << endl;
		cout << "IO> Mandatory parameter ``config_file'' format: see provided Corblivar.conf" << endl;
		cout << "IO> Mandatory parameter ``benchmarks_dir'': folder containing actual benchmark files in GSRC Bookshelf format" << endl;
		cout << "IO> Optional parameter ``solution_file'': re-evaluate w/ given Corblivar solution" << endl;
		cout << "IO> Optional parameter ``TSV density'': average TSV density to be considered across all dies, to be given in \%" << endl;

		exit(1);
	}

	// TSV density given; note special run mode where only thermal-analysis result is
	// output, not all other (time-consuming) date
	if (argc == 6) {
		fp.thermal_analyser_run = true;
	}
	else {
		fp.thermal_analyser_run = false;
	}

	// read in mandatory parameters
	fp.benchmark = argv[1];

	config_file = argv[2];

	blocks_file << argv[3] << fp.benchmark << ".blocks";
	fp.IO_conf.blocks_file = blocks_file.str();

	alignments_file << argv[3] << fp.benchmark << ".alr";
	fp.IO_conf.alignments_file = alignments_file.str();

	pins_file << argv[3] << fp.benchmark << ".pl";
	fp.IO_conf.pins_file = pins_file.str();

	power_density_file << argv[3] << fp.benchmark << ".power";
	fp.IO_conf.power_density_file = power_density_file.str();

	nets_file << argv[3] << fp.benchmark << ".nets";
	fp.IO_conf.nets_file = nets_file.str();

	results_file << fp.benchmark << ".results";
	fp.IO_conf.results.open(results_file.str().c_str());

	// determine path of technology file; same as config file per definition
	last_slash = config_file.find_last_of('/');
	if (last_slash == string::npos) {
		technology_file = "";
	}
	else {
		technology_file = config_file.substr(0, last_slash) + "/";
	}

	// assume minimal log level; actual level to be parsed later on
	fp.log = FloorPlanner::LOG_MINIMAL;

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
	in.open(fp.IO_conf.blocks_file.c_str());
	if (!in.good()) {
		cout << "IO> ";
		cout << "Blocks file missing: " << fp.IO_conf.blocks_file << endl;
		exit(1);
	}
	in.close();

	// alignments file
	in.open(fp.IO_conf.alignments_file.c_str());
	// memorize file availability
	fp.IO_conf.alignments_file_avail = in.good();

	if (!in.good() && fp.logMin()) {
		cout << "IO> ";
		cout << "Note: alignment-requests file missing : " << fp.IO_conf.alignments_file<< endl;
		cout << "IO> Block alignment cannot be performed; is deactivated." << endl;
		cout << endl;
	}
	in.close();

	// pins file
	in.open(fp.IO_conf.pins_file.c_str());
	if (!in.good()) {
		cout << "IO> ";
		cout << "Pins file missing: " << fp.IO_conf.pins_file << endl;
		exit(1);
	}
	in.close();

	// power file
	in.open(fp.IO_conf.power_density_file.c_str());
	// memorize file availability
	fp.IO_conf.power_density_file_avail = in.good();

	if (!in.good()) {
		cout << "IO> ";
		cout << "Note: power density file missing : " << fp.IO_conf.power_density_file << endl;
		cout << "IO> Thermal analysis and optimization cannot be performed; is deactivated." << endl;
		cout << endl;

		// for thermal-analyser runs, the power density files are mandatory
		if (fp.thermal_analyser_run) {
			exit(1);
		}
	}
	in.close();

	// nets file
	in.open(fp.IO_conf.nets_file.c_str());
	if (!in.good()) {
		cout << "IO> ";
		cout << "Nets file missing: " << fp.IO_conf.nets_file << endl;
		exit(1);
	}
	in.close();

	// additional command-line parameters
	//
	// additional parameter for solution file given; consider file for readin
	if (argc > 4) {

		fp.IO_conf.solution_file = argv[4];
		// open file if possible
		fp.IO_conf.solution_in.open(fp.IO_conf.solution_file.c_str());
		if (!fp.IO_conf.solution_in.good())
		{
			cout << "IO> ";
			cout << "No such solution file: " << fp.IO_conf.solution_file << endl;
			exit(1);
		}
	}
	// open new solution file
	else {
		fp.IO_conf.solution_file = fp.benchmark + ".solution";
		fp.IO_conf.solution_out.open(fp.IO_conf.solution_file.c_str());
	}

	// additional parameter for TSV density given, in percent
	if (argc == 6) {
		mask_parameters.TSV_density = atof(argv[5]);
	}
	// otherwise assume a setup w/o regularly spread TSVs, i.e., TSV density is zero
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
	in >> file_version;

	if (file_version != IO::CONFIG_VERSION) {
		cout << "IO> Wrong version of config file; required version is \"" << IO::CONFIG_VERSION << "\"; consider using matching config file!" << endl;
		exit(1);
	}

	// parse in config parameters
	//
	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> tmpstr;
	// append file name to already defined path of technology file
	technology_file += tmpstr;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.log;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.layoutOp.parameters.enhanced_hard_block_rotation;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.layoutOp.parameters.enhanced_soft_block_shaping;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.layoutOp.parameters.packing_iterations;

	// sanity check for packing iterations
	if (fp.layoutOp.parameters.packing_iterations < 0) {
		cout << "IO> Provide a positive packing iterations count or set 0 to disable!" << endl;
		exit(1);
	}

	// sanity check for packing and block rotation
	if (fp.layoutOp.parameters.enhanced_hard_block_rotation && (fp.layoutOp.parameters.packing_iterations > 0)) {
		cout << "IO> Activate only guided hard block rotation OR layout packing; both cannot be performed!" << endl;
		exit(1);
	}

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.layoutOp.parameters.power_aware_block_handling;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.layoutOp.parameters.floorplacement;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.layoutOp.parameters.signal_TSV_clustering;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.SA_parameters.loopFactor;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.SA_parameters.loopLimit;

	// sanity check for positive, non-zero parameters
	if (fp.SA_parameters.loopFactor <= 0.0 || fp.SA_parameters.loopLimit <= 0.0) {
		cout << "IO> Provide positive, non-zero SA loop parameters!" << endl;
		exit(1);
	}

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.SA_parameters.temp_init_factor;

	// sanity check for positive, non-zero factor
	if (fp.SA_parameters.temp_init_factor <= 0.0) {
		cout << "IO> Provide positive, non-zero SA start temperature scaling factor!" << endl;
		exit(1);
	}

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.SA_parameters.temp_factor_phase1;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.SA_parameters.temp_factor_phase1_limit;

	// sanity check for dependent temperature-scaling factors
	if (fp.SA_parameters.temp_factor_phase1 >= fp.SA_parameters.temp_factor_phase1_limit) {
		cout << "IO> Initial cooling factor for SA phase 1 should be smaller than the related final factor!" << endl;
		exit(1);
	}

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.SA_parameters.temp_factor_phase2;

	// sanity check for positive, non-zero parameters
	if (fp.SA_parameters.temp_factor_phase1 <= 0.0 || fp.SA_parameters.temp_factor_phase2 <= 0.0) {
		cout << "IO> Provide positive, non-zero SA cooling factors for phases 1 and 2!" << endl;
		exit(1);
	}

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.SA_parameters.temp_factor_phase3;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.SA_parameters.cost_thermal;

	// memorize if thermal optimization should be performed
	fp.SA_parameters.opt_thermal = (fp.SA_parameters.cost_thermal > 0.0 && fp.IO_conf.power_density_file_avail);

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.SA_parameters.cost_WL;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.SA_parameters.cost_TSVs;

	// memorize if interconnects optimization should be performed
	fp.SA_parameters.opt_interconnects = (fp.SA_parameters.cost_WL > 0.0 || fp.SA_parameters.cost_TSVs > 0.0);

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.SA_parameters.cost_alignment;

	// memorize if alignment optimization should be performed
	fp.SA_parameters.opt_alignment = (fp.SA_parameters.cost_alignment > 0.0 && fp.IO_conf.alignments_file_avail);
	// also memorize in layout-operations handler
	fp.layoutOp.parameters.opt_alignment = fp.SA_parameters.opt_alignment;

	// sanity check for positive cost factors
	if (fp.SA_parameters.cost_thermal < 0.0 || fp.SA_parameters.cost_WL < 0.0 || fp.SA_parameters.cost_TSVs < 0.0 || fp.SA_parameters.cost_alignment < 0.0) {
		cout << "IO> Provide positive cost factors!" << endl;
		exit(1);
	}

	// sanity check for sum of cost factors
	if (abs(fp.SA_parameters.cost_thermal + fp.SA_parameters.cost_WL + fp.SA_parameters.cost_TSVs + fp.SA_parameters.cost_alignment - 1.0) > 0.1) {
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
	fp.power_blurring_parameters = mask_parameters;

	in.close();

	// technology file parsing
	//
	// initially test file
	in.open(technology_file.c_str());
	if (!in.good()) {
		cout << "IO> ";
		cout << "No such technology file: " << technology_file << endl;
		exit(1);
	}

	if (fp.logMin()) {
		cout << "IO> Parsing technology file ..." << endl;
	}

	// reset tmpstr
	tmpstr = "";

	// sanity check for file version
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> file_version;

	if (file_version != IO::TECHNOLOGY_VERSION) {
		cout << file_version << endl;
		cout << "IO> Wrong version of technology file; required version is \"" << IO::TECHNOLOGY_VERSION << "\"; consider using matching technology file!" << endl;
		exit(1);
	}

	// parse in technology parameters
	//
	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.IC.layers;
	// also memorize in layout-operations handler
	fp.layoutOp.parameters.layers = fp.IC.layers;

	// sanity check for positive, non-zero layer
	if (fp.IC.layers <= 0) {
		cout << "IO> Provide positive, non-zero layer count!" << endl;
		exit(1);
	}

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.IC.outline_x;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.IC.outline_y;

	// sanity check for positive, non-zero dimensions
	if (fp.IC.outline_x <= 0.0 || fp.IC.outline_y <= 0.0) {
		cout << "IO> Provide positive, non-zero outline dimensions!" << endl;
		exit(1);
	}

	// determine aspect ratio and area
	fp.IC.die_AR = fp.IC.outline_x / fp.IC.outline_y;
	fp.IC.die_area = fp.IC.outline_x * fp.IC.outline_y;
	fp.IC.stack_area = fp.IC.die_area * fp.IC.layers;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.IC.blocks_scale;

	// sanity check for block scaling factor
	if (fp.IC.blocks_scale <= 0.0) {
		cout << "IO> Provide a positive, non-zero block scaling factor!" << endl;
		exit(1);
	}

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.IC.outline_shrink;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.IC.die_thickness;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.IC.Si_active_thickness;

	// determine thickness of passive Si layer
	fp.IC.Si_passive_thickness = fp.IC.die_thickness - fp.IC.Si_active_thickness;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.IC.BEOL_thickness;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.IC.bond_thickness;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.IC.TSV_dimension;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.IC.TSV_pitch;

	// determine Cu-Si area ratio for TSV groups
	fp.IC.TSV_group_Cu_Si_ratio = (fp.IC.TSV_dimension * fp.IC.TSV_dimension) /
		((fp.IC.TSV_pitch * fp.IC.TSV_pitch) - (fp.IC.TSV_dimension * fp.IC.TSV_dimension));
	// determine Cu area fraction for TSV groups
	fp.IC.TSV_group_Cu_area_ratio = (fp.IC.TSV_dimension * fp.IC.TSV_dimension) /
		(fp.IC.TSV_pitch * fp.IC.TSV_pitch);

	in.close();

	if (fp.logMin()) {
		cout << "IO> Done; technology and config values:" << endl;

		// log
		cout << "IO>  Loglevel (1 to 3 for minimal, medium, maximal): " << fp.log << endl;

		// general 3D IC setup
		cout << "IO>  Chip -- Layers for 3D IC: " << fp.IC.layers << endl;
		cout << "IO>  Chip -- Fixed die outline (width, x-dimension) [um]: " << fp.IC.outline_x << endl;
		cout << "IO>  Chip -- Fixed die outline (height, y-dimension) [um]: " << fp.IC.outline_y << endl;
		cout << "IO>  Chip -- Block scaling factor: " << fp.IC.blocks_scale << endl;
		cout << "IO>  Chip -- Final die outline shrink: " << fp.IC.outline_shrink << endl;

		// technology parameters
		cout << "IO>  Technology -- Die thickness [um]: " << fp.IC.die_thickness << endl;
		cout << "IO>  Technology -- Active Si layer thickness [um]: " << fp.IC.Si_active_thickness << endl;
		cout << "IO>  Technology -- Passive Si layer thickness [um]: " << fp.IC.Si_passive_thickness << endl;
		cout << "IO>  Technology -- BEOL layer thickness [um]: " << fp.IC.BEOL_thickness << endl;
		cout << "IO>  Technology -- BCB bonding layer thickness [um]: " << fp.IC.bond_thickness << endl;
		cout << "IO>  Technology -- TSV dimension [um]: " << fp.IC.TSV_dimension << endl;
		cout << "IO>  Technology -- TSV pitch [um]: " << fp.IC.TSV_pitch << endl;
		cout << "IO>  Technology -- TSV groups; Cu-Si area ratio: " << fp.IC.TSV_group_Cu_Si_ratio << endl;
		cout << "IO>  Technology -- TSV groups; Cu area fraction: " << fp.IC.TSV_group_Cu_area_ratio << endl;

		// layout generation options
		cout << "IO>  SA -- Layout generation; guided hard block rotation: " << fp.layoutOp.parameters.enhanced_hard_block_rotation << endl;
		cout << "IO>  SA -- Layout generation; guided soft block shaping: " << fp.layoutOp.parameters.enhanced_soft_block_shaping << endl;
		cout << "IO>  SA -- Layout generation; packing iterations: " << fp.layoutOp.parameters.packing_iterations << endl;
		cout << "IO>  SA -- Layout generation; power-aware block handling: " << fp.layoutOp.parameters.power_aware_block_handling << endl;
		cout << "IO>  SA -- Layout generation; floorplacement handling: " << fp.layoutOp.parameters.floorplacement << endl;

		// SA loop setup
		cout << "IO>  SA -- Inner-loop operation-factor a (ops = N^a for N blocks): " << fp.SA_parameters.loopFactor << endl;
		cout << "IO>  SA -- Outer-loop upper limit: " << fp.SA_parameters.loopLimit << endl;

		// SA cooling schedule
		cout << "IO>  SA -- Start temperature scaling factor: " << fp.SA_parameters.temp_init_factor << endl;
		cout << "IO>  SA -- Initial temperature-scaling factor for phase 1 (adaptive cooling): " << fp.SA_parameters.temp_factor_phase1 << endl;
		cout << "IO>  SA -- Final temperature-scaling factor for phase 1 (adaptive cooling): " << fp.SA_parameters.temp_factor_phase1_limit << endl;
		cout << "IO>  SA -- Temperature-scaling factor for phase 2 (reheating and freezing): " << fp.SA_parameters.temp_factor_phase2 << endl;
		cout << "IO>  SA -- Temperature-scaling factor for phase 3 (brief reheating, escaping local minima) : " << fp.SA_parameters.temp_factor_phase3 << endl;

		// SA cost factors
		cout << "IO>  SA -- Cost factor for thermal distribution: " << fp.SA_parameters.cost_thermal << endl;
		if (!fp.IO_conf.power_density_file_avail) {
			cout << "IO>     Note: thermal optimization is disabled since no power density file is available" << endl;
		}
		cout << "IO>  SA -- Cost factor for wirelength: " << fp.SA_parameters.cost_WL << endl;
		cout << "IO>  SA -- Cost factor for TSVs: " << fp.SA_parameters.cost_TSVs << endl;
		cout << "IO>  SA -- Cost factor for block alignment: " << fp.SA_parameters.cost_alignment << endl;
		if (!fp.IO_conf.alignments_file_avail) {
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
	while (tmpstr != "data_start" && !fp.IO_conf.solution_in.eof()) {
		fp.IO_conf.solution_in >> tmpstr;
	}

	tuples = 0;
	while (!fp.IO_conf.solution_in.eof()) {
		fp.IO_conf.solution_in >> tmpstr;

		// new die; new CBL
		if (tmpstr == "CBL") {
			// drop "["
			fp.IO_conf.solution_in >> tmpstr;

			// layer id
			fp.IO_conf.solution_in >> cur_layer;

			// drop "]"
			fp.IO_conf.solution_in >> tmpstr;
		}
		// new CBL tuple; new block
		else if (tmpstr == "tuple") {
			// drop tuple id
			fp.IO_conf.solution_in >> tmpstr;
			// drop ":"
			fp.IO_conf.solution_in >> tmpstr;
			// drop "("
			fp.IO_conf.solution_in >> tmpstr;

			// block id
			fp.IO_conf.solution_in >> block_id;
			// find related block
			tuple.S = Block::findBlock(block_id, fp.blocks);
			if (tuple.S == nullptr) {
				cout << "IO> Block " << block_id << " cannot be retrieved; ensure solution file and benchmark file match!" << endl;
				exit(1);
			}

			// memorize layer in block itself
			tuple.S->layer = cur_layer;

			// direction L
			fp.IO_conf.solution_in >> dir;
			// parse direction; unsigned 
			if (dir == static_cast<unsigned>(Direction::VERTICAL)) {
				tuple.L = Direction::VERTICAL;
			}
			else {
				tuple.L = Direction::HORIZONTAL;
			}

			// T-junctions
			fp.IO_conf.solution_in >> tuple.T;

			// block width
			fp.IO_conf.solution_in >> tuple.S->bb.w;

			// block height
			fp.IO_conf.solution_in >> tuple.S->bb.h;

			// drop ");"
			fp.IO_conf.solution_in >> tmpstr;

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
	int id;
	string block_id;
	Block const* b1;
	Block const* b2;
	string type_str;
	int signals;
	CorblivarAlignmentReq::Type type_x;
	CorblivarAlignmentReq::Type type_y;
	CorblivarAlignmentReq::Global_Type type;
	double alignment_x;
	double alignment_y;

	// sanity check for unavailable file
	if (!fp.IO_conf.alignments_file_avail) {
		return;
	}

	if (fp.logMed()) {
		cout << "IO> ";
		cout << "Parsing alignment requests..." << endl;
	}

	// open file
	al_in.open(fp.IO_conf.alignments_file.c_str());

	// reset alignments
	alignments.clear();

	// drop file header
	while (tmpstr != "data_start" && !al_in.eof()) {
		al_in >> tmpstr;
	}

	// parse alignment tuples
	// e.g.
	// ( STRICT 64 sb1 sb2 MIN 50.0 MIN 100.0 )
	id = 0;
	while (!al_in.eof()) {

		// drop "("
		al_in >> tmpstr;

		// due to some empty lines at the end, we may have reached eof just now
		if (al_in.eof()) {
			break;
		}

		// global alignment type
		al_in >> type_str;

		if (type_str == "STRICT") {
			type = CorblivarAlignmentReq::Global_Type::STRICT;
		}
		else if (type_str == "FLEXIBLE") {
			type = CorblivarAlignmentReq::Global_Type::FLEXIBLE;
		}
		else {
			cout << "IO> Unknown global alignment type: " << type_str << "; ensure alignment-requests file has correct format!" << endl;
			exit(1);
		}

		// signals
		al_in >> signals;

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
		al_in >> type_str;

		if (type_str == "MIN") {
			type_x = CorblivarAlignmentReq::Type::MIN;
		}
		else if (type_str == "MAX") {
			type_x = CorblivarAlignmentReq::Type::MAX;
		}
		else if (type_str == "OFFSET") {
			type_x = CorblivarAlignmentReq::Type::OFFSET;
		}
		else if (type_str == "UNDEF") {
			type_x = CorblivarAlignmentReq::Type::UNDEF;
		}
		else {
			cout << "IO> Unknown alignment-request type: " << type_str << "; ensure alignment-requests file has correct format!" << endl;
			exit(1);
		}

		// alignment value for x-dimension
		al_in >> alignment_x;

		// alignment type for y-dimension
		al_in >> type_str;

		if (type_str == "MIN") {
			type_y = CorblivarAlignmentReq::Type::MIN;
		}
		else if (type_str == "MAX") {
			type_y = CorblivarAlignmentReq::Type::MAX;
		}
		else if (type_str == "OFFSET") {
			type_y = CorblivarAlignmentReq::Type::OFFSET;
		}
		else if (type_str == "UNDEF") {
			type_y = CorblivarAlignmentReq::Type::UNDEF;
		}
		else {
			cout << "IO> Unknown alignment-request type: " << type_str << "; ensure alignment-requests file has correct format!" << endl;
			exit(1);
		}

		// alignment value for y-dimension
		al_in >> alignment_y;

		// drop ");"
		al_in >> tmpstr;

		// generate and store successfully parsed request
		alignments.push_back(CorblivarAlignmentReq(id, type, signals, b1, b2, type_x, alignment_x, type_y, alignment_y));

		id++;
	}

	if (IO::DBG) {
		for (CorblivarAlignmentReq const& req : alignments) {
			cout << "DBG_IO> " << req.tupleString() << endl;
		}
	}

	if (fp.logMed()) {
		cout << "IO> ";
		cout << "Done; parsed " << id << " alignment requests" << endl << endl;
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
	blocks_in.open(fp.IO_conf.blocks_file.c_str());
	pins_in.open(fp.IO_conf.pins_file.c_str());
	power_in.open(fp.IO_conf.power_density_file.c_str());

	// drop power density file header line
	if (fp.IO_conf.power_density_file_avail) {
		while (tmpstr != "end" && !power_in.eof())
			power_in >> tmpstr;
		// if we reached eof, there was no header line; reset the input stream
		if (power_in.eof()) {
			power_in.clear() ;
			power_in.seekg(0, power_in.beg);
		}
	}

	// reset blocks
	fp.IC.blocks_area = 0.0;
	fp.blocks.clear();
	// reset terminals
	fp.terminals.clear();

	// reset blocks power statistics
	fp.power_stats.max = fp.power_stats.range = fp.power_stats.avg = 0.0;
	fp.power_stats.min = -1;

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

		// init block / pin
		Block new_block = Block(id);
		Pin new_pin = Pin(id);

		// parse block type
		blocks_in >> tmpstr;

		// terminal pins: store separately
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
				pins_in >> new_pin.bb.ll.x;
				pins_in >> new_pin.bb.ll.y;
			}

			// store pin
			fp.terminals.push_back(new_pin);

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
			new_block.bb.w *= fp.IC.blocks_scale;
			new_block.bb.h *= fp.IC.blocks_scale;

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
			new_block.bb.area *= pow(fp.IC.blocks_scale, 2);

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
		if (fp.IO_conf.power_density_file_avail) {
			if (!power_in.eof()) {
				power_in >> new_block.power_density;
				// GSRC benchmarks provide power density in 10^5 W/m^2
				// which equals 10^-1 uW/um^2; scale by factor 10 in order
				// to obtain uW/um^2
				//
				// (TODO) scaling up ignored in order to limit
				// power-density to reasonable values
				//new_block.power_density *= 10.0;
			}
			else {
				if (fp.logMin()) {
					cout << "IO>  Some blocks have no power value assigned, consider checking the power density file!" << endl;
				}
			}
		}

		// track block power statistics
		power += new_block.power();
		fp.power_stats.max = max(fp.power_stats.max, new_block.power_density);
		if (fp.power_stats.min == -1) {
			fp.power_stats.min = new_block.power_density;
		}
		fp.power_stats.min = min(fp.power_stats.min, new_block.power_density);
		fp.power_stats.avg += new_block.power_density;

		// memorize summed blocks area and largest block, needs to fit into die
		fp.IC.blocks_area += new_block.bb.area;
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
	fp.IC.stack_deadspace = fp.IC.stack_area - fp.IC.blocks_area;

	// determine if floorplacement case, i.e., some very large blocks exist
	blocks_avg_area = fp.IC.blocks_area / fp.blocks.size();
	floorplacement = false;
	for (Block& block : fp.blocks) {

		if (block.bb.area >= FloorPlanner::FP_AREA_RATIO_LIMIT * blocks_avg_area) {

			floorplacement = true;
			// also mark block as floorplacement instance
			block.floorplacement = true;
		}
	}
	// update config parameter, i.e., deactivate floorplacement if not required
	fp.layoutOp.parameters.floorplacement &= floorplacement;

	// determine block power statistics
	fp.power_stats.avg /= fp.blocks.size();
	fp.power_stats.range = fp.power_stats.max - fp.power_stats.min;

	// scale terminal pins
	fp.scaleTerminalPins();

	// sanity check of fixed outline
	blocks_outline_ratio = fp.IC.blocks_area / fp.IC.stack_area;
	if (blocks_outline_ratio > 1.0) {
		cout << "IO>  Chip too small; consider increasing the die outline or layers count" << endl;
		cout << "IO>  Summed Blocks/dies area ratio: " << blocks_outline_ratio << endl;
		exit(1);
	}
	// sanity check for largest block
	if (blocks_max_area > fp.IC.die_area) {
		cout << "IO>  Die outline too small; consider increasing it" << endl;
		cout << "IO>  Largest-block/die area ratio: " << blocks_max_area / fp.IC.die_area << endl;
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
		if (!fp.layoutOp.parameters.floorplacement && floorplacement) {
			cout << "IO>   Note: floorplacement is ignored since it's deactivated" << endl;
		}

		// blocks power
		cout << "IO>  Summed blocks power [W]: " << power;
		if (power != 0.0) {
			cout << "; min power density [uW/um^2]: " << fp.power_stats.min;
			cout << ", max power density [uW/um^2]: " << fp.power_stats.max;
			cout << ", avg power density [uW/um^2]: " << fp.power_stats.avg << endl;
		}
		else {
			cout << endl;
		}

		// blocks area
		cout << "IO>  Summed blocks area [cm^2]: " << fp.IC.blocks_area * 1.0e-8;
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
	Block const* block;
	Pin const* pin;
	int id;
	bool block_not_found, pin_not_found;
	unsigned to_parse_nets;

	if (fp.logMed()) {
		cout << "IO> ";
		cout << "Parsing nets..." << endl;
	}

	// reset nets
	fp.nets.clear();

	// open nets file
	in.open(fp.IO_conf.nets_file.c_str());

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

			// parse block / pin id
			in >> net_block;

			// try to interpret as terminal pin
			pin = Pin::findPin(net_block, fp.terminals);
			if (pin != nullptr) {
				// mark net as net w/ external pin
				new_net.hasExternalPin = true;
				// store terminal
				new_net.terminals.push_back(move(pin));
				// pin found
				pin_not_found = false;
			}
			else {
				// pin not found
				pin_not_found = true;
			}

			// try to interpret as regular block 
			if (pin_not_found) {
				block = Block::findBlock(net_block, fp.blocks);
				if (block != nullptr) {
					// store block
					new_net.blocks.push_back(move(block));
					// block found
					block_not_found = false;
				}
				else {
					// block not found
					block_not_found = true;
				}
			}

			// drop "B"
			in >> tmpstr;

			// log pin parsing failure
			if (fp.logMin()) {
				if (block_not_found && !pin_not_found) {
					cout << "IO>  Net " << id << "'s block \"" << net_block << "\"";
					cout << " cannot be retrieved; consider checking net / blocks file" << endl;
				}
				else if (block_not_found && pin_not_found) {
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

			for (Block const* block : n.blocks) {
				cout << "DBG_IO> ";
				cout << " block " << block->id << endl;
			}

			for (Pin const* pin : n.terminals) {
				cout << "DBG_IO> ";
				cout << " pin " << pin->id << endl;
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
	enum FLAGS : int {POWER = 0, THERMAL = 1, TSV_DENSITY = 2};
	int flag, flag_start, flag_stop;
	double max_temp, min_temp;
	int id;

	// sanity check
	if (fp.thermalAnalyzer.power_maps.empty() || fp.thermalAnalyzer.thermal_map.empty()) {
		return;
	}

	if (fp.logMed()) {
		cout << "IO> ";

		if (fp.thermal_analyser_run) {
			cout << "Generating thermal map ..." << endl;
		}
		else {
			cout << "Generating power maps, TSV-density maps, and thermal map ..." << endl;
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
	if (fp.thermal_analyser_run) {
		flag_start = flag_stop = FLAGS::THERMAL;
	}
	else {
		flag_start = FLAGS::POWER;
		flag_stop = FLAGS::TSV_DENSITY;
	}
	//
	// actual map generation	
	for (flag = flag_start; flag <= flag_stop; flag++) {

		// power and TSV-density maps for all layers
		if (flag == FLAGS::POWER || flag == FLAGS::TSV_DENSITY) {
			layer_limit = fp.IC.layers;
		}
		// thermal map only for layer 0
		else if (flag == FLAGS::THERMAL) {
			layer_limit = 1;
		}

		for (cur_layer = 0; cur_layer < layer_limit; cur_layer++) {
			// build up file names
			stringstream gp_out_name;
			stringstream data_out_name;
			if (flag == FLAGS::POWER) {
				gp_out_name << fp.benchmark << "_" << cur_layer + 1 << "_power.gp";
				data_out_name << fp.benchmark << "_" << cur_layer + 1 << "_power.data";
			}
			else if (flag == FLAGS::THERMAL) {
				gp_out_name << fp.benchmark << "_" << cur_layer + 1 << "_thermal.gp";
				data_out_name << fp.benchmark << "_" << cur_layer + 1 << "_thermal.data";
			}
			else if (flag == FLAGS::TSV_DENSITY) {
				gp_out_name << fp.benchmark << "_" << cur_layer + 1 << "_TSV_density.gp";
				data_out_name << fp.benchmark << "_" << cur_layer + 1 << "_TSV_density.data";
			}

			// init file stream for gnuplot script
			gp_out.open(gp_out_name.str().c_str());
			// init file stream for data file
			data_out.open(data_out_name.str().c_str());

			// file header for data file
			if (flag == FLAGS::POWER) {
				data_out << "# X Y power" << endl;
			}
			else if (flag == FLAGS::THERMAL) {
				data_out << "# X Y thermal" << endl;
			}
			else if (flag == FLAGS::TSV_DENSITY) {
				data_out << "# X Y TSV_density" << endl;
			}

			// output grid values for power maps
			if (flag == FLAGS::POWER) {

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
			else if (flag == FLAGS::THERMAL) {
				max_temp = 0.0;
				min_temp = 1.0e6;

				for (x = 0; x < ThermalAnalyzer::THERMAL_MAP_DIM; x++) {
					for (y = 0; y < ThermalAnalyzer::THERMAL_MAP_DIM; y++) {
						data_out << x << "	" << y << "	" << fp.thermalAnalyzer.thermal_map[x][y].temp << endl;
						// also track max and min temp
						max_temp = max(max_temp, fp.thermalAnalyzer.thermal_map[x][y].temp);
						min_temp = min(min_temp, fp.thermalAnalyzer.thermal_map[x][y].temp);
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
			else if (flag == FLAGS::TSV_DENSITY) {

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
			if (flag == FLAGS::POWER) {
				gp_out << "set title \"Padded and Scaled Power Map - " << fp.benchmark << ", Layer " << cur_layer + 1 << "\"" << endl;
			}
			else if (flag == FLAGS::THERMAL) {
				gp_out << "set title \"Thermal Map - " << fp.benchmark << ", Layer " << cur_layer + 1 << "\"" << endl;
			}
			else if (flag == FLAGS::TSV_DENSITY) {
				gp_out << "set title \"TSV-Density Map - " << fp.benchmark << ", Layer " << cur_layer + 1 << "\"" << endl;
			}

			gp_out << "set terminal pdfcairo enhanced font \"Gill Sans, 12\"" << endl;
			gp_out << "set output \"" << gp_out_name.str() << ".pdf\"" << endl;
			gp_out << "set size square" << endl;

			// different 2D ranges for maps; consider dummy data row and
			// column, since gnuplot option corners2color cuts off last row
			// and column
			if (flag == FLAGS::POWER) {
				gp_out << "set xrange [0:" << ThermalAnalyzer::POWER_MAPS_DIM << "]" << endl;
				gp_out << "set yrange [0:" << ThermalAnalyzer::POWER_MAPS_DIM << "]" << endl;
			}
			else if (flag == FLAGS::THERMAL	|| flag == FLAGS::TSV_DENSITY) {
				gp_out << "set xrange [0:" << ThermalAnalyzer::THERMAL_MAP_DIM << "]" << endl;
				gp_out << "set yrange [0:" << ThermalAnalyzer::THERMAL_MAP_DIM << "]" << endl;
			}

			// power maps: scale, label for cbrange
			if (flag == FLAGS::POWER) {
				// label for power density
				gp_out << "set cblabel \"Power Density [10^{-2} {/Symbol m}W/{/Symbol m}m^2]\"" << endl;
			}
			// thermal maps: scale, label for cbrange
			else if (flag == FLAGS::THERMAL) {
				// fixed scale to avoid remapping to extended range
				gp_out << "set cbrange [" << min_temp << ":" << max_temp << "]" << endl;
				// thermal estimation, correlates w/ power density
				gp_out << "set cblabel \"Estimated Temperature [K]\"" << endl;
			}
			// TSV-density maps: scale, label for cbrange
			else if (flag == FLAGS::TSV_DENSITY) {
				// fixed scale
				gp_out << "set cbrange [0:100]" << endl;
				// (TODO) also possible: fixed log scale to emphasize both
				// low densities (single TSVs) as well as large densities
				// (TSV cluster, vertical buses)
				//gp_out << "set log cb" << endl;
				//gp_out << "set cbrange [0.1:100]" << endl;
				// label for power density
				gp_out << "set cblabel \"TSV-Density [%]\"" << endl;
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
			if (flag == FLAGS::POWER && ThermalAnalyzer::POWER_MAPS_PADDED_BINS > 0) {
				gp_out << "set obj 1 rect from ";
				gp_out << ThermalAnalyzer::POWER_MAPS_PADDED_BINS << ", " << ThermalAnalyzer::POWER_MAPS_PADDED_BINS << " to ";
				gp_out << ThermalAnalyzer::POWER_MAPS_DIM - ThermalAnalyzer::POWER_MAPS_PADDED_BINS << ", ";
				gp_out << ThermalAnalyzer::POWER_MAPS_DIM - ThermalAnalyzer::POWER_MAPS_PADDED_BINS << " ";
				gp_out << "front fillstyle empty border rgb \"white\" linewidth 3" << endl;
			}

			// for thermal maps: draw rectangles for hotspot regions
			if (flag == FLAGS::THERMAL) {

				id = 1;

				for (x = 0; x < ThermalAnalyzer::THERMAL_MAP_DIM; x++) {
					for (y = 0; y < ThermalAnalyzer::THERMAL_MAP_DIM; y++) {

						// mark bins belonging to a hotspot region
						if (fp.thermalAnalyzer.thermal_map[x][y].hotspot_id != ThermalAnalyzer::HOTSPOT_UNDEFINED &&
								fp.thermalAnalyzer.thermal_map[x][y].hotspot_id != ThermalAnalyzer::HOTSPOT_BACKGROUND) {
							gp_out << "set obj " << id << " rect from ";
							gp_out << x << ", " << y << " to ";
							gp_out << x + 1 << ", " << y + 1 << " ";
							gp_out << "front fillstyle empty border ";
							gp_out << "rgb \"white\" linewidth 1";
							gp_out << endl;

							id++;
						}

						// for clustering dbg, also mark
						// background and undefined bins
						if (Clustering::DBG_HOTSPOT_PLOT) {

							gp_out << "set obj " << id << " rect from ";
							gp_out << x << ", " << y << " to ";
							gp_out << x + 1 << ", " << y + 1 << " ";
							gp_out << "front fillstyle empty border ";

							if (fp.thermalAnalyzer.thermal_map[x][y].hotspot_id == ThermalAnalyzer::HOTSPOT_UNDEFINED) {
								gp_out << "rgb \"red\" linewidth 1";
							}
							else if (fp.thermalAnalyzer.thermal_map[x][y].hotspot_id == ThermalAnalyzer::HOTSPOT_BACKGROUND) {
								gp_out << "rgb \"black\" linewidth 1";
							}

							gp_out << endl;

							id++;
						}
					}
				}
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
	if (fp.thermal_analyser_run) {
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
	ratio_inv = 1.0 / fp.IC.die_AR;
	tics = max(fp.IC.outline_x, fp.IC.outline_y) / 5;

	// color for alignment rects; for fulfilled alignment, green-ish color
	alignment_color_fulfilled = "#00A000";
	// color for alignment rects; for failed alignment, red-ish color
	alignment_color_failed = "#A00000";
	// color for alignment rects; for undefined alignment, blue-ish color
	alignment_color_undefined = "#0000A0";

	for (cur_layer = 0; cur_layer < fp.IC.layers; cur_layer++) {
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
		gp_out << "set xrange [0:" << fp.IC.outline_x << "]" << endl;
		gp_out << "set yrange [0:" << fp.IC.outline_y << "]" << endl;
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
			gp_out << " at " << cur_block.bb.ll.x + 0.01 * fp.IC.outline_x;
			gp_out << "," << cur_block.bb.ll.y + 0.01 * fp.IC.outline_y;
			gp_out << " font \"Gill Sans,4\"";
			// prevents generating subscripts for underscore in labels
			gp_out << " noenhanced" << endl;
		}

		// output TSVs (blocks)
		for (TSV_Island const& TSV_group : fp.TSVs) {

			if (TSV_group.layer != cur_layer) {
				continue;
			}

			// block rectangles
			gp_out << "set obj rect";
			gp_out << " from " << TSV_group.bb.ll.x << "," << TSV_group.bb.ll.y;
			gp_out << " to " << TSV_group.bb.ur.x << "," << TSV_group.bb.ur.y;
			gp_out << " fillcolor rgb \"#704a30\" fillstyle solid";
			gp_out << endl;

			// label
			gp_out << "set label \"" << TSV_group.id << "\"";
			gp_out << " at " << TSV_group.bb.ll.x + 0.01 * fp.IC.outline_x;
			gp_out << "," << TSV_group.bb.ll.y + 0.01 * fp.IC.outline_y;
			gp_out << " font \"Gill Sans,2\"";
			// prevents generating subscripts for underscore in labels
			gp_out << " noenhanced" << endl;
		}

		// check alignment fulfillment; draw accordingly colored rectangles around
		// affected blocks
		//
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
						if (alignment_rect.w >= req.alignment_x) {

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
						if (alignment_rect.w <= req.alignment_x) {

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
						if (Math::doubleComp(alignment_rect.w,  req.alignment_x)) {

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
						if (alignment_rect.h >= req.alignment_y) {

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
						if (alignment_rect.h <= req.alignment_y) {

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
						if (Math::doubleComp(alignment_rect.h,  req.alignment_y)) {

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
							gp_out << " to " << alignment_rect.ll.x + 0.01 * fp.IC.outline_x << "," << alignment_rect.ll.y + 0.01 * fp.IC.outline_y;
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
							gp_out << " to " << alignment_rect.ll.x + 0.01 * fp.IC.outline_x << "," << alignment_rect.ll.y + 0.01 * fp.IC.outline_y;
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
	for (cur_layer = 0; cur_layer < fp.IC.layers; cur_layer++) {

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
			file << "	" << cur_block.bb.w * Math::SCALE_UM_M;
			file << "	" << cur_block.bb.h * Math::SCALE_UM_M;
			file << "	" << cur_block.bb.ll.x * Math::SCALE_UM_M;
			file << "	" << cur_block.bb.ll.y * Math::SCALE_UM_M;
			file << "	" << ThermalAnalyzer::HEAT_CAPACITY_SI;
			file << "	" << ThermalAnalyzer::THERMAL_RESISTIVITY_SI;
			file << endl;
		}

		// dummy block to describe layer outline
		file << "outline" << cur_layer + 1;
		file << "	" << fp.IC.outline_x * Math::SCALE_UM_M;
		file << "	" << fp.IC.outline_y * Math::SCALE_UM_M;
		file << "	0.0";
		file << "	0.0";
		file << "	" << ThermalAnalyzer::HEAT_CAPACITY_SI;
		file << "	" << ThermalAnalyzer::THERMAL_RESISTIVITY_SI;
		file << endl;

		// close file stream
		file.close();
	}

	/// generate floorplans for passive Si and bonding layer; considering TSVs (modelled via densities)
	for (cur_layer = 0; cur_layer < fp.IC.layers; cur_layer++) {

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

		// for thermal-analysis fitting runs, we consider one common TSV density
		// for the whole chip outline
		if (fp.thermal_analyser_run) {

			file << "Si_passive_" << cur_layer + 1;
			file << "	" << fp.IC.outline_x * Math::SCALE_UM_M;
			file << "	" << fp.IC.outline_y * Math::SCALE_UM_M;
			file << "	0.0";
			file << "	0.0";
			file << "	" << ThermalAnalyzer::heatCapSi(fp.IC.TSV_group_Cu_Si_ratio, fp.power_blurring_parameters.TSV_density);
			file << "	" << ThermalAnalyzer::thermResSi(fp.IC.TSV_group_Cu_area_ratio, fp.power_blurring_parameters.TSV_density);
			file << endl;

			file_bond << "bond_" << cur_layer + 1;
			file_bond << "	" << fp.IC.outline_x * Math::SCALE_UM_M;
			file_bond << "	" << fp.IC.outline_y * Math::SCALE_UM_M;
			file_bond << "	0.0";
			file_bond << "	0.0";
			file_bond << "	" << ThermalAnalyzer::heatCapBond(fp.IC.TSV_group_Cu_Si_ratio, fp.power_blurring_parameters.TSV_density);
			file_bond << "	" << ThermalAnalyzer::thermResBond(fp.IC.TSV_group_Cu_area_ratio, fp.power_blurring_parameters.TSV_density);
			file_bond << endl;
		}
		// for regular runs, i.e., Corblivar runs, we have to consider different
		// TSV densities for each grid bin, given in the power_maps
		else {

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
					file << "	" << fp.thermalAnalyzer.power_maps_dim_x * Math::SCALE_UM_M;
					file << "	" << fp.thermalAnalyzer.power_maps_dim_y * Math::SCALE_UM_M;
					/// bin lower-left corner; float precision required in
					//order to avoid grid coordinate mismatches
					file << "	" << static_cast<float>(map_x * fp.thermalAnalyzer.power_maps_dim_x * Math::SCALE_UM_M);
					file << "	" << static_cast<float>(map_y * fp.thermalAnalyzer.power_maps_dim_x * Math::SCALE_UM_M);
					// thermal properties, depending on bin's TSV density
					file << "	" << ThermalAnalyzer::heatCapSi(fp.IC.TSV_group_Cu_Si_ratio, fp.thermalAnalyzer.power_maps[cur_layer][x][y].TSV_density);
					file << "	" << ThermalAnalyzer::thermResSi(fp.IC.TSV_group_Cu_area_ratio, fp.thermalAnalyzer.power_maps[cur_layer][x][y].TSV_density);
					file << endl;

					// put grid block as floorplan blocks; bonding layer
					file_bond << "bond_" << cur_layer + 1 << "_" << map_x << ":" << map_y;
					/// bin dimensions
					file_bond << "	" << fp.thermalAnalyzer.power_maps_dim_x * Math::SCALE_UM_M;
					file_bond << "	" << fp.thermalAnalyzer.power_maps_dim_y * Math::SCALE_UM_M;
					/// bin lower-left corner; float precision required in
					//order to avoid grid coordinate mismatches
					file_bond << "	" << static_cast<float>(map_x * fp.thermalAnalyzer.power_maps_dim_x * Math::SCALE_UM_M);
					file_bond << "	" << static_cast<float>(map_y * fp.thermalAnalyzer.power_maps_dim_x * Math::SCALE_UM_M);
					// thermal properties, depending on bin's TSV density
					file_bond << "	" << ThermalAnalyzer::heatCapBond(fp.IC.TSV_group_Cu_Si_ratio, fp.thermalAnalyzer.power_maps[cur_layer][x][y].TSV_density);
					file_bond << "	" << ThermalAnalyzer::thermResBond(fp.IC.TSV_group_Cu_area_ratio, fp.thermalAnalyzer.power_maps[cur_layer][x][y].TSV_density);
					file_bond << endl;
				}
			}
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
	file << "	" << fp.IC.outline_x * Math::SCALE_UM_M;
	file << "	" << fp.IC.outline_y * Math::SCALE_UM_M;
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
	for (cur_layer = 0; cur_layer < fp.IC.layers; cur_layer++) {

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
	for (cur_layer = 0; cur_layer < fp.IC.layers; cur_layer++) {

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

	for (cur_layer = 0; cur_layer < fp.IC.layers; cur_layer++) {

		file << "# BEOL (interconnects) layer " << cur_layer + 1 << endl;
		file << 4 * cur_layer << endl;
		file << "Y" << endl;
		file << "N" << endl;
		file << ThermalAnalyzer::HEAT_CAPACITY_BEOL << endl;
		file << ThermalAnalyzer::THERMAL_RESISTIVITY_BEOL << endl;
		file << fp.IC.BEOL_thickness * Math::SCALE_UM_M << endl;
		file << fp.benchmark << "_HotSpot_BEOL.flp" << endl;
		file << endl;

		file << "# Active Si layer; design layer " << cur_layer + 1 << endl;
		file << 4 * cur_layer + 1 << endl;
		file << "Y" << endl;
		file << "Y" << endl;
		file << ThermalAnalyzer::HEAT_CAPACITY_SI << endl;
		file << ThermalAnalyzer::THERMAL_RESISTIVITY_SI << endl;
		file << fp.IC.Si_active_thickness * Math::SCALE_UM_M << endl;
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
		file << fp.IC.Si_passive_thickness * Math::SCALE_UM_M << endl;
		file << fp.benchmark << "_HotSpot_Si_passive_" << cur_layer + 1 << ".flp" << endl;
		file << endl;

		if (cur_layer < (fp.IC.layers - 1)) {
			file << "# bond layer " << cur_layer + 1 << "; for F2B bonding to next die " << cur_layer + 2 << endl;
			file << 4 * cur_layer + 3 << endl;
			file << "Y" << endl;
			file << "N" << endl;
			// dummy values, proper values (depending on TSV densities) are in
			// the actual floorplan file
			file << ThermalAnalyzer::HEAT_CAPACITY_BOND << endl;
			file << ThermalAnalyzer::THERMAL_RESISTIVITY_BOND << endl;
			file << fp.IC.bond_thickness * Math::SCALE_UM_M << endl;
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
