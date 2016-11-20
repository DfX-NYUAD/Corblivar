/*
 * =====================================================================================
 *
 *    Description:  Corblivar IO handler
 *
 *    Copyright (C) 2013-2016 Johann Knechtel, johann aett jknechtel dot de
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
#include "CorblivarCore.hpp"

/// parse program parameter, config file, and further files
void IO::parseParametersFiles(FloorPlanner& fp, int const& argc, char** argv) {
	int file_version;
	size_t last_slash;
	std::ifstream in;
	std::string config_file, technology_file;
	std::stringstream results_file;
	std::stringstream blocks_file;
	std::stringstream alignments_file;
	std::stringstream pins_file;
	std::stringstream power_density_file;
	std::stringstream nets_file;
	std::string tmpstr;
	ThermalAnalyzer::MaskParameters mask_parameters;

	std::stringstream GT_fp_file;
	// (TODO) handle individual pins for each block
	//std::stringstream GT_pins_file;
	std::stringstream GT_power_file;

	// print command-line parameters
	if (argc < 4) {
		std::cout << "IO> Usage: " << argv[0] << " benchmark_name config_file benchmarks_dir [solution_file] [TSV_density]" << std::endl;
		std::cout << "IO> " << std::endl;
		std::cout << "IO> Mandatory parameter ``benchmark_name'': any name, should be same as benchmark's files names" << std::endl;
		std::cout << "IO> Mandatory parameter ``config_file'' format: see provided Corblivar.conf" << std::endl;
		std::cout << "IO> Mandatory parameter ``benchmarks_dir'': folder containing actual benchmark files" << std::endl;
		std::cout << "IO> Optional parameter ``solution_file'': re-evaluate w/ given Corblivar solution" << std::endl;
		std::cout << "IO> Optional parameter ``TSV density'': average TSV density to be considered across all dies, to be given in \%" << std::endl;

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

	// initially assume benchmark not to be in GATech format/syntax
	fp.IO_conf.GT_benchmark = false;

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

	GT_fp_file << argv[3] << fp.benchmark << ".fpi";
	fp.IO_conf.GT_fp_file = GT_fp_file.str();

	// (TODO) handle individual pins for each block
	//GT_pins_file << argv[3] << fp.benchmark << ".plf";
	//fp.IO_conf.GT_pins_file = GT_pins_file.str();

	GT_power_file << argv[3] << fp.benchmark << ".pow";
	fp.IO_conf.GT_power_file = GT_power_file.str();

	// determine path of technology file; same as config file per definition
	last_slash = config_file.find_last_of('/');
	if (last_slash == std::string::npos) {
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
		std::cout << "IO> ";
		std::cout << "No such config file: " << config_file << std::endl;
		exit(1);
	}
	in.close();

	// blocks file; determine whether benchmark is GSRC or GATech type
	in.open(fp.IO_conf.blocks_file.c_str());
	if (!in.good()) {
		std::cout << "IO> GSRC-style blocks file missing: " << fp.IO_conf.blocks_file << std::endl;

		// if (GSRC) blocks file not found, check for unified GATech floorplan file
		//
		std::cout << "IO> Checking for GATech floorplan file: " << fp.IO_conf.GT_fp_file << std::endl;
		in.close();
		in.open(fp.IO_conf.GT_fp_file.c_str());
		if (!in.good()) {
			std::cout << "IO>  Failure; GATech floorplan file missing: " << fp.IO_conf.GT_fp_file << std::endl;
			exit(1);
		}
		else {
			fp.IO_conf.GT_benchmark = true;
			std::cout << "IO>  Success; GATech floorplan file available" << std::endl;
		}
	}
	in.close();

	// alignments file
	in.open(fp.IO_conf.alignments_file.c_str());
	// memorize file availability
	fp.IO_conf.alignments_file_avail = in.good();

	if (!in.good() && fp.logMin()) {
		std::cout << "IO> ";
		std::cout << "Note: alignment-requests file missing : " << fp.IO_conf.alignments_file<< std::endl;
		std::cout << "IO> Block alignment cannot be performed; is deactivated." << std::endl;
		std::cout << std::endl;
	}
	in.close();

	// pins file
	//
	if (fp.IO_conf.GT_benchmark) {
		// (TODO) handle individual pins for each block
		//in.open(fp.IO_conf.GT_pins_file.c_str());
		//if (!in.good()) {
		//	std::cout << "IO> Pins file missing: " << fp.IO_conf.GT_pins_file << std::endl;
		//	exit(1);
		//}
	}
	else {
		in.open(fp.IO_conf.pins_file.c_str());
		if (!in.good()) {
			std::cout << "IO> Pins file missing: " << fp.IO_conf.pins_file << std::endl;
			exit(1);
		}
	}
	in.close();

	// power file
	if (fp.IO_conf.GT_benchmark) {
		in.open(fp.IO_conf.GT_power_file.c_str());
	}
	else {
		in.open(fp.IO_conf.power_density_file.c_str());
	}
	// memorize file availability
	fp.IO_conf.power_density_file_avail = in.good();

	if (!in.good()) {
		std::cout << "IO> ";
		if (fp.IO_conf.GT_benchmark) {
			std::cout << "Note: GATech power file missing : " << fp.IO_conf.GT_power_file << std::endl;
		}
		else {
			std::cout << "Note: power density file missing : " << fp.IO_conf.power_density_file << std::endl;
		}
		std::cout << "IO> Thermal analysis and optimization cannot be performed; is deactivated." << std::endl;
		std::cout << std::endl;

		// for thermal-analyser runs, the power density files are mandatory
		if (fp.thermal_analyser_run) {
			exit(1);
		}
	}
	in.close();

	// nets file; separate only for GSRC, encapsulated in GATech floorplan file
	if (!fp.IO_conf.GT_benchmark) {
		in.open(fp.IO_conf.nets_file.c_str());
		if (!in.good()) {
			std::cout << "IO> ";
			std::cout << "Nets file missing: " << fp.IO_conf.nets_file << std::endl;
			exit(1);
		}
		in.close();
	}

	// additional command-line parameters
	//
	// additional parameter for solution file given; consider file for readin
	if (argc > 4) {

		fp.IO_conf.solution_file = argv[4];
		// open file if possible
		fp.IO_conf.solution_in.open(fp.IO_conf.solution_file.c_str());
		if (!fp.IO_conf.solution_in.good())
		{
			std::cout << "IO> ";
			std::cout << "No such solution file: " << fp.IO_conf.solution_file << std::endl;
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
		std::cout << "IO> Parsing config file ..." << std::endl;
	}

	// sanity check for file version
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> file_version;

	if (file_version != IO::CONFIG_VERSION) {
		std::cout << "IO> Wrong version of config file; required version is \"" << IO::CONFIG_VERSION << "\"; consider using matching config file!" << std::endl;
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
		std::cout << "IO> Provide a positive packing iterations count or set 0 to disable!" << std::endl;
		exit(1);
	}

	// sanity check for packing and block rotation
	if (fp.layoutOp.parameters.enhanced_hard_block_rotation && (fp.layoutOp.parameters.packing_iterations > 0)) {
		std::cout << "IO> Activate only guided hard block rotation OR layout packing; both cannot be performed!" << std::endl;
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
	in >> fp.layoutOp.parameters.shrink_die;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.layoutOp.parameters.trivial_HPWL;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.layoutOp.parameters.signal_TSV_clustering;

	// TSV clustering is only applicable if non-trivial HPWL are estimated
	fp.layoutOp.parameters.signal_TSV_clustering &= !fp.layoutOp.parameters.trivial_HPWL;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.opt_flags.alignment_WL_estimate;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.schedule.loop_factor;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.schedule.loop_limit;

	// sanity check for positive, non-zero parameters
	if (fp.schedule.loop_factor <= 0.0 || fp.schedule.loop_limit <= 0.0) {
		std::cout << "IO> Provide positive, non-zero SA loop parameters!" << std::endl;
		exit(1);
	}

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.schedule.temp_init_factor;

	// sanity check for positive, non-zero factor
	if (fp.schedule.temp_init_factor <= 0.0) {
		std::cout << "IO> Provide positive, non-zero SA start temperature scaling factor!" << std::endl;
		exit(1);
	}

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.schedule.temp_factor_phase1;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.schedule.temp_factor_phase1_limit;

	// sanity check for dependent temperature-scaling factors
	if (fp.schedule.temp_factor_phase1 >= fp.schedule.temp_factor_phase1_limit) {
		std::cout << "IO> Initial cooling factor for SA phase 1 should be smaller than the related final factor!" << std::endl;
		exit(1);
	}

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.schedule.temp_factor_phase2;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.schedule.temp_factor_phase3;

	// sanity check for positive, non-zero parameters
	if (fp.schedule.temp_factor_phase1 <= 0.0 || fp.schedule.temp_factor_phase2 <= 0.0 || fp.schedule.temp_factor_phase3 <= 0.0) {
		std::cout << "IO> Provide positive, non-zero SA cooling factors for phases 1, 2 and 3!" << std::endl;
		exit(1);
	}

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.weights.area_outline;

	// sanity check for non-zero cost factor; optimization shall be always guided
	// towards area and fixed-outline fitting since the latter relates to the SA
	// optimization flow
	if (fp.weights.area_outline <= 0.0) {
		std::cout << "IO> Provide positive, non-zero cost factor for area and fixed-outline fitting!" << std::endl;
		exit(1);
	}

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.weights.thermal;

	// memorize if thermal optimization should be performed
	fp.opt_flags.thermal = (fp.weights.thermal > 0.0 && fp.IO_conf.power_density_file_avail);

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.weights.WL;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.weights.routing_util;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.weights.TSVs;

	// memorize if interconnects optimization should be performed
	fp.opt_flags.interconnects = (fp.weights.WL > 0.0 || fp.weights.routing_util > 0.0 || fp.weights.TSVs > 0.0);

	// memorize if routing utilization should be performed; may be separated from
	// interconnects optimization
	fp.opt_flags.routing_util = (fp.weights.routing_util > 0.0);

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.weights.alignment;

	// memorize if alignment optimization should be performed
	fp.opt_flags.alignment = (fp.weights.alignment > 0.0 && fp.IO_conf.alignments_file_avail);
	// also memorize in layout-operations handler
	fp.layoutOp.parameters.opt_alignment = fp.opt_flags.alignment;

	// update flag for rough estimate of WL of massive interconnects; only to be
	// applied when block-alignment is _not_optimized, otherwise the actual alignment
	// provides a more accurate estimate
	fp.opt_flags.alignment_WL_estimate = (fp.opt_flags.alignment_WL_estimate && !fp.opt_flags.alignment);

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.weights.timing;

	// memorize if timing optimization should be performed
	fp.opt_flags.timing = fp.weights.timing > 0.0;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.weights.voltage_assignment;

	// memorize if voltage optimization should be performed; to be updated below after
	// technology file is parsed
	fp.opt_flags.voltage_assignment = fp.weights.voltage_assignment > 0.0;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.weights.thermal_leakage;

	// memorize if thermal-leakage mitigation should be performed; requires also power maps
	fp.opt_flags.thermal_leakage = (fp.weights.thermal_leakage > 0.0 && fp.IO_conf.power_density_file_avail);

	// sanity check for positive cost factors
	if (
		fp.weights.thermal < 0.0 ||
		fp.weights.WL < 0.0 ||
		fp.weights.routing_util < 0.0 ||
		fp.weights.TSVs < 0.0 ||
		fp.weights.alignment < 0.0 ||
		fp.weights.timing < 0.0 ||
		fp.weights.voltage_assignment < 0.0 ||
		fp.weights.thermal_leakage < 0.0
	) {
		std::cout << "IO> Provide positive cost factors!" << std::endl;
		exit(1);
	}

	// sanity check for sum of cost factors
	if (std::abs(
			fp.weights.area_outline +
			fp.weights.thermal +
			fp.weights.WL +
			fp.weights.routing_util +
			fp.weights.TSVs +
			fp.weights.alignment +
			fp.weights.timing +
			fp.weights.voltage_assignment +
			fp.weights.thermal_leakage
	- 1.0) > 0.1) {
		std::cout << "IO> Cost factors should sum up to approx. 1!" << std::endl;
		exit(1);
	}

	// thermal-related leakage; weight factors
	//
	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.leakageAnalyzer.parameters.weight_entropy;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.leakageAnalyzer.parameters.weight_correlation;

	// sanity check for positive cost factors
	if (
		fp.leakageAnalyzer.parameters.weight_entropy < 0.0 ||
		fp.leakageAnalyzer.parameters.weight_correlation < 0.0
	) {
		std::cout << "IO> Provide positive thermal-leakage cost factors!" << std::endl;
		exit(1);
	}

	// sanity check for sum of cost factors
	if (std::abs(
			fp.leakageAnalyzer.parameters.weight_entropy +
			fp.leakageAnalyzer.parameters.weight_correlation
	- 1.0) > 0.1) {
		std::cout << "IO> Thermal-leakage cost factors should sum up to approx. 1!" << std::endl;
		exit(1);
	}

	// voltage assignment; weight factors
	//
	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.voltageAssignment.parameters.weight_power_saving;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.voltageAssignment.parameters.weight_corners;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.voltageAssignment.parameters.weight_modules_count;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.voltageAssignment.parameters.weight_power_variation;

	// sanity check for positive cost factors
	if (
		fp.voltageAssignment.parameters.weight_power_saving < 0.0 ||
		fp.voltageAssignment.parameters.weight_corners < 0.0 ||
		fp.voltageAssignment.parameters.weight_modules_count < 0.0 ||
		fp.voltageAssignment.parameters.weight_power_variation < 0.0
	) {
		std::cout << "IO> Provide positive voltage-assignment cost factors!" << std::endl;
		exit(1);
	}

	// sanity check for sum of cost factors
	if (std::abs(
			fp.voltageAssignment.parameters.weight_power_saving +
			fp.voltageAssignment.parameters.weight_corners +
			fp.voltageAssignment.parameters.weight_modules_count +
			fp.voltageAssignment.parameters.weight_power_variation
	- 1.0) > 0.1) {
		std::cout << "IO> Voltage-assignment cost factors should sum up to approx. 1!" << std::endl;
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
		std::cout << "IO> Provide a positive, non-zero power blurring impulse factor!" << std::endl;
		exit(1);
	}
	if (mask_parameters.mask_boundary_value <= 0.0) {
		std::cout << "IO> Provide a positive, non-zero power blurring mask boundary value!" << std::endl;
		exit(1);
	}

	// sanity check for reasonable mask parameters
	if (mask_parameters.impulse_factor <= mask_parameters.mask_boundary_value) {
		std::cout << "IO> Provide a power blurring impulse factor larger than the power blurring mask boundary value!" << std::endl;
		exit(1);
	}

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> mask_parameters.power_density_scaling_padding_zone;

	// sanity check for positive parameter
	if (mask_parameters.power_density_scaling_padding_zone < 1.0) {
		std::cout << "IO> Provide a positive (greater or equal 1.0) power-density scaling factor!" << std::endl;
		exit(1);
	}

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> mask_parameters.power_density_scaling_TSV_region;

	// sanity check for parameter range
	if (mask_parameters.power_density_scaling_TSV_region > 1.0 || mask_parameters.power_density_scaling_TSV_region < 0.0) {
		std::cout << "IO> Provide a power-density down-scaling factor for TSV regions between 0.0 and 1.0!" << std::endl;
		exit(1);
	}

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> mask_parameters.temp_offset;

	// sanity check for positive parameter
	if (mask_parameters.temp_offset < 0.0) {
		std::cout << "IO> Provide a positive temperature offset!" << std::endl;
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
		std::cout << "IO> ";
		std::cout << "No such technology file: " << technology_file << std::endl;
		exit(1);
	}

	if (fp.logMin()) {
		std::cout << "IO> Parsing technology file ..." << std::endl;
	}

	// reset tmpstr
	tmpstr = "";

	// sanity check for file version
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> file_version;

	if (file_version != IO::TECHNOLOGY_VERSION) {
		std::cout << file_version << std::endl;
		std::cout << "IO> Wrong version of technology file; required version is \"" << IO::TECHNOLOGY_VERSION << "\"; consider using matching technology file!" << std::endl;
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
	// also memorize in voltage-volumes handler
	fp.voltageAssignment.parameters.layers = fp.IC.layers;

	// sanity check for positive, non-zero layer
	if (fp.IC.layers <= 0) {
		std::cout << "IO> Provide positive, non-zero layer count!" << std::endl;
		exit(1);
	}

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.IC.outline_x;
	// also memorize for layout operations
	fp.layoutOp.parameters.outline.x = fp.IC.outline_x;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.IC.outline_y;
	// also memorize for layout operations
	fp.layoutOp.parameters.outline.y = fp.IC.outline_y;

	// sanity check for positive, non-zero dimensions
	if (fp.IC.outline_x <= 0.0 || fp.IC.outline_y <= 0.0) {
		std::cout << "IO> Provide positive, non-zero outline dimensions!" << std::endl;
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
		std::cout << "IO> Provide a positive, non-zero block scaling factor!" << std::endl;
		exit(1);
	}

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.IC.power_scale;

	// sanity check for power scaling factor
	if (fp.IC.power_scale <= 0.0) {
		std::cout << "IO> Provide a positive, non-zero power scaling factor!" << std::endl;
		exit(1);
	}

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.IC.outline_shrink;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.techParameters.die_thickness;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.techParameters.Si_active_thickness;

	// determine thickness of passive Si layer
	fp.techParameters.Si_passive_thickness = fp.techParameters.die_thickness - fp.techParameters.Si_active_thickness;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.techParameters.BEOL_thickness;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.techParameters.bond_thickness;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.techParameters.TSV_dimension;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.techParameters.TSV_pitch;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.techParameters.TSV_frame_dim;

	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.techParameters.TSV_per_cluster_limit;

	// determine Cu area fraction for TSV groups
	fp.techParameters.TSV_group_Cu_area_ratio = (fp.techParameters.TSV_dimension * fp.techParameters.TSV_dimension) /
		(fp.techParameters.TSV_pitch * fp.techParameters.TSV_pitch);

	// multiple-voltages values
	//
	unsigned voltages_count;

	// voltages count
	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> voltages_count;

	// sanity check for enough bits to encode all voltage levels
	if (voltages_count > MultipleVoltages::MAX_VOLTAGES) {
		std::cout << "IO> Recompile with increased MultipleVoltages::MAX_VOLTAGES; voltage levels given, i.e., bits required to encode: " << voltages_count;
		std::cout << "; count of bits that can be encoded according to MultipleVoltages::MAX_VOLTAGES: " << MultipleVoltages::MAX_VOLTAGES << std::endl;
		exit(1);
	}

	// actual voltages; drop header
	in >> tmpstr;
	while (tmpstr != "values" && !in.eof())
		in >> tmpstr;

	// actual voltages; parse values
	for (unsigned v = 1; v <= voltages_count; v++) {
		in >> tmpstr;
		fp.voltageAssignment.parameters.voltages.push_back(atof(tmpstr.c_str()));
	}

	// sanity check for at least one voltage defined
	if (fp.voltageAssignment.parameters.voltages.empty() || voltages_count == 0) {
		std::cout << "IO> Check the voltage, power-scaling and delay-scaling factors; indicated are " << voltages_count << " values, provided are:";
		std::cout << " voltages: " << fp.voltageAssignment.parameters.voltages.size();
		std::cout << " power-scaling: " << fp.voltageAssignment.parameters.voltages_power_factors.size();
		std::cout << " delay-scaling: " << fp.voltageAssignment.parameters.voltages_delay_factors.size();
		std::cout << std::endl;
		exit(1);
	}

	// sanity check for voltages order; from lowest to highest
	for (auto iter = std::next(fp.voltageAssignment.parameters.voltages.begin()); iter != fp.voltageAssignment.parameters.voltages.end(); ++iter) {

		if (*std::prev(iter) > *iter) {
			std::cout << "IO> Check the voltages; they have to be provided in the order from lowest to highest!" << std::endl;
			exit(1);
		}
	}

	// deactivate voltage assignment in case only one voltage shall be considered
	fp.opt_flags.voltage_assignment &= fp.voltageAssignment.parameters.voltages.size() > 1;

	// power-scaling factors; drop header
	in >> tmpstr;
	while (tmpstr != "values" && !in.eof())
		in >> tmpstr;

	// power-scaling factors; parse values
	for (unsigned v = 1; v <= voltages_count; v++) {
		in >> tmpstr;
		fp.voltageAssignment.parameters.voltages_power_factors.push_back(atof(tmpstr.c_str()));
	}

	// delay-scaling factors; drop header
	in >> tmpstr;
	while (tmpstr != "values" && !in.eof())
		in >> tmpstr;

	// delay-scaling factors; parse values
	for (unsigned v = 1; v <= voltages_count; v++) {
		in >> tmpstr;
		fp.voltageAssignment.parameters.voltages_delay_factors.push_back(atof(tmpstr.c_str()));
	}

	// sanity check for appropriate, i.e., positive non-zero values
	for (auto& v : fp.voltageAssignment.parameters.voltages) {
		if (v <= 0.0) {
			std::cout << "IO> Check the provided voltage values, they should be all positive and non-zero!" << std::endl;
			exit(1);
		}
	}
	for (auto& v : fp.voltageAssignment.parameters.voltages_power_factors) {
		if (v <= 0.0) {
			std::cout << "IO> Check the provided power-scaling values, they should be all positive and non-zero!" << std::endl;
			exit(1);
		}
	}
	for (auto& v : fp.voltageAssignment.parameters.voltages_delay_factors) {
		if (v <= 0.0) {
			std::cout << "IO> Check the provided delay-scaling values, they should be all positive and non-zero!" << std::endl;
			exit(1);
		}
	}

	// delay threshold
	in >> tmpstr;
	while (tmpstr != "value" && !in.eof())
		in >> tmpstr;
	in >> fp.IC.delay_threshold;
	// memorize initially parsed threshold separately
	fp.IC.delay_threshold_initial = fp.IC.delay_threshold;

	// sanity check for appropriate, i.e., positive non-zero threshold
	if (fp.IC.delay_threshold <= 0.0) {
		std::cout << "IO> Check the provided delay threshold, it should be positive and non-zero!" << std::endl;
		exit(1);
	}

	// the achievable frequency is derived from this delay threshold/constraint:
	// f = 1.0 / delay; delay is given in ns, scale to seconds for f in [Hz]
	fp.IC.frequency = 1.0 / (fp.IC.delay_threshold * 1.0e-09);

	in.close();

	if (fp.logMin()) {
		std::cout << "IO> Done; technology and config values:" << std::endl;

		// log
		std::cout << "IO>  Loglevel (1 to 3 for minimal, medium, maximal): " << fp.log << std::endl;

		// general 3D IC setup
		std::cout << "IO>  Chip -- Layers for 3D IC: " << fp.IC.layers << std::endl;
		std::cout << "IO>  Chip -- Fixed die outline (width, x-dimension) [um]: " << fp.IC.outline_x << std::endl;
		std::cout << "IO>  Chip -- Fixed die outline (height, y-dimension) [um]: " << fp.IC.outline_y << std::endl;
		std::cout << "IO>  Chip -- Block scaling factor: " << fp.IC.blocks_scale << std::endl;
		std::cout << "IO>  Chip -- Final die outline shrink: " << fp.IC.outline_shrink << std::endl;

		// technology parameters
		std::cout << "IO>  Technology -- Power scaling factor: " << fp.IC.power_scale << std::endl;
		std::cout << "IO>  Technology -- Die thickness [um]: " << fp.techParameters.die_thickness << std::endl;
		std::cout << "IO>  Technology -- Active Si layer thickness [um]: " << fp.techParameters.Si_active_thickness << std::endl;
		std::cout << "IO>  Technology -- Passive Si layer thickness [um]: " << fp.techParameters.Si_passive_thickness << std::endl;
		std::cout << "IO>  Technology -- BEOL layer thickness [um]: " << fp.techParameters.BEOL_thickness << std::endl;
		std::cout << "IO>  Technology -- BCB bonding layer thickness [um]: " << fp.techParameters.bond_thickness << std::endl;
		std::cout << "IO>  Technology -- TSV dimension [um]: " << fp.techParameters.TSV_dimension << std::endl;
		std::cout << "IO>  Technology -- TSV pitch [um]: " << fp.techParameters.TSV_pitch << std::endl;
		std::cout << "IO>  Technology -- TSV frames; enforces minimal TSV density [um]: " << fp.techParameters.TSV_frame_dim << std::endl;
		std::cout << "IO>  Technology -- TSV islands; upper limit for TSV per island: " << fp.techParameters.TSV_per_cluster_limit << std::endl;
		std::cout << "IO>  Technology -- TSV islands; Cu vs. Si area fraction: " << fp.techParameters.TSV_group_Cu_area_ratio << std::endl;

		// technology parameters for multi-voltage domains
		//
		std::cout << "IO>  Technology -- Multi-voltage domains; voltage levels: ";
		for (unsigned v = 0; v < fp.voltageAssignment.parameters.voltages.size(); v++) {

			// last value shall have no tailing comma
			if (v == fp.voltageAssignment.parameters.voltages.size() - 1) {
				std::cout << fp.voltageAssignment.parameters.voltages[v];
			}
			else {
				std::cout << fp.voltageAssignment.parameters.voltages[v] << ", ";
			}
		}
		std::cout << std::endl;

		std::cout << "IO>  Technology -- Multi-voltage domains; power-scaling factors: ";
		for (unsigned v = 0; v < fp.voltageAssignment.parameters.voltages_power_factors.size(); v++) {

			// last value shall have no tailing comma
			if (v == fp.voltageAssignment.parameters.voltages_power_factors.size() - 1) {
				std::cout << fp.voltageAssignment.parameters.voltages_power_factors[v];
			}
			else {
				std::cout << fp.voltageAssignment.parameters.voltages_power_factors[v] << ", ";
			}
		}
		std::cout << std::endl;

		std::cout << "IO>  Technology -- Multi-voltage domains; delay-scaling factors: ";
		for (unsigned v = 0; v < fp.voltageAssignment.parameters.voltages_delay_factors.size(); v++) {

			// last value shall have no tailing comma
			if (v == fp.voltageAssignment.parameters.voltages_delay_factors.size() - 1) {
				std::cout << fp.voltageAssignment.parameters.voltages_delay_factors[v];
			}
			else {
				std::cout << fp.voltageAssignment.parameters.voltages_delay_factors[v] << ", ";
			}
		}
		std::cout << std::endl;

		std::cout << "IO>  Technology -- Global delay threshold (covering module and net delay, in [ns]): " << fp.IC.delay_threshold << std::endl;

		// layout generation options
		std::cout << "IO>  SA -- Layout generation; guided hard block rotation: " << fp.layoutOp.parameters.enhanced_hard_block_rotation << std::endl;
		std::cout << "IO>  SA -- Layout generation; guided soft block shaping: " << fp.layoutOp.parameters.enhanced_soft_block_shaping << std::endl;
		std::cout << "IO>  SA -- Layout generation; packing iterations: " << fp.layoutOp.parameters.packing_iterations << std::endl;
		std::cout << "IO>  SA -- Layout generation; power-aware block handling: " << fp.layoutOp.parameters.power_aware_block_handling << std::endl;
		std::cout << "IO>  SA -- Layout generation; floorplacement handling: " << fp.layoutOp.parameters.floorplacement << std::endl;
		std::cout << "IO>  SA -- Layout generation; adaptive shrinking of dies: " << fp.layoutOp.parameters.shrink_die << std::endl;
		std::cout << "IO>  SA -- Layout generation; trivial HPWL estimation: " << fp.layoutOp.parameters.trivial_HPWL << std::endl;
		std::cout << "IO>  SA -- Layout generation; signal-TSV clustering: " << fp.layoutOp.parameters.signal_TSV_clustering << std::endl;
		if (fp.layoutOp.parameters.trivial_HPWL) {
			std::cout << "IO>     Note: signal-TSV clustering is disabled since trivial HPWL is applied" << std::endl;
		}
		std::cout << "IO>  SA -- Layout generation; rough estimate of WL for massive interconnects (w/o block-alignment optimization): " << fp.opt_flags.alignment_WL_estimate << std::endl;

		// SA loop setup
		std::cout << "IO>  SA -- Inner-loop operation-factor a (ops = N^a for N blocks): " << fp.schedule.loop_factor << std::endl;
		std::cout << "IO>  SA -- Outer-loop upper limit: " << fp.schedule.loop_limit << std::endl;

		// SA cooling schedule
		std::cout << "IO>  SA -- Start temperature scaling factor: " << fp.schedule.temp_init_factor << std::endl;
		std::cout << "IO>  SA -- Initial temperature-scaling factor for phase 1 (adaptive cooling): " << fp.schedule.temp_factor_phase1 << std::endl;
		std::cout << "IO>  SA -- Final temperature-scaling factor for phase 1 (adaptive cooling): " << fp.schedule.temp_factor_phase1_limit << std::endl;
		std::cout << "IO>  SA -- Temperature-scaling factor for phase 2 (reheating and freezing): " << fp.schedule.temp_factor_phase2 << std::endl;
		std::cout << "IO>  SA -- Temperature-scaling factor for phase 3 (brief reheating, escaping local minima) : " << fp.schedule.temp_factor_phase3 << std::endl;

		// SA cost factors
		std::cout << "IO>  SA -- Cost factor for are and fixed-outline fitting: " << fp.weights.area_outline << std::endl;
		std::cout << "IO>  SA -- Cost factor for thermal distribution: " << fp.weights.thermal << std::endl;
		if (!fp.opt_flags.thermal && fp.opt_flags.thermal_leakage) {
			std::cout << "IO>     Note: thermal analysis (not optimization) is conducted since thermal-leakage mitigation is activated" << std::endl;
		}
		if (!fp.IO_conf.power_density_file_avail) {
			std::cout << "IO>     Note: thermal optimization is disabled since no power density file is available" << std::endl;
		}
		std::cout << "IO>  SA -- Cost factor for wirelength: " << fp.weights.WL << std::endl;
		std::cout << "IO>  SA -- Cost factor for routing utilization: " << fp.weights.routing_util << std::endl;
		std::cout << "IO>  SA -- Cost factor for TSVs: " << fp.weights.TSVs << std::endl;
		std::cout << "IO>  SA -- Cost factor for block alignment: " << fp.weights.alignment << std::endl;
		if (!fp.IO_conf.alignments_file_avail) {
			std::cout << "IO>     Note: block alignment is disabled since no alignment-requests file is available" << std::endl;
		}
		std::cout << "IO>  SA -- Cost factor for timing optimization: " << fp.weights.timing << std::endl;
		if (!fp.opt_flags.timing && fp.opt_flags.voltage_assignment) {
			std::cout << "IO>     Note: timing analysis (not optimization) is conducted since voltage assignment is activated" << std::endl;
		}
		std::cout << "IO>  SA -- Cost factor for thermal-leakage mitigation: " << fp.weights.thermal_leakage << std::endl;
		std::cout << "IO>  Thermal-leakage mitigation -- Internal cost factor - Spatial entropy of power maps: " << fp.leakageAnalyzer.parameters.weight_entropy << std::endl;
		std::cout << "IO>  Thermal-leakage mitigation -- Internal cost factor - Pearson correlation of power and thermal map (lowest layer): " << fp.leakageAnalyzer.parameters.weight_correlation << std::endl;
		std::cout << "IO>  SA -- Cost factor for voltage assignment: " << fp.weights.voltage_assignment << std::endl;
		std::cout << "IO>  Voltage assignment -- Internal cost factor - Power saving: " << fp.voltageAssignment.parameters.weight_power_saving << std::endl;
		std::cout << "IO>  Voltage assignment -- Internal cost factor - Power-ring corner minimization: " << fp.voltageAssignment.parameters.weight_corners << std::endl;
		std::cout << "IO>  Voltage assignment -- Internal cost factor - Volume-count minimization: " << fp.voltageAssignment.parameters.weight_modules_count << std::endl;
		std::cout << "IO>  Voltage assignment -- Internal cost factor - Volume-variation minimization: " << fp.voltageAssignment.parameters.weight_power_variation << std::endl;

		// power blurring mask parameters
		std::cout << "IO>  Power-blurring mask parameterization -- TSV density: " << mask_parameters.TSV_density << std::endl;
		std::cout << "IO>  Power-blurring mask parameterization -- Impulse factor: " << mask_parameters.impulse_factor << std::endl;
		std::cout << "IO>  Power-blurring mask parameterization -- Impulse scaling-factor: " << mask_parameters.impulse_factor_scaling_exponent << std::endl;
		std::cout << "IO>  Power-blurring mask parameterization -- Mask-boundary value: " << mask_parameters.mask_boundary_value << std::endl;
		std::cout << "IO>  Power-blurring mask parameterization -- Power-density scaling factor (padding zone): " << mask_parameters.power_density_scaling_padding_zone << std::endl;
		std::cout << "IO>  Power-blurring mask parameterization -- Power-density down-scaling factor (TSV regions): " << mask_parameters.power_density_scaling_TSV_region << std::endl;
		std::cout << "IO>  Power-blurring mask parameterization -- Temperature offset: " << mask_parameters.temp_offset << std::endl;

		std::cout << std::endl;
	}

	// final update of thermal-analysis flag; required when thermal-related leakage is to be analyzed but still requires power-density files
	if (!fp.opt_flags.thermal && fp.opt_flags.thermal_leakage) {
		fp.opt_flags.thermal = (true && fp.IO_conf.power_density_file_avail);
	}
}

/// parse Corblivar solution file, to rerun Corbliar w/ previous data
void IO::parseCorblivarFile(FloorPlanner& fp, CorblivarCore& corb) {
	std::string tmpstr;
	CornerBlockList::Tuple tuple;
	unsigned tuples;
	int cur_layer;
	std::string block_id;
	unsigned dir;
	double width, height;

	if (fp.logMed()) {
		std::cout << "IO> ";
		std::cout << "Initializing Corblivar data from solution file ..." << std::endl;
	}

	// drop solution file header
	while (tmpstr != "data_start" && !fp.IO_conf.solution_in.eof()) {
		fp.IO_conf.solution_in >> tmpstr;
	}

	tuples = 0;
	cur_layer = -1;

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
				std::cout << "IO> Block " << block_id << " cannot be retrieved; ensure solution file and benchmark file match!" << std::endl;
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
			fp.IO_conf.solution_in >> width;

			// block height
			fp.IO_conf.solution_in >> height;

			// reshape block accordingly
			tuple.S->bb.ur.x = tuple.S->bb.ll.x + width;
			tuple.S->bb.ur.y = tuple.S->bb.ll.y + height;
			tuple.S->bb.w = width;
			tuple.S->bb.h = height;

			// recalculate the base delay, which relies on the dimensions
			tuple.S->base_delay = TimingPowerAnalyser::baseDelay(height, width);

			// drop ");"
			fp.IO_conf.solution_in >> tmpstr;

			// sanity check for same number of dies
			if (cur_layer > fp.getLayers() - 1) {
				std::cout << "IO> Parsing error: solution file contains at least " << cur_layer + 1 << " dies; config file is set for " << fp.getLayers() << " dies!" << std::endl;
				exit(1);
			}

			// store successfully parsed tuple into CBL
			corb.editDie(cur_layer).editCBL().insert(std::move(tuple));
			tuples++;
		}
	}

	// sanity check for same number of blocks
	if (fp.getBlocks().size() != tuples) {
		std::cout << "IO> Parsing error: solution file contains " << tuples << " tuples/blocks; read in benchmark contains " << fp.getBlocks().size() << " blocks!" << std::endl;
		exit(1);
	}

	if (fp.logMed()) {
		std::cout << "IO> ";
		std::cout << "Done; parsed " << tuples << " tuples" << std::endl << std::endl;
	}
}

/// parse alignment-requests file
void IO::parseAlignmentRequests(FloorPlanner& fp, std::vector<CorblivarAlignmentReq>& alignments) {
	std::ifstream al_in;
	std::string tmpstr;
	int id;
	std::string block_id;
	Block const* b1;
	Block const* b2;
	std::string handling_str;
	std::string type_str;
	int signals;
	CorblivarAlignmentReq::Type type_x;
	CorblivarAlignmentReq::Type type_y;
	CorblivarAlignmentReq::Handling handling;
	double alignment_x;
	double alignment_y;

	// sanity check for unavailable file
	if (!fp.IO_conf.alignments_file_avail) {
		return;
	}

	if (fp.logMed()) {
		std::cout << "IO> ";
		std::cout << "Parsing alignment requests..." << std::endl;
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

		// global alignment handling
		al_in >> handling_str;

		if (handling_str == "STRICT") {
			handling = CorblivarAlignmentReq::Handling::STRICT;
		}
		else if (handling_str == "FLEXIBLE") {
			handling = CorblivarAlignmentReq::Handling::FLEXIBLE;
		}
		else {
			std::cout << "IO> Unknown global alignment handling: " << handling_str << "; ensure alignment-requests file has correct format!" << std::endl;
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
			if (block_id == RBOD::ID) {
				// link dummy block to alignment request
				b1 = &fp.RBOD;
			}
			// otherwise, we triggered some parsing error
			else {
				std::cout << "IO> Block " << block_id << " cannot be retrieved; ensure alignment-requests file and benchmark file match!" << std::endl;
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
			if (block_id == RBOD::ID) {
				// link dummy block to alignment request
				b2 = &fp.RBOD;
			}
			// otherwise, we triggered some parsing error
			else {
				std::cout << "IO> Block " << block_id << " cannot be retrieved; ensure alignment-requests file and benchmark file match!" << std::endl;
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
			std::cout << "IO> Unknown alignment-request type: " << type_str << "; ensure alignment-requests file has correct format!" << std::endl;
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
			std::cout << "IO> Unknown alignment-request type: " << type_str << "; ensure alignment-requests file has correct format!" << std::endl;
			exit(1);
		}

		// alignment value for y-dimension
		al_in >> alignment_y;

		// drop ");"
		al_in >> tmpstr;

		// generate and store successfully parsed request
		alignments.push_back(CorblivarAlignmentReq(id, handling, signals, b1, b2, type_x, alignment_x, type_y, alignment_y));

		id++;
	}

	// update blocks' status according to alignments
	for (CorblivarAlignmentReq& req : alignments) {

		// memorize blocks with STRICT alignment request as not to be rotated;
		// only if alignment is actually to be considered
		if (req.handling == CorblivarAlignmentReq::Handling::STRICT && fp.opt_flags.alignment) {

			if (req.s_i != &fp.RBOD) {
				req.s_i->rotatable = false;
			}

			if (req.s_j != &fp.RBOD) {
				req.s_j->rotatable = false;
			}
		}

		// memorize pointer to vertical-bus requests
		//
		// link pointer to blocks only now, i.e., after all blocks are handled;
		// otherwise, some new alignments will trigger reallocation of alignments
		// vector and thus invalidate previous pointer
		if (req.vertical_bus()) {

			if (req.s_i != &fp.RBOD) {
				req.s_i->alignments_vertical_bus.push_back(&req);
			}

			if (req.s_j != &fp.RBOD) {
				req.s_j->alignments_vertical_bus.push_back(&req);
			}
		}
	}

	if (IO::DBG) {
		for (CorblivarAlignmentReq const& req : alignments) {
			std::cout << "DBG_IO> " << req.tupleString() << std::endl;
		}
	}

	if (fp.logMed()) {
		std::cout << "IO> ";
		std::cout << "Done; parsed " << id << " alignment requests" << std::endl << std::endl;
	}
}

/// parse blocks file
void IO::parseBlocks(FloorPlanner& fp) {
	std::ifstream blocks_in, pins_in, power_in;
	std::string tmpstr;
	double power = 0.0;
	double blocks_max_area = 0.0, blocks_avg_area = 0.0;
	int soft_blocks = 0;
	double blocks_outline_ratio;
	std::string id;
	int to_parse_soft_blocks, to_parse_hard_blocks, to_parse_terminals;
	bool floorplacement;
	unsigned numerical_id;
	double GT_fp_height, GT_fp_width;
	bool GT_soft_blocks;
	bool GT_terminals;

	if (fp.logMed()) {
		std::cout << "IO> ";
		std::cout << "Parsing blocks..." << std::endl;
	}

	// open GT benchmark files
	if (fp.IO_conf.GT_benchmark) {
		blocks_in.open(fp.IO_conf.GT_fp_file.c_str());
		// (TODO) handle individual pins for each block
		//pins_in.open(fp.IO_conf.GT_pins_file.c_str());
		power_in.open(fp.IO_conf.GT_power_file.c_str());
	}
	// open GSRC benchmark files
	else {
		blocks_in.open(fp.IO_conf.blocks_file.c_str());
		pins_in.open(fp.IO_conf.pins_file.c_str());
		power_in.open(fp.IO_conf.power_density_file.c_str());
	}

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

	// first block id shall be distinct from the dummy id
	numerical_id = Block::DUMMY_NUM_ID;
	numerical_id++;

	// initial parsing for GSRC benchmarks
	//
	if (!fp.IO_conf.GT_benchmark) {

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
	}
	// initial parsing for GATech benchmarks
	//
	else {
		// number of blocks and terminal (to be parsed) not given in the files
		to_parse_soft_blocks = to_parse_hard_blocks = to_parse_terminals = -1;

		// drop header lines until floorplan  section to be parsed
		GT_soft_blocks = false;
		while (tmpstr != "*FLOORPLAN" && !blocks_in.eof())
			blocks_in >> tmpstr;
		// parse floorplan width
		blocks_in >> GT_fp_width;
		// parse floorplan height
		blocks_in >> GT_fp_height;
		// drop "*END"
		blocks_in >> tmpstr;
		
		// apply non-zero outlines
		if (GT_fp_width != 0 && GT_fp_height != 0) {

			fp.layoutOp.parameters.outline.x = fp.IC.outline_x = GT_fp_width;
			fp.layoutOp.parameters.outline.y = fp.IC.outline_y = GT_fp_height;

			// update aspect ratio and area
			fp.IC.die_AR = fp.IC.outline_x / fp.IC.outline_y;
			fp.IC.die_area = fp.IC.outline_x * fp.IC.outline_y;
			fp.IC.stack_area = fp.IC.die_area * fp.IC.layers;

			std::cout << "IO> Chip outline updated from GATech floorplan file:" << std::endl;
			std::cout << "IO>  Chip -- Fixed die outline (width, x-dimension) [um]: " << fp.IC.outline_x << std::endl;
			std::cout << "IO>  Chip -- Fixed die outline (height, y-dimension) [um]: " << fp.IC.outline_y << std::endl;
		}
	}

	// parse blocks and pins
	while (!blocks_in.eof()) {

		// parse block identifier
		blocks_in >> id;

		// init block / pin
		Block new_block = Block(id, numerical_id);
		Pin new_pin = Pin(id);

		// GSRC: each line contains a block or terminal pin, two examples are below
		//
		// bk1 hardrectilinear 4 (0, 0) (0, 133) (336, 133) (336, 0)
		// BLOCK_7 softrectangular 2464 0.33 3.0
		// VSS terminal
		//
		//
		// GATech: each line contains a soft/hard block, examples given below
		//
		//*HARDBLOCKS
		//<block_name> <block_width> <block_height>
		//*END
		//
		//*SOFTBLOCKS
		//<block_name> <block_area> <min_aspect_ratio> <max_aspect_ratio>
		//*END
		//
		// m0 9.000000 9.000000
		// m0 81 0.33 3

		// parse block type; only for GSRC format
		if (!fp.IO_conf.GT_benchmark)
			blocks_in >> tmpstr;

		// check for begin/end of different sections; only for GATech format
		//
		if (fp.IO_conf.GT_benchmark && id == "*HARDBLOCKS") {

			// continue w/ parsing of hard blocks
			GT_soft_blocks = false;
			GT_terminals = false;

			// parse first block's actual identifier
			blocks_in >> id;
			new_block.id = id;
		}
		else if (fp.IO_conf.GT_benchmark && id == "*SOFTBLOCKS") {

			// continue w/ parsing of soft blocks
			GT_soft_blocks = true;
			GT_terminals = false;

			// parse first block's actual identifier
			blocks_in >> id;
			new_block.id = id;
		}
		else if (fp.IO_conf.GT_benchmark && id == "*TERMINALS") {

			// continue w/ parsing of terminal pins
			GT_terminals = true;

			// parse first pin's actual identifier
			blocks_in >> id;
			new_pin.id = id;
		}
		else if (fp.IO_conf.GT_benchmark && id == "*NETS") {

			// nets are parsed separately, abort parsing
			break;
		}
		else if (fp.IO_conf.GT_benchmark && id == "*END") {

			// current section done, continue parsing from beginning, in order
			// to search for other new section or eof
			continue;
		}

		// GSRC terminal pins: stored separately, parse from pins file
		if (!fp.IO_conf.GT_benchmark && tmpstr == "terminal") {

			// parse pins file for related coordinates
			while (tmpstr != id && !pins_in.eof()) {
				pins_in >> tmpstr;
			}

			// pin cannot be found; log
			if (pins_in.eof() && fp.logMin()) {
				std::cout << "IO>  Coordinates for pin \"" << id << "\" cannot be retrieved, consider checking the pins file!" << std::endl;
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
		// GATech terminal pins: stored within floorplan file (stream blocks_in),
		// syntax similar to hardblocks:
		//
		// "*TERMINALS"
		// "<pin_name> <pin_x_coordinate> <pin_y_coordinate>"
		// "*END"
		//
		else if (fp.IO_conf.GT_benchmark && GT_terminals) {

			blocks_in >> new_pin.bb.ll.x;
			blocks_in >> new_pin.bb.ll.y;

			// store pin now
			fp.terminals.push_back(new_pin);

			// skip further block related handling
			continue;
		}
		// GSRC/GATech hard blocks: parse dimensions
		else if ((!fp.IO_conf.GT_benchmark && tmpstr == "hardrectilinear") ||
				(fp.IO_conf.GT_benchmark && !GT_soft_blocks)) {

			// GSRC parsing
			//
			if (!fp.IO_conf.GT_benchmark) {
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
			}
			// GATech parsing
			else {
				blocks_in >> new_block.bb.w;
				blocks_in >> new_block.bb.h;
			}

			// scale up dimensions
			new_block.bb.w *= fp.IC.blocks_scale;
			new_block.bb.h *= fp.IC.blocks_scale;

			// calculate block area
			new_block.bb.area = new_block.bb.w * new_block.bb.h;

			// also increment numerical id for next block
			numerical_id++;
		}
		// GSRC/GATech soft blocks: parse area and AR range
		else if ((!fp.IO_conf.GT_benchmark && tmpstr == "softrectangular") ||
				(fp.IO_conf.GT_benchmark && GT_soft_blocks)) {

			// parse area, min AR, max AR
			blocks_in >> new_block.bb.area;
			blocks_in >> new_block.AR.min;
			blocks_in >> new_block.AR.max;

			// scale up blocks area
			new_block.bb.area *= std::pow(fp.IC.blocks_scale, 2);

			// mark block as soft
			new_block.soft = true;
			// init block dimensions randomly
			new_block.shapeRandomlyByAR();

			// memorize soft blocks count
			soft_blocks++;

			// also increment numerical id for next block
			numerical_id++;
		}
		// due to some blank lines at the end, we may have reached eof just now
		else if (blocks_in.eof()) {
			break;
		}
		// unknown block type
		else {
			std::cout << "IO>  Unknown block type: " << tmpstr << std::endl;
			std::cout << "IO>  Consider checking the benchmark syntax!" << std::endl;
			exit(1);
		}

		// determine power density
		if (fp.IO_conf.power_density_file_avail) {

			// GSRC handling: values are expected to be listed in the very
			// same order the parsed-in blocks
			//
			if (!fp.IO_conf.GT_benchmark) {
				if (!power_in.eof()) {

					// unscaled value; due to different voltage levels scaled
					// density can be obtained via power_density()
					power_in >> new_block.power_density_unscaled;

					// however, apply general power-scaling factor
					new_block.power_density_unscaled *= fp.IC.power_scale;

					// backup original value
					new_block.power_density_unscaled_back = new_block.power_density_unscaled;
				}
				else {
					if (fp.logMin()) {
						std::cout << "IO>  Some blocks have no power value assigned, consider checking the power density file!" << std::endl;
					}
				}
			}
			// GATech handling: _power_ (not power density) values are given by indexed lines, e.g.,
			// m0 5.40256466490364e-05
			else {

				// parse power file for related block id
				while (tmpstr != id && !power_in.eof()) {
					power_in >> tmpstr;
				}

				// related line cannot be found; log
				if (power_in.eof() && fp.logMin()) {
					std::cout << "IO>  Power value for block \"" << id << "\" cannot be retrieved, consider checking the power file!" << std::endl;
				}
				// related line found, parse power value
				else {

					// raw, unscaled value
					power_in >> new_block.power_density_unscaled;
					// read-in value is absolute power; normalize to
					// density via area;
					new_block.power_density_unscaled /= new_block.bb.area;
					// note that density is assumed to be in uW/um^2;
					// area is given already in um^2 but just read-in
					// power was/is in W; scale up to uW
					new_block.power_density_unscaled *= 1.0e6;

					// finally, apply general power-scaling factor
					new_block.power_density_unscaled *= fp.IC.power_scale;

					// backup original value
					new_block.power_density_unscaled_back = new_block.power_density_unscaled;
				}

				// reset power file stream for next search
				power_in.clear() ;
				power_in.seekg(0, power_in.beg);
			}
		}

		// copy the global power and delay scaling factors along with the
		// voltages; they are required for dynamic calculation of power_density(),
		// delay() and voltage()
		new_block.voltages_power_factors = fp.voltageAssignment.parameters.voltages_power_factors;
		new_block.voltages_delay_factors = fp.voltageAssignment.parameters.voltages_delay_factors;
		new_block.voltages = fp.voltageAssignment.parameters.voltages;

		// init feasible voltages with highest possible voltage
		new_block.resetVoltageAssignment();

		// calculate the base delay; according to [Lin10]
		new_block.base_delay = TimingPowerAnalyser::baseDelay(new_block.bb.h, new_block.bb.w);

		// track block power statistics
		power += new_block.power();
		fp.power_stats.max = std::max(fp.power_stats.max, new_block.power_density());
		if (fp.power_stats.min == -1) {
			fp.power_stats.min = new_block.power_density();
		}
		fp.power_stats.min = std::min(fp.power_stats.min, new_block.power_density());
		fp.power_stats.avg += new_block.power_density();

		// memorize summed blocks area and largest block, needs to fit into die
		fp.IC.blocks_area += new_block.bb.area;
		blocks_max_area = std::max(blocks_max_area, new_block.bb.area);

		// store block
		fp.blocks.push_back(std::move(new_block));
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
	fp.scaleTerminalPins(Point(fp.IC.outline_x, fp.IC.outline_y));

	// sanity check of fixed outline
	blocks_outline_ratio = fp.IC.blocks_area / fp.IC.stack_area;
	if (blocks_outline_ratio > 1.0) {
		std::cout << "IO>  Chip too small; consider increasing the die outline or layers count" << std::endl;
		std::cout << "IO>  Summed Blocks/dies area ratio: " << blocks_outline_ratio << std::endl;
		exit(1);
	}
	// sanity check for largest block
	if (blocks_max_area > fp.IC.die_area) {
		std::cout << "IO>  Die outline too small; consider increasing it" << std::endl;
		std::cout << "IO>  Largest-block/die area ratio: " << blocks_max_area / fp.IC.die_area << std::endl;
		exit(1);
	}

	// sanity check for parsed blocks
	if (to_parse_soft_blocks != -1 && to_parse_hard_blocks != -1) {
		if (fp.blocks.size() != static_cast<unsigned>(to_parse_soft_blocks + to_parse_hard_blocks)) {
			std::cout << "IO>  Not all given blocks could be parsed; consider checking the benchmark syntax!" << std::endl;
			std::cout << "IO>   Parsed hard blocks: " << fp.blocks.size() - soft_blocks << ", expected hard blocks count: " << to_parse_hard_blocks << std::endl;
			exit(1);
		}
	}

	// sanity check for parsed terminals
	if (to_parse_terminals != -1) {
		if (fp.terminals.size() != static_cast<unsigned>(to_parse_terminals)) {
			std::cout << "IO>  Not all given terminals could be parsed; consider checking the benchmark syntax!" << std::endl;
			std::cout << "IO>   Parsed pins: " << fp.terminals.size() << ", expected pins count: " << to_parse_terminals << std::endl;
			exit(1);
		}
	}

	// logging
	if (fp.logMed()) {
		std::cout << "IO> ";
		std::cout << "Done; " << fp.blocks.size() << " blocks read in, " << fp.terminals.size() << " terminal pins read in" << std::endl;
		std::cout << "IO>  Soft blocks: " << soft_blocks << ", hard blocks: " << fp.blocks.size() - soft_blocks << std::endl;

		// floorplacement
		std::cout << "IO>  Largest-block / Average-block area ratio: " << blocks_max_area / blocks_avg_area << ", to be handled as floorplacement: " << floorplacement << std::endl;
		if (!fp.layoutOp.parameters.floorplacement && floorplacement) {
			std::cout << "IO>   Note: floorplacement is ignored since it's deactivated" << std::endl;
		}

		// blocks power
		std::cout << "IO>  Summed blocks power [W]: " << power;
		if (power != 0.0) {
			std::cout << "; min power density [uW/um^2]: " << fp.power_stats.min;
			std::cout << ", max power density [uW/um^2]: " << fp.power_stats.max;
			std::cout << ", avg power density [uW/um^2]: " << fp.power_stats.avg << std::endl;
		}
		else {
			std::cout << std::endl;
		}

		// blocks area
		std::cout << "IO>  Summed blocks area [cm^2]: " << fp.IC.blocks_area * 1.0e-8;
		std::cout << "; single die area [cm^2]: " << fp.IC.die_area * 1.0e-8;
		std::cout << "; summed blocks area / summed dies area: " << blocks_outline_ratio << std::endl;
		std::cout << std::endl;
	}
}

/// parse nets file
void IO::parseNets(FloorPlanner& fp) {
	std::ifstream in;
	std::string tmpstr;
	int i, net_degree;
	std::string net_block;
	Block const* block;
	Pin const* pin;
	int id_num;
	std::string id;
	bool block_not_found, pin_not_found;
	int to_parse_nets;
	unsigned count_input, count_output, count_degree;

	if (fp.logMed()) {
		std::cout << "IO> ";
		std::cout << "Parsing nets..." << std::endl;
	}

	// reset nets
	fp.nets.clear();

	// open nets file
	//
	// GSRC, separate file
	if (!fp.IO_conf.GT_benchmark) {
		in.open(fp.IO_conf.nets_file.c_str());
	}
	// GATech, open floorplan file which also contains the nets
	else {
		in.open(fp.IO_conf.GT_fp_file.c_str());
	}


	// drop other parts, until nets section is reached
	//
	// GSRC
	if (!fp.IO_conf.GT_benchmark) {

		while (tmpstr != "NumNets" && !in.eof())
			in >> tmpstr;
		// drop ":"
		in >> tmpstr;
	}
	// GATech
	else {
		while (tmpstr != "*NETS" && !in.eof())
			in >> tmpstr;
	}

	// memorize how many nets to be parsed
	//
	// GSRC
	if (!fp.IO_conf.GT_benchmark) {
		in >> to_parse_nets;
	}
	// GATech, info not given
	else {
		to_parse_nets = -1;
	}

	// init counter
	count_input = count_output = count_degree = 0;
	id_num = 0;

	// parse nets
	//
	// GATech syntax:
	// "*NETS"
	// "- <net_name1> <no_of_connections>"
	// "<block_name> <input(I) or output(O) pin> <block_pin_name>"
	// "<pin_name> <input(I) or output (O)>"
	// "*END"
	//
	// GSRC syntax:
	// "NetDegree : 2"
	// "p3 B"
	// "sb73 B"
	//
	while (!in.eof()) {

		// GSRC, generate own net id
		if (!fp.IO_conf.GT_benchmark) {

			id = std::to_string(id_num);
			id_num++;
		}
		// GATech, further parsing
		else {

			// drop "-"
			in >> tmpstr;
			// still, check whether end of nets section has been reached
			if (tmpstr == "*END")
				break;

			// parse net id
			in >> id;
		}

		// GSRC and GATech, init new net
		Net new_net = Net(id);

		// GSRC, prepare parsing net degree
		if (!fp.IO_conf.GT_benchmark) {

			// drop until net degree is reached
			// "NetDegree : 2"
			while (tmpstr != "NetDegree" && !in.eof()) {
				in >> tmpstr;
			}

			// drop ":"
			in >> tmpstr;
		}

		// parse net degree
		in >> net_degree;

		count_degree += net_degree;

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
				new_net.terminals.push_back(std::move(pin));
				// pin found
				pin_not_found = false;

				// GSRC, determine net type based on order of pins and
				// blocks appearing in net
				if (!fp.IO_conf.GT_benchmark) {

					// mark each net with a pin as first element as
					// input net
					if (i == 0) {
						new_net.inputNet = true;
					}
					// mark each net with a pin as 2nd or later
					// element as output net
					else {
						new_net.outputNet = true;
					}
				}
				// GATech, parse net type
				else {

					// determine whether pin/net is input or output
					// type
					in >> tmpstr;

					if (tmpstr == "I") {
						new_net.inputNet = true;
					}
					else if (tmpstr == "O") {
						new_net.outputNet = true;
					}
				}

				// update net counter
				if (new_net.inputNet)
					count_input++;
				if (new_net.outputNet)
					count_output++;
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
					new_net.blocks.push_back(std::move(block));
					// block found
					block_not_found = false;

					// GATech, parse further tokens given for blocks
					//
					// "<block_name> <input(I) or output(O) pin> <block_pin_name>"
					if (fp.IO_conf.GT_benchmark) {

						// drop I/O flag
						in >> tmpstr;
						// (TODO) handle individual pins for each block
						// drop pin name
						in >> tmpstr;
					}
				}
				else {
					// block not found
					block_not_found = true;
				}
			}

			// GSRC, drop "B" found for both blocks and terminals
			if (!fp.IO_conf.GT_benchmark) {
				in >> tmpstr;
			}

			// log pin parsing failure
			if (fp.logMin()) {
				if (block_not_found && !pin_not_found) {
					std::cout << "IO>  Net " << new_net.id << "'s block \"" << net_block << "\"";
					std::cout << " cannot be retrieved; consider checking net / blocks file" << std::endl;
				}
				else if (block_not_found && pin_not_found) {
					std::cout << "IO>  Net " << new_net.id << "'s terminal pin \"" << net_block << "\"";
					std::cout << " cannot be retrieved; consider checking net / blocks file" << std::endl;
				}
			}
		}

		// memorize the first block as source/driver; applies to regular
		// internal nets or global output nets, i.e., doesn't apply to
		// global input nets
		//
		if (!new_net.inputNet) {
			new_net.source = new_net.blocks.front();
		}

		// store net
		fp.nets.push_back(std::move(new_net));
	}

	// close nets file
	in.close();

	if (IO::DBG) {
		for (Net const& n : fp.nets) {
			std::cout << "DBG_IO> ";
			std::cout << "net " << n.id << std::endl;

			for (Block const* block : n.blocks) {
				std::cout << "DBG_IO> ";
				std::cout << " block " << block->id << std::endl;
			}

			for (Pin const* pin : n.terminals) {
				std::cout << "DBG_IO> ";
				std::cout << " pin " << pin->id << std::endl;
			}
		}
	}

	// GSRC, sanity check for parsed nets
	//
	if (to_parse_nets != -1) {
		if (fp.nets.size() != static_cast<unsigned>(to_parse_nets)) {
			std::cout << "IO>  Not all given nets could be parsed; consider checking the benchmark format, should comply w/ GSRC Bookshelf" << std::endl;
			std::cout << "IO>   Parsed nets: " << fp.nets.size() << ", expected nets count: " << to_parse_nets << std::endl;
			exit(1);
		}
	}

	if (fp.logMed()) {
		std::cout << "IO> Done; " << fp.nets.size() << " nets read in" << std::endl;
		std::cout << "IO>  Avg net degree: " << static_cast<double>(count_degree) / fp.nets.size() << std::endl;
		std::cout << "IO>  Input nets: " << count_input << "; output nets: " << count_output << std::endl;
		std::cout << std::endl;
	}

}

/// output gnuplot maps
void IO::writeMaps(FloorPlanner& fp, int const& flag_parameter, std::string const& benchmark_suffix) {
	std::ofstream gp_out;
	std::ofstream data_out;
	int cur_layer;
	int layer_limit;
	unsigned x, y;
	int flag, flag_start, flag_stop;
	double max_temp, min_temp;
	int id;

	// sanity check
	if (fp.thermalAnalyzer.power_maps.empty() || fp.thermalAnalyzer.thermal_map.empty()) {
		return;
	}

	if (fp.logMed()) {
		std::cout << "IO> ";

		if (fp.thermal_analyser_run) {
			std::cout << "Generating thermal map ..." << std::endl;
		}
		else if (fp.opt_flags.routing_util) {
			std::cout << "Generating power maps, routing-utilization maps, TSV-density maps, and thermal map ..." << std::endl;
		}
		else {
			std::cout << "Generating power maps, TSV-density maps, and thermal map ..." << std::endl;
		}

		if (benchmark_suffix != "") {
			std::cout << "IO>  Benchmark suffix: " << benchmark_suffix << std::endl;
		}
	}

	// generate set of maps; integer encoding
	//
	// flag == 0: generate power maps
	// flag == 1: generate thermal map
	// flag == 2: generate thermal map using HotSpot results
	// flag == 3: generate TSV-density map
	// flag == 4: generate original power maps (not padded, not adapted)
	// flag == 5: generate routing-utilization map
	//
	// for regular runs, generate all sets; for thermal-analyzer runs, only generate
	// the required thermal map
	flag_start = flag_stop = -1;
	// also, check whether a particular flag was passed as parameter
	if (flag_parameter != -1) {
		flag_start = flag_stop = flag_parameter;
	}
	if (fp.thermal_analyser_run) {
		flag_start = flag_stop = MAPS_FLAGS::THERMAL;
	}
	else if (fp.opt_flags.routing_util) {
		flag_start = MAPS_FLAGS::POWER;
		flag_stop = MAPS_FLAGS::ROUTING;
	}
	else {
		flag_start = MAPS_FLAGS::POWER;
		flag_stop = MAPS_FLAGS::POWER_ORIG;
	}
	//
	// actual map generation	
	for (flag = flag_start; flag <= flag_stop; flag++) {

		// thermal map (power blurring) only for layer 0
		if (flag == MAPS_FLAGS::THERMAL) {
			layer_limit = 1;
		}
		// power, thermal (HotSpot), routing-utilization and TSV-density maps for all layers
		else {
			layer_limit = fp.IC.layers;
		}

		for (cur_layer = 0; cur_layer < layer_limit; cur_layer++) {
			// build up file names
			std::stringstream gp_out_name;
			std::stringstream data_out_name;
			if (flag == MAPS_FLAGS::POWER) {
				gp_out_name << fp.benchmark << benchmark_suffix << "_" << cur_layer + 1 << "_power.gp";
				data_out_name << fp.benchmark << benchmark_suffix << "_" << cur_layer + 1 << "_power.data";
			}
			else if (flag == MAPS_FLAGS::POWER_ORIG) {
				gp_out_name << fp.benchmark << benchmark_suffix << "_" << cur_layer + 1 << "_power_orig.gp";
				data_out_name << fp.benchmark << benchmark_suffix << "_" << cur_layer + 1 << "_power_orig.data";
			}
			else if (flag == MAPS_FLAGS::THERMAL) {
				gp_out_name << fp.benchmark << benchmark_suffix << "_" << cur_layer + 1 << "_thermal.gp";
				data_out_name << fp.benchmark << benchmark_suffix << "_" << cur_layer + 1 << "_thermal.data";
			}
			else if (flag == MAPS_FLAGS::THERMAL_HOTSPOT) {
				gp_out_name << fp.benchmark << benchmark_suffix << "_" << cur_layer + 1 << "_HotSpot.gp";

				// note that this file is not written here, but actually
				// to be generated separately by HotSpot; it is only
				// required here to be referenced in the gp script
				//
				// also note that layer ids must match with active Si
				// layers, where the order is defined in the lcf file in
				// writeHotSpotFiles
				data_out_name << fp.benchmark << benchmark_suffix << "_HotSpot.steady.grid.gp_data.layer_" << (1 + 4 * cur_layer);
			}
			else if (flag == MAPS_FLAGS::TSV_DENSITY) {
				gp_out_name << fp.benchmark << benchmark_suffix << "_" << cur_layer + 1 << "_TSV_density.gp";
				data_out_name << fp.benchmark << benchmark_suffix << "_" << cur_layer + 1 << "_TSV_density.data";
			}
			else if (flag == MAPS_FLAGS::ROUTING) {
				gp_out_name << fp.benchmark << benchmark_suffix << "_" << cur_layer + 1 << "_routing_util.gp";
				data_out_name << fp.benchmark << benchmark_suffix << "_" << cur_layer + 1 << "_routing_util.data";
			}

			// init file stream for gnuplot script
			gp_out.open(gp_out_name.str().c_str());
			// init file stream for data file;
			// don't open (overwrite) for HotSpot data
			if (flag != MAPS_FLAGS::THERMAL_HOTSPOT) {
				data_out.open(data_out_name.str().c_str());
			}

			// file header for data file
			if (flag == MAPS_FLAGS::POWER || flag == MAPS_FLAGS::POWER_ORIG) {
				data_out << "# X Y power" << std::endl;
			}
			else if (flag == MAPS_FLAGS::THERMAL) {
				data_out << "# X Y thermal" << std::endl;
			}
			else if (flag == MAPS_FLAGS::TSV_DENSITY) {
				data_out << "# X Y TSV_density" << std::endl;
			}
			else if (flag == MAPS_FLAGS::ROUTING) {
				data_out << "# X Y routing_util" << std::endl;
			}

			// output grid values for power maps
			if (flag == MAPS_FLAGS::POWER) {

				for (x = 0; x < ThermalAnalyzer::POWER_MAPS_DIM; x++) {
					for (y = 0; y < ThermalAnalyzer::POWER_MAPS_DIM; y++) {
						data_out << x << "	" << y << "	" << fp.thermalAnalyzer.power_maps[cur_layer][x][y].power_density << std::endl;
					}

					// add dummy data point, required since gnuplot option corners2color cuts last row and column of dataset
					data_out << x << "	" << ThermalAnalyzer::POWER_MAPS_DIM << "	" << "0.0" << std::endl;

					// blank line marks new row for gnuplot
					data_out << std::endl;
				}

				// add dummy data row, required since gnuplot option corners2color cuts last row and column of dataset
				for (y = 0; y <= ThermalAnalyzer::POWER_MAPS_DIM; y++) {
					data_out << ThermalAnalyzer::POWER_MAPS_DIM << "	" << y << "	" << "0.0" << std::endl;
				}

			}
			// output grid values for original power maps
			else if (flag == MAPS_FLAGS::POWER_ORIG) {

				// not padded, dimensions like thermal map
				for (x = 0; x < ThermalAnalyzer::THERMAL_MAP_DIM; x++) {
					for (y = 0; y < ThermalAnalyzer::THERMAL_MAP_DIM; y++) {
						data_out << x << "	" << y << "	" << fp.thermalAnalyzer.power_maps_orig[cur_layer][x][y].power_density << std::endl;
					}

					// add dummy data point, required since gnuplot option corners2color cuts last row and column of dataset
					data_out << x << "	" << ThermalAnalyzer::THERMAL_MAP_DIM << "	" << "0.0" << std::endl;

					// blank line marks new row for gnuplot
					data_out << std::endl;
				}

				// add dummy data row, required since gnuplot option corners2color cuts last row and column of dataset
				for (y = 0; y <= ThermalAnalyzer::THERMAL_MAP_DIM; y++) {
					data_out << ThermalAnalyzer::THERMAL_MAP_DIM << "	" << y << "	" << "0.0" << std::endl;
				}

			}
			// output grid values for thermal maps
			else if (flag == MAPS_FLAGS::THERMAL) {
				max_temp = 0.0;
				min_temp = 1.0e6;

				for (x = 0; x < ThermalAnalyzer::THERMAL_MAP_DIM; x++) {
					for (y = 0; y < ThermalAnalyzer::THERMAL_MAP_DIM; y++) {
						data_out << x << "	" << y << "	" << fp.thermalAnalyzer.thermal_map[x][y].temp << std::endl;
						// also track max and min temp
						max_temp = std::max(max_temp, fp.thermalAnalyzer.thermal_map[x][y].temp);
						min_temp = std::min(min_temp, fp.thermalAnalyzer.thermal_map[x][y].temp);
					}

					// add dummy data point, required since gnuplot option corners2color cuts last row and column of dataset
					data_out << x << "	" << ThermalAnalyzer::THERMAL_MAP_DIM << "	" << "0.0" << std::endl;

					// blank line marks new row for gnuplot
					data_out << std::endl;
				}

				// add dummy data row, required since gnuplot option corners2color cuts last row and column of dataset
				for (y = 0; y <= ThermalAnalyzer::THERMAL_MAP_DIM; y++) {
					data_out << ThermalAnalyzer::THERMAL_MAP_DIM << "	" << y << "	" << "0.0" << std::endl;
				}
			}
			// output grid values for TSV-density maps; consider only bin bins
			// w/in die outline, not in padded zone
			else if (flag == MAPS_FLAGS::TSV_DENSITY) {

				for (x = 0; x < ThermalAnalyzer::THERMAL_MAP_DIM; x++) {
					for (y = 0; y < ThermalAnalyzer::THERMAL_MAP_DIM; y++) {
						// access map bins w/ offset related to
						// padding zone
						data_out << x << "	" << y << "	" << fp.thermalAnalyzer.power_maps[cur_layer][x + ThermalAnalyzer::POWER_MAPS_PADDED_BINS][y + ThermalAnalyzer::POWER_MAPS_PADDED_BINS].TSV_density << std::endl;
					}

					// add dummy data point, required since gnuplot option corners2color cuts last row and column of dataset
					data_out << x << "	" << ThermalAnalyzer::THERMAL_MAP_DIM << "	" << "0.0" << std::endl;

					// blank line marks new row for gnuplot
					data_out << std::endl;
				}

				// add dummy data row, required since gnuplot option corners2color cuts last row and column of dataset
				for (y = 0; y <= ThermalAnalyzer::THERMAL_MAP_DIM; y++) {
					data_out << ThermalAnalyzer::THERMAL_MAP_DIM << "	" << y << "	" << "0.0" << std::endl;
				}
			}
			// output grid values for routing-utilization maps
			else if (flag == MAPS_FLAGS::ROUTING) {

				for (x = 0; x < RoutingUtilization::UTIL_MAPS_DIM; x++) {
					for (y = 0; y < RoutingUtilization::UTIL_MAPS_DIM; y++) {
						data_out << x << "	" << y << "	" << fp.routingUtil.util_maps[cur_layer][x][y].utilization << std::endl;
					}

					// add dummy data point, required since gnuplot option corners2color cuts last row and column of dataset
					data_out << x << "	" << RoutingUtilization::UTIL_MAPS_DIM << "	" << "0.0" << std::endl;

					// blank line marks new row for gnuplot
					data_out << std::endl;
				}

				// add dummy data row, required since gnuplot option corners2color cuts last row and column of dataset
				for (y = 0; y <= RoutingUtilization::UTIL_MAPS_DIM; y++) {
					data_out << RoutingUtilization::UTIL_MAPS_DIM << "	" << y << "	" << "0.0" << std::endl;
				}

			}

			// close file stream for data file
			if (flag != MAPS_FLAGS::THERMAL_HOTSPOT) {
				data_out.close();
			}

			// file header for gnuplot script
			if (flag == MAPS_FLAGS::POWER) {
				gp_out << "set title \"Padded and Adapted Power Map - " << fp.benchmark << benchmark_suffix << ", Layer " << cur_layer + 1 << "\" noenhanced" << std::endl;
			}
			else if (flag == MAPS_FLAGS::POWER_ORIG) {
				gp_out << "set title \"Power Map - " << fp.benchmark << benchmark_suffix << ", Layer " << cur_layer + 1 << "\" noenhanced" << std::endl;
			}
			else if (flag == MAPS_FLAGS::THERMAL || flag == MAPS_FLAGS::THERMAL_HOTSPOT) {
				gp_out << "set title \"Thermal Map - " << fp.benchmark << benchmark_suffix << ", Layer " << cur_layer + 1 << "\" noenhanced" << std::endl;
			}
			else if (flag == MAPS_FLAGS::TSV_DENSITY) {
				gp_out << "set title \"TSV-Density Map - " << fp.benchmark << benchmark_suffix << ", Layer " << cur_layer + 1 << "\" noenhanced" << std::endl;
			}
			else if (flag == MAPS_FLAGS::ROUTING) {
				gp_out << "set title \"Routing-Utilization Map - " << fp.benchmark << benchmark_suffix << ", Layer " << cur_layer + 1 << "\" noenhanced" << std::endl;
			}

			gp_out << "set terminal pdfcairo enhanced font \"Gill Sans, 12\"" << std::endl;
			gp_out << "set output \"" << gp_out_name.str() << ".pdf\"" << std::endl;
			gp_out << "set size square" << std::endl;

			// different 2D ranges for maps; consider dummy data row and
			// column, since gnuplot option corners2color cuts off last row
			// and column
			if (flag == MAPS_FLAGS::POWER) {
				gp_out << "set xrange [0:" << ThermalAnalyzer::POWER_MAPS_DIM << "]" << std::endl;
				gp_out << "set yrange [0:" << ThermalAnalyzer::POWER_MAPS_DIM << "]" << std::endl;
			}
			// other dimensions, not padded
			else if (flag == MAPS_FLAGS::POWER_ORIG) {
				gp_out << "set xrange [0:" << ThermalAnalyzer::THERMAL_MAP_DIM << "]" << std::endl;
				gp_out << "set yrange [0:" << ThermalAnalyzer::THERMAL_MAP_DIM << "]" << std::endl;
			}
			else if (flag == MAPS_FLAGS::THERMAL	|| flag == MAPS_FLAGS::THERMAL_HOTSPOT || flag == MAPS_FLAGS::TSV_DENSITY) {
				gp_out << "set xrange [0:" << ThermalAnalyzer::THERMAL_MAP_DIM << "]" << std::endl;
				gp_out << "set yrange [0:" << ThermalAnalyzer::THERMAL_MAP_DIM << "]" << std::endl;
			}
			else if (flag == MAPS_FLAGS::ROUTING) {
				gp_out << "set xrange [0:" << RoutingUtilization::UTIL_MAPS_DIM << "]" << std::endl;
				gp_out << "set yrange [0:" << RoutingUtilization::UTIL_MAPS_DIM << "]" << std::endl;
			}

			// power maps
			if (flag == MAPS_FLAGS::POWER || flag == MAPS_FLAGS::POWER_ORIG) {
				// label for power density
				gp_out << "set cblabel \"Power Density [10^{-2} {/Symbol m}W/{/Symbol m}m^2]\"" << std::endl;
			}
			// thermal maps (power blurring)
			else if (flag == MAPS_FLAGS::THERMAL) {
				// fixed scale to avoid remapping to extended range
				gp_out << "set cbrange [" << min_temp << ":" << max_temp << "]" << std::endl;
				// thermal estimation, correlates w/ power density
				gp_out << "set cblabel \"Estimated Temperature [K]\"" << std::endl;
			}
			// thermal maps (HotSpot)
			else if (flag == MAPS_FLAGS::THERMAL_HOTSPOT) {
				// label for HotSpot results
				gp_out << "set cblabel \"Temperature [K], from HotSpot\"" << std::endl;
			}
			// TSV-density maps
			else if (flag == MAPS_FLAGS::TSV_DENSITY) {
				// fixed scale
				gp_out << "set cbrange [0:100]" << std::endl;
				// (TODO) also possible: fixed log scale to emphasize both
				// low densities (single TSVs) as well as large densities
				// (TSV cluster, vertical buses)
				//gp_out << "set log cb" << std::endl;
				//gp_out << "set cbrange [0.1:100]" << std::endl;
				// label for power density
				gp_out << "set cblabel \"TSV-Density [%]\"" << std::endl;
			}
			// routing-util maps
			else if (flag == MAPS_FLAGS::ROUTING) {
				// label for utilization
				gp_out << "set cblabel \"Estimated Routing Utilization\"" << std::endl;
			}

			// tics
			gp_out << "set tics front" << std::endl;
			gp_out << "set grid xtics ytics ztics" << std::endl;
			// pm3d algorithm determines an average value for each pixel,
			// considering sourrounding pixels;
			// skip this behaviour w/ ``corners2color''; c1 means to select
			// the lower-left value, practically loosing one row and column in
			// the overall plot (compensated for by dummy data; see also
			// http://gnuplot.sourceforge.net/demo/pm3d.html
			gp_out << "set pm3d map corners2color c1" << std::endl;
			//// color printable as gray
			//gp_out << "set palette rgbformulae 30,31,32" << std::endl;
			// mathlab color palette; see
			// http://www.gnuplotting.org/matlab-colorbar-with-gnuplot/
			gp_out << "set palette defined ( 0 \"#000090\",\\" << std::endl;
			gp_out << "1 \"#000fff\",\\" << std::endl;
			gp_out << "2 \"#0090ff\",\\" << std::endl;
			gp_out << "3 \"#0fffee\",\\" << std::endl;
			gp_out << "4 \"#90ff70\",\\" << std::endl;
			gp_out << "5 \"#ffee00\",\\" << std::endl;
			gp_out << "6 \"#ff7000\",\\" << std::endl;
			gp_out << "7 \"#ee0000\",\\" << std::endl;
			gp_out << "8 \"#7f0000\")" << std::endl;

			// for padded power maps: draw rectangle for unpadded core
			if (flag == MAPS_FLAGS::POWER && ThermalAnalyzer::POWER_MAPS_PADDED_BINS > 0) {
				gp_out << "set obj 1 rect from ";
				gp_out << ThermalAnalyzer::POWER_MAPS_PADDED_BINS << ", " << ThermalAnalyzer::POWER_MAPS_PADDED_BINS << " to ";
				gp_out << ThermalAnalyzer::POWER_MAPS_DIM - ThermalAnalyzer::POWER_MAPS_PADDED_BINS << ", ";
				gp_out << ThermalAnalyzer::POWER_MAPS_DIM - ThermalAnalyzer::POWER_MAPS_PADDED_BINS << " ";
				gp_out << "front fillstyle empty border rgb \"white\" linewidth 3" << std::endl;
			}

			// for original power maps, mark leakage-analyzer related partitions for dbg mode
			else if (flag == POWER_ORIG) {
				if (LeakageAnalyzer::DBG_GP) {

					// sanity check if some partitions are defined
					if (fp.leakageAnalyzer.power_partitions.empty()) {
						continue;
					}

					id = 1;

					for (auto& cur_part : fp.leakageAnalyzer.power_partitions[cur_layer]) {

						// determine random color for each partition, in order to (hopefully) have clear visual separation between adjacent partitions
						int r,g,b;

						r = Math::randI(0, 256);
						g = Math::randI(0, 256);
						b = Math::randI(0, 256);

						for (auto& bin : cur_part.second) {

							gp_out << "set obj " << id << " rect from ";
							gp_out << bin.x << ", " << bin.y << " to ";
							gp_out << bin.x + 1 << ", " << bin.y + 1 << " ";
							gp_out << "front fillstyle empty border ";

							gp_out << " rgb \"#";
							gp_out << std::hex << std::setfill('0') << std::setw(2) << r;
							gp_out << std::hex << std::setfill('0') << std::setw(2) << g;
							gp_out << std::hex << std::setfill('0') << std::setw(2) << b;
							gp_out << std::dec;
							gp_out << "\" linewidth 1" << std::endl;

							gp_out << std::endl;

							id++;
						}
					}
				}
			}

			// for thermal maps (power blurring): draw rectangles for hotspot regions
			else if (flag == MAPS_FLAGS::THERMAL) {

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
							gp_out << std::endl;

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

							gp_out << std::endl;

							id++;
						}
					}
				}
			}

			// for thermal maps (HotSpot): draw rectangles for floorplan
			// blocks, which have to scaled to grid dimensions
			else if (flag == MAPS_FLAGS::THERMAL_HOTSPOT) {

				double scaling_factor_x = static_cast<double>(ThermalAnalyzer::THERMAL_MAP_DIM) / fp.IC.outline_x;
				double scaling_factor_y = static_cast<double>(ThermalAnalyzer::THERMAL_MAP_DIM) / fp.IC.outline_y;

				// output blocks
				for (Block const& cur_block : fp.blocks) {

					if (cur_block.layer != cur_layer) {
						continue;
					}

					// block rectangles, only white boundary;
					// down-scaled to grid dimension
					gp_out << "set obj rect front";
					gp_out << " from " << cur_block.bb.ll.x * scaling_factor_x << "," << cur_block.bb.ll.y * scaling_factor_y;
					gp_out << " to " << cur_block.bb.ur.x * scaling_factor_x << "," << cur_block.bb.ur.y * scaling_factor_y;
					gp_out << " fillstyle empty border rgb \"white\"";
					gp_out << std::endl;
				}
			}

			gp_out << "splot \"" << data_out_name.str() << "\" using 1:2:3 notitle" << std::endl;

			// close file stream for gnuplot script
			gp_out.close();
		}
	}

	if (fp.logMed()) {
		std::cout << "IO> ";
		std::cout << "Done" << std::endl << std::endl;
	}
}

/// output gnuplot for SA annealing schedule
void IO::writeTempSchedule(FloorPlanner const& fp) {
	std::ofstream gp_out;
	std::ofstream data_out;
	bool valid_solutions, first_valid_sol;

	// sanity check
	if (fp.tempSchedule.empty()) {
		return;
	}

	if (fp.logMed()) {
		std::cout << "IO> ";
		std::cout << "Generating GP scripts for SA temperature-schedule ..." << std::endl;
	}

	// build up file names
	std::stringstream gp_out_name;
	std::stringstream data_out_name;
	gp_out_name << fp.benchmark << "_TempSchedule.gp";
	data_out_name << fp.benchmark << "_TempSchedule.data";

	// init file stream for gnuplot script
	gp_out.open(gp_out_name.str().c_str());
	// init file stream for data file
	data_out.open(data_out_name.str().c_str());

	// output data: SA step and SA temp
	data_out << "# Step Temperature (index 0)" << std::endl;

	for (FloorPlanner::TempStep step : fp.tempSchedule) {
		data_out << step.step << " " << step.temp << std::endl;
	}

	// two blank lines trigger gnuplot to interpret data file as separate data sets
	data_out << std::endl;
	data_out << std::endl;

	// output data: SA step and avg costs phase 1
	data_out << "# Step Avg_Cost_Phase_1 (index 1)" << std::endl;

	// memorize if valid solutions are given at all
	valid_solutions = false;

	for (FloorPlanner::TempStep step : fp.tempSchedule) {

		data_out << step.step << " " << step.avg_cost << std::endl;

		// output data until first valid solution is found
		if (step.new_best_sol_found) {

			valid_solutions = true;

			break;
		}
	}

	if (valid_solutions) {

		// two blank lines trigger gnuplot to interpret data file as separate data sets
		data_out << std::endl;
		data_out << std::endl;

		// output data: markers for best-solution steps
		data_out << "# Step Temperature (only steps w/ new best solutions, index 2)" << std::endl;

		for (FloorPlanner::TempStep step : fp.tempSchedule) {

			if (step.new_best_sol_found) {
				data_out << step.step << " " << step.temp << std::endl;
			}
		}

		// two blank lines trigger gnuplot to interpret data file as separate data sets
		data_out << std::endl;
		data_out << std::endl;

		// output data: SA step and avg costs phase 2
		data_out << "# Step Avg_Cost_Phase_2 (index 3)" << std::endl;

		first_valid_sol = false;
		for (FloorPlanner::TempStep step : fp.tempSchedule) {

			// output data only after first solution is found and only for
			// cost larger 0
			if (first_valid_sol && step.avg_cost > 0.0) {
				data_out << step.step << " " << step.avg_cost << std::endl;
			}

			if (step.new_best_sol_found) {
				first_valid_sol = true;
			}
		}

		// two blank lines trigger gnuplot to interpret data file as separate data sets
		data_out << std::endl;
		data_out << std::endl;

		// output data: SA step and best costs phase 2
		data_out << "# Step Best_Cost_Phase_2 (index 4)" << std::endl;

		first_valid_sol = false;
		for (FloorPlanner::TempStep step : fp.tempSchedule) {

			// output data only after first solution is found
			if (first_valid_sol) {
				data_out << step.step << " " << step.cost_best_sol << std::endl;
			}

			if (step.new_best_sol_found) {
				first_valid_sol = true;
			}
		}
	}

	// close file stream
	data_out.close();

	// gp header
	gp_out << "set title \"Temperature and Cost Schedule - " << fp.benchmark << "\" noenhanced" << std::endl;
	gp_out << "set output \"" << gp_out_name.str() << ".pdf\"" << std::endl;

	// general settings for more attractive plots, extracted from
	// http://youinfinitesnake.blogspot.de/2011/02/attractive-scientific-plots-with.html
	gp_out << "set terminal pdfcairo font \"Gill Sans, 12\" linewidth 4 rounded" << std::endl;
	gp_out << "# Line style for axes" << std::endl;
	gp_out << "set style line 80 lt rgb \"#808080\"" << std::endl;
	gp_out << "# Line style for grid" << std::endl;
	gp_out << "set style line 81 lt 0  # dashed" << std::endl;
	gp_out << "set style line 81 lt rgb \"#808080\"  # grey" << std::endl;
	gp_out << "set grid back linestyle 81" << std::endl;
	gp_out << "# Remove border on top and right." << std::endl;
	gp_out << "# Also, put it in grey; no need for so much emphasis on a border." << std::endl;
	gp_out << "set border 3 back linestyle 80" << std::endl;
	gp_out << "set xtics nomirror" << std::endl;
	gp_out << "set ytics nomirror" << std::endl;
	gp_out << "# Line styles: try to pick pleasing colors, rather" << std::endl;
	gp_out << "# than strictly primary colors or hard-to-see colors" << std::endl;
	gp_out << "# like gnuplot's default yellow. Make the lines thick" << std::endl;
	gp_out << "# so they're easy to see in small plots in papers." << std::endl;
	gp_out << "set style line 1 lt rgb \"#A00000\" lw 2 pt 1" << std::endl;
	gp_out << "set style line 2 lt rgb \"#00A000\" lw 2 pt 6" << std::endl;
	gp_out << "set style line 3 lt rgb \"#5060D0\" lw 2 pt 2" << std::endl;
	gp_out << "set style line 4 lt rgb \"#F25900\" lw 2 pt 9" << std::endl;
	gp_out << "set style line 5 lt rgb \"#7806A0\" lw 2 pt 9" << std::endl;

	// specific settings: labels
	gp_out << "set xlabel \"SA Step\"" << std::endl;
	gp_out << "set ylabel \"SA Temperature\"" << std::endl;
	gp_out << "set y2label \"Normalized Cost\"" << std::endl;
	// specific settings: key, labels box
	gp_out << "set key box lt rgb \"#808080\" out bottom center" << std::endl;
	// specific settings: log scale (for SA temp)
	gp_out << "set log y" << std::endl;
	gp_out << "set mytics 10" << std::endl;
	// second, indepentend scale for cost values
	gp_out << "set y2tics nomirror" << std::endl;
	gp_out << "set mytics 10" << std::endl;
	// cut cost above 1 in order to emphasize cost trend
	gp_out << "set y2range [:1]" << std::endl;

	// gp data plot command
	gp_out << "plot \"" << data_out_name.str() << "\" index 0 using 1:2 title \"SA Temperature\" with lines linestyle 2, \\" << std::endl;
	// there may be no valid solutions, then only the costs for phase 1 are plotted
	// besides the temperature schedule
	if (!valid_solutions) {
		gp_out << "\"" << data_out_name.str() << "\" index 1";
		gp_out << " using 1:2 title \"Avg. Cost\" with lines linestyle 3 axes x1y2" << std::endl;
	}
	// otherwise, we consider both cost and the best solutions data sets
	else {
		gp_out << "\"" << data_out_name.str() << "\" index 2 using 1:2 title \"Best Solutions\" with points linestyle 1, \\" << std::endl;
		gp_out << "\"" << data_out_name.str() << "\" index 1";
		gp_out << " using 1:2 title \"Avg. Cost - SA Phase 1\" with lines linestyle 3 axes x1y2, \\" << std::endl;
		gp_out << "\"" << data_out_name.str() << "\" index 3";
		gp_out << " using 1:2 title \"Avg. Cost - SA Phase 2\" with lines linestyle 5 axes x1y2, \\" << std::endl;
		gp_out << "\"" << data_out_name.str() << "\" index 4";
		gp_out << " using 1:2 title \"Best Cost - SA Phase 2\" with lines linestyle 4 axes x1y2" << std::endl;
	}

	// close file stream
	gp_out.close();

	if (fp.logMed()) {
		std::cout << "IO> ";
		std::cout << "Done" << std::endl << std::endl;
	}
}

/// generate gnuplot for floorplans
void IO::writeFloorplanGP(FloorPlanner const& fp, std::vector<CorblivarAlignmentReq> const& alignment, std::string const& benchmark_suffix) {
	std::ofstream gp_out;
	int cur_layer;
	double ratio_inv;
	int tics;
	Rect alignment_rect, alignment_rect_tmp;
	int req_x_fulfilled, req_y_fulfilled;
	std::string alignment_color_fulfilled;
	std::string alignment_color_failed;
	std::string alignment_color_undefined;

	// sanity check, not for thermal-analysis runs
	if (fp.thermal_analyser_run) {
		return;
	}

	if (fp.logMed()) {
		std::cout << "IO> Generating GP scripts for floorplan ..." << std::endl;
		if (benchmark_suffix != "") {
			std::cout << "IO>  Benchmark suffix: " << benchmark_suffix << std::endl;
		}
	}

	// GP parameters
	ratio_inv = 1.0 / fp.IC.die_AR;
	tics = std::max(fp.IC.outline_x, fp.IC.outline_y) / 5;

	// color for alignment rects; for fulfilled alignment, green-ish color
	alignment_color_fulfilled = "#00A000";
	// color for alignment rects; for failed alignment, red-ish color
	alignment_color_failed = "#A00000";
	// color for alignment rects; for undefined alignment, blue-ish color
	alignment_color_undefined = "#0000A0";

	for (cur_layer = 0; cur_layer < fp.IC.layers; cur_layer++) {
		// build up file name
		std::stringstream out_name;
		out_name << fp.benchmark << benchmark_suffix << "_" << cur_layer + 1 << ".gp";

		// init file stream
		gp_out.open(out_name.str().c_str());

		// file header
		gp_out << "set title \"Floorplan - " << fp.benchmark << ", Layer " << cur_layer + 1 << "\" noenhanced" << std::endl;
		gp_out << "set terminal pdfcairo enhanced font \"Gill Sans, 12\"" << std::endl;
		gp_out << "set output \"" << out_name.str() << ".pdf\"" << std::endl;
		gp_out << "set size ratio " << ratio_inv << std::endl;
		gp_out << "set xrange [0:" << fp.IC.outline_x << "]" << std::endl;
		gp_out << "set yrange [0:" << fp.IC.outline_y << "]" << std::endl;
		gp_out << "set xlabel \"Width [{/Symbol m}m]\"" << std::endl;
		gp_out << "set ylabel \"Height [{/Symbol m}m]\"" << std::endl;
		gp_out << "set xtics " << tics << std::endl;
		gp_out << "set ytics " << tics << std::endl;
		gp_out << "set mxtics 4" << std::endl;
		gp_out << "set mytics 4" << std::endl;
		gp_out << "set tics front" << std::endl;
		gp_out << "set grid xtics ytics mxtics mytics" << std::endl;

		// output blocks
		for (Block const& cur_block : fp.blocks) {

			if (cur_block.layer != cur_layer) {
				continue;
			}

			// block rectangles
			gp_out << "set obj rect";
			gp_out << " from " << cur_block.bb.ll.x << "," << cur_block.bb.ll.y;
			gp_out << " to " << cur_block.bb.ur.x << "," << cur_block.bb.ur.y;
			// soft and hard blocks shall have different colors
			if (cur_block.soft) {
				gp_out << " fillcolor rgb \"#ac9d93\" fillstyle solid";
			}
			else {
				gp_out << " fillcolor rgb \"#91A1AB\" fillstyle solid";
			}
			gp_out << std::endl;

			// label
			gp_out << "set label \"" << cur_block.id << "\"";
			gp_out << " at " << cur_block.bb.ll.x + 0.01 * fp.IC.outline_x;
			gp_out << "," << cur_block.bb.ll.y + 0.01 * fp.IC.outline_y;
			gp_out << " font \"Gill Sans,4\"";
			// prevents generating subscripts for underscore in labels
			gp_out << " noenhanced" << std::endl;

			// feasible voltages, as bitset
			if (MultipleVoltages::DBG_FLOORPLAN) {

				gp_out << "set label \"";
				gp_out << cur_block.feasible_voltages;
				gp_out << "\"";

				gp_out << " at " << cur_block.bb.ur.x - 0.03 * fp.IC.outline_x;
				gp_out << "," << cur_block.bb.ll.y + 0.01 * fp.IC.outline_y;
				gp_out << " font \"Gill Sans,2\"";
				// prevents generating subscripts for underscore in labels
				gp_out << " noenhanced" << std::endl;
			}
		}

		// output voltage volumes; die-wise as islands comprised of several boxes
		for (unsigned m = 0; m < fp.voltageAssignment.selected_modules.size(); m++) {

			MultipleVoltages::CompoundModule* module = fp.voltageAssignment.selected_modules[m];

			bool label_put = false;

			// boost data structures for current module
			//
			BoostPolygonSet outline;
			std::vector<BoostRect> voltage_island_rects;

			// compose all (likely overlapping) bbs of this module into a
			// non-overlapping BoostPolygonSet
			//
			for (auto& bb : module->outline[cur_layer]) {

				if (bb.area == 0) {
					continue;
				}

				outline += BoostRect(bb.ll.x, bb.ll.y, bb.ur.x, bb.ur.y);
			}

			// subtract all by other modules covered areas; only consider yet
			// unplotted modules; this avoids subtracting overlaps anytime for
			// all intersecting modules which would, in turn, disables
			// properly closed voltage islands
			//
			for (unsigned m2 = m + 1; m2 < fp.voltageAssignment.selected_modules.size(); m2++) {

				MultipleVoltages::CompoundModule* other_module = fp.voltageAssignment.selected_modules[m2];

				if (other_module->id() == module->id()) {
					continue;
				}

				for (auto& other_bb : other_module->outline[cur_layer]) {

					if (other_bb.area == 0) {
						continue;
					}

					outline -= BoostRect(other_bb.ll.x, other_bb.ll.y, other_bb.ur.x, other_bb.ur.y);
				}
			}

			// get rectangles from outline BoostPolygonSet, put them into
			// voltage_island_rects
			outline.get(voltage_island_rects);

			// walk all partial rects of the voltage island
			for (auto& rect : voltage_island_rects) {

				// box outline
				gp_out << "set obj rect";
				gp_out << " from " << bp::xl(rect) << "," << bp::yl(rect);
				gp_out << " to " << bp::xh(rect) << "," << bp::yh(rect);
				gp_out << " fillstyle transparent solid 0.5 border";
				// color depending on voltage, whereas to color range goes from
				// #11ff00 (green) to #ff6600 (orange); only R and G values are scaled
				gp_out << " fillcolor rgb \"#";
				gp_out << std::hex;
				if (fp.voltageAssignment.parameters.voltages.size() == 1) {
					gp_out << static_cast<int>(0x11);
					gp_out << static_cast<int>(0xff);
				}
				else {
					gp_out << static_cast<int>(0x11 + module->min_voltage_index() * (0xee / (fp.voltageAssignment.parameters.voltages.size() - 1) ));
					gp_out << static_cast<int>(0xff - module->min_voltage_index() * (0x99 / (fp.voltageAssignment.parameters.voltages.size() - 1) ));
				}
				gp_out << std::dec;
				gp_out << "00\"" << std::endl;

				// put label once label; to module assigned blocks and their shared voltage
				if (!label_put) {
					gp_out << "set label \"" << module->id() << "\\n" << fp.voltageAssignment.parameters.voltages[module->min_voltage_index()] << " V\"";
					gp_out << " at " << bp::xl(rect) + 0.01 * fp.IC.outline_x;
					gp_out << "," << bp::yh(rect) - 0.01 * fp.IC.outline_y;
					gp_out << " font \"Gill Sans,2\"";
					// prevents generating subscripts for underscore in labels
					gp_out << " noenhanced" << std::endl;

					label_put = true;
				}
			}
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
			gp_out << std::endl;

			// label, only for larger islands not for single TSVs
			if (TSV_group.TSVs_count > 1) {
				gp_out << "set label \"" << TSV_group.id << "\"";
				gp_out << " at " << TSV_group.bb.ll.x + 0.01 * fp.IC.outline_x;
				gp_out << "," << TSV_group.bb.ll.y + 0.01 * fp.IC.outline_y;
				gp_out << " font \"Gill Sans,2\"";
				// prevents generating subscripts for underscore in labels
				gp_out << " noenhanced" << std::endl;
			}
		}

		// output dummy TSVs
		for (TSV_Island const& TSV_group : fp.dummy_TSVs) {

			if (TSV_group.layer != cur_layer) {
				continue;
			}

			// block rectangles
			gp_out << "set obj rect";
			gp_out << " from " << TSV_group.bb.ll.x << "," << TSV_group.bb.ll.y;
			gp_out << " to " << TSV_group.bb.ur.x << "," << TSV_group.bb.ur.y;
			gp_out << " fillcolor rgb \"#704a30\" fillstyle solid";
			gp_out << std::endl;
		}

		// check alignment fulfillment; draw accordingly colored rectangles around
		// affected blocks
		//
		if (fp.opt_flags.alignment) {

			for (Block const& cur_block : fp.blocks) {

				if (cur_block.layer != cur_layer) {
					continue;
				}

				// for each alignment request defined for the block; draw the
				// related intersection/offset to illustrate block alignment
				for (CorblivarAlignmentReq const& req :  alignment) {

					if (req.s_i->numerical_id == cur_block.numerical_id || req.s_j->numerical_id == cur_block.numerical_id) {

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
							alignment_rect.ll.x = std::min(req.s_i->bb.ll.x, req.s_j->bb.ll.x);
							alignment_rect.ur.x = std::max(req.s_i->bb.ur.x, req.s_j->bb.ur.x);
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
							alignment_rect.ll.y = std::min(req.s_i->bb.ll.y, req.s_j->bb.ll.y);
							alignment_rect.ur.y = std::max(req.s_i->bb.ur.y, req.s_j->bb.ur.y);
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
								gp_out << std::endl;
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
								gp_out << std::endl;
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
							gp_out << std::endl;

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
							gp_out << std::endl;
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
								gp_out << std::endl;
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
								gp_out << std::endl;
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
							gp_out << std::endl;

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
							gp_out << std::endl;
						}
					}
				}
			}
		}

		// file footer
		gp_out << "plot NaN notitle" << std::endl;

		// close file stream
		gp_out.close();
	}

	if (fp.logMed()) {
		std::cout << "IO> ";
		std::cout << "Done" << std::endl << std::endl;
	}
}

/// generate files for HotSpot steady-state thermal simulation
void IO::writeHotSpotFiles(FloorPlanner const& fp, std::string const& benchmark_suffix) {
	std::ofstream file, file_bond;
	int cur_layer;
	int x, y;
	int map_x, map_y;
	float x_ll, y_ll;
	float bin_w, bin_h;

	if (fp.logMed()) {
		std::cout << "IO> Generating files for HotSpot 3D-thermal simulation..." << std::endl;
		if (benchmark_suffix != "") {
			std::cout << "IO>  Benchmark suffix: " << benchmark_suffix << std::endl;
		}
	}

	/// generate floorplan files
	for (cur_layer = 0; cur_layer < fp.IC.layers; cur_layer++) {

		// build up file name
		std::stringstream fp_file;
		fp_file << fp.benchmark << benchmark_suffix << "_HotSpot_Si_active_" << cur_layer + 1 << ".flp";

		// init file stream
		file.open(fp_file.str().c_str());

		// file header
		file << "# Line Format: <unit-name>\\t<width>\\t<height>\\t<left-x>\\t<bottom-y>\\t<specific-heat>\\t<resistivity>" << std::endl;
		file << "# all dimensions are in meters" << std::endl;
		file << "# comment lines begin with a '#'" << std::endl;
		file << "# comments and empty lines are ignored" << std::endl;
		file << std::endl;

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
			file << std::endl;
		}

		// dummy block to describe layer outline
		file << "outline_" << cur_layer + 1;
		file << "	" << fp.IC.outline_x * Math::SCALE_UM_M;
		file << "	" << fp.IC.outline_y * Math::SCALE_UM_M;
		file << "	0.0";
		file << "	0.0";
		file << "	" << ThermalAnalyzer::HEAT_CAPACITY_SI;
		file << "	" << ThermalAnalyzer::THERMAL_RESISTIVITY_SI;
		file << std::endl;

		// close file stream
		file.close();
	}

	/// generate floorplans for passive Si and bonding layer; considering TSVs (modelled via densities)
	for (cur_layer = 0; cur_layer < fp.IC.layers; cur_layer++) {

		// build up file names
		std::stringstream Si_fp_file;
		Si_fp_file << fp.benchmark << benchmark_suffix << "_HotSpot_Si_passive_" << cur_layer + 1 << ".flp";
		std::stringstream bond_fp_file;
		bond_fp_file << fp.benchmark << benchmark_suffix << "_HotSpot_bond_" << cur_layer + 1 << ".flp";

		// init file streams
		file.open(Si_fp_file.str().c_str());
		file_bond.open(bond_fp_file.str().c_str());

		// file headers
		file << "# Line Format: <unit-name>\\t<width>\\t<height>\\t<left-x>\\t<bottom-y>\\t<specific-heat>\\t<resistivity>" << std::endl;
		file << "# all dimensions are in meters" << std::endl;
		file << "# comment lines begin with a '#'" << std::endl;
		file << "# comments and empty lines are ignored" << std::endl;
		file_bond << "# Line Format: <unit-name>\\t<width>\\t<height>\\t<left-x>\\t<bottom-y>\\t<specific-heat>\\t<resistivity>" << std::endl;
		file_bond << "# all dimensions are in meters" << std::endl;
		file_bond << "# comment lines begin with a '#'" << std::endl;
		file_bond << "# comments and empty lines are ignored" << std::endl;

		// for thermal-analysis fitting runs, we consider one common TSV density
		// for the whole chip outline
		if (fp.thermal_analyser_run) {

			file << "Si_passive_" << cur_layer + 1;
			file << "	" << fp.IC.outline_x * Math::SCALE_UM_M;
			file << "	" << fp.IC.outline_y * Math::SCALE_UM_M;
			file << "	0.0";
			file << "	0.0";
			file << "	" << ThermalAnalyzer::heatCapSi(fp.techParameters.TSV_group_Cu_area_ratio, fp.power_blurring_parameters.TSV_density);
			file << "	" << ThermalAnalyzer::thermResSi(fp.techParameters.TSV_group_Cu_area_ratio, fp.power_blurring_parameters.TSV_density);
			file << std::endl;

			file_bond << "bond_" << cur_layer + 1;
			file_bond << "	" << fp.IC.outline_x * Math::SCALE_UM_M;
			file_bond << "	" << fp.IC.outline_y * Math::SCALE_UM_M;
			file_bond << "	0.0";
			file_bond << "	0.0";
			file_bond << "	" << ThermalAnalyzer::heatCapBond(fp.techParameters.TSV_group_Cu_area_ratio, fp.power_blurring_parameters.TSV_density);
			file_bond << "	" << ThermalAnalyzer::thermResBond(fp.techParameters.TSV_group_Cu_area_ratio, fp.power_blurring_parameters.TSV_density);
			file_bond << std::endl;
		}
		// for regular runs, i.e., Corblivar runs, we have to consider different
		// TSV densities for each grid bin, given in the power_maps
		else {
			// walk power-map grid to obtain specific TSV densities of bins
			for (x = ThermalAnalyzer::POWER_MAPS_PADDED_BINS; x < ThermalAnalyzer::THERMAL_MAP_DIM + ThermalAnalyzer::POWER_MAPS_PADDED_BINS; x++) {

				// adapt index for final thermal map according to padding
				map_x = x - ThermalAnalyzer::POWER_MAPS_PADDED_BINS;

				// pre-calculate bin's lower-left corner coordinates;
				// float precision required to avoid grid coordinate
				// mismatches
				x_ll = static_cast<float>(map_x * fp.thermalAnalyzer.power_maps_dim_x * Math::SCALE_UM_M);

				// pre-calculate bin dimensions; float precision required
				// to avoid grid coordinate mismatches; re-calculation
				// only required for lower and upper bounds
				//
				// lower bound, regular bin dimension; value also used
				// until reaching upper bound
				if (x == ThermalAnalyzer::POWER_MAPS_PADDED_BINS) {
					bin_w = static_cast<float>(fp.thermalAnalyzer.power_maps_dim_x * Math::SCALE_UM_M);
				}
				// upper bound, limit bin dimension according to overall
				// chip outline; scale down slightly is required to avoid
				// rounding errors during HotSpot's grid mapping
				else if (x == (ThermalAnalyzer::THERMAL_MAP_DIM + ThermalAnalyzer::POWER_MAPS_PADDED_BINS - 1)) {
					bin_w = 0.999 * static_cast<float>(fp.IC.outline_x * Math::SCALE_UM_M - x_ll);
				}

				for (y = ThermalAnalyzer::POWER_MAPS_PADDED_BINS; y < ThermalAnalyzer::THERMAL_MAP_DIM + ThermalAnalyzer::POWER_MAPS_PADDED_BINS; y++) {
					// adapt index for final thermal map according to padding
					map_y = y - ThermalAnalyzer::POWER_MAPS_PADDED_BINS;

					// pre-calculate bin's lower-left corner
					// coordinates; float precision required to avoid
					// grid coordinate mismatches
					y_ll = static_cast<float>(map_y * fp.thermalAnalyzer.power_maps_dim_y * Math::SCALE_UM_M);

					// pre-calculate bin dimensions; float precision required
					// to avoid grid coordinate mismatches; re-calculation
					// only required for lower and upper bounds
					//
					// lower bound, regular bin dimension; value also used
					// until reaching upper bound
					if (y == ThermalAnalyzer::POWER_MAPS_PADDED_BINS) {
						bin_h = static_cast<float>(fp.thermalAnalyzer.power_maps_dim_y * Math::SCALE_UM_M);
					}
					// upper bound, limit bin dimension according to
					// overall chip outline; scale down slightly is
					// required to avoid rounding errors during
					// HotSpot's grid mapping
					else if (y == (ThermalAnalyzer::THERMAL_MAP_DIM + ThermalAnalyzer::POWER_MAPS_PADDED_BINS - 1)) {
						bin_h = 0.999 * static_cast<float>(fp.IC.outline_y * Math::SCALE_UM_M - y_ll);
					}

					// put grid block as floorplan blocks; passive Si layer
					file << "Si_passive_" << cur_layer + 1 << "_" << map_x << ":" << map_y;
					/// bin dimensions
					file << "	" << bin_w;
					file << "	" << bin_h;
					/// bin's lower-left corner
					file << "	" << x_ll;
					file << "	" << y_ll;
					// thermal properties, depending on bin's TSV density
					file << "	" << ThermalAnalyzer::heatCapSi(fp.techParameters.TSV_group_Cu_area_ratio, fp.thermalAnalyzer.power_maps[cur_layer][x][y].TSV_density);
					file << "	" << ThermalAnalyzer::thermResSi(fp.techParameters.TSV_group_Cu_area_ratio, fp.thermalAnalyzer.power_maps[cur_layer][x][y].TSV_density);
					file << std::endl;

					// put grid block as floorplan blocks; bonding layer
					file_bond << "bond_" << cur_layer + 1 << "_" << map_x << ":" << map_y;
					/// bin dimensions
					file_bond << "	" << bin_w;
					file_bond << "	" << bin_h;
					/// bin's lower-left corner
					file_bond << "	" << x_ll;
					file_bond << "	" << y_ll;
					// thermal properties, depending on bin's TSV density
					file_bond << "	" << ThermalAnalyzer::heatCapBond(fp.techParameters.TSV_group_Cu_area_ratio, fp.thermalAnalyzer.power_maps[cur_layer][x][y].TSV_density);
					file_bond << "	" << ThermalAnalyzer::thermResBond(fp.techParameters.TSV_group_Cu_area_ratio, fp.thermalAnalyzer.power_maps[cur_layer][x][y].TSV_density);
					file_bond << std::endl;
				}
			}
		}

		// close file streams
		file.close();
		file_bond.close();
	}

	/// generate dummy floorplan for BEOL layer; TSVs are not to be considered
	//
	for (cur_layer = 0; cur_layer < fp.IC.layers; cur_layer++) {


		// build up file name
		std::stringstream BEOL_fp_file;
		BEOL_fp_file << fp.benchmark << benchmark_suffix << "_HotSpot_BEOL_" << cur_layer + 1 << ".flp";

		// init file stream
		file.open(BEOL_fp_file.str().c_str());

		// file header
		file << "# Line Format: <unit-name>\\t<width>\\t<height>\\t<left-x>\\t<bottom-y>\\t<specific-heat>\\t<resistivity>" << std::endl;
		file << "# all dimensions are in meters" << std::endl;
		file << "# comment lines begin with a '#'" << std::endl;
		file << "# comments and empty lines are ignored" << std::endl;
		file << std::endl;

		// dummy blocks representing bb over all wires; related power consumption
		// also modeled in ptrace file
		for (Block const& cur_wire : fp.wires) {

			if (cur_wire.layer != cur_layer) {
				continue;
			}

			file << cur_wire.id << " ";
			file << "	" << cur_wire.bb.w * Math::SCALE_UM_M;
			file << "	" << cur_wire.bb.h * Math::SCALE_UM_M;
			file << "	" << cur_wire.bb.ll.x * Math::SCALE_UM_M;
			file << "	" << cur_wire.bb.ll.y * Math::SCALE_UM_M;
			file << "	" << ThermalAnalyzer::HEAT_CAPACITY_BEOL;
			file << "	" << ThermalAnalyzer::THERMAL_RESISTIVITY_BEOL;
			file << std::endl;
		}

		// dummy BEOL outline ``block''
		file << "BEOL_" << cur_layer + 1;;
		file << "	" << fp.IC.outline_x * Math::SCALE_UM_M;
		file << "	" << fp.IC.outline_y * Math::SCALE_UM_M;
		file << "	0.0";
		file << "	0.0";
		file << "	" << ThermalAnalyzer::HEAT_CAPACITY_BEOL;
		file << "	" << ThermalAnalyzer::THERMAL_RESISTIVITY_BEOL;
		file << std::endl;

		// close file stream
		file.close();
	}

	/// generate power-trace file
	//
	// build up file name
	std::stringstream power_file;
	power_file << fp.benchmark << benchmark_suffix << "_HotSpot.ptrace";

	// init file stream
	file.open(power_file.str().c_str());

	// block sequence in trace file has to follow layer files, thus build up file
	// according to layer structure
	//
	// output block labels in first line
	for (cur_layer = 0; cur_layer < fp.IC.layers; cur_layer++) {

		// output dummy blocks representing wires first, since they are placed in
		// the BEOL layer, coming before the active Si layer
		for (Block const& cur_wire : fp.wires) {

			if (cur_wire.layer != cur_layer) {
				continue;
			}

			file << cur_wire.id << " ";
		}

		// dummy BEOL outline block
		file << "BEOL_" << cur_layer + 1 << " ";

		// actual blocks
		for (Block const& cur_block : fp.blocks) {

			if (cur_block.layer != cur_layer) {
				continue;
			}

			file << cur_block.id << " ";
		}

		// dummy outline block
		file << "outline_" << cur_layer + 1 << " ";
	}
	file << std::endl;

	// output block power in second line
	for (cur_layer = 0; cur_layer < fp.IC.layers; cur_layer++) {

		// dummy blocks representing wires along with their power consumption
		for (Block const& cur_wire : fp.wires) {

			if (cur_wire.layer != cur_layer) {
				continue;
			}

			// actual power encoded in power_density_unscaled, see
			// ThermalAnalyzer::adaptPowerMapsWires
			file << cur_wire.power_density_unscaled << " ";
		}

		// dummy BEOL outline block
		file << "0.0 ";

		for (Block const& cur_block : fp.blocks) {

			if (cur_block.layer != cur_layer) {
				continue;
			}

			file << cur_block.power() << " ";
		}

		// dummy outline block
		file << "0.0 ";
	}
	file << std::endl;

	// close file stream
	file.close();

	/// generate 3D-IC description file
	// build up file name
	std::stringstream stack_file;
	stack_file << fp.benchmark << benchmark_suffix << "_HotSpot.lcf";

	// init file stream
	file.open(stack_file.str().c_str());

	// file header
	file << "#Lines starting with # are used for commenting" << std::endl;
	file << "#Blank lines are also ignored" << std::endl;
	file << std::endl;
	file << "#File Format:" << std::endl;
	file << "#<Layer Number>" << std::endl;
	file << "#<Lateral heat flow Y/N?>" << std::endl;
	file << "#<Power Dissipation Y/N?>" << std::endl;
	file << "#<Specific heat capacity in J/(m^3K)>" << std::endl;
	file << "#<Resistivity in (m-K)/W>" << std::endl;
	file << "#<Thickness in m>" << std::endl;
	file << "#<floorplan file>" << std::endl;
	file << std::endl;

	for (cur_layer = 0; cur_layer < fp.IC.layers; cur_layer++) {

		file << "# BEOL (interconnects) layer " << cur_layer + 1 << std::endl;
		file << 4 * cur_layer << std::endl;
		file << "Y" << std::endl;
		file << "Y" << std::endl;
		file << ThermalAnalyzer::HEAT_CAPACITY_BEOL << std::endl;
		file << ThermalAnalyzer::THERMAL_RESISTIVITY_BEOL << std::endl;
		file << fp.techParameters.BEOL_thickness * Math::SCALE_UM_M << std::endl;
		file << fp.benchmark << benchmark_suffix << "_HotSpot_BEOL_" << cur_layer + 1 << ".flp" << std::endl;
		file << std::endl;

		file << "# Active Si layer; design layer " << cur_layer + 1 << std::endl;
		file << 4 * cur_layer + 1 << std::endl;
		file << "Y" << std::endl;
		file << "Y" << std::endl;
		file << ThermalAnalyzer::HEAT_CAPACITY_SI << std::endl;
		file << ThermalAnalyzer::THERMAL_RESISTIVITY_SI << std::endl;
		file << fp.techParameters.Si_active_thickness * Math::SCALE_UM_M << std::endl;
		file << fp.benchmark << benchmark_suffix << "_HotSpot_Si_active_" << cur_layer + 1 << ".flp" << std::endl;
		file << std::endl;

		file << "# Passive Si layer " << cur_layer + 1 << std::endl;
		file << 4 * cur_layer + 2 << std::endl;
		file << "Y" << std::endl;
		file << "N" << std::endl;
		// dummy values, proper values (depending on TSV densities) are in the
		// actual floorplan file
		file << ThermalAnalyzer::HEAT_CAPACITY_SI << std::endl;
		file << ThermalAnalyzer::THERMAL_RESISTIVITY_SI << std::endl;
		file << fp.techParameters.Si_passive_thickness * Math::SCALE_UM_M << std::endl;
		file << fp.benchmark << benchmark_suffix << "_HotSpot_Si_passive_" << cur_layer + 1 << ".flp" << std::endl;
		file << std::endl;

		if (cur_layer < (fp.IC.layers - 1)) {
			file << "# bond layer " << cur_layer + 1 << "; for F2B bonding to next die " << cur_layer + 2 << std::endl;
			file << 4 * cur_layer + 3 << std::endl;
			file << "Y" << std::endl;
			file << "N" << std::endl;
			// dummy values, proper values (depending on TSV densities) are in
			// the actual floorplan file
			file << ThermalAnalyzer::HEAT_CAPACITY_BOND << std::endl;
			file << ThermalAnalyzer::THERMAL_RESISTIVITY_BOND << std::endl;
			file << fp.techParameters.bond_thickness * Math::SCALE_UM_M << std::endl;
			file << fp.benchmark << benchmark_suffix << "_HotSpot_bond_" << cur_layer + 1 << ".flp" << std::endl;
			file << std::endl;
		}
	}

	// close file stream
	file.close();

	if (fp.logMed()) {
		std::cout << "IO> Done" << std::endl << std::endl;
	}
}
