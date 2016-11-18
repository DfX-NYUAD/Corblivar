/*
 * =====================================================================================
 *
 *    Description: Calculates the linear Parson correlation of Corblivar's power maps and the corresponding HotSpot's thermal maps
 *
 *    Copyright (C) 2016 Johann Knechtel, johann aett nyu dot edu
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
#include <random>
#include <chrono>

// logging flags
static constexpr bool DBG = true;

// type definitions, for shorter notation
static constexpr unsigned VARIATIONS_FRAME_DIM = 1;
typedef	std::array< std::array< std::array<double, VARIATIONS_FRAME_DIM> , ThermalAnalyzer::THERMAL_MAP_DIM>, ThermalAnalyzer::THERMAL_MAP_DIM> variations_data_layer_type;
typedef	std::vector< variations_data_layer_type > variations_data_type;

// forward declaration
void parseHotSpotFiles(FloorPlanner& fp, unsigned frame, variations_data_type& temperature_variations);

int main (int argc, char** argv) {
	FloorPlanner fp;
	double corr;
	bool correlation_reduced;

	variations_data_type temperature_variations;
	variations_data_type power_variations;

	// construct a trivial random generator engine from a time-based seed:
	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	std::default_random_engine random_generator(seed);

	std::cout << std::endl;
	std::cout << "Thermal Side-Channel Leakage Verification: Determine Entropy and Correlation of Power and Thermal Maps" << std::endl;
	std::cout << "------------------------------------------------------------------------------------------------------" << std::endl;
	std::cout << "WARNING: File handling implicitly assumes that the dimensions of power and thermal maps are all the same, both within HotSpot and Corblivar; parsing and calculation will most likely fail if there are dimension mismatches!" << std:: endl;
	std::cout << std::endl;

	// parse program parameter, config file, and further files
	IO::parseParametersFiles(fp, argc, argv);
	// parse blocks
	IO::parseBlocks(fp);
	// parse nets
	IO::parseNets(fp);

	// init Corblivar core
	CorblivarCore corb = CorblivarCore(fp.getLayers(), fp.getBlocks().size());

	// parse alignment request
	IO::parseAlignmentRequests(fp, corb.editAlignments());

	// init thermal analyzer, only reasonable after parsing config file
	fp.initThermalAnalyzer();

	// init routing-utilization analyzer
	fp.initRoutingUtilAnalyzer();

	// no solution file found; error
	if (!fp.inputSolutionFileOpen()) {
		std::cout << "Corblivar> ";
		std::cout << "ERROR: Solution file required for call of " << argv[0] << std::endl << std::endl;
		exit(1);
	}

	// required solution file found; parse from file, and generate layout and all data such as power and thermal maps
	//
	// read from file
	IO::parseCorblivarFile(fp, corb);

	// assume read in data as currently best solution
	corb.storeBestCBLs();

	// overall cost is not determined; cost cannot be determined since no
	// normalization during SA search was performed
	//
	// generates also all required files
	fp.finalize(corb, false);

	std::cout << std::endl;

	// iteratively try to reduce the worst correlations over each layer
	//
	// TODO memorize whether correlation was reduced for each layer individually
	correlation_reduced = true;
	while (correlation_reduced) {

		// clear local variations_data_type
		//
		power_variations.clear();
		temperature_variations.clear();

		// allocate vectors
		for (int layer = 0; layer < fp.getLayers(); layer++) {

			power_variations.emplace_back(variations_data_layer_type());
			temperature_variations.emplace_back(variations_data_layer_type());
		}

		// generate new power variations and gather related HotSpot simulation temperature data
		//
		for (unsigned frame = 0; frame < VARIATIONS_FRAME_DIM; frame++) {

			// first, randomly vary power densities in blocks
			//
			for (Block const& b : fp.getBlocks()) {

				// restore original value, used as mean for Gaussian distribution of power variations
				b.power_density_unscaled = b.power_density_unscaled_back;

				// calculate new power value, based on Gaussian distribution with std dev of 10% of power value
				std::normal_distribution<double> gaussian(b.power_density_unscaled, b.power_density_unscaled * 0.1);

				b.power_density_unscaled = gaussian(random_generator);

				if (DBG) {
					std::cout << "Block " << b.id << ":" << std::endl;
					std::cout << " Original power = " << b.power_density_unscaled_back << std::endl;
					std::cout << " New random power = " << b.power_density_unscaled << std::endl;
				}
			}

			// second, generate new power maps
			//
			fp.editThermalAnalyzer().generatePowerMaps(fp.getLayers(), fp.getBlocks(), fp.getOutline(), fp.getPowerBlurringParameters());

			// copy data from Corblivar power maps into local data structure power_variations
			//
			for (int layer = 0; layer < fp.getLayers(); layer++) {
				for (unsigned x = 0; x < ThermalAnalyzer::THERMAL_MAP_DIM; x++) {
					for (unsigned y = 0; y < ThermalAnalyzer::THERMAL_MAP_DIM; y++) {

						power_variations[layer][x][y][frame] = fp.getPowerMapsOrig()[layer][x][y].power_density;
					}
				}
			}

			// third, run HotSpot on this new map
			//
			// TODO generate new ptrace file first; not all HotSpot files
			IO::writeHotSpotFiles(fp);
			// TODO HotSpot.sh system call

			// fourth, read in the new HotSpot results into local data structure temperature_variations
			//
			parseHotSpotFiles(fp, frame, temperature_variations);
		}

		// TODO
		// calculate and memorize new Pearson correlation for each bin of grid
		//
		

		// TODO
		// adapt TSV densities in original power map; set to 100% for bins within frame of 90-100% of worst correlation
		//
		// only if new worst correlation is smaller than previous worst correlation; otherwise, the new correlation actually increased again, that is, the previous TSV adaption
		// was not beneficial anymore, and further dummy-TSV insertions will most likely also not be helpful
		//
		// in general, ignore upper-most layer; don't place TSVs there

		correlation_reduced = false;
	}

	// (TODO) probably not required
	//// restore original power values
	//for (Block const& b : fp.getBlocks()) {

	//	b.power_density_unscaled = b.power_density_unscaled_back;
	//}
}

void parseHotSpotFiles(FloorPlanner& fp, unsigned frame, variations_data_type& temperature_variations) {
	std::ifstream layer_file;
	int x, y;
	double temp;

	for (int layer = 0; layer < fp.getLayers(); layer++) {

		// file name
		std::stringstream layer_file_name;
		// HotSpot files for active Si layer; offsets defined accordingly to file generation in IO::writeMaps, IO::writeHotSpotFiles
		layer_file_name << fp.getBenchmark() << "_HotSpot.steady.grid.gp_data.layer_" << (1 + 4 * layer);

		// open file
		layer_file.open(layer_file_name.str().c_str());
		if (!layer_file.good()) {
			std::cout << "HotSpot file \"" << layer_file_name.str() << "\" missing!" << std::endl;
			exit(1);
		}

		// parse file
		//
		// syntax: X Y TEMP

		while (!layer_file.eof()) {

			layer_file >> x;
			layer_file >> y;
			layer_file >> temp;

			// drop the dummy data points, inserted for gnuplot
			//
			if (x == ThermalAnalyzer::THERMAL_MAP_DIM || y == ThermalAnalyzer::THERMAL_MAP_DIM) {
				continue;
			}

			// memorize temperature value for its respective bin
			temperature_variations[layer][x][y][frame] = temp;

			// DBG output
			if (DBG) {
				std::cout << "Temp for [layer= " << layer << "][x= " << x << "][y= " << y << "]: " << temperature_variations[layer][x][y][frame] << std::endl;
				std::cout << "Power for [layer= " << layer << "][x= " << x << "][y= " << y << "]: " << fp.getPowerMapsOrig()[layer][x][y].power_density << std::endl;
			}
		}

		// close file
		layer_file.close();
	}
}
