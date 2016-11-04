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

// DBG flag
static constexpr bool DBG = false;

// type definitions, for shorter notation
typedef	std::array< std::array<double, ThermalAnalyzer::THERMAL_MAP_DIM>, ThermalAnalyzer::THERMAL_MAP_DIM> thermal_maps_layer_type;
typedef	std::vector< thermal_maps_layer_type > thermal_maps_type;

// forward declaration
void calculatePearsonCorr(FloorPlanner& fp, thermal_maps_type& thermal_maps);
void parseHotSpotFiles(FloorPlanner& fp, thermal_maps_type& thermal_maps);

// (TODO) compare actual correlation to correlation estimation, resulting from power-blurring thermal analysis
//
int main (int argc, char** argv) {
	FloorPlanner fp;
	thermal_maps_type thermal_maps_HotSpot;

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
		fp.finalize(corb, false);
		std::cout << std::endl;

	// (TODO) HotSpot.sh system call
	//

	// now, read in the HotSpot simulation result
	//
	parseHotSpotFiles(fp, thermal_maps_HotSpot);

	// now, we may calculate the correlation of the power maps and thermal maps
	//
	calculatePearsonCorr(fp, thermal_maps_HotSpot);
}

void parseHotSpotFiles(FloorPlanner& fp, thermal_maps_type& thermal_maps) {
	std::ifstream layer_file;
	int x, y;
	double temp;

	thermal_maps.clear();

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

		// init maps structure
		thermal_maps.emplace_back(thermal_maps_layer_type());

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

			// memorize temp values in thermal map
			thermal_maps[layer][x][y] = temp;

			// DBG output
			if (DBG) {
				std::cout << "Temp for [layer= " << layer << "][x= " << x << "][y= " << y << "]: " << thermal_maps[layer][x][y] << std::endl;
				std::cout << "Power for [layer= " << layer << "][x= " << x << "][y= " << y << "]: " << fp.getThermalAnalyzer().getPowerMapsOrig()[layer][x][y].power_density << std::endl;
			}
		}

		// close file
		layer_file.close();
	}
}

void calculatePearsonCorr(FloorPlanner& fp, thermal_maps_type& thermal_maps) {
	double avg_power, avg_temp;
	double std_dev_power, std_dev_temp;
	double cov;
	double cur_power_dev, cur_temp_dev;
	double correlation;

	for (int layer = 0; layer < fp.getLayers(); layer++) {

		avg_power = avg_temp = 0.0;
		cov = std_dev_power = std_dev_temp = 0.0;
		correlation = 0.0;

		// first pass: determine avg values
		//
		for (int x = 0; x < ThermalAnalyzer::THERMAL_MAP_DIM; x++) {
			for (int y = 0; y < ThermalAnalyzer::THERMAL_MAP_DIM; y++) {

				avg_power += fp.getThermalAnalyzer().getPowerMapsOrig()[layer][x][y].power_density;
				avg_temp += thermal_maps[layer][x][y];
			}
		}
		avg_power /= std::pow(ThermalAnalyzer::THERMAL_MAP_DIM, 2);
		avg_temp /= std::pow(ThermalAnalyzer::THERMAL_MAP_DIM, 2);

		// DBG output
		if (DBG) {
			std::cout << "Avg power for layer " << layer << ": " << avg_power << std::endl;
			std::cout << "Avg temp for layer " << layer << ": " << avg_temp << std::endl;
			std::cout << std::endl;
		}
		
		// second pass: determine covariance and standard deviations
		//
		for (int x = 0; x < ThermalAnalyzer::THERMAL_MAP_DIM; x++) {
			for (int y = 0; y < ThermalAnalyzer::THERMAL_MAP_DIM; y++) {

				// deviations of current values from avg values
				cur_power_dev = fp.getThermalAnalyzer().getPowerMapsOrig()[layer][x][y].power_density - avg_power;
				cur_temp_dev = thermal_maps[layer][x][y] - avg_temp;

				// covariance
				cov += cur_power_dev * cur_temp_dev;

				// standard deviation, to be sqrd later on
				std_dev_power += std::pow(cur_power_dev, 2.0);
				std_dev_temp += std::pow(cur_temp_dev, 2.0);
			}
		}
		std_dev_power = std::sqrt(std_dev_power);
		std_dev_temp = std::sqrt(std_dev_temp);

		// calculate Pearson correlation: covariance over product of standard deviations
		//
		correlation = cov / (std_dev_power * std_dev_temp);

		// DBG output
		if (DBG) {
			std::cout << "Standard deviation of power for layer " << layer << ": " << std_dev_power << std::endl;
			std::cout << "Standard deviation of temp for layer " << layer << ": " << std_dev_temp << std::endl;
			std::cout << "Covariance of temp and power for layer " << layer << ": " << cov << std::endl;
			std::cout << std::endl;
		}

		std::cout << "Pearson correlation of temp and power for layer " << layer << ": " << correlation << std::endl;
		std::cout << std::endl;
	}
}
