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
static constexpr bool DBG = false;
static constexpr bool DBG_PARSING = false;

// global fixed parameters
static constexpr unsigned SAMPLING_ITERATIONS = 10;
// for the Gaussian distribution of power values; the std dev is set up from the mean value and this factor
static constexpr double MEAN_TO_STD_DEV_FACTOR = 0.1;
// for dummy TSV insertion, consider all bins with correlations above this fraction of the worst correlation per layer
static constexpr double MAX_CORR_RANGE = 0.99;

// type definitions, for shorter notation
typedef	std::array< std::array< std::array<double, SAMPLING_ITERATIONS> , ThermalAnalyzer::THERMAL_MAP_DIM>, ThermalAnalyzer::THERMAL_MAP_DIM> samples_data_layer_type;
typedef	std::vector< samples_data_layer_type > samples_data_type;
typedef std::array< std::array<double, ThermalAnalyzer::THERMAL_MAP_DIM>, ThermalAnalyzer::THERMAL_MAP_DIM> correlations_layer_type;
// copied from Variation_TSC
typedef	std::array< std::array<ThermalAnalyzer::ThermalMapBin, ThermalAnalyzer::THERMAL_MAP_DIM>, ThermalAnalyzer::THERMAL_MAP_DIM> thermal_maps_layer_type;
typedef	std::vector< thermal_maps_layer_type > thermal_maps_type;

// forward declaration
void parseHotSpotFiles(FloorPlanner& fp, unsigned sampling_iter, samples_data_type& temp_samples);
void writeHotSpotPtrace(FloorPlanner& fp);
void writeHotSpotFiles__passiveSi_bonding(FloorPlanner& fp);
// copied and adapted from Variation_TSC
void parseHotSpotFiles(FloorPlanner& fp, std::string const& benchmark_suffix, thermal_maps_type& thermal_maps);

int main (int argc, char** argv) {
	FloorPlanner fp;

	samples_data_type temp_samples;
	samples_data_type power_samples;

	double avg_power, avg_temp;
	double std_dev_power, std_dev_temp;
	double cur_power_dev, cur_temp_dev;

	double cov;

	double corr;
	double max_corr;
	double avg_corr;
	double std_dev_corr;
	int count_corr;

	std::vector< correlations_layer_type > correlations;

	bool run;
	std::vector<double> correlation_avgs;
	std::vector<double> prev_correlation_avgs;
	std::vector<double> max_correlation_avgs;
	std::vector<double> prev_max_correlation_avgs;
	std::vector<double> std_dev_correlation_avgs;
	std::vector<double> prev_std_dev_correlation_avgs;

	int adapted_bins = 0;
	int prev_adapted_bins;

	std::list<std::string> dummy_TSVs_to_delete;

	std::vector<TSV_Island> original_dummy_TSVs = fp.getDummyTSVs();

	thermal_maps_type thermal_maps_HotSpot;
	double entropy;

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
	run = true;
	while (run) {

		// clear local samples_data_type
		//
		power_samples.clear();
		temp_samples.clear();
		correlations.clear();

		// allocate vectors
		for (int layer = 0; layer < fp.getLayers(); layer++) {

			power_samples.emplace_back(samples_data_layer_type());
			temp_samples.emplace_back(samples_data_layer_type());
			correlations.emplace_back(correlations_layer_type());
		}

		// generate power data and gather related HotSpot simulation temperature data
		//
		for (unsigned sampling_iter = 0; sampling_iter < SAMPLING_ITERATIONS; sampling_iter++) {

			std::cout << std::endl;
			std::cout << "Sampling iteration: " << (sampling_iter + 1) << "/" << SAMPLING_ITERATIONS << std::endl;
			std::cout << "------------------------------" << std::endl;

			// first, randomly vary power densities in blocks
			//
			for (Block const& b : fp.getBlocks()) {

				// restore original value, used as mean for Gaussian distribution of power densities
				b.power_density_unscaled = b.power_density_unscaled_back;

				// calculate new power value, based on Gaussian distribution
				std::normal_distribution<double> gaussian(b.power_density_unscaled, b.power_density_unscaled * MEAN_TO_STD_DEV_FACTOR);

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

			// copy data from Corblivar power maps into local data structure power_samples
			//
			for (int layer = 0; layer < fp.getLayers(); layer++) {
				for (unsigned x = 0; x < ThermalAnalyzer::THERMAL_MAP_DIM; x++) {
					for (unsigned y = 0; y < ThermalAnalyzer::THERMAL_MAP_DIM; y++) {

						power_samples[layer][x][y][sampling_iter] = fp.getThermalAnalyzer().getPowerMapsOrig()[layer][x][y].power_density;
					}
				}
			}

			// third, run HotSpot on this new map
			//
			// generate new ptrace file first
			writeHotSpotPtrace(fp);
			// HotSpot.sh system call
			system(std::string("./HotSpot.sh " + fp.getBenchmark() + " " + std::to_string(fp.getLayers())).c_str());

			// fourth, read in the new HotSpot results into local data structure temp_samples
			//
			parseHotSpotFiles(fp, sampling_iter, temp_samples);


			if (DBG) {
				std::cout << "Printing gathered power/temperature data for sampling iteration " << sampling_iter << std::endl;
				std::cout << std::endl;

				for (int layer = 0; layer < fp.getLayers(); layer++) {
					std::cout << " Layer " << layer << std::endl;
					std::cout << std::endl;

					for (unsigned x = 0; x < ThermalAnalyzer::THERMAL_MAP_DIM; x++) {
						for (unsigned y = 0; y < ThermalAnalyzer::THERMAL_MAP_DIM; y++) {

							std::cout << "  Power[" << x << "][" << y << "]: " << power_samples[layer][x][y][sampling_iter] << std::endl;
							std::cout << "  Temp [" << x << "][" << y << "]: " << temp_samples[layer][x][y][sampling_iter] << std::endl;
						}
					}
				}
			}
		}

		// calculate avg Pearson correlation over all bins
		//
		prev_correlation_avgs = correlation_avgs;
		correlation_avgs.clear();
		prev_max_correlation_avgs = max_correlation_avgs;
		max_correlation_avgs.clear();
		prev_std_dev_correlation_avgs = std_dev_correlation_avgs;
		std_dev_correlation_avgs.clear();

		std::cout << std::endl;
		std::cout << "Sampling results" << std::endl;
		std::cout << "----------------" << std::endl;

		for (int layer = 0; layer < fp.getLayers(); layer++) {

			// dbg output
			if (DBG) {
				std::cout << std::endl;
				std::cout << "Pearson correlations on layer " << layer << std::endl;
				std::cout << std::endl;
			}

			avg_corr = 0.0;
			count_corr = 0;
			max_corr = 0.0;

			for (unsigned x = 0; x < ThermalAnalyzer::THERMAL_MAP_DIM; x++) {
				for (unsigned y = 0; y < ThermalAnalyzer::THERMAL_MAP_DIM; y++) {

					avg_power = avg_temp = 0.0;
					cov = std_dev_power = std_dev_temp = 0.0;
					corr = 0.0;

					// first pass: determine avg values
					//
					for (unsigned sampling_iter = 0; sampling_iter < SAMPLING_ITERATIONS; sampling_iter++) {

						avg_power += power_samples[layer][x][y][sampling_iter];
						avg_temp += temp_samples[layer][x][y][sampling_iter];
					}
					avg_power /= SAMPLING_ITERATIONS;
					avg_temp /= SAMPLING_ITERATIONS;

					// dbg output
					if (DBG) {
						std::cout << "Bin: " << x << ", " << y << std::endl;
						std::cout << " Avg power: " << avg_power << std::endl;
						std::cout << " Avg temp: " << avg_temp << std::endl;
					}
					
					// second pass: determine covariance and standard deviations
					//
					for (unsigned sampling_iter = 0; sampling_iter < SAMPLING_ITERATIONS; sampling_iter++) {

						// deviations of current values from avg values
						cur_power_dev = power_samples[layer][x][y][sampling_iter] - avg_power;
						cur_temp_dev = temp_samples[layer][x][y][sampling_iter] - avg_temp;

						// covariance
						cov += cur_power_dev * cur_temp_dev;

						// standard deviation, calculate its sqrt later on
						std_dev_power += std::pow(cur_power_dev, 2.0);
						std_dev_temp += std::pow(cur_temp_dev, 2.0);
					}
					cov /= SAMPLING_ITERATIONS;
					std_dev_power /= SAMPLING_ITERATIONS;
					std_dev_temp /= SAMPLING_ITERATIONS;

					std_dev_power = std::sqrt(std_dev_power);
					std_dev_temp = std::sqrt(std_dev_temp);

					// calculate Pearson correlation: covariance over product of standard deviations
					//
					corr = cov / (std_dev_power * std_dev_temp);

					// consider only valid correlations values
					if (!std::isnan(corr)) {

						avg_corr += corr;
						count_corr++;

						// also memorize the max/worst correlation; note that worst correlation may in theory also be negative but this is unlikely for
						// power-thermal correlations
						max_corr = std::max(max_corr, corr);
					}

					// memorize all correlation values separately; also memorize nan (otherwise we should initially reset correlations array to some pre-defined
					// dummy value which could serve the same purpose, to not consider that related bin)
					correlations[layer][x][y] = corr;

					// dbg output
					if (DBG) {
						std::cout << " Correlation: " << correlations[layer][x][y] << std::endl;
						if (std::isnan(correlations[layer][x][y])) {
							std::cout << "  NAN, because of zero power; to be skipped" << std::endl;
						}
					}
				}
			}
			avg_corr /= count_corr;

			// now, also calculate the std dev of correlations 
			std_dev_corr = 0.0;
			for (unsigned x = 0; x < ThermalAnalyzer::THERMAL_MAP_DIM; x++) {
				for (unsigned y = 0; y < ThermalAnalyzer::THERMAL_MAP_DIM; y++) {

					// consider only valid correlations values
					if (!std::isnan(correlations[layer][x][y])) {

						std_dev_corr += std::pow(correlations[layer][x][y] - avg_corr, 2.0);
					}
				}
			}
			std_dev_corr /= count_corr;
			std_dev_corr = std::sqrt(std_dev_corr);

			std::cout << "Avg Pearson correlations over all bins on layer " << layer << ": " << avg_corr << std::endl;
			std::cout << "Max Pearson correlation over all bins on layer " << layer << ": " << max_corr << std::endl;
			std::cout << "Std dev of Pearson correlation over all bins on layer " << layer << ": " << std_dev_corr << std::endl;

			correlation_avgs.push_back(avg_corr);
			max_correlation_avgs.push_back(max_corr);
			std_dev_correlation_avgs.push_back(std_dev_corr);
		}

		// compare new avg correlations with previous correlations
		//
		// if new avg is larger than previous values, then the previous TSV adaption was not beneficial, and further dummy-TSV insertions will probably also not be helpful
		//
		if (!prev_correlation_avgs.empty()) {

			// initially assume no improvements
			run = false;

			// now, check for each layer individually
			//
			for (int layer = 0; layer < fp.getLayers(); layer++) {

				if (correlation_avgs[layer] < prev_correlation_avgs[layer]) {
					run = true;
					break;
				}
			}
		}

		// at least for one layer there was an improvement compared to the previous iteration; so we try further; note that this may worsen other layers on the other hand
		//
		if (run) {
			prev_adapted_bins = adapted_bins;
			dummy_TSVs_to_delete.clear();

			std::cout << std::endl;
			std::cout << "Adapt TSV densities via dummy TSVs" << std::endl;
			std::cout << "----------------------------------" << std::endl;
			std::cout << std::endl;
			std::cout << " Dummy TSVs before: " << fp.getDummyTSVs().size() << std::endl;

			// adapt TSV densities in power maps by adding dummy TSVs
			//
			// in general, ignore upper-most layer; don't place dummy TSVs there
			for (int layer = 0; layer < (fp.getLayers() - 1); layer++) {

				int adapted_bins_layer = 0;

				for (unsigned x = 0; x < ThermalAnalyzer::THERMAL_MAP_DIM; x++) {
					for (unsigned y = 0; y < ThermalAnalyzer::THERMAL_MAP_DIM; y++) {

						// consider only valid correlations values
						if (std::isnan(correlations[layer][x][y])) {
							continue;
						}

						// consider all bins within range starting from MAX_CORR_RANGE times the worst correlation and more
						//
						if (correlations[layer][x][y] > (max_correlation_avgs[layer] * MAX_CORR_RANGE)) {

							// we have to add a dummy TSV, otherwise the resulting power-density edit will be lost during the next call of generatePowerMaps()
							//
							// first determine the bb of the related grid bin
							Rect bb;
							bb.ll.x = x * (fp.getOutline().x / ThermalAnalyzer::THERMAL_MAP_DIM);
							bb.ll.y = y * (fp.getOutline().y / ThermalAnalyzer::THERMAL_MAP_DIM);
							bb.ur.x = (x + 1) * (fp.getOutline().x / ThermalAnalyzer::THERMAL_MAP_DIM);
							bb.ur.y = (y + 1) * (fp.getOutline().y / ThermalAnalyzer::THERMAL_MAP_DIM);

							// generate id
							std::string id = std::string("dummy_"
										+ std::to_string(x) + "_"
										+ std::to_string(y) + "_"
										+ std::to_string(layer));

							// now, insert a dummy TSV
							fp.editDummyTSVs().emplace_back(TSV_Island(
									id,
									// one TSV count
									1,
									// TSV pitch; required for proper scaling
									// of TSV island
									fp.getTechParameters().TSV_pitch,
									// reference point for placement
									// of dummy TSV is frame bb
									bb,
									// layer assignment
									layer
								));

							// track number of adapted bins
							adapted_bins++;
							adapted_bins_layer++;

							// also track ids of inserted TSVs; may have to be deleted, in case this iteration will be evaluated (in next iteration) to
							// have actually increased the correlation again
							//
							dummy_TSVs_to_delete.push_back(id);
						}
					}
				}
				std::cout << std::endl;
				std::cout << "Adapted bins on layer " << layer << ": " << adapted_bins_layer << std::endl;
			}
			std::cout << std::endl;
			std::cout << " Dummy TSVs after: " << fp.getDummyTSVs().size() << std::endl;

			// now we may generate/adapt the TSV densities in underlying power_maps
			//
			// note that this will also re-scale power_maps.power_density, so we should not access those values during this iteration any more
			//
			fp.editThermalAnalyzer().adaptPowerMapsTSVs(fp.getLayers(), fp.getTSVs(), fp.getDummyTSVs(), fp.getPowerBlurringParameters());

			// generate new HotSpot files for Si and bonding layers; adapted by TSVs
			writeHotSpotFiles__passiveSi_bonding(fp);

			// prepare next run
			//
			// restore original power values
			for (Block const& b : fp.getBlocks()) {

				b.power_density_unscaled = b.power_density_unscaled_back;
			}
			// generate new/original ptrace file
			writeHotSpotPtrace(fp);

			std::cout << std::endl;
			std::cout << "Continue with next sampling round" << std::endl;
			std::cout << "---------------------------------" << std::endl;
		}
	}

	std::cout << std::endl;
	std::cout << "No further improvements on all layers; revert last iteration" << std::endl;
	std::cout << "------------------------------------------------------------" << std::endl;
	std::cout << std::endl;

	// remove dummy TSVs added during previous iteration
	for (std::string const& id : dummy_TSVs_to_delete) {

		unsigned dummy_TSVs_count = fp.getDummyTSVs().size();

		for (unsigned i = 0; i < dummy_TSVs_count; i++) {

			// we may have reached the end of vector already, in case some islands have been already deleted
			if (i >= fp.getDummyTSVs().size()) {
				break;
			}
			// otherwise, check whether the ids match, and then delete the island
			else if (id == fp.getDummyTSVs()[i].id) {
				fp.editDummyTSVs().erase(fp.editDummyTSVs().begin() + i);
			}
		}
	}

	// restore original power values
	for (Block const& b : fp.getBlocks()) {

		b.power_density_unscaled = b.power_density_unscaled_back;
	}

	// generate original power maps
	//
	fp.editThermalAnalyzer().generatePowerMaps(fp.getLayers(), fp.getBlocks(), fp.getOutline(), fp.getPowerBlurringParameters());

	// generate/adapt the TSV densities, including only the original dummy TSVs
	//
	fp.editThermalAnalyzer().adaptPowerMapsTSVs(fp.getLayers(), fp.getTSVs(), original_dummy_TSVs, fp.getPowerBlurringParameters());

	// restore original HotSpot files
	IO::writeHotSpotFiles(fp);

	// final HotSpot.sh system call for original files
	system(std::string("./HotSpot.sh " + fp.getBenchmark() + " " + std::to_string(fp.getLayers())).c_str());

	// re-generate original power maps
	//
	fp.editThermalAnalyzer().generatePowerMaps(fp.getLayers(), fp.getBlocks(), fp.getOutline(), fp.getPowerBlurringParameters());

	// generate/adapt the TSV densities, now including dummy TSVs
	//
	fp.editThermalAnalyzer().adaptPowerMapsTSVs(fp.getLayers(), fp.getTSVs(), fp.getDummyTSVs(), fp.getPowerBlurringParameters());

	// write final HotSpot files, now with new dummy TSVs considered in underlying power_maps
	IO::writeHotSpotFiles(fp, "_postprocessed");

	// write final floorplan, with dummy TSVs
	IO::writeFloorplanGP(fp, corb.getAlignments(), "_postprocessed");

	// write gp template map for HotSpot data
	IO::writeMaps(fp, IO::MAPS_FLAGS::THERMAL_HOTSPOT, "_postprocessed");

	// final HotSpot.sh system call
	system(std::string("./HotSpot.sh " + fp.getBenchmark() + "_postprocessed " + std::to_string(fp.getLayers())).c_str());

	std::cout << std::endl;
	std::cout << "Log for previous iteration's final results" << std::endl;
	std::cout << "------------------------------------------" << std::endl;
	std::cout << std::endl;

	std::cout << "Overall number of adapted bins / dummy TSVs: " << fp.getDummyTSVs().size() << std::endl;
	std::cout << std::endl;

	// final log
	for (int layer = 0; layer < fp.getLayers(); layer++) {

		std::cout << "Avg Pearson correlations over all bins on layer " << layer << ": " << prev_correlation_avgs[layer] << std::endl;
		std::cout << "Max Pearson correlation over all bins on layer " << layer << ": " << prev_max_correlation_avgs[layer] << std::endl;
		std::cout << "Std dev of Pearson correlation over all bins on layer " << layer << ": " << prev_std_dev_correlation_avgs[layer] << std::endl;
	}

	// now, read in the final HotSpot simulation result
	//
	parseHotSpotFiles(fp, "_postprocessed", thermal_maps_HotSpot);


	std::cout << "Leakage metrics for final result" << std::endl;
	std::cout << "--------------------------------" << std::endl;
	std::cout << std::endl;

	// now, we may calculate the correlation and spatial entropy
	//
	for (int layer = 0; layer < fp.getLayers(); layer++) {

		corr = LeakageAnalyzer::determinePearsonCorr(fp.getThermalAnalyzer().getPowerMapsOrig()[layer], &thermal_maps_HotSpot[layer]);
		entropy = fp.editLeakageAnalyzer().determineSpatialEntropy(layer, fp.getThermalAnalyzer().getPowerMapsOrig()[layer]);

		std::cout << "Pearson correlation of (HotSpot) temp and power for layer " << layer << ": " << corr << std::endl;
		std::cout << std::endl;

		std::cout << "Spatial entropy of power map for layer " << layer << ": "	<< entropy << std::endl;
		std::cout << std::endl;
	}
}

void parseHotSpotFiles(FloorPlanner& fp, unsigned sampling_iter, samples_data_type& temp_samples) {
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
			temp_samples[layer][x][y][sampling_iter] = temp;

			// DBG output
			if (DBG_PARSING) {
				std::cout << "Temp for [layer= " << layer << "][x= " << x << "][y= " << y << "]: " << temp_samples[layer][x][y][sampling_iter] << std::endl;
				std::cout << "Power for [layer= " << layer << "][x= " << x << "][y= " << y << "]: " << fp.getThermalAnalyzer().getPowerMapsOrig()[layer][x][y].power_density << std::endl;
			}
		}

		// close file
		layer_file.close();
	}
}

// copied from IO::writeHotSpotFiles; adapter for getter on FloorPlanner
//
void writeHotSpotPtrace(FloorPlanner& fp) {
	std::ofstream file;
	int cur_layer;

	/// generate power-trace file
	//
	// build up file name
	std::stringstream power_file;
	power_file << fp.getBenchmark() << "_HotSpot.ptrace";

	// init file stream
	file.open(power_file.str().c_str());

	// block sequence in trace file has to follow layer files, thus build up file
	// according to layer structure
	//
	// output block labels in first line
	for (cur_layer = 0; cur_layer < fp.getLayers(); cur_layer++) {

		// output dummy blocks representing wires first, since they are placed in
		// the BEOL layer, coming before the active Si layer
		for (Block const& cur_wire : fp.getWires()) {

			if (cur_wire.layer != cur_layer) {
				continue;
			}

			file << cur_wire.id << " ";
		}

		// dummy BEOL outline block
		file << "BEOL_" << cur_layer + 1 << " ";

		// actual blocks
		for (Block const& cur_block : fp.getBlocks()) {

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
	for (cur_layer = 0; cur_layer < fp.getLayers(); cur_layer++) {

		// dummy blocks representing wires along with their power consumption
		for (Block const& cur_wire : fp.getWires()) {

			if (cur_wire.layer != cur_layer) {
				continue;
			}

			// actual power encoded in power_density_unscaled, see
			// ThermalAnalyzer::adaptPowerMapsWires
			file << cur_wire.power_density_unscaled << " ";
		}

		// dummy BEOL outline block
		file << "0.0 ";

		for (Block const& cur_block : fp.getBlocks()) {

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
}

// copied from IO::writeHotSpotFiles; adapter for getter on FloorPlanner
//
void writeHotSpotFiles__passiveSi_bonding(FloorPlanner& fp) {
	std::ofstream file, file_bond;
	int cur_layer;
	int x, y;
	int map_x, map_y;
	float x_ll, y_ll;
	float bin_w, bin_h;

	/// generate floorplans for passive Si and bonding layer; considering TSVs (modelled via densities)
	for (cur_layer = 0; cur_layer < fp.getLayers(); cur_layer++) {

		// build up file names
		std::stringstream Si_fp_file;
		Si_fp_file << fp.getBenchmark() << "_HotSpot_Si_passive_" << cur_layer + 1 << ".flp";
		std::stringstream bond_fp_file;
		bond_fp_file << fp.getBenchmark() << "_HotSpot_bond_" << cur_layer + 1 << ".flp";

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

		// walk power-map grid to obtain specific TSV densities of bins
		for (x = ThermalAnalyzer::POWER_MAPS_PADDED_BINS; x < ThermalAnalyzer::THERMAL_MAP_DIM + ThermalAnalyzer::POWER_MAPS_PADDED_BINS; x++) {

			// adapt index for final thermal map according to padding
			map_x = x - ThermalAnalyzer::POWER_MAPS_PADDED_BINS;

			// pre-calculate bin's lower-left corner coordinates;
			// float precision required to avoid grid coordinate
			// mismatches
			x_ll = static_cast<float>(map_x * fp.getThermalAnalyzer().power_maps_dim_x * Math::SCALE_UM_M);

			// pre-calculate bin dimensions; float precision required
			// to avoid grid coordinate mismatches; re-calculation
			// only required for lower and upper bounds
			//
			// lower bound, regular bin dimension; value also used
			// until reaching upper bound
			if (x == ThermalAnalyzer::POWER_MAPS_PADDED_BINS) {
				bin_w = static_cast<float>(fp.getThermalAnalyzer().power_maps_dim_x * Math::SCALE_UM_M);
			}
			// upper bound, limit bin dimension according to overall
			// chip outline; scale down slightly is required to avoid
			// rounding errors during HotSpot's grid mapping
			else if (x == (ThermalAnalyzer::THERMAL_MAP_DIM + ThermalAnalyzer::POWER_MAPS_PADDED_BINS - 1)) {
				bin_w = 0.999 * static_cast<float>(fp.getOutline().x * Math::SCALE_UM_M - x_ll);
			}

			for (y = ThermalAnalyzer::POWER_MAPS_PADDED_BINS; y < ThermalAnalyzer::THERMAL_MAP_DIM + ThermalAnalyzer::POWER_MAPS_PADDED_BINS; y++) {
				// adapt index for final thermal map according to padding
				map_y = y - ThermalAnalyzer::POWER_MAPS_PADDED_BINS;

				// pre-calculate bin's lower-left corner
				// coordinates; float precision required to avoid
				// grid coordinate mismatches
				y_ll = static_cast<float>(map_y * fp.getThermalAnalyzer().power_maps_dim_y * Math::SCALE_UM_M);

				// pre-calculate bin dimensions; float precision required
				// to avoid grid coordinate mismatches; re-calculation
				// only required for lower and upper bounds
				//
				// lower bound, regular bin dimension; value also used
				// until reaching upper bound
				if (y == ThermalAnalyzer::POWER_MAPS_PADDED_BINS) {
					bin_h = static_cast<float>(fp.getThermalAnalyzer().power_maps_dim_y * Math::SCALE_UM_M);
				}
				// upper bound, limit bin dimension according to
				// overall chip outline; scale down slightly is
				// required to avoid rounding errors during
				// HotSpot's grid mapping
				else if (y == (ThermalAnalyzer::THERMAL_MAP_DIM + ThermalAnalyzer::POWER_MAPS_PADDED_BINS - 1)) {
					bin_h = 0.999 * static_cast<float>(fp.getOutline().y * Math::SCALE_UM_M - y_ll);
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
				file << "	" << ThermalAnalyzer::heatCapSi(fp.getTechParameters().TSV_group_Cu_area_ratio, fp.getThermalAnalyzer().getPowerMaps()[cur_layer][x][y].TSV_density);
				file << "	" << ThermalAnalyzer::thermResSi(fp.getTechParameters().TSV_group_Cu_area_ratio, fp.getThermalAnalyzer().getPowerMaps()[cur_layer][x][y].TSV_density);
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
				file_bond << "	" << ThermalAnalyzer::heatCapBond(fp.getTechParameters().TSV_group_Cu_area_ratio, fp.getThermalAnalyzer().getPowerMaps()[cur_layer][x][y].TSV_density);
				file_bond << "	" << ThermalAnalyzer::thermResBond(fp.getTechParameters().TSV_group_Cu_area_ratio, fp.getThermalAnalyzer().getPowerMaps()[cur_layer][x][y].TSV_density);
				file_bond << std::endl;
			}
		}

		// close file streams
		file.close();
		file_bond.close();
	}
}

// copied and adapted from Variation_TSC
//
void parseHotSpotFiles(FloorPlanner& fp, std::string const& benchmark_suffix, thermal_maps_type& thermal_maps) {
	std::ifstream layer_file;
	int x, y;
	double temp;

	thermal_maps.clear();

	for (int layer = 0; layer < fp.getLayers(); layer++) {

		// file name
		std::stringstream layer_file_name;
		// HotSpot files for active Si layer; offsets defined accordingly to file generation in IO::writeMaps, IO::writeHotSpotFiles
		layer_file_name << fp.getBenchmark() << benchmark_suffix << "_HotSpot.steady.grid.gp_data.layer_" << (1 + 4 * layer);

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
			thermal_maps[layer][x][y].temp = temp;

			// DBG output
			if (DBG) {
				std::cout << "Temp for [layer= " << layer << "][x= " << x << "][y= " << y << "]: " << thermal_maps[layer][x][y].temp << std::endl;
				std::cout << "Power for [layer= " << layer << "][x= " << x << "][y= " << y << "]: " << fp.getThermalAnalyzer().getPowerMapsOrig()[layer][x][y].power_density << std::endl;
			}
		}

		// close file
		layer_file.close();
	}
}
