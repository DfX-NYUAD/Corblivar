/*
 * =====================================================================================
 *
 *    Description:  Corblivar thermal-related side-channel leakage analyzer, based on
 *    Pearson correlation of power and thermal maps and spatial entropy of power maps
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
#include "LeakageAnalyzer.hpp"
// required Corblivar headers
#include "ThermalAnalyzer.hpp"

void LeakageAnalyzer::determineSpatialEntropies(int const& layers,
		std::vector< std::array< std::array<ThermalAnalyzer::PowerMapBin, ThermalAnalyzer::THERMAL_MAP_DIM>, ThermalAnalyzer::THERMAL_MAP_DIM> > const& power_maps_orig) {

	// first, the power maps have to be partitioned/classified
	//
	this->partitionPowerMaps(layers, power_maps_orig);
}

void LeakageAnalyzer::partitionPowerMaps(int const& layers,
		std::vector< std::array< std::array<ThermalAnalyzer::PowerMapBin, ThermalAnalyzer::THERMAL_MAP_DIM>, ThermalAnalyzer::THERMAL_MAP_DIM> > const& power_maps_orig) {
	double power_avg;
	double power_std_dev;
	unsigned m;
	// layer-wise power values
	std::vector<Bin> power_values;

	this->power_partitions.clear();

	for (int layer = 0; layer < layers; layer++) {

		// allocate empty vector
		this->power_partitions.push_back(std::vector< std::vector<Bin> >());

		// put power values along with their coordinates into vector; also track avg power
		//
		power_values.clear();
		power_values.reserve(std::pow(ThermalAnalyzer::THERMAL_MAP_DIM, 2));
		power_avg = 0.0;
		for (int x = 0; x < ThermalAnalyzer::THERMAL_MAP_DIM; x++) {
			for (int y = 0; y < ThermalAnalyzer::THERMAL_MAP_DIM; y++) {

				power_values.push_back({
						x, y,
						power_maps_orig[layer][x][y].power_density
					});

				power_avg += power_values.back().value;
			}
		}
		power_avg /= std::pow(ThermalAnalyzer::THERMAL_MAP_DIM, 2);

		// sort vector according to power values
		std::sort(power_values.begin(), power_values.end(),
				// lambda expression
				[&](Bin b1, Bin b2) {
					return b1.value < b2.value;
				}
			 );

		if (DBG_VERBOSE) {
			for (auto const& p : power_values) {
				std::cout << "DBG>  Power[" << p.x << "][" << p.y << "]: " << p.value << std::endl;
			}
		}

		// determine first cut: index of first value larger than avg
		for (m = 0; m < power_values.size(); m++) {
			
			if (power_values[m].value > power_avg) {
				break;
			}
		}

		// start recursive calls; partition these two ranges iteratively further
		//
		// note that upper-boundary element is left out for actual calculations, but required as upper boundary for traversal of data structures
		this->partitionPowerMapHelper(0, m, layer, power_values);
		this->partitionPowerMapHelper(m, power_values.size(), layer, power_values);

		// now, all partitions along with their power bins are determined and stored in power_partitions
		//


		// dbg logging
		if (DBG) {
			std::cout << "DBG> Partitions on layer " << layer << ": " << this->power_partitions[layer].size() << std::endl;

			for (auto const& cur_part : this->power_partitions[layer]) {

				// determine avg power for current partition
				power_avg = 0.0;
				for (auto const& bin : cur_part) {
					power_avg += bin.value;
				}
				power_avg /= cur_part.size();

				// determine sum of squared diffs for std dev
				power_std_dev = 0.0;
				for (auto const& bin : cur_part) {
					power_std_dev += std::pow(bin.value - power_avg, 2.0);
				}
				// determine std dev
				power_std_dev /= cur_part.size();
				power_std_dev = std::sqrt(power_std_dev);
				
				std::cout << "DBG>  Partition" << std::endl;
				std::cout << "DBG>   Size: " << cur_part.size() << std::endl;
				std::cout << "DBG>   Std dev power: " << power_std_dev << std::endl;
				std::cout << "DBG>   Avg power: " << power_avg << std::endl;
				// min value is represented by first bin, since the underlying data of power_values was sorted by power
				std::cout << "DBG>   Min power: " << cur_part.front().value << std::endl;
				// max value is represented by last bin, since the underlying data of power_values was sorted by power
				std::cout << "DBG>   Max power: " << cur_part.back().value << std::endl;

				if (DBG_VERBOSE) {
					for (auto const& bin : cur_part) {
						std::cout << "DBG>   Power[" << bin.x << "][" << bin.y << "]: " << bin.value << std::endl;
					}
				}
			}
		}
	}
}

/// note that power_partitions are updated in this function
inline void LeakageAnalyzer::partitionPowerMapHelper(unsigned const& lower_bound, unsigned const& upper_bound, int const& layer, std::vector<Bin> const& power_values) {
	double avg, std_dev;
	unsigned range;
	unsigned m, i;

	// sanity check for proper ranges
	if (upper_bound <= lower_bound) {
		return;
	}

	range = upper_bound - lower_bound;

	// determine avg power for given data range
	avg = 0.0;
	for (i = lower_bound; i < upper_bound; i++) {
		avg += power_values[i].value;
	}
	avg /= range;

	// determine sum of squared diffs for std dev
	std_dev = 0.0;
	for (i = lower_bound; i < upper_bound; i++) {
		std_dev += std::pow(power_values[i].value - avg, 2.0);
	}
	// determine std dev
	std_dev /= range;
	std_dev = std::sqrt(std_dev);
	
	if (DBG_VERBOSE) {
		std::cout << "DBG> Current range: " << lower_bound << ", " << upper_bound - 1 << std::endl;
		std::cout << "DBG>  Std dev: " << std_dev << std::endl;
		std::cout << "DBG>  Avg: " << avg << std::endl;
		std::cout << "DBG>  Min: " << power_values[lower_bound].value << std::endl;
		std::cout << "DBG>  Max: " << power_values[upper_bound - 1].value << std::endl;
	}

	// determine (potential) cut: index of first value larger than avg
	for (m = lower_bound; m < upper_bound; m++) {
		
		if (power_values[m].value > avg) {
			break;
		}
	}

	// check break criterion for recursive partitioning;
	//
	if (
			// ideal case: std dev of this partition is reaching zero
			Math::doubleComp(0.0, std_dev) ||
			// also maintain (a ``soft'', see below) minimal partition size
			//
			// note that some partitions may be much smaller in case their previous cut was largely skewed towards one boundary; a possible countermeasure here would be
			// to implement the check as look-ahead, but this also triggers some partitions to have a rather large leakage in practice
			(range < MIN_PARTITION_SIZE) ||
			// look-ahead checks still required, to avoid trivial sub-partitions with only one element
			((m - lower_bound) == 1) ||
			((upper_bound - m) == 1)
	   ) {

		// if criterion reached, then memorize this current partition as new partition
		//
		std::vector<Bin> partition;
		for (i = lower_bound; i < upper_bound; i++) {
			partition.push_back(power_values[i]);
		}
		this->power_partitions[layer].push_back(partition);
		
		return;
	}
	// continue recursively
	//
	else {

		// recursive call for the two new sub-partitions
		// note that upper-boundary element is left out for actual calculations, but required as upper boundary for traversal of data structures
		this->partitionPowerMapHelper(lower_bound, m, layer, power_values);
		this->partitionPowerMapHelper(m, upper_bound, layer, power_values);
	}
}
