/**
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
#ifndef _CORBLIVAR_LEAKAGEANALYZER
#define _CORBLIVAR_LEAKAGEANALYZER

// library includes
#include "Corblivar.incl.hpp"
// Corblivar includes, if any
#include "ThermalAnalyzer.hpp"
// forward declarations, if any

/// Corblivar thermal-related leakage analyzer
class LeakageAnalyzer {
	private:
		/// debugging code switch (private)
		static constexpr bool DBG = false;
		/// debugging code switch (private)
		static constexpr bool DBG_BASIC = false;
		/// debugging code switch (private)
		static constexpr bool DBG_VERBOSE = false;
		/// debugging code switch (private)
		static constexpr bool DBG_GP = false;

		/// minimal size of partitions, equal to 1% of power-map size/bins
		static constexpr int MIN_PARTITION_SIZE = (ThermalAnalyzer::THERMAL_MAP_DIM * ThermalAnalyzer::THERMAL_MAP_DIM) / 100;

	// public data
	public:
		struct Parameters {
			/// internal weights, used for internal cost terms
			double weight_entropy;
			/// internal weights, used for internal cost terms
			double weight_correlation;
		} parameters;

		/// max evaluation values have to memorized as well, in order to enable
		/// comparison during different SA iterations
		struct max_values {
			double entropy;
			double correlation;
		} max_values;

	// PODs, to be declared early on
	public:
		struct Bin {
			unsigned x;
			unsigned y;
			double value;
		};

	// private data, functions
	private:
		/// power partitions; outer vector: layers; middle vector: partitions (of layer), inner pair: id and vector of coordinates/indices of bins (of partition), related
		/// to indices of ThermalAnalyzer::power_maps_orig
		std::vector< std::vector< std::pair<std::string, std::vector<Bin>> > > power_partitions;

		/// sum of Manhattan distances from each array bin to all other bins; used for calculation of spatial entropy
		std::array< std::array<int, ThermalAnalyzer::THERMAL_MAP_DIM>, ThermalAnalyzer::THERMAL_MAP_DIM> distances_summed;

		/// Manhattan distances, in 1D, for one bin to another bin; used for calculation of spatial entropy
		std::array< std::array<unsigned, ThermalAnalyzer::THERMAL_MAP_DIM>, ThermalAnalyzer::THERMAL_MAP_DIM> distances;

		/// nested-means based partitioning of power maps
		///
		/// the values of power maps are sorted in a 1D data structure and then ``natural'' breaks are determined by
		/// recursively bi-partitioning these values, where the mean is the boundary; the partitioning stops once the min/max values are
		void partitionPowerMap(int const& layer,
				std::array< std::array<ThermalAnalyzer::PowerMapBin, ThermalAnalyzer::THERMAL_MAP_DIM>, ThermalAnalyzer::THERMAL_MAP_DIM> const& power_map);

		/// helper for recursive calls for partitioning of power maps
		///
		/// note that the upper bound is excluded
		inline void partitionPowerMapHelper(int const& layer, unsigned const& lower_bound, unsigned const& upper_bound, std::vector<Bin> const& power_values);

		/// helper to init distance arrays, which are used as look-up tables for spatial entropy
		inline void initDistances() {
			int dist;

			// sum of distances for one bin in 2D array to all other bins in same 2D array
			//
			for (int x = 0; x < ThermalAnalyzer::THERMAL_MAP_DIM; x++) {
				for (int y = 0; y < ThermalAnalyzer::THERMAL_MAP_DIM; y++) {

					// for each bin, calculate the sum of distances to all other bins
					dist = 0;

					for (int i = 0; i < ThermalAnalyzer::THERMAL_MAP_DIM; i++) {
						for (int j = 0; j < ThermalAnalyzer::THERMAL_MAP_DIM; j++) {

							// Manhattan distance should suffice for grid coordinates/distances
							//
							dist += std::abs(x - i) + std::abs(y - j);
						}
					}

					this->distances_summed[x][y] = dist;
				}
			}

			// all distances for one bin in 1D array to all other bins in same 1D array
			//
			for (unsigned x = 0; x < ThermalAnalyzer::THERMAL_MAP_DIM; x++) {

				// no need to walk ranges [x][0--x]; they will be covered by symmetric counterparts below; also, by initializing y = x, the calculation of y - x is
				// always valid on unsigned
				for (unsigned y = x; y < ThermalAnalyzer::THERMAL_MAP_DIM; y++) {

					// Manhattan distance should suffice for grid coordinates/distances
					//
					this->distances[x][y] = y - x;
					// symmetric counterparts
					this->distances[y][x] = this->distances[x][y];
				}
			}
		}

	// constructors, destructors, if any non-implicit
	public:
		/// default constructor
		LeakageAnalyzer() {
			this->initDistances();
		}

	// public data, functions
	public:
		friend class IO;
	
		/// Pearson correlation of power and thermal map
		static double determinePearsonCorr(
				std::array< std::array<ThermalAnalyzer::PowerMapBin, ThermalAnalyzer::THERMAL_MAP_DIM>, ThermalAnalyzer::THERMAL_MAP_DIM> const& power_map,
				std::array< std::array<ThermalAnalyzer::ThermalMapBin, ThermalAnalyzer::THERMAL_MAP_DIM>, ThermalAnalyzer::THERMAL_MAP_DIM> const* thermal_map
			);
		
		/// Spatial entropy of original power map, as proposed by Claramunt
		double determineSpatialEntropy(int const& layer,
				std::array< std::array<ThermalAnalyzer::PowerMapBin, ThermalAnalyzer::THERMAL_MAP_DIM>, ThermalAnalyzer::THERMAL_MAP_DIM> const& power_map
			);
};

#endif
