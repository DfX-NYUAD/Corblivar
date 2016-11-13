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

		/// table of Manhattan distances b/w array indices; used for calculation of spatial entropy; in double already here to avoid casting during read
		std::array< std::array<double, ThermalAnalyzer::THERMAL_MAP_DIM>, ThermalAnalyzer::THERMAL_MAP_DIM> distances;

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

		/// helper to init distance matrix (used as look-up table for spatial entropy)
		inline void initDistances() {

			for (unsigned i = 0; i < ThermalAnalyzer::THERMAL_MAP_DIM; i++) {

				// j shall not be less than i
				//
				// this way the result of j - i is always positive; also, there is no need to calculate values in the range of [i][0--j], since they are already
				// calculated as counterparts below
				//
				for (unsigned j = i; j < ThermalAnalyzer::THERMAL_MAP_DIM; j++) {

					this->distances[i][j] = j - i;
					// assign symmetric counterpart here as well
					this->distances[j][i] = this->distances[i][j];
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
