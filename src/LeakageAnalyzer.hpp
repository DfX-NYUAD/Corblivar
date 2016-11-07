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
			int x;
			int y;
			double value;
		};

	// private data, functions
	private:
		/// power partitions; outer vector: layers, middle vector: partitions (of layer), inner vector: coordinates/indices of bins (of partition), related to indices of
		/// ThermalAnalyzer::power_maps_orig
		std::vector< std::vector< std::vector<Bin> > > power_partitions;

		/// nested-means based partitioning of power maps
		///
		/// the values of power maps are sorted in a 1D data structure and then ``natural'' breaks are determined by
		/// recursively bi-partitioning these values, where the mean is the boundary; the partitioning stops once the min/max values are
		void partitionPowerMaps(int const& layers,
				std::vector< std::array< std::array<ThermalAnalyzer::PowerMapBin, ThermalAnalyzer::THERMAL_MAP_DIM>, ThermalAnalyzer::THERMAL_MAP_DIM> > const& power_maps_orig
			);

		/// helper for recursive calls for partitioning of power maps
		///
		/// note that the upper bound is excluded
		inline void partitionPowerMapHelper(unsigned const& lower_bound, unsigned const& upper_bound, int const& layer, std::vector<Bin> const& power_values);

	// constructors, destructors, if any non-implicit
	public:

	// public data, functions
	public:
		friend class IO;
	
		/// Pearson correlation of power and thermal maps
		// TODO implement here as well, for power-blurring estimates
		//void calculatePearsonCorr(FloorPlanner& fp, thermal_maps_type& thermal_maps);
		
		/// spatial entropy of original power maps
		void determineSpatialEntropies(int const& layers,
				std::vector< std::array< std::array<ThermalAnalyzer::PowerMapBin, ThermalAnalyzer::THERMAL_MAP_DIM>, ThermalAnalyzer::THERMAL_MAP_DIM> > const& power_maps_orig
			);
};

#endif
