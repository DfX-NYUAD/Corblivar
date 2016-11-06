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
class Point;

/// Corblivar thermal-related leakage analyzer
class LeakageAnalyzer {
	private:
		/// debugging code switch (private)
		static constexpr bool DBG = true;
		/// debugging code switch (private)
		static constexpr bool DBG_VERBOSE = false;

	// public data
	public:
		// TODO data structure for actual partitions, encapsulated per layer

	// PODs, to be declared early on
	public:
		//struct MaskParameters {
		//	double TSV_density;
		//	double mask_boundary_value;
		//	double impulse_factor;
		//	double impulse_factor_scaling_exponent;
		//	double power_density_scaling_padding_zone;
		//	double power_density_scaling_TSV_region;
		//	double temp_offset;
		//};

	// private data, functions
	private:
		/// internal data used for partitioning, only for one layer at a time
		/// Point encodes the x,y indices of the corresponding power-map bin
		std::vector< std::pair<double, Point> > power_values;
		/// contains the ranges of determined partitions, with lower and upper value corresponding to indices of power_values
		std::vector< std::pair<unsigned, unsigned> > partition_ranges;

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
		inline void partitionPowerMapHelper(unsigned lower_bound, unsigned upper_bound);

	// constructors, destructors, if any non-implicit
	public:

	// public data, functions
	public:
		friend class IO;
	
		/// Pearson correlation of power and thermal maps
		// TODO implement here as well, for power-blurring estimates
		//void calculatePearsonCorr(FloorPlanner& fp, thermal_maps_type& thermal_maps);
		
		/// spatial entropy of original power maps
		// TODO actual implementation in cpp
		void determineSpatialEntropies(int const& layers,
				std::vector< std::array< std::array<ThermalAnalyzer::PowerMapBin, ThermalAnalyzer::THERMAL_MAP_DIM>, ThermalAnalyzer::THERMAL_MAP_DIM> > const& power_maps_orig
			) {
			this->partitionPowerMaps(layers, power_maps_orig);
		}
};

#endif
