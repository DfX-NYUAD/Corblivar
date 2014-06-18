/*
 * =====================================================================================
 *
 *    Description:  Corblivar thermal analyzer, based on power blurring
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
#ifndef _CORBLIVAR_THERMALANALYZER
#define _CORBLIVAR_THERMALANALYZER

// library includes
#include "Corblivar.incl.hpp"
// Corblivar includes, if any
#include "Block.hpp"
// forward declarations, if any
class Point;
class Net;
class CorblivarAlignmentReq;

class ThermalAnalyzer {
	// debugging code switch (private)
	private:
		static constexpr bool DBG_CALLS = false;
		static constexpr bool DBG = false;
		static constexpr bool DBG_INSANE = false;

	// PODs, to be declared early on
	public:
		struct MaskParameters {
			double TSV_density;
			double mask_boundary_value;
			double impulse_factor;
			double impulse_factor_scaling_exponent;
			double power_density_scaling_padding_zone;
			double power_density_scaling_TSV_region;
			double temp_offset;
		};
		struct PowerMapBin {
			double power_density;
			double TSV_density;
		};
		struct Temp {
			double cost_temp;
			double max_temp;
		};

	// private data, functions
	private:

		// thermal modeling: dimensions
		// represents the thermal map's dimension
		static constexpr int THERMAL_MAP_DIM = 64;
		// represents the thermal mask's dimension (i.e., the 2D gauss function
		// representing the thermal impulse response);
		// note that value should be uneven!
		static constexpr int THERMAL_MASK_DIM = 11;
		// represents the center index of the center originated mask; int division
		// discards remainder, i.e., is equal to floor() for positive int
		static constexpr int THERMAL_MASK_CENTER = THERMAL_MASK_DIM / 2;
		// represents the amount of padded bins at power maps' boundaries
		static constexpr int POWER_MAPS_PADDED_BINS = THERMAL_MASK_CENTER;
		// represents the power maps' dimension
		// (note that maps are padded at the boundaries according to mask
		// dim in order to handle boundary values for convolution)
		static constexpr int POWER_MAPS_DIM = THERMAL_MAP_DIM + (THERMAL_MASK_DIM - 1);

		// thermal modeling: thermal masks and maps
		// thermal_masks[i][x/y], whereas thermal_masks[0] relates to the mask for
		// layer 0 obtained by considering heat source in layer 0,
		// thermal_masks[1] relates to the mask for layer 0 obtained by
		// considering heat source in layer 1 and so forth.  Note that the masks
		// are only 1D for the separated convolution.
		vector< array<double,THERMAL_MASK_DIM> > thermal_masks;
		// power_maps[i][x][y], whereas power_maps[0] relates to the map for layer
		// 0 and so forth.
		vector< array<array<PowerMapBin, POWER_MAPS_DIM>, POWER_MAPS_DIM> > power_maps;
		// thermal map for layer 0 (lowest layer), i.e., hottest layer
		array<array<double,THERMAL_MAP_DIM>,THERMAL_MAP_DIM> thermal_map;

		// thermal modeling: parameters for generating power maps
		double power_maps_dim_x, power_maps_dim_y;
		double power_maps_bin_area;
		double blocks_offset_x, blocks_offset_y;
		double padding_right_boundary_blocks_distance, padding_upper_boundary_blocks_distance;
		array<double, POWER_MAPS_DIM + 1> power_maps_bins_ll_x, power_maps_bins_ll_y;
		static constexpr double PADDING_ZONE_BLOCKS_DISTANCE_LIMIT = 0.01;
		/// material parameters for thermal 3D-IC simulation using HotSpot
		/// Note: properties for heat spread and heat sink also from [Park09] (equal default
		/// HotSpot configuration values)
		// [Park09]; derived from 700 J/(kg*K) to J/(m^3*K) considering Si density of 2330 kg/m^3
		static constexpr double HEAT_CAPACITY_SI = 1.631e06;
		// [Park09]
		static constexpr double THERMAL_RESISTIVITY_SI = 8.510638298e-03;
		// [Sridhar10]; derived considering a factor of appr. 1.35 for Si/BEOL heat capacity
		static constexpr double HEAT_CAPACITY_BEOL = HEAT_CAPACITY_SI / 1.35;
		// [Sridhar10]
		static constexpr double THERMAL_RESISTIVITY_BEOL = 0.4444;
		// [Park09]; BCB polymer
		static constexpr double HEAT_CAPACITY_BOND = 2.298537e06;
		// [Park09]; BCB polymer
		static constexpr double THERMAL_RESISTIVITY_BOND = 5.0;
		// [Park09]
		static constexpr double HEAT_CAPACITY_CU = 3.546401e06;
		// [Park09]
		static constexpr double THERMAL_RESISTIVITY_CU = 2.53164557e-03;
		// [Park09]
		static constexpr double DENSITY_SI = 2330.0;
		// [Park09]
		static constexpr double DENSITY_BOND = 1051.0;
		// [Park09]
		static constexpr double DENSITY_CU = 8933.0;

		// TSV-groups-related parameters; derived from values above and
		// considering TSV dimensions
		//
		// heat capacity of TSV-group-Si compound: mass-weighted mean;  C_group =
		// m_Cu / m_group * C_Cu + (1 - m_Cu / m_group) * C_Si = 1 / (1 + (p_Si *
		// A_Si) / (p_Cu * A_Cu)) * C_Cu + 1 / (1 + (p_Cu * A_Cu) / (p_Si * A_Si))
		// * C_si
		// http://answers.yahoo.com/question/index?qid=20110804231514AApjFkc ,
		// http://www.google.de/url?sa=t&rct=j&q=&esrc=s&source=web&cd=1&cad=rja&ved=0CCwQFjAA&url=http%3A%2F%2Fpubs.acs.org%2Fdoi%2Fabs%2F10.1021%2Fma902122u&ei=6JXuUfD-K9GKswaPzIGgDw&usg=AFQjCNFX7TTz6SQ_ZlLkt5nwGcLh-abdzQ&sig2=Jd7U_ZTSDs_7KYWTmXaA7g
		//
		// note that TSV_density is to be passed as percent
		inline static double heatCapSi(double const& TSV_group_Cu_Si_ratio, double const& TSV_density) {
			if (TSV_density == 0.0) {
				return HEAT_CAPACITY_SI;
			}
			else {
				return
					HEAT_CAPACITY_CU / (1.0 + ( (DENSITY_SI / DENSITY_CU) / ((TSV_density * 0.01) * TSV_group_Cu_Si_ratio) ) ) +
					HEAT_CAPACITY_SI / (1.0 + ( (DENSITY_CU / DENSITY_SI) * ((TSV_density * 0.01) * TSV_group_Cu_Si_ratio) ) );
			}
		};
		// similar for Bond scenario; TSV group's area-ratio for Cu-Si applies to Cu-Bond as well
		//
		// note that TSV_density is to be passed as percent
		inline static double heatCapBond(double const& TSV_group_Cu_Si_ratio, double const& TSV_density) {
			if (TSV_density == 0.0) {
				return HEAT_CAPACITY_BOND;
			}
			else {
				return
					HEAT_CAPACITY_CU / (1.0 + ( (DENSITY_BOND / DENSITY_CU) / ((TSV_density * 0.01) * TSV_group_Cu_Si_ratio) ) ) +
					HEAT_CAPACITY_BOND / (1.0 + ( (DENSITY_CU / DENSITY_BOND) * ((TSV_density * 0.01) * TSV_group_Cu_Si_ratio) ) );
			}
		}
		//
		// thermal resistivity of compounds, to be derived as parallel joint
		// resistance of Si, Cu where TSV density impacts the area fractions
		//
		// note that TSV_density is to be passed as percent
		inline static double thermResSi(double const& TSV_group_Cu_area_ratio, double const& TSV_density) {
			if (TSV_density == 0.0) {
				return THERMAL_RESISTIVITY_SI;
			}
			else {
				return 1.0 /
					(
					 (((TSV_density * 0.01) * TSV_group_Cu_area_ratio) / THERMAL_RESISTIVITY_CU) +
					 ((1.0 - (TSV_density * 0.01) * TSV_group_Cu_area_ratio) / THERMAL_RESISTIVITY_SI)
					);
			}
		}
		//
		// thermal resistivity of compounds, to be derived as parallel joint
		// resistance of Bond, Cu where TSV density impacts the area fractions
		//
		// note that TSV_density is to be passed as percent
		inline static double thermResBond(double const& TSV_group_Cu_area_ratio, double const& TSV_density) {
			if (TSV_density == 0.0) {
				return THERMAL_RESISTIVITY_BOND;
			}
			else {
				return 1.0 /
					(
					 (((TSV_density * 0.01) * TSV_group_Cu_area_ratio) / THERMAL_RESISTIVITY_CU) +
					 ((1.0 - (TSV_density * 0.01) * TSV_group_Cu_area_ratio) / THERMAL_RESISTIVITY_BOND)
					);
			}
		}

	// constructors, destructors, if any non-implicit
	public:

	// public data, functions
	public:
		friend class IO;

		// thermal modeling: handlers
		void initThermalMasks(int const& layers, bool const& log, MaskParameters const& parameters);
		void initPowerMaps(int const& layers, Point const& die_outline);
		void generatePowerMaps(int const& layers, vector<Block> const& blocks, Point const& die_outline, MaskParameters const& parameters, bool const& extend_boundary_blocks_into_padding_zone = true);
		void adaptPowerMaps(int const& layers, vector<TSV_Group> const& TSVs, vector<Net> const& nets, double const& TSV_pitch, MaskParameters const& parameters);
		// thermal-analyzer routine based on power blurring,
		// i.e., convolution of thermals masks and power maps
		void performPowerBlurring(Temp& ret, int const& layers, MaskParameters const& parameters);
};

#endif
