/*
 * =====================================================================================
 *
 *    Description:  Corblivar thermal analyzer, based on power blurring
 *
 *         Author:  Johann Knechtel, johann.knechtel@ifte.de
 *        Company:  Institute of Electromechanical and Electronic Design, www.ifte.de
 *
 * =====================================================================================
 */
#ifndef _CORBLIVAR_THERMALANALYZER
#define _CORBLIVAR_THERMALANALYZER

// library includes
#include "Corblivar.incl.hpp"
// Corblivar includes, if any
// forward declarations, if any
class Block;

class ThermalAnalyzer {
	private:
		// debugging code switches
		static constexpr bool DBG_CALLS = false;
		static constexpr bool DBG = false;

	private:
		/// material parameters for thermal 3D-IC simulation using HotSpot
		/// Note: properties for heat spread and heat sink also from [Park09] (equal default
		/// HotSpot configuration values)
		// [Park09]; derived from 700 J/(kg*K) to J/(m^3*K) considering Si density of 2330 kg/m^3
		static constexpr double HEAT_CAPACITY_SI = 1631000.0;
		// [Park09]
		static constexpr double THERMAL_RESISTIVITY_SI = 0.008510638;
		// [Sridhar10]; derived considering a factor of appr. 1.35 for Si/BEOL heat capacity
		static constexpr double HEAT_CAPACITY_BEOL = 1208150.0;
		// [Sridhar10]
		static constexpr double THERMAL_RESISTIVITY_BEOL = 0.4444;
		// [Park09]
		static constexpr double HEAT_CAPACITY_BOND = 2298537.0;
		// [Park09]
		static constexpr double THERMAL_RESISTIVITY_BOND = 5.0;
		// 2um active Si layer; [Sridhar10]
		static constexpr double THICKNESS_SI_ACTIVE = 0.000002;
		// 100um thick dies; own value
		static constexpr double THICKNESS_SI = 0.0001 - THICKNESS_SI_ACTIVE;
		// 12um BEOL; [Sridhar10]
		static constexpr double THICKNESS_BEOL = 0.000012;
		// 20um BCB bond; [Sridhar10]
		static constexpr double THICKNESS_BOND = 0.00002;

		// thermal modeling: dimensions
		// represents the thermal map's dimension
		static constexpr int thermal_map_dim = 64;
		// represents the thermal mask's dimension (i.e., the 2D gauss function
		// representing the thermal impulse response);
		// note that value should be uneven!
		static constexpr int mask_dim = 7;
		// represents the center index of the center originated mask
		static constexpr int mask_center = mask_dim / 2;
		// represents the amount of padded bins at power maps' boundaries
		static constexpr int mask_dim_half = mask_center;
		// represents the power maps' dimension
		// (note that maps are padded at the boundaries according to mask
		// dim in order to handle boundary values for convolution)
		static constexpr int power_maps_dim = thermal_map_dim + (mask_dim - 1);

		// thermal modeling: vector masks and maps
		// mask[i][x/y], whereas mask[0] relates to the mask for layer 0 obtained
		// by considering heat source in layer 0, mask[1] relates to the mask for
		// layer 0 obtained by considering heat source in layer 1 and so forth.
		// Note that the masks are only 1D for the separated convolution
		vector< array<double,mask_dim> > thermal_masks;
		// map[i][x][y], whereas map[0] relates to the map for layer 0 and so forth.
		mutable vector< array<array<double,power_maps_dim>,power_maps_dim> > power_maps;
		// thermal map for layer 0 (lowest layer), i.e., hottest layer
		mutable array<array<double,thermal_map_dim>,thermal_map_dim> thermal_map;

		// thermal modeling: parameters for generating power maps
		double power_maps_dim_x, power_maps_dim_y;
		double power_maps_bin_area;
		double offset_x, offset_y;
		double padding_right_boundary_blocks_distance, padding_upper_boundary_blocks_distance;
		array<double,power_maps_dim> power_maps_bins_ll_x, power_maps_bins_ll_y;

	public:
		friend class IO;

		// POD
		struct MaskParameters {
			double mask_boundary_value;
			double impulse_factor;
			double impulse_factor_scaling_exponent;
		};

		// thermal modeling: handlers
		void initThermalMasks(int const& layers, bool const& log, MaskParameters const& parameters);
		void initPowerMaps(int const& layers, double const& outline_x, double const& outline_y);
		void generatePowerMaps(int const& layers, map<int, Block*> const& blocks, double const& outline_x, double const& outline_y, bool const& extend_boundary_blocks_into_padding_zone = true) const;
		// thermal-analyzer routine based on power blurring,
		// i.e., convolution of thermals masks and power maps
		// returns max value of convoluted 2D matrix
		double performPowerBlurring(int const& layers, double& max_cost_temp, bool const& set_max_cost = false, bool const& normalize = true) const;
};

#endif
