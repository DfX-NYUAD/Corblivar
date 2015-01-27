/*
 * =====================================================================================
 *
 *    Description:  Corblivar routing-congestion analyzer
 *
 *    Copyright (C) 2015 Johann Knechtel, johann.knechtel@ifte.de, www.ifte.de
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
#ifndef _CORBLIVAR_ROUTINGCONGESTION
#define _CORBLIVAR_ROUTINGCONGESTION

// library includes
#include "Corblivar.incl.hpp"
// Corblivar includes, if any
#include "Block.hpp"
// forward declarations, if any
class Point;

class RoutingCongestion {
	// debugging code switch (private)
	private:
		static constexpr bool DBG = false;
		static constexpr bool DBG_CALLS = false;

	// public data
	public:

		// dimensions for routing-congestion map
		static constexpr int CONG_MAPS_DIM = 64;

	// PODs, to be declared early on
	public:
		struct CongBin {
			double utilization;
		};
		struct CongestionResult {
			double cost_utilization;
			double max_utilization;
			double std_dev_utilization;
			std::vector< std::array< std::array<CongBin, CONG_MAPS_DIM>, CONG_MAPS_DIM> > *cong_maps = nullptr;
		};

	// private data, functions
	private:

		// congestion maps [i][x][y] whereas i relates to the layer
		std::vector< std::array< std::array<CongBin, CONG_MAPS_DIM>, CONG_MAPS_DIM> > cong_maps;

		// parameters for generating congestion maps
		double cong_maps_dim_x, cong_maps_dim_y;
		double cong_maps_bin_area;
		std::array<double, CONG_MAPS_DIM + 1> cong_maps_bins_ll_x, cong_maps_bins_ll_y;


	// constructors, destructors, if any non-implicit
	public:

	// public data, functions
	public:
		friend class IO;

		// congestion analysis: handlers
		void initCongMaps(int const& layers, Point const& die_outline);
		void resetCongMaps(int const& layers);
		void adaptCongMap(int const& layer, Rect const& net_bb, double const& net_weight = 1.0);
};

#endif
