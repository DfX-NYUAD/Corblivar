/**
 * =====================================================================================
 *
 *    Description:  Corblivar routing-utilization analyzer
 *
 *    Copyright (C) 2015-2016 Johann Knechtel, johann aett jknechtel dot de
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
#ifndef _CORBLIVAR_ROUTING_UTIL
#define _CORBLIVAR_ROUTING_UTIL

// library includes
#include "Corblivar.incl.hpp"
// Corblivar includes, if any
// forward declarations, if any
class Point;
class Rect;

/// Corblivar routing-utilization analyzer
class RoutingUtilization {
	private:
		/// debugging code switch (private)
		static constexpr bool DBG = false;
		/// debugging code switch (private)
		static constexpr bool DBG_CALLS = false;

	// public data
	public:

		/// dimensions for routing-utilization map
		static constexpr unsigned UTIL_MAPS_DIM = 64;

	// PODs, to be declared early on
	public:
		/// POD for bin
		struct UtilBin {
			double utilization;
		};
		/// POD for overall result
		struct UtilResult {
			double cost;
			double avg_util;
			double max_util;
		};

	// private data, functions
	private:

		/// utilization maps [i][x][y] whereas i relates to the layer
		std::vector< std::array< std::array<UtilBin, UTIL_MAPS_DIM>, UTIL_MAPS_DIM> > util_maps;

		/// parameters for generating utilization maps
		double util_maps_dim_x, util_maps_dim_y;
		/// parameters for generating utilization maps
		double util_maps_bin_area;
		/// helper variables for generating utilization maps
		std::array<double, UTIL_MAPS_DIM + 1> util_maps_bins_ll_x, util_maps_bins_ll_y;


	// constructors, destructors, if any non-implicit
	public:

	// public data, functions
	public:
		friend class IO;

		/// utilization analysis: handlers
		void initUtilMaps(int const& layers, Point const& die_outline);
		/// utilization analysis: handlers
		void resetUtilMaps(int const& layers);
		/// utilization analysis: handlers
		void adaptUtilMap(int const& layer, Rect const& net_bb, double const& net_weight = 1.0);
		/// utilization analysis: handlers
		UtilResult determCost() const;
};

#endif
