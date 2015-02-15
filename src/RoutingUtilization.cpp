/*
 * =====================================================================================
 *
 *    Description:  Corblivar routing-utilization analyzer
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

// own Corblivar header
#include "RoutingUtilization.hpp"
// required Corblivar headers
#include "Rect.hpp"
#include "Net.hpp"
#include "Block.hpp"
#include "Math.hpp"

// memory allocation
constexpr int RoutingUtilization::UTIL_MAPS_DIM;

void RoutingUtilization::resetUtilMaps(int const& layers) {
	int i;
	RoutingUtilization::UtilBin init_bin;

	// reset the maps w/ zero values
	init_bin.utilization = 0.0;
	for (i = 0; i < layers; i++) {
		for (auto& partial_map : this->util_maps[i]) {
			partial_map.fill(init_bin);
		}
	}
}

void RoutingUtilization::initUtilMaps(int const& layers, Point const& die_outline) {
	unsigned b;
	int i;

	if (RoutingUtilization::DBG_CALLS) {
		std::cout << "-> RoutingUtilization::initUtilMap()" << std::endl;
	}

	this->util_maps.clear();

	// allocate util-maps arrays
	for (i = 0; i < layers; i++) {
		this->util_maps.emplace_back(
			std::array<std::array<RoutingUtilization::UtilBin, RoutingUtilization::UTIL_MAPS_DIM>, RoutingUtilization::UTIL_MAPS_DIM>()
		);
	}

	// init maps w/ zero values
	this->resetUtilMaps(layers);

	// scale of util map dimensions
	this->util_maps_dim_x = die_outline.x / RoutingUtilization::UTIL_MAPS_DIM;
	this->util_maps_dim_y = die_outline.y / RoutingUtilization::UTIL_MAPS_DIM;

	// predetermine map bins' area and lower-left corner coordinates; note that the
	// last bin represents the upper-right coordinates for the penultimate bin
	this->util_maps_bin_area = this->util_maps_dim_x * this->util_maps_dim_y;
	for (b = 0; b <= RoutingUtilization::UTIL_MAPS_DIM; b++) {
		this->util_maps_bins_ll_x[b] = b * this->util_maps_dim_x;
	}
	for (b = 0; b <= RoutingUtilization::UTIL_MAPS_DIM; b++) {
		this->util_maps_bins_ll_y[b] = b * this->util_maps_dim_y;
	}

	if (RoutingUtilization::DBG_CALLS) {
		std::cout << "<- RoutingUtilization::initUtilMap" << std::endl;
	}
}

RoutingUtilization::UtilResult RoutingUtilization::determCost() const {
	unsigned x, y;
	unsigned layer;
	UtilResult ret;

	ret.cost = ret.avg_util = ret.max_util = 0.0;

	for (layer = 0; layer < this->util_maps.size(); layer++) {
		for (x = 0; x < this->util_maps[0].size(); x++) {
			for (y = 0; y < this->util_maps[0][0].size(); y++) {

				// determine max util
				if (this->util_maps[layer][x][y].utilization > ret.max_util) {
					ret.max_util = this->util_maps[layer][x][y].utilization;
				}

				// sum up util, required for avg util
				ret.avg_util += this->util_maps[layer][x][y].utilization;
			}
		}
	}

	ret.avg_util /= this->util_maps.size();
	ret.avg_util /= this->util_maps[0].size();
	ret.avg_util /= this->util_maps[0][0].size();

	// cost: avg and max util
	ret.cost = ret.avg_util * ret.max_util;

	return ret;
}

void RoutingUtilization::adaptUtilMap(int const& layer, Rect const& net_bb, double const& net_weight) {
	int x, y;
	double util;
	int x_lower, x_upper, y_lower, y_upper;
	Rect bb_ext;

	if (RoutingUtilization::DBG_CALLS) {
		std::cout << "-> RoutingUtilization::adaptUtilMap(" << layer << ", " << net_bb.area << ", " << net_weight << ")" << std::endl;
	}

	// determine index boundaries for utilization map; based on intersection of map and
	// net's bb; note that cast to int truncates toward zero, i.e., performs like
	// floor for positive numbers
	x_lower = static_cast<int>(net_bb.ll.x / this->util_maps_dim_x);
	y_lower = static_cast<int>(net_bb.ll.y / this->util_maps_dim_y);
	// +1 in order to efficiently emulate the result of ceil(); limit upper
	// bound to util-maps dimensions
	x_upper = std::min(static_cast<int>(net_bb.ur.x / this->util_maps_dim_x) + 1, RoutingUtilization::UTIL_MAPS_DIM);
	y_upper = std::min(static_cast<int>(net_bb.ur.y / this->util_maps_dim_y) + 1, RoutingUtilization::UTIL_MAPS_DIM);

	if (RoutingUtilization::DBG) {
		std::cout << "DBG_ROUTING_UTIL>  Affected util-map bins: " << x_lower << "," << y_lower
			<< " to " <<
			x_upper << "," << y_upper << std::endl;
	}

	// simple routing-utilization model: even distribution, as discussed in
	// [Meister11]; this model is surprisingly accurate for practical benchmarks;
	// calculate utilization according to wirelength, covered area, and net weight
	//
	// for area and wirelength, we consider the (by above floor and ceil index
	// boundaries) slightly extended bb
	bb_ext.w = this->util_maps_bins_ll_x[x_upper] - this->util_maps_bins_ll_x[x_lower];
	bb_ext.h = this->util_maps_bins_ll_y[y_upper] - this->util_maps_bins_ll_y[y_lower];
	bb_ext.area = bb_ext.w * bb_ext.h;

	util = net_weight * ((bb_ext.w + bb_ext.h) / bb_ext.area);

	// walk util-map bins covering intersection; adapt routing utilization
	for (x = x_lower; x < x_upper; x++) {
		for (y = y_lower; y < y_upper; y++) {

			// adapt map on affected layer
			this->util_maps[layer][x][y].utilization += util;
		}
	}

	if (RoutingUtilization::DBG_CALLS) {
		std::cout << "<- RoutingUtilization::adaptPowerMaps" << std::endl;
	}
}
