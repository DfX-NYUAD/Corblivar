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

// own Corblivar header
#include "RoutingCongestion.hpp"
// required Corblivar headers
#include "Rect.hpp"
#include "Net.hpp"
#include "Block.hpp"
#include "Math.hpp"

// memory allocation
constexpr int RoutingCongestion::CONG_MAPS_DIM;

void RoutingCongestion::resetCongMaps(int const& layers) {
	int i;
	RoutingCongestion::CongBin init_bin;

	// reset the maps w/ zero values
	init_bin.utilization = 0.0;
	for (i = 0; i < layers; i++) {
		for (auto& partial_map : this->cong_maps[i]) {
			partial_map.fill(init_bin);
		}
	}
}

void RoutingCongestion::initCongMaps(int const& layers, Point const& die_outline) {
	unsigned b;
	int i;

	if (RoutingCongestion::DBG_CALLS) {
		std::cout << "-> RoutingCongestion::initCongMap()" << std::endl;
	}

	this->cong_maps.clear();

	// allocate cong-maps arrays
	for (i = 0; i < layers; i++) {
		this->cong_maps.emplace_back(
			std::array<std::array<RoutingCongestion::CongBin, RoutingCongestion::CONG_MAPS_DIM>, RoutingCongestion::CONG_MAPS_DIM>()
		);
	}

	// init maps w/ zero values
	this->resetCongMaps(layers);

	// scale of cong map dimensions
	this->cong_maps_dim_x = die_outline.x / RoutingCongestion::CONG_MAPS_DIM;
	this->cong_maps_dim_y = die_outline.y / RoutingCongestion::CONG_MAPS_DIM;

	// predetermine map bins' area and lower-left corner coordinates; note that the
	// last bin represents the upper-right coordinates for the penultimate bin
	this->cong_maps_bin_area = this->cong_maps_dim_x * this->cong_maps_dim_y;
	for (b = 0; b <= RoutingCongestion::CONG_MAPS_DIM; b++) {
		this->cong_maps_bins_ll_x[b] = b * this->cong_maps_dim_x;
	}
	for (b = 0; b <= RoutingCongestion::CONG_MAPS_DIM; b++) {
		this->cong_maps_bins_ll_y[b] = b * this->cong_maps_dim_y;
	}

	if (RoutingCongestion::DBG_CALLS) {
		std::cout << "<- RoutingCongestion::initCongMap" << std::endl;
	}
}

void RoutingCongestion::adaptCongMap(int const& layer, Rect const& net_bb, double const& net_weight) {
	int x, y;
	double util;
	int x_lower, x_upper, y_lower, y_upper;
	Rect bb_ext;

	if (RoutingCongestion::DBG_CALLS) {
		std::cout << "-> RoutingCongestion::adaptCongMap(" << layer << ", " << net_bb.area << ", " << net_weight << ")" << std::endl;
	}

	// determine index boundaries for congestion map; based on intersection of map and
	// net's bb; note that cast to int truncates toward zero, i.e., performs like
	// floor for positive numbers
	x_lower = static_cast<int>(net_bb.ll.x / this->cong_maps_dim_x);
	y_lower = static_cast<int>(net_bb.ll.y / this->cong_maps_dim_y);
	// +1 in order to efficiently emulate the result of ceil(); limit upper
	// bound to cong-maps dimensions
	x_upper = std::min(static_cast<int>(net_bb.ur.x / this->cong_maps_dim_x) + 1, RoutingCongestion::CONG_MAPS_DIM);
	y_upper = std::min(static_cast<int>(net_bb.ur.y / this->cong_maps_dim_y) + 1, RoutingCongestion::CONG_MAPS_DIM);

	if (RoutingCongestion::DBG) {
		std::cout << "DBG_ROUTING_CONG>  Affected cong-map bins: " << x_lower << "," << y_lower
			<< " to " <<
			x_upper << "," << y_upper << std::endl;
	}

	// simple routing-utilization model: even distribution, as discussed in
	// [Meister11]; this model is surprisingly accurate for practical benchmarks;
	// calculate utilization according to wirelength, covered area, and net weight
	//
	// for area and wirelength, we consider the (by above floor and ceil index
	// boundaries) slightly extended bb
	bb_ext.w = this->cong_maps_bins_ll_x[x_upper] - this->cong_maps_bins_ll_x[x_lower];
	bb_ext.h = this->cong_maps_bins_ll_y[y_upper] - this->cong_maps_bins_ll_y[y_lower];
	bb_ext.area = bb_ext.w * bb_ext.h;

	util = net_weight * ((bb_ext.w + bb_ext.h) / bb_ext.area);

	// walk cong-map bins covering intersection; adapt routing utilization
	for (x = x_lower; x < x_upper; x++) {
		for (y = y_lower; y < y_upper; y++) {

			// adapt map on affected layer
			this->cong_maps[layer][x][y].utilization += util;
		}
	}

	if (RoutingCongestion::DBG_CALLS) {
		std::cout << "<- RoutingCongestion::adaptPowerMaps" << std::endl;
	}
}
