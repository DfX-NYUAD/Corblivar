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
	Rect bin, bin_intersect;
	int x_lower, x_upper, y_lower, y_upper;
	Rect bb;

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
	// [Meister11]; this model is surprisingly accurate for practical benchmarks
	//
	// calculate utilization according to wirelength, covered area, and net weight
	util = net_weight * (net_bb.w + net_bb.h) / net_bb.area;

	// walk cong-map bins covering intersection; adapt routing utilization
	for (x = x_lower; x < x_upper; x++) {
		for (y = y_lower; y < y_upper; y++) {

			// consider full utilization for fully covered bins
			if (x_lower < x && x < (x_upper - 1) && y_lower < y && y < (y_upper - 1)) {

				// adapt map on affected layer
				this->cong_maps[layer][x][y].utilization += util;
			}
			// else consider utilization according to partial intersection
			// with current bin
			else {
				// determine real coords of map bin
				bin.ll.x = this->cong_maps_bins_ll_x[x];
				bin.ll.y = this->cong_maps_bins_ll_y[y];
				// note that +1 is guaranteed to be within bounds of
				// cong_maps_bins_ll_x/y (size =
				// RoutingCongestion::CONG_MAPS_DIM + 1); the related last
				// tuple describes the upper-right corner coordinates of
				// the right/top boundary
				bin.ur.x = this->cong_maps_bins_ll_x[x + 1];
				bin.ur.y = this->cong_maps_bins_ll_y[y + 1];

				// determine intersection
				bin_intersect = Rect::determineIntersection(bin, net_bb);
				// normalize to full bin area
				bin_intersect.area /= this->cong_maps_bin_area;

				// adapt map on affected layer
				this->cong_maps[layer][x][y].utilization += util * bin_intersect.area;
			}
		}
	}

	if (RoutingCongestion::DBG_CALLS) {
		std::cout << "<- RoutingCongestion::adaptPowerMaps" << std::endl;
	}
}
