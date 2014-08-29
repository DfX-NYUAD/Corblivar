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

// own Corblivar header
#include "ThermalAnalyzer.hpp"
// required Corblivar headers
#include "Rect.hpp"
#include "Net.hpp"
#include "Block.hpp"
#include "Math.hpp"
#include "CorblivarAlignmentReq.hpp"

// memory allocation
constexpr int ThermalAnalyzer::POWER_MAPS_DIM;

void ThermalAnalyzer::initThermalMap() {
	int x, y;

	if (ThermalAnalyzer::DBG_CALLS) {
		cout << "-> ThermalAnalyzer::initThermalMap()" << endl;
	}

	// init map data structure
	for (x = 0; x < ThermalAnalyzer::THERMAL_MAP_DIM; x++) {
		for (y = 0; y < ThermalAnalyzer::THERMAL_MAP_DIM; y++) {

			this->thermal_map[x][y] = {
					// init w/ zero temp value
					0.0,
					// grid-map coordinates
					x,
					y,
					// hotspot/blob region id; -1 encodes background
					-1,
					// allocation for neighbor's list; to be
					// initialized during clustering
					list<ThermalMapBin*>()
			};
		}
	}

	// build-up neighbor relations for thermal-map grid;
	// inner core
	for (x = 1; x < ThermalAnalyzer::THERMAL_MAP_DIM - 1; x++) {
		for (y = 1; y < ThermalAnalyzer::THERMAL_MAP_DIM - 1; y++) {

			this->thermal_map[x][y].neighbors.push_back(&this->thermal_map[x-1][y-1]);
			this->thermal_map[x][y].neighbors.push_back(&this->thermal_map[x-1][y]);
			this->thermal_map[x][y].neighbors.push_back(&this->thermal_map[x-1][y+1]);
			this->thermal_map[x][y].neighbors.push_back(&this->thermal_map[x][y+1]);
			this->thermal_map[x][y].neighbors.push_back(&this->thermal_map[x+1][y+1]);
			this->thermal_map[x][y].neighbors.push_back(&this->thermal_map[x+1][y]);
			this->thermal_map[x][y].neighbors.push_back(&this->thermal_map[x+1][y-1]);
			this->thermal_map[x][y].neighbors.push_back(&this->thermal_map[x][y-1]);
      	}
      }

	// build-up neighbor relations for thermal-map grid;
	// outer rows and columns 
	x = 0;
	for (y = 1; y < ThermalAnalyzer::THERMAL_MAP_DIM - 1; y++) {

		this->thermal_map[x][y].neighbors.push_back(&this->thermal_map[x][y+1]);
		this->thermal_map[x][y].neighbors.push_back(&this->thermal_map[x+1][y+1]);
		this->thermal_map[x][y].neighbors.push_back(&this->thermal_map[x+1][y]);
		this->thermal_map[x][y].neighbors.push_back(&this->thermal_map[x+1][y-1]);
		this->thermal_map[x][y].neighbors.push_back(&this->thermal_map[x][y-1]);
	}

	x = ThermalAnalyzer::THERMAL_MAP_DIM - 1;
	for (y = 1; y < ThermalAnalyzer::THERMAL_MAP_DIM - 1; y++) {

		this->thermal_map[x][y].neighbors.push_back(&this->thermal_map[x-1][y-1]);
		this->thermal_map[x][y].neighbors.push_back(&this->thermal_map[x-1][y]);
		this->thermal_map[x][y].neighbors.push_back(&this->thermal_map[x-1][y+1]);
		this->thermal_map[x][y].neighbors.push_back(&this->thermal_map[x][y+1]);
		this->thermal_map[x][y].neighbors.push_back(&this->thermal_map[x][y-1]);
	}

	y = 0;
	for (x = 1; x < ThermalAnalyzer::THERMAL_MAP_DIM - 1; x++) {

		this->thermal_map[x][y].neighbors.push_back(&this->thermal_map[x-1][y]);
		this->thermal_map[x][y].neighbors.push_back(&this->thermal_map[x-1][y+1]);
		this->thermal_map[x][y].neighbors.push_back(&this->thermal_map[x][y+1]);
		this->thermal_map[x][y].neighbors.push_back(&this->thermal_map[x+1][y+1]);
		this->thermal_map[x][y].neighbors.push_back(&this->thermal_map[x+1][y]);
	}

	y = ThermalAnalyzer::THERMAL_MAP_DIM - 1;
	for (x = 1; x < ThermalAnalyzer::THERMAL_MAP_DIM - 1; x++) {

		this->thermal_map[x][y].neighbors.push_back(&this->thermal_map[x-1][y-1]);
		this->thermal_map[x][y].neighbors.push_back(&this->thermal_map[x-1][y]);
		this->thermal_map[x][y].neighbors.push_back(&this->thermal_map[x+1][y]);
		this->thermal_map[x][y].neighbors.push_back(&this->thermal_map[x+1][y-1]);
		this->thermal_map[x][y].neighbors.push_back(&this->thermal_map[x][y-1]);
	}

	// build-up neighbor relations for thermal-map grid;
	// corner points
	this->thermal_map[0][0].neighbors.push_back(&this->thermal_map[0][1]);
	this->thermal_map[0][0].neighbors.push_back(&this->thermal_map[1][1]);
	this->thermal_map[0][0].neighbors.push_back(&this->thermal_map[1][0]);
	this->thermal_map[0][ThermalAnalyzer::THERMAL_MAP_DIM - 1].neighbors.push_back(&this->thermal_map[0][ThermalAnalyzer::THERMAL_MAP_DIM - 2]);
	this->thermal_map[0][ThermalAnalyzer::THERMAL_MAP_DIM - 1].neighbors.push_back(&this->thermal_map[1][ThermalAnalyzer::THERMAL_MAP_DIM - 2]);
	this->thermal_map[0][ThermalAnalyzer::THERMAL_MAP_DIM - 1].neighbors.push_back(&this->thermal_map[1][ThermalAnalyzer::THERMAL_MAP_DIM - 1]);
	this->thermal_map[ThermalAnalyzer::THERMAL_MAP_DIM - 1][ThermalAnalyzer::THERMAL_MAP_DIM - 1].neighbors.push_back(
			&this->thermal_map[ThermalAnalyzer::THERMAL_MAP_DIM - 2][ThermalAnalyzer::THERMAL_MAP_DIM - 1]);
	this->thermal_map[ThermalAnalyzer::THERMAL_MAP_DIM - 1][ThermalAnalyzer::THERMAL_MAP_DIM - 1].neighbors.push_back(
			&this->thermal_map[ThermalAnalyzer::THERMAL_MAP_DIM - 2][ThermalAnalyzer::THERMAL_MAP_DIM - 2]);
	this->thermal_map[ThermalAnalyzer::THERMAL_MAP_DIM - 1][ThermalAnalyzer::THERMAL_MAP_DIM - 1].neighbors.push_back(
			&this->thermal_map[ThermalAnalyzer::THERMAL_MAP_DIM - 1][ThermalAnalyzer::THERMAL_MAP_DIM - 2]);
	this->thermal_map[ThermalAnalyzer::THERMAL_MAP_DIM - 1][0].neighbors.push_back(&this->thermal_map[ThermalAnalyzer::THERMAL_MAP_DIM - 2][0]);
	this->thermal_map[ThermalAnalyzer::THERMAL_MAP_DIM - 1][0].neighbors.push_back(&this->thermal_map[ThermalAnalyzer::THERMAL_MAP_DIM - 2][1]);
	this->thermal_map[ThermalAnalyzer::THERMAL_MAP_DIM - 1][0].neighbors.push_back(&this->thermal_map[ThermalAnalyzer::THERMAL_MAP_DIM - 1][1]);

	if (ThermalAnalyzer::DBG_CALLS) {
		cout << "<- ThermalAnalyzer::initThermalMap" << endl;
	}
}

void ThermalAnalyzer::initPowerMaps(int const& layers, Point const& die_outline) {
	unsigned b;
	int i;
	ThermalAnalyzer::PowerMapBin init_bin;

	if (ThermalAnalyzer::DBG_CALLS) {
		cout << "-> ThermalAnalyzer::initPowerMaps(" << layers << ", " << die_outline.x << ", " << die_outline.y << ")" << endl;
	}

	this->power_maps.clear();

	// allocate power-maps arrays
	for (i = 0; i < layers; i++) {
		this->power_maps.emplace_back(
			array<array<ThermalAnalyzer::PowerMapBin, ThermalAnalyzer::POWER_MAPS_DIM>, ThermalAnalyzer::POWER_MAPS_DIM>()
		);
	}

	// init the maps w/ zero values
	init_bin.power_density = init_bin.TSV_density = 0.0;
	for (i = 0; i < layers; i++) {
		for (auto& partial_map : this->power_maps[i]) {
			partial_map.fill(init_bin);
		}
	}

	// scale power map dimensions to outline of thermal map; this way the padding of
	// power maps doesn't distort the block outlines in the thermal map
	this->power_maps_dim_x = die_outline.x / ThermalAnalyzer::THERMAL_MAP_DIM;
	this->power_maps_dim_y = die_outline.y / ThermalAnalyzer::THERMAL_MAP_DIM;

	// determine offset for blocks, related to padding of power maps
	this->blocks_offset_x = this->power_maps_dim_x * ThermalAnalyzer::POWER_MAPS_PADDED_BINS;
	this->blocks_offset_y = this->power_maps_dim_y * ThermalAnalyzer::POWER_MAPS_PADDED_BINS;

	// determine max distance for blocks' upper/right boundaries to upper/right die
	// outline to be padded
	this->padding_right_boundary_blocks_distance = ThermalAnalyzer::PADDING_ZONE_BLOCKS_DISTANCE_LIMIT * die_outline.x;
	this->padding_upper_boundary_blocks_distance = ThermalAnalyzer::PADDING_ZONE_BLOCKS_DISTANCE_LIMIT * die_outline.y;

	// predetermine map bins' area and lower-left corner coordinates; note that the
	// last bin represents the upper-right coordinates for the penultimate bin
	this->power_maps_bin_area = this->power_maps_dim_x * this->power_maps_dim_y;
	for (b = 0; b <= ThermalAnalyzer::POWER_MAPS_DIM; b++) {
		this->power_maps_bins_ll_x[b] = b * this->power_maps_dim_x;
	}
	for (b = 0; b <= ThermalAnalyzer::POWER_MAPS_DIM; b++) {
		this->power_maps_bins_ll_y[b] = b * this->power_maps_dim_y;
	}

	if (ThermalAnalyzer::DBG_CALLS) {
		cout << "<- ThermalAnalyzer::initPowerMaps" << endl;
	}
}

// Determine masks for lowest layer, i.e., hottest layer.
// Based on a gaussian-like thermal impulse response fuction.
// Note that masks are centered, i.e., the value f(x=0) resides in the middle of the
// (uneven) array.
// Note that masks are 1D, sufficient for the separated convolution in
// performPowerBlurring()
void ThermalAnalyzer::initThermalMasks(int const& layers, bool const& log, MaskParameters const& parameters) {
	int i, ii;
	double scale;
	double layer_impulse_factor;
	int x_y;

	if (ThermalAnalyzer::DBG_CALLS) {
		cout << "-> ThermalAnalyzer::initThermalMasks(" << layers << ", " << log << ")" << endl;
	}

	if (log) {
		cout << "ThermalAnalyzer> ";
		cout << "Initializing thermals masks for power blurring ..." << endl;
	}

	// reset mask arrays
	this->thermal_masks.clear();

	// allocate mask arrays
	for (i = 0; i < layers; i++) {
		this->thermal_masks.emplace_back(
			array<double,ThermalAnalyzer::THERMAL_MASK_DIM>()
		);
	}

	// determine scale factor such that mask_boundary_value is reached at the
	// boundary of the lowermost (2D) mask; based on general 2D gauss equation,
	// determines gauss(x = y) = mask_boundary_value;
	// constant spread (e.g., 1.0) is sufficient since this function fitting
	// only requires two parameters, i.e., varying spread has no impact
	static constexpr double SPREAD = 1.0;
	// scaling is required for function fitting; the maximum of the gauss / exp
	// function is defined by the impulse factor, the minimum by the
	// mask_boundary_value
	scale = sqrt(SPREAD * std::log(parameters.impulse_factor / (parameters.mask_boundary_value))) / sqrt(2.0);
	// normalize factor according to half of mask dimension; i.e., fit spreading of
	// exp function
	scale /=  ThermalAnalyzer::THERMAL_MASK_CENTER;

	// determine all masks, starting from lowest layer, i.e., hottest layer
	for (i = 1; i <= layers; i++) {

		// impulse factor is to be reduced notably for increasing layer count
		layer_impulse_factor = parameters.impulse_factor / pow(i, parameters.impulse_factor_scaling_exponent);

		ii = 0;
		for (x_y = -ThermalAnalyzer::THERMAL_MASK_CENTER; x_y <= ThermalAnalyzer::THERMAL_MASK_CENTER; x_y++) {
			// sqrt for impulse factor is mandatory since the mask is
			// used for separated convolution (i.e., factor will be
			// squared in final convolution result)
			this->thermal_masks[i - 1][ii] = Math::gauss1D(x_y * scale, sqrt(layer_impulse_factor), SPREAD);

			ii++;
		}
	}

	if (ThermalAnalyzer::DBG) {
		// enforce fixed digit count for printing mask
		cout << fixed;
		// dump mask
		for (i = 0; i < layers; i++) {
			cout << "DBG> Thermal 1D mask for point source on layer " << i << ":" << endl;
			for (x_y = 0; x_y < ThermalAnalyzer::THERMAL_MASK_DIM; x_y++) {
				cout << this->thermal_masks[i][x_y] << ", ";
			}
			cout << endl;
		}
		// reset to default floating output
		cout.unsetf(ios_base::floatfield);

		cout << endl;
		cout << "DBG> Note that these values will be multiplied w/ each other in the final 2D mask" << endl;
	}

	if (log) {
		cout << "ThermalAnalyzer> ";
		cout << "Done" << endl << endl;
	}

	if (ThermalAnalyzer::DBG_CALLS) {
		cout << "<- ThermalAnalyzer::initThermalMasks" << endl;
	}
}

void ThermalAnalyzer::generatePowerMaps(int const& layers, vector<Block> const& blocks, Point const& die_outline, MaskParameters const& parameters, bool const& extend_boundary_blocks_into_padding_zone) {
	int i;
	int x, y;
	Rect bin, intersect, block_offset;
	int x_lower, x_upper, y_lower, y_upper;
	bool padding_zone;
	ThermalAnalyzer::PowerMapBin init_bin;

	if (ThermalAnalyzer::DBG_CALLS) {
		cout << "-> ThermalAnalyzer::generatePowerMaps(" << layers << ", " << &blocks << ", (" << die_outline.x << ", " << die_outline.y << "), " << &parameters << ", " << extend_boundary_blocks_into_padding_zone << ")" << endl;
	}

	init_bin.power_density = 0.0;
	init_bin.TSV_density = 0.0;

	// determine maps for each layer
	for (i = 0; i < layers; i++) {

		// reset map to zero
		// note: this also implicitly pads the map w/ zero power density
		for (auto& m : this->power_maps[i]) {
			m.fill(init_bin);
		}

		// consider each block on the related layer
		for (Block const& block : blocks) {

			// drop blocks assigned to other layers
			if (block.layer != i) {
				continue;
			}

			// determine offset, i.e., shifted, block bb; relates to block's
			// bb in padded power map
			block_offset = block.bb;

			// don't offset blocks at the left/lower chip boundaries,
			// implicitly extend them into power-map padding zone; this way,
			// during convolution, the thermal estimate increases for these
			// blocks; blocks not at the boundaries are shifted
			if (extend_boundary_blocks_into_padding_zone && block.bb.ll.x == 0.0) {
			}
			else {
				block_offset.ll.x += this->blocks_offset_x;
			}
			if (extend_boundary_blocks_into_padding_zone && block.bb.ll.y == 0.0) {
			}
			else {
				block_offset.ll.y += this->blocks_offset_y;
			}

			// also consider extending blocks into right/upper padding zone if
			// they are close to the related chip boundaries
			if (
					extend_boundary_blocks_into_padding_zone &&
					abs(die_outline.x - block.bb.ur.x) < this->padding_right_boundary_blocks_distance
			   ) {
				// consider offset twice in order to reach right/uppper
				// boundary related to layout described by padded power map
				block_offset.ur.x = die_outline.x + 2.0 * this->blocks_offset_x;
			}
			// simple shift otherwise; compensate for padding of left/bottom
			// boundaries
			else {
				block_offset.ur.x += this->blocks_offset_x;
			}

			if (
					extend_boundary_blocks_into_padding_zone
					&& abs(die_outline.y - block.bb.ur.y) < this->padding_upper_boundary_blocks_distance
			   ) {
				block_offset.ur.y = die_outline.y + 2.0 * this->blocks_offset_y;
			}
			else {
				block_offset.ur.y += this->blocks_offset_y;
			}

			// determine index boundaries for offset block; based on boundary
			// of blocks and the covered bins; note that cast to int truncates
			// toward zero, i.e., performs like floor for positive numbers
			x_lower = static_cast<int>(block_offset.ll.x / this->power_maps_dim_x);
			y_lower = static_cast<int>(block_offset.ll.y / this->power_maps_dim_y);
			// +1 in order to efficiently emulate the result of ceil(); limit
			// upper bound to power-maps dimenions
			x_upper = min(static_cast<int>(block_offset.ur.x / this->power_maps_dim_x) + 1, ThermalAnalyzer::POWER_MAPS_DIM);
			y_upper = min(static_cast<int>(block_offset.ur.y / this->power_maps_dim_y) + 1, ThermalAnalyzer::POWER_MAPS_DIM);

			// walk power-map bins covering block outline
			for (x = x_lower; x < x_upper; x++) {
				for (y = y_lower; y < y_upper; y++) {

					// determine if bin w/in padding zone
					if (
							x < ThermalAnalyzer::POWER_MAPS_PADDED_BINS
							|| x >= (ThermalAnalyzer::POWER_MAPS_DIM - ThermalAnalyzer::POWER_MAPS_PADDED_BINS)
							|| y < ThermalAnalyzer::POWER_MAPS_PADDED_BINS
							|| y >= (ThermalAnalyzer::POWER_MAPS_DIM - ThermalAnalyzer::POWER_MAPS_PADDED_BINS)
					   ) {
						padding_zone = true;
					}
					else {
						padding_zone = false;
					}

					// consider full block power density for fully covered bins
					if (x_lower < x && x < (x_upper - 1) && y_lower < y && y < (y_upper - 1)) {
						if (padding_zone) {
							this->power_maps[i][x][y].power_density += block.power_density * parameters.power_density_scaling_padding_zone;
						}
						else {
							this->power_maps[i][x][y].power_density += block.power_density;
						}
					}
					// else consider block power according to
					// intersection of current bin and block
					else {
						// determine real coords of map bin
						bin.ll.x = this->power_maps_bins_ll_x[x];
						bin.ll.y = this->power_maps_bins_ll_y[y];
						// note that +1 is guaranteed to be within bounds
						// of power_maps_bins_ll_x/y (size =
						// ThermalAnalyzer::POWER_MAPS_DIM + 1); the
						// related last tuple describes the upper-right
						// corner coordinates of the right/top boundary
						bin.ur.x = this->power_maps_bins_ll_x[x + 1];
						bin.ur.y = this->power_maps_bins_ll_y[y + 1];

						// determine intersection
						intersect = Rect::determineIntersection(bin, block_offset);
						// normalize to full bin area
						intersect.area /= this->power_maps_bin_area;

						if (padding_zone) {
							this->power_maps[i][x][y].power_density += block.power_density * intersect.area * parameters.power_density_scaling_padding_zone;
						}
						else {
							this->power_maps[i][x][y].power_density += block.power_density * intersect.area;
						}
					}
				}
			}
		}
	}

	if (ThermalAnalyzer::DBG_CALLS) {
		cout << "<- ThermalAnalyzer::generatePowerMaps" << endl;
	}
}

void ThermalAnalyzer::adaptPowerMaps(int const& layers, vector<TSV_Group> const& TSVs, vector<Net> const& nets, MaskParameters const& parameters) {
	int x, y;
	Rect aligned_blocks_intersect;
	Rect bin, bin_intersect;
	int x_lower, x_upper, y_lower, y_upper;
	int i;
	Rect bb, prev_bb;

	if (ThermalAnalyzer::DBG_CALLS) {
		cout << "-> ThermalAnalyzer::adaptPowerMaps(" << layers << ", " << &TSVs << ", " << &nets << ", " << &parameters << ")" << endl;
	}

	// consider impact of vertical buses; map TSVs to power maps
	//
	// note that local copies of TSVs groups are used in order to not mess with the
	// actual coordinates of the groups
	for (TSV_Group TSV_group : TSVs) {

		// offset intersection, i.e., account for padded power maps and related
		// offset in coordinates
		TSV_group.bb.ll.x += this->blocks_offset_x;
		TSV_group.bb.ll.y += this->blocks_offset_y;
		TSV_group.bb.ur.x += this->blocks_offset_x;
		TSV_group.bb.ur.y += this->blocks_offset_y;

		// determine index boundaries for offset intersection; based on boundary
		// of intersection and the covered bins; note that cast to int truncates
		// toward zero, i.e., performs like floor for positive numbers
		x_lower = static_cast<int>(TSV_group.bb.ll.x / this->power_maps_dim_x);
		y_lower = static_cast<int>(TSV_group.bb.ll.y / this->power_maps_dim_y);
		// +1 in order to efficiently emulate the result of ceil(); limit upper
		// bound to power-maps dimensions
		x_upper = min(static_cast<int>(TSV_group.bb.ur.x / this->power_maps_dim_x) + 1, ThermalAnalyzer::POWER_MAPS_DIM);
		y_upper = min(static_cast<int>(TSV_group.bb.ur.y / this->power_maps_dim_y) + 1, ThermalAnalyzer::POWER_MAPS_DIM);

		if (ThermalAnalyzer::DBG) {
			cout << "DBG> TSV group " << TSV_group.id << endl;
			cout << "DBG>  Affected power-map bins: " << x_lower << "," << y_lower
				<< " to " <<
				x_upper << "," << y_upper << endl;
		}

		// walk power-map bins covering intersection outline; adapt TSV densities
		for (x = x_lower; x < x_upper; x++) {
			for (y = y_lower; y < y_upper; y++) {

				// consider full TSV density for fully covered bins
				if (x_lower < x && x < (x_upper - 1) && y_lower < y && y < (y_upper - 1)) {

					// adapt map on affected layer
					this->power_maps[TSV_group.layer][x][y].TSV_density += 100.0;
				}
				// else consider TSV density according to partial
				// intersection with current bin
				else {
					// determine real coords of map bin
					bin.ll.x = this->power_maps_bins_ll_x[x];
					bin.ll.y = this->power_maps_bins_ll_y[y];
					// note that +1 is guaranteed to be within
					// bounds of power_maps_bins_ll_x/y (size
					// = ThermalAnalyzer::POWER_MAPS_DIM + 1);
					// the related last tuple describes the
					// upper-right corner coordinates of the
					// right/top boundary
					bin.ur.x = this->power_maps_bins_ll_x[x + 1];
					bin.ur.y = this->power_maps_bins_ll_y[y + 1];

					// determine intersection
					bin_intersect = Rect::determineIntersection(bin, TSV_group.bb);
					// normalize to full bin area
					bin_intersect.area /= this->power_maps_bin_area;

					// adapt map on affected layer
					this->power_maps[TSV_group.layer][x][y].TSV_density += 100.0 * bin_intersect.area;
				}
			}
		}
	}

	// walk power-map bins; adapt power according to TSV densities
	for (x = ThermalAnalyzer::POWER_MAPS_PADDED_BINS; x < ThermalAnalyzer::THERMAL_MAP_DIM + ThermalAnalyzer::POWER_MAPS_PADDED_BINS; x++) {
		for (y = ThermalAnalyzer::POWER_MAPS_PADDED_BINS; y < ThermalAnalyzer::THERMAL_MAP_DIM + ThermalAnalyzer::POWER_MAPS_PADDED_BINS; y++) {

			// sanity check; TSV density should be <= 100%; might be larger
			// due to superposition in calculations above
			for (i = 0; i < layers; i++) {
				this->power_maps[i][x][y].TSV_density = min(100.0, this->power_maps[i][x][y].TSV_density);
			}

			// adapt maps for all layers; the uppermost layer next the
			// heatsink may also contain (dummy) thermal TSVs
			for (i = 0; i < layers; i++) {

				// ignore cases w/o TSVs
				if (this->power_maps[i][x][y].TSV_density == 0.0)
					continue;

				// scaling depends on TSV density; the larger the TSV
				// density, the larger the power down-scaling; note that
				// the factor power_density_scaling_TSV_region ranges b/w
				// 0.0 and 1.0, whereas TSV_density ranges b/w 0.0 and
				// 100.0
				this->power_maps[i][x][y].power_density *=
					1.0 +
					((parameters.power_density_scaling_TSV_region - 1.0) / 100.0) * this->power_maps[i][x][y].TSV_density;
			}
		}
	}

	if (ThermalAnalyzer::DBG_CALLS) {
		cout << "<- ThermalAnalyzer::adaptPowerMaps" << endl;
	}
}

// Thermal-analyzer routine based on power blurring,
// i.e., convolution of thermals masks and power maps into thermal maps.
// Based on a separated convolution using separated 2D gauss function, i.e., 1D gauss
// fct., see http://www.songho.ca/dsp/convolution/convolution.html#separable_convolution
// Returns thermal map of lowest layer, i.e., hottest layer
void ThermalAnalyzer::performPowerBlurring(ThermalAnalysisResult& ret, int const& layers, MaskParameters const& parameters) {
	int layer;
	int x, y, i;
	int map_x, map_y;
	int mask_i;
	double max_temp, avg_temp;
	// required as buffer for separated convolution; note that its dimensions
	// corresponds to a power map, which is required to hold temporary results for 1D
	// convolution of padded power maps
	array< array<double, ThermalAnalyzer::POWER_MAPS_DIM>, ThermalAnalyzer::POWER_MAPS_DIM> thermal_map_tmp;

	if (ThermalAnalyzer::DBG_CALLS) {
		cout << "-> ThermalAnalyzer::performPowerBlurring(" << &ret << ", " << ", " << layers << ", " << &parameters << ")" << endl;
	}

	// init temp map w/ zero
	for (auto& m : thermal_map_tmp) {
		m.fill(0.0);
	}

	// reset temp and hotspot regions id
	//
	// consider temperature offset; temperature offset is a additive factor, and thus
	// not considered during convolution
	for (x = 0; x < ThermalAnalyzer::THERMAL_MAP_DIM; x++) {
		for (y = 0; y < ThermalAnalyzer::THERMAL_MAP_DIM; y++) {

			this->thermal_map[x][y].temp = parameters.temp_offset;
			// -1 encodes background
			this->thermal_map[x][y].hotspot_region_id = -1;
		}
	}

	/// perform 2D convolution by performing two separated 1D convolution iterations;
	/// note that no (kernel) flipping is required since the mask is symmetric
	//
	// start w/ horizontal convolution (with which to start doesn't matter actually)
	for (layer = 0; layer < layers; layer++) {

		// walk power-map grid for horizontal convolution; store into
		// thermal_map_tmp; note that during horizontal convolution we need to
		// walk the full y-dimension related to the padded power map in order to
		// reasonably model the thermal effect in the padding zone during
		// subsequent vertical convolution
		for (y = 0; y < ThermalAnalyzer::POWER_MAPS_DIM; y++) {

			// for the x-dimension during horizontal convolution, we need to
			// restrict the considered range according to the thermal map in
			// order to exploit the padded power map w/o mask boundary checks
			for (x = ThermalAnalyzer::POWER_MAPS_PADDED_BINS; x < ThermalAnalyzer::THERMAL_MAP_DIM + ThermalAnalyzer::POWER_MAPS_PADDED_BINS; x++) {

				// perform horizontal 1D convolution, i.e., multiply
				// input[x] w/ mask
				//
				// e.g., for x = 0, THERMAL_MASK_DIM = 3
				// convol1D(x=0) = input[-1] * mask[0] + input[0] * mask[1] + input[1] * mask[2]
				//
				// can be also illustrated by aligning and multiplying
				// both arrays:
				// input array (power map); unpadded view
				// |x=-1|x=0|x=1|x=2|
				// input array (power map); padded, real view
				// |x=0 |x=1|x=2|x=3|
				// mask:
				// |m=0 |m=1|m=2|
				//
				for (mask_i = 0; mask_i < ThermalAnalyzer::THERMAL_MASK_DIM; mask_i++) {

					// determine power-map index; note that it is not
					// out of range due to the padded power maps
					i = x + (mask_i - ThermalAnalyzer::THERMAL_MASK_CENTER);

					if (ThermalAnalyzer::DBG) {

						// mass of dbg messages; only for insane
						// flag
						if (ThermalAnalyzer::DBG_INSANE) {
							cout << "DBG> y=" << y << ", x=" << x << ", mask_i=" << mask_i << ", i=" << i << endl;
						}

						if (i < 0 || i >= ThermalAnalyzer::POWER_MAPS_DIM) {
							cout << "DBG> Convolution data error; i out of range (should be limited by x)" << endl;
						}
					}

					// convolution; multiplication of mask element and
					// power-map bin
					thermal_map_tmp[x][y] +=
						this->power_maps[layer][i][y].power_density *
						this->thermal_masks[layer][mask_i];
				}
			}
		}
	}

	// continue w/ vertical convolution; here we convolute the temp thermal map (sized
	// like the padded power map) w/ the thermal masks in order to obtain the final
	// thermal map (sized like a non-padded power map)
	for (layer = 0; layer < layers; layer++) {

		// walk power-map grid for vertical convolution; convolute mask w/ data
		// obtained by horizontal convolution (thermal_map_tmp)
		for (x = ThermalAnalyzer::POWER_MAPS_PADDED_BINS; x < ThermalAnalyzer::THERMAL_MAP_DIM + ThermalAnalyzer::POWER_MAPS_PADDED_BINS; x++) {

			// index for final thermal map, considers padding offset
			map_x = x - ThermalAnalyzer::POWER_MAPS_PADDED_BINS;

			for (y = ThermalAnalyzer::POWER_MAPS_PADDED_BINS; y < ThermalAnalyzer::THERMAL_MAP_DIM + ThermalAnalyzer::POWER_MAPS_PADDED_BINS; y++) {

				// index for final thermal map, considers padding offset
				map_y = y - ThermalAnalyzer::POWER_MAPS_PADDED_BINS;

				// perform 1D vertical convolution
				for (mask_i = 0; mask_i < ThermalAnalyzer::THERMAL_MASK_DIM; mask_i++) {

					// determine power-map index; note that it is not
					// out of range due to the temp thermal map (sized
					// like the padded power map)
					i = y + (mask_i - ThermalAnalyzer::THERMAL_MASK_CENTER);

					if (ThermalAnalyzer::DBG) {

						// mass of dbg messages; only for insane
						// flag
						if (ThermalAnalyzer::DBG_INSANE) {
							cout << "DBG> x=" << x << ", y=" << y << ", map_x=" << map_x << ", map_y=" << map_y;
							cout << ", mask_i=" << mask_i << ", i=" << i << endl;
						}

						if (i < 0 || i >= ThermalAnalyzer::POWER_MAPS_DIM) {
							cout << "DBG> Convolution data error; i out of range (should be limited by y)" << endl;
						}
					}

					// convolution; multiplication of mask element and
					// power-map bin
					this->thermal_map[map_x][map_y].temp +=
						thermal_map_tmp[x][i] *
						this->thermal_masks[layer][mask_i];
				}
			}
		}
	}

	// determine max and avg value
	max_temp = avg_temp = 0.0;
	for (x = 0; x < ThermalAnalyzer::THERMAL_MAP_DIM; x++) {
		for (y = 0; y < ThermalAnalyzer::THERMAL_MAP_DIM; y++) {
			max_temp = max(max_temp, this->thermal_map[x][y].temp);
			avg_temp += this->thermal_map[x][y].temp;
		}
	}
	avg_temp /= pow(ThermalAnalyzer::THERMAL_MAP_DIM, 2);

	// determine cost: max temp estimation, weighted w/ avg temp
	ret.cost_temp = avg_temp * max_temp;
	// store max temp
	ret.max_temp = max_temp;
	// also link whole thermal map to result
	ret.thermal_map = &this->thermal_map;

	if (ThermalAnalyzer::DBG_CALLS) {
		cout << "<- ThermalAnalyzer::performPowerBlurring" << endl;
	}
}
