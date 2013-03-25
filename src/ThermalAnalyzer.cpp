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
#include "Corblivar.hpp"

// definitions for const vars, to allocate memory
// initializations see Corblivar.hpp
constexpr int ThermalAnalyzer::thermal_map_dim;

// Thermal-analyzer routine based on power blurring,
// i.e., convolution of thermals masks and power maps into thermal maps.
// Based on a separated convolution using separated 2D gauss function, i.e., 1D gauss fct.
// Returns max value of thermal map of lowest layer, i.e., hottest layer
// Based on http://www.songho.ca/dsp/convolution/convolution.html#separable_convolution
double ThermalAnalyzer::performPowerBlurring(const FloorPlanner& fp, const bool& set_max_cost, const bool& normalize) const {
	int layer;
	int x, y, i;
	int map_x, map_y;
	int mask_i;
	double max_temp;
	// required as buffer for separated convolution; note that its dimensions
	// corresponds to a power map, which is required to hold temporary results for 1D
	// convolution of padded power maps
	array<array<double,ThermalAnalyzer::power_maps_dim>,ThermalAnalyzer::power_maps_dim> thermal_map_tmp;

#ifdef DBG_CALLS_THERMAL
	cout << "-> ThermalAnalyzer::performPowerBlurring(" << &fp << ", " << set_max_cost << ", " << normalize << ")" << endl;
#endif

	// init temp map to zero
	for (auto& m : thermal_map_tmp) {
		m.fill(0.0);
	}

	// reset thermal map to zero
	for (auto& m : this->thermal_map) {
		m.fill(0.0);
	}

	/// perform 2D convolution by performing two separated 1D convolution iterations;
	/// note that no (kernel) flipping is required since the mask is symmetric
	//
	// start w/ horizontal convolution (with which to start doesn't matter actually)
	// TODO replace fp.conf_layer w/ fct parameter
	for (layer = 0; layer < fp.conf_layer; layer++) {

		// walk power-map grid for horizontal convolution; store into thermal_map_tmp;
		// consider offset due to padding of power map; also walk only according
		// to thermal_map_dim which is the relevant input dimension
		for (y = ThermalAnalyzer::mask_dim_half; y < ThermalAnalyzer::thermal_map_dim + ThermalAnalyzer::mask_dim_half; y++) {

			for (x = ThermalAnalyzer::mask_dim_half; x < ThermalAnalyzer::thermal_map_dim + ThermalAnalyzer::mask_dim_half; x++) {

				// perform horizontal 1D convolution, i.e., multiply
				// input[x] w/ mask
				//
				// e.g., for x = 0, mask_dim = 3
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
				for (mask_i = 0; mask_i < ThermalAnalyzer::mask_dim; mask_i++) {

					// determine power-map index; note that it is not
					// out of range due to the padded power maps
					i = x + (mask_i - ThermalAnalyzer::mask_center);

#ifdef DBG_LAYOUT
					cout << "y=" << y << ", x=" << x << ", mask_i=" << mask_i << ", i=" << i << endl;
					if (i < 0 || i >= ThermalAnalyzer::power_maps_dim) {
						cout << "i out of range (should be limited by x)" << endl;
					}
#endif

					// convolution; multplication of mask element and
					// power-map bin
					thermal_map_tmp[x][y] += this->power_maps[layer][i][y] * this->thermal_masks[layer][mask_i];
				}
			}
		}
	}

	// continue w/ vertical convolution
	// TODO replace fp.conf_layer w/ fct parameter
	for (layer = 0; layer < fp.conf_layer; layer++) {

		// walk power-map grid for horizontal convolution; use data from
		// thermal_map_tmp and store into thermal_map
		for (x = ThermalAnalyzer::mask_dim_half; x < ThermalAnalyzer::thermal_map_dim + ThermalAnalyzer::mask_dim_half; x++) {

			// adapt index for thermal map according to padding
			map_x = x - ThermalAnalyzer::mask_dim_half;

			for (y = ThermalAnalyzer::mask_dim_half; y < ThermalAnalyzer::thermal_map_dim + ThermalAnalyzer::mask_dim_half; y++) {

				// adapt index for thermal map according to padding
				map_y = y - ThermalAnalyzer::mask_dim_half;

				// perform 1D vertical convolution
				for (mask_i = 0; mask_i < ThermalAnalyzer::mask_dim; mask_i++) {

					// determine power-map index; note that it is not
					// out of range due to the padded power maps
					i = y + (mask_i - ThermalAnalyzer::mask_center);

#ifdef DBG_LAYOUT
					cout << "x=" << x << ", y=" << y << ", map_x=" << map_x << ", map_y=" << map_y;
					cout << ", mask_i=" << mask_i << ", i=" << i << endl;
					if (i < 0 || i >= ThermalAnalyzer::power_maps_dim) {
						cout << "i out of range (should be limited by y)" << endl;
					}
#endif

					// convolution; multplication of mask element and
					// power-map bin
					this->thermal_map[map_x][map_y] += thermal_map_tmp[x][i] * this->thermal_masks[layer][mask_i];
				}
			}
		}
	}

	// determine max value
	max_temp = 0.0;
	for (x = 0; x < ThermalAnalyzer::thermal_map_dim; x++) {
		for (y = 0; y < ThermalAnalyzer::thermal_map_dim; y++) {
			max_temp = max(max_temp, this->thermal_map[x][y]);
		}
	}

	// memorize max cost; initial sampling
	if (set_max_cost) {
		fp.max_cost_temp = max_temp;
	}

	// normalize to max value from initial sampling
	if (normalize) {
		max_temp /= fp.max_cost_temp;
	}

#ifdef DBG_CALLS_THERMAL
	cout << "<- ThermalAnalyzer::performPowerBlurring : " << max_temp << endl;
#endif

	return max_temp;
}

void ThermalAnalyzer::generatePowerMaps(const FloorPlanner& fp) const {
	int i;
	int x, y;
	Block *block;
	double maps_dim_x, maps_dim_y;
	double offset_x, offset_y;
	array<array<double,ThermalAnalyzer::power_maps_dim>,ThermalAnalyzer::power_maps_dim> map;
	Rect bin, intersect, block_offset;
	int x_lower, x_upper, y_lower, y_upper;

#ifdef DBG_CALLS_THERMAL
	cout << "-> ThermalAnalyzer::generatePowerMaps(" << &fp << ")" << endl;
#endif

	// clear maps
	this->power_maps.clear();
	this->power_maps.reserve(fp.conf_layer);

	// scale power map dimensions to outline of thermal map; this way the padding of
	// power maps doesn't distort the block outlines in the thermal map
	maps_dim_x = fp.conf_outline_x / ThermalAnalyzer::thermal_map_dim;
	maps_dim_y = fp.conf_outline_y / ThermalAnalyzer::thermal_map_dim;
	// determine offset for blocks, related to padding of power maps
	offset_x = (fp.conf_outline_x / ThermalAnalyzer::power_maps_dim) * ThermalAnalyzer::mask_dim_half;
	offset_y = (fp.conf_outline_y / ThermalAnalyzer::power_maps_dim) * ThermalAnalyzer::mask_dim_half;

	// determine maps for each layer
	for (i = 0; i < fp.conf_layer; i++) {

		// init map to zero
		// note: this also implicitly pads the map w/ zero
		for (auto& m : map) {
			m.fill(0.0);
		}

		// consider each block on the related layer
		for (auto& b : fp.blocks) {
			block = b.second;

			// drop blocks assigned to other layers
			if (block->layer != i) {
				continue;
			}

			// determine offset block bb; relates to block's bb in padded
			// power map
			block_offset = block->bb;
			block_offset.ll.x += offset_x;
			block_offset.ur.x += offset_x;
			block_offset.ll.y += offset_y;
			block_offset.ur.y += offset_y;

			// determine index boundaries for block
			x_lower = floor(block_offset.ll.x / maps_dim_x);
			x_upper = ceil(block_offset.ur.x / maps_dim_x);
			y_lower = floor(block_offset.ll.y / maps_dim_y);
			y_upper = ceil(block_offset.ur.y / maps_dim_y);
			// limit boundaries; restricts mapping of blocks' power to power
			// map according to offset thermal-map dimensions
			x_upper = min(x_upper, ThermalAnalyzer::thermal_map_dim + ThermalAnalyzer::mask_dim_half);
			y_upper = min(y_upper, ThermalAnalyzer::thermal_map_dim + ThermalAnalyzer::mask_dim_half);

			// walk power-map bins covering block outline
			for (x = x_lower; x < x_upper; x++) {
				for (y = y_lower; y < y_upper; y++) {

					// determine real coords of map bin
					bin.ll.x = x * maps_dim_x;
					bin.ur.x = bin.ll.x + maps_dim_x;
					bin.ll.y = y * maps_dim_y;
					bin.ur.y = bin.ll.y + maps_dim_y;

					// consider total block power for fully covered bins
					if (x_lower < x && x < (x_upper - 1) && y_lower < y && y < (y_upper - 1)) {
						map[x][y] += block->power;
					}
					// else consider intersection of bin and block at
					// blocks' boundaries
					else {
						intersect = Rect::determineIntersection(bin, block_offset);
						// scale power according to intersection
						// area
						map[x][y] += block->power * (intersect.area / (maps_dim_x * maps_dim_y));
					}
				}
			}
		}

		this->power_maps.push_back(map);
	}

#ifdef DBG_CALLS_THERMAL
	cout << "<- ThermalAnalyzer::generatePowerMaps" << endl;
#endif

}

// Determine masks for lowest layer, i.e., hottest layer.
// Based on a gaussian-like thermal impulse response fuction.
// Note that masks are centered, i.e., the value f(x=0) resides in the middle of the
// (uneven) array.
// Note that masks are 1D, sufficient for the separated convolution in
// performPowerBlurring()
void ThermalAnalyzer::initThermalMasks(const FloorPlanner& fp) {
	int i, ii;
	double range_scale;
	double max_spread;
	double spread;
	double impulse_factor;
	array<double,ThermalAnalyzer::mask_dim> mask;
	int x_y;

#ifdef DBG_CALLS_THERMAL
	cout << "-> ThermalAnalyzer::initThermalMasks(" << &fp << ")" << endl;
#endif

	if (fp.logMed()) {
		cout << "Layout> ";
		cout << "Initializing thermals masks for power blurring ..." << endl;
	}

	// clear masks
	this->thermal_masks.clear();
	this->thermal_masks.reserve(fp.conf_layer);

	// max_spread represents the spreading factor for the widest function g, i.e., relates
	// to mask for point source on layer furthest away
	// TODO vary this parameter
	max_spread = fp.conf_layer;

	// determine range scale factor, i.e. determine spread such that g = 0.01 at
	// boundary corners of kernel
	range_scale = sqrt(max_spread) * sqrt(log(2.0)+log(5.0));
	// normalize range according to mask dimension
	// decrement masks_dim such that subsequent impulse-response calculation
	// determines values for center of each mask bin
	range_scale /=  (ThermalAnalyzer::mask_dim - 1) / 2;

	// determine masks for lowest layer, i.e., hottest layer
	for (i = 1; i <= fp.conf_layer; i++) {
		// TODO vary these calculations
		spread = 1.0 / i;
		impulse_factor = 1.0 / i;

		ii = 0;
		for (x_y = -(ThermalAnalyzer::mask_dim - 1) / 2; x_y <= (ThermalAnalyzer::mask_dim - 1) / 2; x_y++) {
			mask[ii] = Math::gauss1D(x_y * range_scale, impulse_factor, spread);
			ii++;
		}

		this->thermal_masks.push_back(mask);
	}

#ifdef DBG_LAYOUT
	// enforce fixed digit count for printing mask
	cout << fixed;
	// dump mask
	for (i = 0; i < fp.conf_layer; i++) {
		cout << "DBG_LAYOUT> Thermal 1D mask for layer " << i << ":" << endl;
		for (x_y = 0; x_y < ThermalAnalyzer::mask_dim; x_y++) {
			cout << this->thermal_masks[i][x_y] << ", ";
		}
		cout << endl;
	}
	// reset to default floating output
	cout.unsetf(ios_base::floatfield);
#endif

	if (fp.logMed()) {
		cout << "Layout> ";
		cout << "Done" << endl << endl;
	}

#ifdef DBG_CALLS_THERMAL
	cout << "<- ThermalAnalyzer::initThermalMasks" << endl;
#endif

}
