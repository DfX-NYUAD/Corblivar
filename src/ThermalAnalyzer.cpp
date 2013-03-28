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
#include "ThermalAnalyzer.hpp"

// Thermal-analyzer routine based on power blurring,
// i.e., convolution of thermals masks and power maps into thermal maps.
// Based on a separated convolution using separated 2D gauss function, i.e., 1D gauss fct.
// Returns max value of thermal map of lowest layer, i.e., hottest layer
// Based on http://www.songho.ca/dsp/convolution/convolution.html#separable_convolution
double ThermalAnalyzer::performPowerBlurring(int const& layers, double& max_cost_temp, bool const& set_max_cost, bool const& normalize) const {
	int layer;
	int x, y, i;
	int map_x, map_y;
	int mask_i;
	double max_temp;
	// required as buffer for separated convolution; note that its dimensions
	// corresponds to a power map, which is required to hold temporary results for 1D
	// convolution of padded power maps
	array<array<double,ThermalAnalyzer::power_maps_dim>,ThermalAnalyzer::power_maps_dim> thermal_map_tmp;

	if (ThermalAnalyzer::DBG_CALLS) {
		cout << "-> ThermalAnalyzer::performPowerBlurring(" << layers << ", " << max_cost_temp << ", ";
		cout << set_max_cost << ", " << normalize << ")" << endl;
	}

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
	for (layer = 0; layer < layers; layer++) {

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

					if (ThermalAnalyzer::DBG) {
						cout << "y=" << y << ", x=" << x << ", mask_i=" << mask_i << ", i=" << i << endl;
						if (i < 0 || i >= ThermalAnalyzer::power_maps_dim) {
							cout << "i out of range (should be limited by x)" << endl;
						}
					}

					// convolution; multplication of mask element and
					// power-map bin
					thermal_map_tmp[x][y] += this->power_maps[layer][i][y] * this->thermal_masks[layer][mask_i];
				}
			}
		}
	}

	// continue w/ vertical convolution
	for (layer = 0; layer < layers; layer++) {

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

					if (ThermalAnalyzer::DBG) {
						cout << "x=" << x << ", y=" << y << ", map_x=" << map_x << ", map_y=" << map_y;
						cout << ", mask_i=" << mask_i << ", i=" << i << endl;
						if (i < 0 || i >= ThermalAnalyzer::power_maps_dim) {
							cout << "i out of range (should be limited by y)" << endl;
						}
					}

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
		max_cost_temp = max_temp;
	}

	// normalize to max value from initial sampling
	if (normalize) {
		max_temp /= max_cost_temp;
	}

	if (ThermalAnalyzer::DBG_CALLS) {
		cout << "<- ThermalAnalyzer::performPowerBlurring : " << max_temp << endl;
	}

	return max_temp;
}

void ThermalAnalyzer::generatePowerMaps(int const& layers, map<int, Block*> const& blocks) const {
	int i;
	int x, y;
	Block* block;
	Rect bin, intersect, block_offset;
	int x_lower, x_upper, y_lower, y_upper;

	if (ThermalAnalyzer::DBG_CALLS) {
		cout << "-> ThermalAnalyzer::generatePowerMaps(" << layers << ", " << &blocks << ")" << endl;
	}

	// determine maps for each layer
	for (i = 0; i < layers; i++) {

		// reset map to zero
		// note: this also implicitly pads the map w/ zero
		for (auto& m : this->power_maps[i]) {
			m.fill(0.0);
		}

		// consider each block on the related layer
		for (auto& b : blocks) {
			block = b.second;

			// drop blocks assigned to other layers
			if (block->layer != i) {
				continue;
			}

			// determine offset block bb; relates to block's bb in padded
			// power map
			block_offset = block->bb;
			block_offset.ll.x += this->offset_x;
			block_offset.ur.x += this->offset_x;
			block_offset.ll.y += this->offset_y;
			block_offset.ur.y += this->offset_y;

			// determine index boundaries for block
			x_lower = (int) (block_offset.ll.x / this->power_maps_dim_x);
			x_upper = (int) (block_offset.ur.x / this->power_maps_dim_x) + 1;
			y_lower = (int) (block_offset.ll.y / this->power_maps_dim_y);
			y_upper = (int) (block_offset.ur.y / this->power_maps_dim_y) + 1;
			// limit boundaries; restricts mapping of blocks' power to power
			// map according to offset thermal-map dimensions
			x_upper = min(x_upper, ThermalAnalyzer::thermal_map_dim + ThermalAnalyzer::mask_dim_half);
			y_upper = min(y_upper, ThermalAnalyzer::thermal_map_dim + ThermalAnalyzer::mask_dim_half);

			// walk power-map bins covering block outline
			for (x = x_lower; x < x_upper; x++) {
				for (y = y_lower; y < y_upper; y++) {

					// determine real coords of map bin
					bin.ll.x = this->power_maps_bins_ll_x[x];
					bin.ur.x = this->power_maps_bins_ll_x[x + 1];
					bin.ll.y = this->power_maps_bins_ll_y[y];
					bin.ur.y = this->power_maps_bins_ll_y[y + 1];

					// consider total block power for fully covered bins
					if (x_lower < x && x < (x_upper - 1) && y_lower < y && y < (y_upper - 1)) {
						this->power_maps[i][x][y] += block->power;
					}
					// else consider intersection of bin and block at
					// blocks' boundaries
					else {
						intersect = Rect::determineIntersection(bin, block_offset);
						// scale power according to intersection
						// area
						this->power_maps[i][x][y] += block->power * (intersect.area / this->power_maps_bin_area);
					}
				}
			}
		}
	}

	if (ThermalAnalyzer::DBG_CALLS) {
		cout << "<- ThermalAnalyzer::generatePowerMaps" << endl;
	}
}

void ThermalAnalyzer::initPowerMaps(int const& layers, double const& outline_x, double const& outline_y) {
	array<array<double,ThermalAnalyzer::power_maps_dim>,ThermalAnalyzer::power_maps_dim> map;
	unsigned b;
	int i;

	if (ThermalAnalyzer::DBG_CALLS) {
		cout << "-> ThermalAnalyzer::initPowerMaps(" << outline_x << ", " << outline_y << ")" << endl;
	}

	// init power maps
	this->power_maps.clear();
	this->power_maps.reserve(layers);
	// init temp map to zero
	for (auto& m : map) {
		m.fill(0.0);
	}
	// push back temp map in order to allocate power-maps memory
	for (i = 0; i < layers; i++) {
		this->power_maps.push_back(map);
	}

	// scale power map dimensions to outline of thermal map; this way the padding of
	// power maps doesn't distort the block outlines in the thermal map
	this->power_maps_dim_x = outline_x / ThermalAnalyzer::thermal_map_dim;
	this->power_maps_dim_y = outline_y / ThermalAnalyzer::thermal_map_dim;

	// determine offset for blocks, related to padding of power maps
	this->offset_x = (outline_x / ThermalAnalyzer::power_maps_dim) * ThermalAnalyzer::mask_dim_half;
	this->offset_y = (outline_y / ThermalAnalyzer::power_maps_dim) * ThermalAnalyzer::mask_dim_half;

	// predetermine map bins' area and lower-left corner coordinates
	this->power_maps_bin_area = this->power_maps_dim_x * this->power_maps_dim_y;
	for (b = 0; b < this->power_maps_bins_ll_x.size(); b++) {
		this->power_maps_bins_ll_x[b] = b * this->power_maps_dim_x;
	}
	for (b = 0; b < this->power_maps_bins_ll_y.size(); b++) {
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
	array<double,ThermalAnalyzer::mask_dim> mask;
	int x_y;

	if (ThermalAnalyzer::DBG_CALLS) {
		cout << "-> ThermalAnalyzer::initThermalMasks(" << layers << ", " << log << ")" << endl;
	}

	if (log) {
		cout << "Layout> ";
		cout << "Initializing thermals masks for power blurring ..." << endl;
	}

	// clear masks
	this->thermal_masks.clear();
	this->thermal_masks.reserve(layers);

	// determine scale factor such that mask_boundary_value is reached at the boundary
	// of the lowermost (2D) mask; based on general equation to determine x=y for
	// gauss2D such that gauss2D(x=y) = mask_boundary_value; constant spread (e.g.,
	// 1.0) is sufficient since this function fitting only requires two paramaters,
	// i.e., varying spread has no impact
	static constexpr double spread = 1.0;
	scale = sqrt(spread * std::log(parameters.impulse_factor / (parameters.mask_boundary_value))) / sqrt(2.0);
	// normalize factor according to half of mask dimension
	scale /=  ThermalAnalyzer::mask_dim_half;

	// determine masks for lowest layer, i.e., hottest layer
	for (i = 1; i <= layers; i++) {
		// impuls factor is to be reduced notably for increasing layer count
		layer_impulse_factor = parameters.impulse_factor / pow(i, parameters.impulse_factor_scaling_exponent);

		ii = 0;
		for (x_y = -ThermalAnalyzer::mask_dim_half; x_y <= ThermalAnalyzer::mask_dim_half; x_y++) {
			// sqrt for impulse factor is mandatory since the mask is used for
			// separated convolution (i.e., factor will be squared in final
			// convolution result)
			mask[ii] = Math::gauss1D(x_y * scale, sqrt(layer_impulse_factor), spread);
			ii++;
		}

		this->thermal_masks.push_back(mask);
	}

	if (ThermalAnalyzer::DBG) {
		// enforce fixed digit count for printing mask
		cout << fixed;
		// dump mask
		for (i = 0; i < layers; i++) {
			cout << "DBG> Thermal 1D mask for layer 0 w/ point source on layer " << i << ":" << endl;
			for (x_y = 0; x_y < ThermalAnalyzer::mask_dim; x_y++) {
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
		cout << "Layout> ";
		cout << "Done" << endl << endl;
	}

	if (ThermalAnalyzer::DBG_CALLS) {
		cout << "<- ThermalAnalyzer::initThermalMasks" << endl;
	}
}
