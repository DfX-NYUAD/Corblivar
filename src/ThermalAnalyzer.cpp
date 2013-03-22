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

// Thermal-analyzer routine based on power blurring,
// i.e., convolution of thermals masks and power maps into thermal maps.
// Based on a separated convolution using separated 2D gauss function, i.e., 1D gauss fct.
// Returns max value of thermal map of lowest layer, i.e., hottest layer
// Based on http://www.songho.ca/dsp/convolution/convolution.html#separable_convolution
double ThermalAnalyzer::performPowerBlurring(const FloorPlanner& fp, const bool& set_max_cost, const bool& normalize) const {
	int layer;
	int x, y, i;
	int mask_i;
	int mask_center;
	double max_temp;
	// required as buffer for separated convolution
	array<array<double,ThermalAnalyzer::maps_dim>,ThermalAnalyzer::maps_dim> thermal_map_tmp;

#ifdef DBG_CALLS_THERMAL
	cout << "-> ThermalAnalyzer::performPowerBlurring(" << &fp << ", " << set_max_cost << ", " << normalize << ")" << endl;
#endif

	// center index for uneven mask
	mask_center = ThermalAnalyzer::mask_dim / 2;

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

		// walk map grid for horizontal convolution; store into thermal_map_tmp
		// TODO consider padded power map, i.e., power map is larger than
		// thermal-map to generate; drops boundary checks
		for (y = 0; y < ThermalAnalyzer::maps_dim; y++) {
			for (x = 0; x < ThermalAnalyzer::maps_dim; x++) {

				// perform horizontal 1D convolution, i.e., multiply
				// input[x] w/ mask
				//
				// e.g., for x = 0, mask_dim = 3
				// convol1D(x=0) = input[-1] * mask[0] + input[0] * mask[1] + input[1] * mask[2]
				//
				// can be also represented by aligning both arrays:
				// input array (power map):
				// |x=-1|x=0|x=1|x=2|
				// mask:
				// |i=0 |y=1|y=2|
				//
				for (mask_i = 0; mask_i < ThermalAnalyzer::mask_dim; mask_i++) {

					// determine power-map index
					i = x + (mask_i - mask_center);

					// TODO drop after padding power map
					if (i >= 0 && i < ThermalAnalyzer::maps_dim) {
						thermal_map_tmp[x][y] += this->power_maps[layer][i][y] * this->thermal_masks[layer][mask_i];
					}
				}
			}
		}
	}

	// continue w/ vertical convolution
	// TODO replace fp.conf_layer w/ fct parameter
	for (layer = 0; layer < fp.conf_layer; layer++) {

		// walk map grid for horizontal convolution; use data from thermal_map_tmp
		// and store into thermal_map
		// TODO consider padded power map, i.e., power map is larger than
		// thermal-map to generate; drops boundary checks
		for (x = 0; x < ThermalAnalyzer::maps_dim; x++) {
			for (y = 0; y < ThermalAnalyzer::maps_dim; y++) {

				// perform 1D vertical convolution
				for (mask_i = 0; mask_i < ThermalAnalyzer::mask_dim; mask_i++) {

					// determine power-map index
					i = y + (mask_i - mask_center);

					// TODO drop after padding power map
					if (i >= 0 && i < ThermalAnalyzer::maps_dim) {
						this->thermal_map[x][y] += thermal_map_tmp[x][i] * this->thermal_masks[layer][mask_i];
					}
				}
			}
		}
	}

	// determine max value
	max_temp = 0.0;
	for (x = 0; x < ThermalAnalyzer::maps_dim; x++) {
		for (y = 0; y < ThermalAnalyzer::maps_dim; y++) {
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
	array<array<double,ThermalAnalyzer::maps_dim>,ThermalAnalyzer::maps_dim> map;
	Rect bin, intersect;
	int x_lower, x_upper, y_lower, y_upper;

#ifdef DBG_CALLS_THERMAL
	cout << "-> ThermalAnalyzer::generatePowerMaps(" << &fp << ")" << endl;
#endif

	// clear maps
	this->power_maps.clear();
	this->power_maps.reserve(fp.conf_layer);

	// scale map dimensions to outline
	maps_dim_x = fp.conf_outline_x / ThermalAnalyzer::maps_dim;
	maps_dim_y = fp.conf_outline_y / ThermalAnalyzer::maps_dim;

	// determine maps for each layer
	for (i = 0; i < fp.conf_layer; i++) {

		// init map to zero
		for (auto& m : map) {
			m.fill(0.0);
		}

		// consider each block on the related layer
		for (auto& b : fp.blocks) {
			block = b.second;

			if (block->layer != i) {
				continue;
			}
			// TODO consider parts of blocks which are w/in outline
			// required to not guide search towards violating outline by
			// moving blocks outside in order to reduce temperature
			// sanity check; ignore blocks outside outline
			if (block->bb.ur.x > fp.conf_outline_x || block->bb.ur.y > fp.conf_outline_y) {
				continue;
			}

			// determine grid index boundaries for block
			x_lower = floor(block->bb.ll.x / maps_dim_x);
			x_upper = ceil(block->bb.ur.x / maps_dim_x);
			y_lower = floor(block->bb.ll.y / maps_dim_y);
			y_upper = ceil(block->bb.ur.y / maps_dim_y);

			// walk grid bins covering block outline
			for (x = x_lower; x < x_upper; x++) {
				for (y = y_lower; y < y_upper; y++) {

					// determine real coords of abstract grid bin
					bin.ll.x = x * maps_dim_x;
					bin.ur.x = bin.ll.x + maps_dim_x;
					bin.ll.y = y * maps_dim_y;
					bin.ur.y = bin.ll.y + maps_dim_y;

					// sum up block power in fully covered grid bins
					if (x_lower < x && x < (x_upper - 1) && y_lower < y && y < (y_upper - 1)) {
						map[x][y] += block->power;
					}
					// consider intersection of bin and block at
					// blocks' boundaries
					else {
						intersect = Rect::determineIntersection(bin, block->bb);
						// scale power according to intersection
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
