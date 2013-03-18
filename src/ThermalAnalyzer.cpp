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

/// material parameters for thermal 3D-IC simulation using HotSpot
/// Note: properties for heat spread and heat sink also from [Park09] (equal default
/// HotSpot configuration values)
// [Park09]; derived from 700 J/(kg*K) to J/(m^3*K) considering Si density of 2330 kg/m^3
const double ThermalAnalyzer::HEAT_CAPACITY_SI = 1631000.0;
// [Park09]
const double ThermalAnalyzer::THERMAL_RESISTIVITY_SI = 0.008510638;
// [Sridhar10]; derived considering a factor of appr. 1.35 for Si/BEOL heat capacity
const double ThermalAnalyzer::HEAT_CAPACITY_BEOL = 1208150.0;
// [Sridhar10]
const double ThermalAnalyzer::THERMAL_RESISTIVITY_BEOL = 0.4444;
// [Park09]
const double ThermalAnalyzer::HEAT_CAPACITY_BOND = 2298537.0;
// [Park09]
const double ThermalAnalyzer::THERMAL_RESISTIVITY_BOND = 5.0;
// 100um thick dies; own value
const double ThermalAnalyzer::THICKNESS_SI = 0.0001;
// 2um active Si layer; [Sridhar10]
const double ThermalAnalyzer::THICKNESS_SI_ACTIVE = 0.000002;
// 12um BEOL; [Sridhar10]
const double ThermalAnalyzer::THICKNESS_BEOL = 0.000012;
// 20um BCB bond; [Sridhar10]
const double ThermalAnalyzer::THICKNESS_BOND = 0.00002;

// thermal-analyzer routine based on power blurring,
// i.e., convolution of thermals masks and power maps
// returns max value of convoluted 2D matrix
double ThermalAnalyzer::performPowerBlurring(const FloorPlanner& fp, const bool& set_max_cost, const bool& normalize) {
	unsigned i;
	int n;
	int maps_dim;
	int x, y;
	int mask_center_x, mask_center_y;
	int mask_x, mask_y;
	int mask_x_size, mask_y_size;
	int mask_flipped_x, mask_flipped_y;
	int power_x, power_y;
	double max_temp;

	// TODO realize as config parameter / determine considering smallest block
	maps_dim = 64;

	// generate power maps for current layout
	this->generatePowerMaps(fp, maps_dim);

	// init grid of thermal map
	this->thermal_map.clear();
	this->thermal_map.resize(maps_dim);
	for (n = 0; n < maps_dim; n++) {
		this->thermal_map[n].resize(maps_dim, 0.0);
	}

	// determine thermal map for lowest layer, i.e., hottest layer;
	// perform convolution of thermal masks and power maps
	for (i = 0; i < this->thermal_masks.size(); i++) {
		mask_x_size = this->thermal_masks[i].size();
		mask_y_size = this->thermal_masks[i][0].size();

		// determine center index of mask grid
		mask_center_x = mask_x_size / 2;
		mask_center_y = mask_y_size / 2;

		// walk thermal-map grid
		for (x = 0; x < maps_dim; x++) {
			for (y = 0; y < maps_dim; y++) {

				// walk mask grid
				for (mask_x = 0; mask_x < mask_x_size; mask_x++) {
					mask_flipped_x = mask_x_size - mask_x - 1;

					for (mask_y = 0; mask_y < mask_y_size; mask_y++) {
						mask_flipped_y = mask_y_size - mask_y - 1;

						// power bin to consider
						power_x = x + (mask_x - mask_center_x);
						power_y = y + (mask_y - mask_center_y);

						// consider only bins within map bounds
						if (power_x >= 0 && power_x < maps_dim && power_y >= 0 && power_y < maps_dim) {
							// multiply mask bin w/ related
							// power-map bin for convolution
							this->thermal_map[x][y] += this->thermal_masks[i][mask_flipped_x][mask_flipped_y]
								* this->power_maps[i][power_x][power_y];
						}
					}
				}
			}
		}
	}

	// determine max value
	max_temp = 0.0;
	for (x = 0; x < maps_dim; x++) {
		for (y = 0; y < maps_dim; y++) {
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

	return max_temp;
}

void ThermalAnalyzer::generatePowerMaps(const FloorPlanner& fp, const int& maps_dim) {
	int i, n;
	int x, y;
	Block *block;
	double maps_dim_x, maps_dim_y;
	vector<vector<double>> map;
	Rect bin, intersect;
	int x_lower, x_upper, y_lower, y_upper;

	// clear maps
	this->power_maps.clear();
	this->power_maps.reserve(fp.conf_layer);

	// scale map dimensions to outline
	maps_dim_x = fp.conf_outline_x / maps_dim;
	maps_dim_y = fp.conf_outline_y / maps_dim;

	// determine maps for each layer
	for (i = 0; i < fp.conf_layer; i++) {

		// init grid of map
		map.clear();
		map.resize(maps_dim);
		for (n = 0; n < maps_dim; n++) {
			map[n].resize(maps_dim, 0.0);
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
}

// determine masks for lowest layer, i.e., hottest layer
// based on a gaussian-like thermal impulse response fuction
void ThermalAnalyzer::initThermalMasks(const FloorPlanner& fp) {
	int i;
	int masks_dim;
	double range_scale;
	double max_spread;
	double spread;
	double impulse_factor;
	vector<vector<double>> mask;
	vector<double> mask_col;
	int x, y;

	if (fp.logMed()) {
		cout << "Layout> ";
		cout << "Initializing thermals masks for power blurring ..." << endl;
	}

	// clear masks
	this->thermal_masks.clear();
	this->thermal_masks.reserve(fp.conf_layer);

	// TODO vary this parameter; should be uneven
	// TODO realize as config parameter
	masks_dim = 17;

	mask.reserve(masks_dim);
	mask_col.reserve(masks_dim);

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
	range_scale /=  (masks_dim - 1) / 2;

	// determine masks for lowest layer, i.e., hottest layer
	for (i = 1; i <= fp.conf_layer; i++) {
		// TODO vary these calculations
		spread = 1.0 / i;
		impulse_factor = 1.0 / i;

		mask.clear();

		for (x = -(masks_dim - 1) / 2; x <= (masks_dim - 1) / 2; x++) {
			mask_col.clear();

			for (y = -(masks_dim - 1) / 2; y <= (masks_dim - 1) / 2; y++) {
				mask_col.push_back(Math::gauss2D(x * range_scale, y * range_scale, impulse_factor, spread));
			}

			mask.push_back(mask_col);
		}

		this->thermal_masks.push_back(mask);
	}

#ifdef DBG_LAYOUT
	// enforce fixed digit count for printing mask
	cout << fixed;
	// dump mask
	for (i = 0; i < fp.conf_layer; i++) {
		cout << "DBG_LAYOUT> Thermal mask for layer " << i << ":" << endl;
		for (y = masks_dim - 1; y >= 0; y--) {
			for (x = 0; x < masks_dim; x++) {
				cout << this->thermal_masks[i][x][y] << "	";
			}
			cout << endl;
		}
	}
	// reset to default floating output
	cout.unsetf(ios_base::floatfield);
#endif

	if (fp.logMed()) {
		cout << "Layout> ";
		cout << "Done" << endl << endl;
	}

}
