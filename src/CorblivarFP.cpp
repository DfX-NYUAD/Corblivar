/*
 * =====================================================================================
 *
 *    Description:  Corblivar floorplanning file (SA operations and related handler)
 *
 *         Author:  Johann Knechtel, johann.knechtel@ifte.de
 *        Company:  Institute of Electromechanical and Electronic Design, www.ifte.de
 *
 * =====================================================================================
 */
#include "Corblivar.hpp"

bool CorblivarFP::SA(CorblivarLayoutRep &chip) {
	int i, ii;
	int innerLoopMax;
	double accepted_ops_ratio;
	double accepted_ops_ratio_offset;
	double accepted_ops_ratio_boundary_1, accepted_ops_ratio_boundary_2;
	bool annealed;
	bool op_success;
	double cur_cost, best_cost, prev_cost, cost_diff, avg_cost, fitting_cost;
	vector<double> cost_hist;
	double cur_temp, init_temp;
	double r;
	vector<double> init_cost_interconnects;
	bool cur_layout_fits_in_outline;
	int layout_fit_counter;
	double layout_fit_ratio;
	bool valid_layout_found;
	bool accept;

	// init max cost
	this->max_cost_WL = 0.0;
	this->max_cost_TSVs = 0.0;
	this->max_cost_temp = 0.0;
	this->max_cost_IR = 0.0;
	this->max_cost_alignments = 0.0;

	// init SA parameter: inner loop count
	innerLoopMax = this->conf_SA_loopFactor * pow((double) this->blocks.size(), (double) 4/3);

	/// initial solution-space sampling
	/// i.e., outline max cost for various parameters
	//
	if (this->logMed()) {
		cout << "SA> Perform initial solution-space sampling..." << endl;
	}

	// backup initial CBLs
	chip.backupCBLs();

	// perform some random operations, track max costs
	i = 0;
	while (i < SA_SAMPLING_LOOP_FACTOR * innerLoopMax) {

		op_success = this->performRandomLayoutOp(chip);

		if (op_success) {
			// generate layout
			chip.generateLayout(this->conf_log);

			// determine and memorize cost of interconnects
			init_cost_interconnects = this->determCostInterconnects();

			// memorize max cost
			this->max_cost_WL = max(this->max_cost_WL, init_cost_interconnects[0]);
			this->max_cost_TSVs = max(this->max_cost_TSVs, init_cost_interconnects[1]);

			i++;
		}
	}
	// log
	if (this->logMed()) {
		cout << "SA> Cost parameter normalization set up..." << endl;
	}

	// restore initial CBLs
	chip.restoreCBLs();

	// perform some random operations, track cost
	layout_fit_counter = 0;
	i = 1;
	while (i <= SA_SAMPLING_LOOP_FACTOR * innerLoopMax) {

		op_success = this->performRandomLayoutOp(chip);

		if (op_success) {
			// generate layout
			chip.generateLayout(this->conf_log);

			cost_hist.push_back(this->determLayoutCost(cur_layout_fits_in_outline, (double) layout_fit_counter / i));

			// memorize count of solutions fitting into outline
			if (cur_layout_fits_in_outline) {
				layout_fit_counter++;
			}

			i++;
		}
	}

	// restore initial CBLs
	chip.restoreCBLs();

	// init SA parameter: start temp, depends on std dev of costs
	// Huang et al 1986
	init_temp = cur_temp = Math::stdDev(cost_hist);
	if (this->logMed()) {
		cout << "SA> Initial temperature: " << init_temp << endl;
	}

	// perform some random operations, track acceptance ratio for temperature = 0.0
	// i.e., consider only solutions w/ improved cost
	i = 1;
	accepted_ops_ratio = 0.0;
	layout_fit_counter = 0;
	while (i <= SA_SAMPLING_LOOP_FACTOR * innerLoopMax) {

		op_success = this->performRandomLayoutOp(chip);

		if (op_success) {

			// init cost
			if (i == 1) {
				chip.generateLayout(this->conf_log);
				cur_cost = this->determLayoutCost(cur_layout_fits_in_outline, 0.0);
			}

			prev_cost = cur_cost;

			// generate layout
			chip.generateLayout(this->conf_log);

			// evaluate layout, new cost
			cur_cost = this->determLayoutCost(cur_layout_fits_in_outline, (double) layout_fit_counter / i);
			// cost difference
			cost_diff = cur_cost - prev_cost;

			// solution w/ worse cost, revert
			if (cost_diff >= 0.0) {
				// revert last op
				this->performRandomLayoutOp(chip, true);
				// reset cost according to reverted CBL
				cur_cost = prev_cost;
			}
			// accept solution w/ improved cost
			else {
				// update ops count
				accepted_ops_ratio++;
			}

			// memorize count of solutions fitting into outline
			if (cur_layout_fits_in_outline) {
				layout_fit_counter++;
			}

			i++;
		}
	}
	// determine ratio of accepted ops
	accepted_ops_ratio_offset = accepted_ops_ratio / i;
	if (this->logMed()) {
		cout << "SA> Acceptance ratio offset: " << accepted_ops_ratio_offset << endl;
	}

	/// derive related temperature-schedule boundaries
	// upper boundary; for fast cooling
	accepted_ops_ratio_boundary_1 = this->conf_SA_temp_phase_trans_12_factor * accepted_ops_ratio_offset;
	// lower boundary; for slow cooling
	accepted_ops_ratio_boundary_2 = this->conf_SA_temp_phase_trans_23_factor * accepted_ops_ratio_offset;

	if (this->logMed()) {
		cout << "SA> Temperature-update factors (dependent of acceptance ratio r): " << endl;
		cout << "SA>  r > " << accepted_ops_ratio_boundary_1 << ": " << this->conf_SA_temp_factor_phase1 << endl;
		cout << "SA>  " << accepted_ops_ratio_boundary_2 << " < r <= " << accepted_ops_ratio_boundary_1 << ": ";
		cout << this->conf_SA_temp_factor_phase2 << endl;
		// below lower boundary; peform reheating
		cout << "SA>  r <= " << accepted_ops_ratio_boundary_2 << ": " << this->conf_SA_temp_factor_phase3 << endl;
		cout << "SA>   Note: this factor will decrease w/ increasing optimization step" << endl;
	}

	if (this->logMed()) {
		cout << "SA> Done" << endl;
		cout << "SA> Start annealing process..." << endl;
	}

	// restore initial CBLs
	chip.restoreCBLs();

	// init loop parameters
	i = 1;
	annealed = valid_layout_found = false;
	layout_fit_ratio = 0.0;
	// dummy large value to accept first fitting solution
	best_cost = 100.0 * Math::stdDev(cost_hist);

	/// outer loop: annealing -- temperature steps
	while (!annealed) {

		if (this->logMed()) {
			cout << "SA> Optimization step: " << i << "/" << this->conf_SA_loopLimit << endl;
		}

		// init loop parameters
		ii = 1;
		avg_cost = 0.0;
		accepted_ops_ratio = 0.0;
		layout_fit_counter = 0.0;

		// init cost for current layout and fitting ratio
		chip.generateLayout(this->conf_log);
		cur_cost = this->determLayoutCost(cur_layout_fits_in_outline, layout_fit_ratio);

		// inner loop: layout operations
		while (ii <= innerLoopMax) {

			// perform random layout op
			op_success = this->performRandomLayoutOp(chip);
			if (op_success) {

				prev_cost = cur_cost;

				// generate layout
				chip.generateLayout(this->conf_log);

				// evaluate layout, new cost
				cur_cost = this->determLayoutCost(cur_layout_fits_in_outline, layout_fit_ratio);
				// cost difference
				cost_diff = cur_cost - prev_cost;
#ifdef DBG_SA
				cout << "DBG_SA> Inner step: " << ii << "/" << innerLoopMax << endl;
				cout << "DBG_SA> Cost diff: " << cost_diff << endl;
#endif

				// revert solution w/ worse or same cost, depending on temperature
				accept = true;
				if (cost_diff >= 0.0) {
					r = Math::randF01();
					if (r > exp(- cost_diff / cur_temp)) {
#ifdef DBG_SA
						cout << "DBG_SA> Revert op" << endl;
#endif
						accept = false;

						// revert last op
						this->performRandomLayoutOp(chip, true);
						// reset cost according to reverted CBL
						cur_cost = prev_cost;
					}
				}

				// solution accepted
				if (accept) {
					// update ops count
					accepted_ops_ratio++;
					// sum up cost for subsequent avg determination
					avg_cost += cur_cost;

					if (cur_layout_fits_in_outline) {
						// update count of solutions fitting into outline
						layout_fit_counter++;

						// in order to compare different fitting
						// solutions equally, redetermine cost w/
						// fitting ratio 1.0
						fitting_cost = this->determLayoutCost(cur_layout_fits_in_outline, 1.0);

						// memorize best solution which fits into outline
						if (fitting_cost < best_cost) {
							if (this->logMax()) {
								cout << "SA> Currently best solution found; (adapted) cost: " << fitting_cost << endl;
							}

							best_cost = fitting_cost;
							chip.storeBestCBLs();
							valid_layout_found = true;
						}
					}
				}
				// not accepted, but would fit into outline
				else if (cur_layout_fits_in_outline) {
					// update count of solutions fitting into outline
					layout_fit_counter++;
				}

				// consider next loop iteration
				ii++;
			}
		}

		// determine ratio of solutions fitting into outline in prev temp step;
		// note that during the temp step this ratio is fixed in order to avoid
		// sudden changes of related cost terms during few iterations
		layout_fit_ratio = (double) layout_fit_counter / innerLoopMax;

		// determine avg cost for temp step
		avg_cost /= accepted_ops_ratio;
		// determine accepted-ops ratio
		accepted_ops_ratio /= innerLoopMax;

		if (this->logMed()) {
			cout << "SA> Step done:" << endl;
			cout << "SA>  accept-ops ratio: " << accepted_ops_ratio << endl;
			cout << "SA>  valid-layouts ratio: " << layout_fit_ratio << endl;
			cout << "SA>  temp: " << cur_temp << endl;
			cout << "SA>  avg cost: " << avg_cost << endl;
		}

		/// reduce temp
		// phase 1; fast cooling
		if (accepted_ops_ratio > accepted_ops_ratio_boundary_1) {
			cur_temp *= this->conf_SA_temp_factor_phase1;
		}
		// phase 2; slow cooling
		else if (accepted_ops_ratio_boundary_2 < accepted_ops_ratio && accepted_ops_ratio <= accepted_ops_ratio_boundary_1) {
			cur_temp *= this->conf_SA_temp_factor_phase2;
		}
		// phase 3; reheating; accepted_ops_ratio <= accepted_ops_ratio_boundary_2
		// heating-up factor is steadily decreased w/ increasing step count to
		// enable convergence
		else {
			if (valid_layout_found) {
				cur_temp *= this->conf_SA_temp_factor_phase3 * (1.0 - (double) i / this->conf_SA_loopLimit);
			}
			// if no layout was found; heating up is increased exponentially
			else {
				cur_temp *= pow(this->conf_SA_temp_factor_phase3, 2.0) * (1.0 - (double) i / this->conf_SA_loopLimit);
			}
		}

		// consider next step
		i++;
		// consider as annealed if some maximum temperature steps are done
		annealed = (i > this->conf_SA_loopLimit);
	}

	// apply best solution, if available, as final solution
	chip.applyBestCBLs(this->conf_log);
	// generate final layout
	chip.generateLayout(this->conf_log);

	// log results for feasible solution
	if (valid_layout_found && this->logMin()) {
		cout << "SA> Final (adapted) cost: " << best_cost << endl;
		this->results << "Cost: " << best_cost << endl;
		this->results << "CBLs: " << endl;
		this->results << chip.CBLsString() << endl;
	}

	if (this->logMed()) {
		cout << "SA> Done" << endl;
		cout << endl;
	}

	return valid_layout_found;
}

bool CorblivarFP::performRandomLayoutOp(CorblivarLayoutRep &chip, bool revertLastOp) {
	int op;
	int die1, die2, tuple1, tuple2, t;

	// revert last op
	if (revertLastOp) {
		op = this->last_op;
	}
	// perform new, random op
	else {
		// see OP_ constants (encoding ``op-codes'') in class CorblivarLayoutRep
		// to set op-code ranges
		// recall that randI(x,y) is [x,y)
		this->last_op = op = Math::randI(0, 6);
	}

	// specific op handler
	switch (op) {
		case CorblivarLayoutRep::OP_SWAP_BLOCKS_WI_DIE:
			if (!revertLastOp) {
				die1 = Math::randI(0, chip.dies.size());
				// sanity check for dies w/ one or zero tuples
				if (chip.dies[die1]->CBL.size() <= 1) {
					return false;
				}

				tuple1 = Math::randI(0, chip.dies[die1]->CBL.size());
				tuple2 = Math::randI(0, chip.dies[die1]->CBL.size());
				// ensure that tuples are different
				while (tuple1 == tuple2) {
					tuple2 = Math::randI(0, chip.dies[die1]->CBL.size());
				}

				chip.switchBlocksWithinDie(die1, tuple1, tuple2);
			}
			else {
				chip.switchBlocksWithinDie(this->last_op_die1, this->last_op_tuple2, this->last_op_tuple1);
			}

			break;

		case CorblivarLayoutRep::OP_SWAP_BLOCKS_ACROSS_DIE:
			if (!revertLastOp) {
				die1 = Math::randI(0, chip.dies.size());
				die2 = Math::randI(0, chip.dies.size());
				// ensure that dies are different
				while (die1 == die2) {
					die2 = Math::randI(0, chip.dies.size());
				}
				// sanity check for empty dies
				if (chip.dies[die1]->CBL.empty() || chip.dies[die2]->CBL.empty()) {
					return false;
				}

				tuple1 = Math::randI(0, chip.dies[die1]->CBL.size());
				tuple2 = Math::randI(0, chip.dies[die2]->CBL.size());

				chip.switchBlocksAcrossDies(die1, die2, tuple1, tuple2);
			}
			else {
				chip.switchBlocksAcrossDies(this->last_op_die2, this->last_op_die1, this->last_op_tuple2, this->last_op_tuple1);
			}

			break;

		case CorblivarLayoutRep::OP_MOVE_TUPLE:
			if (!revertLastOp) {
				die1 = Math::randI(0, chip.dies.size());
				die2 = Math::randI(0, chip.dies.size());
				// ensure that dies are different
				while (die1 == die2) {
					die2 = Math::randI(0, chip.dies.size());
				}
				// sanity check for empty (origin) die
				if (chip.dies[die1]->CBL.empty()) {
					return false;
				}

				tuple1 = Math::randI(0, chip.dies[die1]->CBL.size());
				tuple2 = Math::randI(0, chip.dies[die2]->CBL.size());

				chip.moveTupleAcrossDies(die1, die2, tuple1, tuple2);
			}
			else {
				chip.moveTupleAcrossDies(this->last_op_die2, this->last_op_die1, this->last_op_tuple2, this->last_op_tuple1);
			}

			break;

		case CorblivarLayoutRep::OP_SWITCH_TUPLE_DIR:
			if (!revertLastOp) {
				die1 = Math::randI(0, chip.dies.size());
				// sanity check for empty dies
				if (chip.dies[die1]->CBL.empty()) {
					return false;
				}

				tuple1 = Math::randI(0, chip.dies[die1]->CBL.size());

				chip.switchTupleDirection(die1, tuple1);
			}
			else {
				chip.switchTupleDirection(this->last_op_die1, this->last_op_tuple1);
			}

			break;

		case CorblivarLayoutRep::OP_SWITCH_TUPLE_JUNCTS:
			if (!revertLastOp) {
				die1 = Math::randI(0, chip.dies.size());
				// sanity check for empty dies
				if (chip.dies[die1]->CBL.empty()) {
					return false;
				}

				tuple1 = Math::randI(0, chip.dies[die1]->CBL.size());
				t = chip.dies[die1]->CBL.T[tuple1];

				this->last_op_juncts = t;

				if (t == 0) {
					t++;
				}
				else {
					if (Math::randB()) {
						t++;
					}
					else {
						t = max(0, t - 1);
					}
				}

				chip.switchTupleJunctions(die1, tuple1, t);
			}
			else {
				chip.switchTupleJunctions(this->last_op_die1, this->last_op_tuple1, this->last_op_juncts);
			}

			break;

		case CorblivarLayoutRep::OP_SWITCH_BLOCK_ORIENT:
			if (!revertLastOp) {
				die1 = Math::randI(0, chip.dies.size());
				// sanity check for empty dies
				if (chip.dies[die1]->CBL.empty()) {
					return false;
				}

				tuple1 = Math::randI(0, chip.dies[die1]->CBL.size());

				chip.switchBlockOrientation(die1, tuple1);
			}
			else {
				chip.switchBlockOrientation(this->last_op_die1, this->last_op_tuple1);
			}

			break;
	}

	// memorize op elements
	this->last_op_die1 = die1;
	this->last_op_die2 = die2;
	this->last_op_tuple1 = tuple1;
	this->last_op_tuple2 = tuple2;

	// op succeeded
	return true;
}

// cost factors must be normalized to their respective max values; i.e., for
// optimized solutions, cost will be significantly smaller than 1
double CorblivarFP::determLayoutCost(bool &layout_fits_in_fixed_outline, double ratio_feasible_solutions_fixed_outline) {
	double cost_total, cost_temp, cost_IR, cost_alignments;
	double cost_area_outline;
	vector<double> cost_interconnects;
	double max_outline_x;
	double max_outline_y;
	Block *cur_block;
	map<int, Block*>::iterator b;
	int i, non_empty_dies;
	vector<double> dies_AR;
	vector<double> dies_outline;

	// cost temperature distribution
	// TODO consider only for layouts fitting into outline
	// TODO max value? initial sampling most likely doesn't fit into outline
	cost_temp = 0.0;
	if (layout_fits_in_fixed_outline) {
		this->determCostThermalDistr();
	}

	// TODO Cost IR
	cost_IR = 0.0;

	// cost interconnects
	cost_interconnects = this->determCostInterconnects();
	// normalize to max value from initial sampling
	cost_interconnects[0] /= this->max_cost_WL;
	cost_interconnects[1] /= this->max_cost_TSVs;

	// TODO Cost (Failed) Alignments
	cost_alignments = 0.0;

	// cost function; cost terms which are determined considering full 3D layout
	cost_total = this->conf_SA_cost_temp * cost_temp
		+ this->conf_SA_cost_WL * cost_interconnects[0]
		+ this->conf_SA_cost_TSVs * cost_interconnects[1]
		+ this->conf_SA_cost_IR * cost_IR
	;

	// cost function; cost terms which are determined considering particular dies
	// i.e., outline, area
	//
	// determine outline and area
	layout_fits_in_fixed_outline = true;
	non_empty_dies = 0;
	for (i = 0; i < this->conf_layer; i++) {
		// determine outline and area for blocks on all dies separately
		max_outline_x = max_outline_y = 0.0;
		for (b = this->blocks.begin(); b != this->blocks.end(); ++b) {
			cur_block = (*b).second;
			if (cur_block->layer == i) {
				// update max outline coords
				max_outline_x = max(max_outline_x, cur_block->bb.ur.x);
				max_outline_y = max(max_outline_y, cur_block->bb.ur.y);
			}
		}
		dies_outline.push_back(max_outline_x * max_outline_y);

		// determine aspect ratio; used to guide optimization for fixed outline (Chen 2006)
		if (max_outline_x > 0.0 && max_outline_y > 0.0) {
			dies_AR.push_back(max_outline_x / max_outline_y);
			non_empty_dies++;
		}
		// dummy value for empty dies; implies cost of 0.0 for this die, i.e. does
		// not impact cost function
		else {
			dies_AR.push_back(this->outline_AR);
		}

		// memorize whether layout fits into outline
		max_outline_x /= this->conf_outline_x;
		max_outline_y /= this->conf_outline_y;
		layout_fits_in_fixed_outline = layout_fits_in_fixed_outline && (max_outline_x <= 1.0 && max_outline_y <= 1.0);
	}

	// determine cost factors
	cost_area_outline = 0.0;
	for (i = 0; i < this->conf_layer; i++) {
		/// adaptive cost model: terms for area and AR mismatch are _mutually_
		/// depending on ratio of feasible solutions (solutions fitting into outline)
		cost_area_outline +=
			// cost term for area
			(0.5 * this->conf_SA_cost_area_outline * (1.0 + ratio_feasible_solutions_fixed_outline)
			 	// consider normalized area (blocks' outline) w.r.t. die outline
				* (dies_outline[i] / (this->conf_outline_x * this->conf_outline_y)))
			// cost term for aspect ratio mismatch
			+ (0.5 * this->conf_SA_cost_area_outline * (1.0 - ratio_feasible_solutions_fixed_outline)
				* pow(dies_AR[i] - this->outline_AR, 2.0))
			;
	}
	// determine average of layer-dependent cost factors
	cost_area_outline /= non_empty_dies;

	// sum up cost function
	cost_total += cost_area_outline;

#ifdef DBG_SA
	cout << "Layout> ";
	cout << "Layout cost: " << cost_total << endl;
#endif

	return cost_total;
}

double CorblivarFP::determCostThermalDistr() {
	unsigned i;
	int n;
	int maps_dim;
	int x, y;
	int mask_center_x, mask_center_y;
	int mask_x, mask_y;
	int mask_x_size, mask_y_size;
	int mask_flipped_x, mask_flipped_y;
	int power_x, power_y;

	// TODO realize as config parameter / determine considering smallest block
	maps_dim = 64;

	// generate power maps for current layout
	this->generatePowerMaps(maps_dim);

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

	// TODO drop
	this->finalize();

	return 0.0;
}

// TODO logging, dbg
void CorblivarFP::generatePowerMaps(int maps_dim) {
	int i, n;
	int x, y;
	map<int, Block*>::iterator block_it;
	Block *block;
	double maps_dim_x, maps_dim_y;
	vector< vector<double> > map;
	Rect bin, intersect;
	int x_lower, x_upper, y_lower, y_upper;

	// clear maps
	this->power_maps.clear();

	// scale map dimensions to outline
	maps_dim_x = this->conf_outline_x / maps_dim;
	maps_dim_y = this->conf_outline_y / maps_dim;

	// determine maps for each layer
	for (i = 0; i < this->conf_layer; i++) {

		// init grid of map
		map.clear();
		map.resize(maps_dim);
		for (n = 0; n < maps_dim; n++) {
			map[n].resize(maps_dim, 0.0);
		}

		// consider each block on the related layer
		for (block_it = this->blocks.begin(); block_it != this->blocks.end(); ++block_it) {
			block = (*block_it).second;

			if (block->layer != i) {
				continue;
			}
			// TODO consider parts of blocks which are w/in outline
			// required to not guide search towards violating outline by
			// moving blocks outside in order to reduce temperature
			// sanity check; ignore blocks outside outline
			if (block->bb.ur.x > this->conf_outline_x || block->bb.ur.y > this->conf_outline_y) {
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

					// determine intersection of grid bin and block
					// TODO only for boundaries required
					intersect = Rect::determineIntersection(bin, block->bb);

					// add block power to bin, scaled accordingly to
					// intersection
					map[x][y] += block->power * (intersect.area / (maps_dim_x * maps_dim_y));
				}
			}
		}

		this->power_maps.push_back(map);
	}
}

// determine masks for lowest layer, i.e., hottest layer
// based on a gaussian-like thermal impulse response fuction
// TODO logging, dbg
void CorblivarFP::initThermalMasks() {
	int i;
	int masks_dim;
	double range_scale;
	double max_spread;
	double spread;
	double impulse_factor;
	vector< vector<double> > mask;
	vector<double> mask_col;
	int x, y;

	// clear masks
	this->thermal_masks.clear();

	// TODO vary this parameter; should be uneven
	// TODO realize as config parameter
	masks_dim = 17;

	// max_spread represents the spreading factor for the widest function g, i.e., relates
	// to mask for point source on layer furthest away
	// TODO vary this parameter
	max_spread = this->conf_layer;

	// determine range scale factor, i.e. determine spread such that g = 0.01 at
	// boundary corners of kernel
	range_scale = sqrt(max_spread) * sqrt(log(2.0)+log(5.0));
	// normalize range according to mask dimension
	// decrement masks_dim such that subsequent impulse-response calculation
	// determines values for center of each mask bin
	range_scale /=  (masks_dim - 1) / 2;

	// determine masks for lowest layer, i.e., hottest layer
	for (i = 1; i <= this->conf_layer; i++) {
		// TODO vary these calculations
		spread = 1.0 / i;
		impulse_factor = 1.0 / i;

		mask.clear();

		for (x = -(masks_dim - 1) / 2; x <= (masks_dim - 1) / 2; x++) {
			mask_col.clear();

			for (y = -(masks_dim - 1) / 2; y <= (masks_dim - 1) / 2; y++) {
				mask_col.push_back(Math::gaussImpulseResponse(x * range_scale, y * range_scale, impulse_factor, spread));
			}

			mask.push_back(mask_col);
		}

		this->thermal_masks.push_back(mask);
	}

#ifdef DBG_SA
	// enforce fixed digit count for printing mask
	cout << fixed;
	// dump mask
	for (i = 0; i < this->conf_layer; i++) {
		cout << "DBG_SA> Thermal mask for layer " << i << ":" << endl;
		for (y = masks_dim - 1; y >= 0; y--) {
			for (x = 0; x < masks_dim; x++) {
				cout << masks[i][x][y] << "	";
			}
			cout << endl;
		}
	}
	// reset to default floating output
	cout.unsetf(ios_base::floatfield);
#endif

}

// return[0]: HPWL
// return[1]: TSVs
// TODO recode; currently hotspot
vector<double> CorblivarFP::determCostInterconnects() {
	unsigned n, b;
	int i, ii;
	double HPWL;
	int TSVs;
	Net *cur_net;
	vector<Rect> blocks_to_consider;
	Rect bb;
	bool blocks_above_considered;

	HPWL = 0.0;
	TSVs = 0;
	vector<double> ret;

	// determine HPWL and TSVs for each net
	for (n = 0; n < this->nets.size(); n++) {
		cur_net = this->nets[n];

		// determine HPWL on each layer separately
		for (i = 0; i < this->conf_layer; i++) {
#ifdef DBG_SA
			cout << "DBG_SA> Determine interconnects for net " << cur_net->id << " on layer " << i << " and above" << endl;
#endif

			// consider all related blocks:
			// blocks on this layer and blocks on layer above --- thus we
			// include TSVs in HPWL estimate assuming they are subsequently
			// placed in the related bounding box
			blocks_to_consider.clear();

			// blocks on current layer
			for (b = 0; b < cur_net->blocks.size(); b++) {
				if (cur_net->blocks[b]->layer == i) {
					blocks_to_consider.push_back(cur_net->blocks[b]->bb);
#ifdef DBG_SA
					cout << "DBG_SA> 	Consider block " << cur_net->blocks[b]->id << " on layer " << i << endl;
#endif
				}
			}
			// ignore cases where no blocks on current layer (no blocks
			// require connecting to upper layers)
			if (blocks_to_consider.empty()) {
				continue;
			}

			// blocks on layer above, not necessarily adjacent
			// thus stepwise consider upper layers until some blocks are found
			blocks_above_considered = false;
			ii = i + 1;
			while (true) {
				for (b = 0; b < cur_net->blocks.size(); b++) {
					if (cur_net->blocks[b]->layer == ii) {
						blocks_to_consider.push_back(cur_net->blocks[b]->bb);
						blocks_above_considered = true;
#ifdef DBG_SA
						cout << "DBG_SA> 	Consider block " << cur_net->blocks[b]->id << " on layer " << ii << endl;
#endif
					}
				}

				// loop handler
				if (blocks_above_considered) {
					break;
				}
				else {
					if (ii == this->conf_layer) {
						break;
					}
					else {
						ii++;
					}
				}
			}
			// ignore cases where only one block needs to be considered; these
			// cases (single blocks on uppermost layer) are already covered
			// while considering layers below
			if (blocks_to_consider.size() == 1) {
#ifdef DBG_SA
				cout << "DBG_SA> 	Ignore single block on uppermost layer" << endl;
#endif
				continue;
			}

			// update TSVs counter if connecting to blocks on some upper layer
			if (blocks_above_considered) {
				TSVs += (ii - i);
#ifdef DBG_SA
				cout << "DBG_SA> 	TSVs required: " << (ii - i) << endl;
#endif
			}

			// determine HPWL of related blocks using their bounding box
			bb = Rect::determBoundingBox(blocks_to_consider);
			HPWL += bb.w;
			HPWL += bb.h;
#ifdef DBG_SA
			cout << "DBG_SA> 	HPWL of bounding box of blocks to consider: " << (bb.w + bb. h) << endl;
#endif
		}
	}

	ret.push_back(HPWL);
	ret.push_back(TSVs);

	return ret;
}
