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

// main handler
bool FloorPlanner::performSA(const CorblivarCore& corb) {
	int i, ii;
	int innerLoopMax;
	double accepted_ops_ratio;
	double accepted_ops_ratio_offset;
	double accepted_ops_ratio_boundary_1, accepted_ops_ratio_boundary_2;
	bool op_success;
	double cur_cost, best_cost, prev_cost, cost_diff, avg_cost, fitting_cost;
	Cost cost;
	vector<double> cost_hist;
	double cur_temp, init_temp;
	double r;
	vector<double> init_cost_interconnects;
	int layout_fit_counter;
	double layout_fit_ratio;
	bool valid_layout_found;
	bool accept;
	bool phase_two, phase_two_transit;
	double loop_factor, reheat_factor;

	// reset max cost
	this->max_cost_WL = 0.0;
	this->max_cost_TSVs = 0.0;
	this->max_cost_temp = 0.0;
	this->max_cost_alignments = 0.0;

	// backup initial CBLs
	corb.backupCBLs();

	// init SA parameter: inner loop count
	innerLoopMax = this->conf_SA_loopFactor * pow((double) this->blocks.size(), (double) 4/3);

	/// initial sampling
	//
	if (this->logMed()) {
		cout << "SA> Perform initial solution-space sampling..." << endl;
	}

	// init cost
	corb.generateLayout();
	cur_cost = this->determCost().cost;

	// perform some random operations, for SA temperature = 0.0
	// i.e., consider only solutions w/ improved cost
	// track acceptance ratio and cost (phase one, area and AR mismatch)
	i = 1;
	accepted_ops_ratio = 0.0;
	layout_fit_counter = 0;
	cost_hist.reserve(SA_SAMPLING_LOOP_FACTOR * innerLoopMax);

	while (i <= SA_SAMPLING_LOOP_FACTOR * innerLoopMax) {

		op_success = this->performRandomLayoutOp(corb);

		if (op_success) {

			prev_cost = cur_cost;

			// generate layout
			corb.generateLayout();

			// evaluate layout, new cost
			cost = this->determCost((double) layout_fit_counter / (SA_SAMPLING_LOOP_FACTOR * innerLoopMax));
			cur_cost = cost.cost;
			// cost difference
			cost_diff = cur_cost - prev_cost;

			// solution w/ worse cost, revert
			if (cost_diff >= 0.0) {
				// revert last op
				this->performRandomLayoutOp(corb, true);
				// reset cost according to reverted CBL
				cur_cost = prev_cost;
			}
			// accept solution w/ improved cost
			else {
				// update ops count
				accepted_ops_ratio++;
			}
			// store cost
			cost_hist.push_back(cur_cost);

			// memorize count of solutions fitting into outline
			if (cost.fits_fixed_outline) {
				layout_fit_counter++;
			}

			i++;
		}
	}

	// init SA parameter: start temp, depends on std dev of costs [Huan86, see
	// Shahookar91]
	init_temp = cur_temp = Math::stdDev(cost_hist) * SA_INIT_TEMP_FACTOR;
	if (this->logMax()) {
		cout << "SA> Initial temperature: " << init_temp << endl;
	}

	// determine ratio of accepted ops
	accepted_ops_ratio_offset = accepted_ops_ratio / i;
	if (this->logMax()) {
		cout << "SA> Acceptance ratio offset: " << accepted_ops_ratio_offset << endl;
	}

	/// derive related temperature-schedule boundaries
	// upper boundary; for fast cooling
	accepted_ops_ratio_boundary_1 = this->conf_SA_temp_phase_trans_12_factor * accepted_ops_ratio_offset;
	// lower boundary; for slow cooling
	accepted_ops_ratio_boundary_2 = this->conf_SA_temp_phase_trans_23_factor * accepted_ops_ratio_offset;

	if (this->logMax()) {
		cout << "SA> Temperature-scaling factors (dependent of acceptance ratio r): " << endl;
		cout << "SA>  r > " << accepted_ops_ratio_boundary_1 << ": " << this->conf_SA_temp_factor_phase1 << endl;
		cout << "SA>  " << accepted_ops_ratio_boundary_2 << " < r <= " << accepted_ops_ratio_boundary_1 << ": ";
		cout << this->conf_SA_temp_factor_phase2 << endl;
		// below lower boundary; peform reheating
		cout << "SA>  r <= " << accepted_ops_ratio_boundary_2 << ": " << this->conf_SA_temp_factor_phase3 << endl;
		cout << "SA>   Note: this ``reheating'' factor will decrease w/ increasing optimization step" << endl;
	}

	if (this->logMed()) {
		cout << "SA> Done" << endl;
		cout << "SA> " << endl;
		cout << "SA> Perform simulated annealing process..." << endl;
		cout << "SA> Phase I: packing blocks into outline..." << endl;
	}
	if (this->logMax()) {
		cout << "SA> " << endl;
	}

	// restore initial CBLs
	corb.restoreCBLs();

	/// main SA loop
	//
	// init loop parameters
	i = 1;
	phase_two = phase_two_transit = false;
	valid_layout_found = false;
	layout_fit_ratio = 0.0;
	// dummy large value to accept first fitting solution
	best_cost = 100.0 * Math::stdDev(cost_hist);

	/// outer loop: annealing -- temperature steps
	while (i <= this->conf_SA_loopLimit) {

		if (this->logMax()) {
			cout << "SA> Optimization step: " << i << "/" << this->conf_SA_loopLimit << endl;
		}

		// init loop parameters
		ii = 1;
		avg_cost = 0.0;
		accepted_ops_ratio = 0.0;
		layout_fit_counter = 0.0;
		phase_two_transit = false;

		// init cost for current layout and fitting ratio
		corb.generateLayout();
		cur_cost = this->determCost(layout_fit_ratio, phase_two).cost;

		// inner loop: layout operations
		while (ii <= innerLoopMax) {

			// perform random layout op
			op_success = this->performRandomLayoutOp(corb);

			if (op_success) {

				prev_cost = cur_cost;

				// generate layout
				corb.generateLayout();

				// evaluate layout, new cost
				cost = this->determCost(layout_fit_ratio, phase_two);
				cur_cost = cost.cost;
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
						this->performRandomLayoutOp(corb, true);
						// reset cost according to reverted CBL
						cur_cost = prev_cost;
					}
				}

				// solution to be accepted, i.e., previously not reverted
				if (accept) {
					// update ops count
					accepted_ops_ratio++;
					// sum up cost for subsequent avg determination
					avg_cost += cur_cost;

					if (cost.fits_fixed_outline) {
						// update count of solutions fitting into outline
						layout_fit_counter++;

						// switch to SA phase two when
						// first fitting solution is found
						if (!phase_two) {
							phase_two = phase_two_transit = true;

							if (this->logMax()) {
								cout << "SA> " << endl;
							}
							if (this->logMed()) {
								cout << "SA> Phase II: optimizing within outline; switch cost function ..." << endl;
							}
							if (this->logMax()) {
								cout << "SA> " << endl;
							}
						}

						// in order to compare different fitting
						// solutions equally, redetermine cost w/
						// fitting ratio 1.0
						//
						// during switch to phase two, initialize
						// current cost as max cost for further
						// normalization (phase_two_transit)
						fitting_cost = this->determCost(1.0, phase_two, phase_two_transit).cost;

						// memorize best solution which fits into outline
						if (fitting_cost < best_cost) {
							if (this->logMax()) {
								cout << "SA> Currently best solution found; (adapted) cost: " << fitting_cost << endl;
							}

							best_cost = fitting_cost;
							corb.storeBestCBLs();
							valid_layout_found = true;
						}
					}
				}
				// not accepted, but would fit into outline
				else if (cost.fits_fixed_outline) {
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
		layout_fit_ratio = (double) layout_fit_counter / ii;

		// determine avg cost for temp step
		avg_cost /= accepted_ops_ratio;
		// determine accepted-ops ratio
		accepted_ops_ratio /= ii;

		if (this->logMax()) {
			cout << "SA> Step done:" << endl;
			cout << "SA>  accept-ops ratio: " << accepted_ops_ratio << endl;
			cout << "SA>  valid-layouts ratio: " << layout_fit_ratio << endl;
			cout << "SA>  avg cost: " << avg_cost << endl;
			cout << "SA>  temp: " << cur_temp << endl;
		}

		/// reduce temp
		// phase 1; fast cooling
		if (accepted_ops_ratio > accepted_ops_ratio_boundary_1) {
			cur_temp *= this->conf_SA_temp_factor_phase1;

			if (this->logMax()) {
				cout << "SA>  temp factor: " << this->conf_SA_temp_factor_phase1 << " (phase 1)" << endl;
			}
		}
		// phase 2; slow cooling
		else if (accepted_ops_ratio_boundary_2 < accepted_ops_ratio && accepted_ops_ratio <= accepted_ops_ratio_boundary_1) {
			cur_temp *= this->conf_SA_temp_factor_phase2;

			if (this->logMax()) {
				cout << "SA>  temp factor: " << this->conf_SA_temp_factor_phase2 << " (phase 2)" << endl;
			}
		}
		// phase 3; reheating; accepted_ops_ratio <= accepted_ops_ratio_boundary_2
		// heating-up factor is steadily decreased w/ increasing step count to
		// enable convergence
		else {
			loop_factor = 1.0 - (double) i / this->conf_SA_loopLimit;

			if (valid_layout_found) {
				reheat_factor = this->conf_SA_temp_factor_phase3;

				cur_temp *= loop_factor * reheat_factor;
			}
			// if no layout was found; heating up is increased exponentially
			else {
				reheat_factor = pow(this->conf_SA_temp_factor_phase3, 2.0);

				cur_temp *= loop_factor * reheat_factor;
			}

			if (this->logMax()) {
				cout << "SA>  temp factor: " << loop_factor * reheat_factor << " (phase 3)" << endl;
			}
		}

		// consider next outer step
		i++;
	}

	if (this->logMed()) {
		cout << "SA> Done" << endl;
		cout << endl;
	}

	return valid_layout_found;
}

void FloorPlanner::finalize(const CorblivarCore& corb) {
	struct timeb end;
	stringstream runtime;
	bool valid_solution;
	double cost;
	double area, temp;
	CostInterconn interconn;

	// apply best solution, if available, as final solution
	valid_solution = corb.applyBestCBLs(this->logMin());
	// generate final layout
	corb.generateLayout();

	// determine cost for valid solutions
	if (valid_solution) {

		// determine overall cost
		cost = this->determCost(1.0, true).cost;

		// determine area cost, invert weight
		area = (1.0 / this->conf_SA_cost_area_outline) * this->determCostAreaOutline(1.0).cost;

		// determine non-normalized WL and TSVs cost
		interconn = this->determCostInterconnects(false, false);

		// determine non-normalized temperature cost
		temp = this->determCostThermalDistr(false, false);

		// TODO alignment costs
		if (this->logMin()) {
			cout << "SA> Final (adapted) cost: " << cost << endl;
			cout << "SA>  Max blocks-outline / die-outline ratio: " << area << endl;
			cout << "SA>  HPWL: " << interconn.HPWL << endl;
			cout << "SA>  TSVs: " << interconn.TSVs << endl;
			cout << "SA>  Temp cost (no real temp): " << temp << endl;
			cout << endl;
			this->results << "Final (adapted) cost: " << cost << endl;
			this->results << " Max die occupation [\%]: " << area << endl;
			this->results << " HPWL: " << interconn.HPWL << endl;
			this->results << " TSVs: " << interconn.TSVs << endl;
			this->results << " Temp cost (no real temp): " << temp << endl;
		}
	}

	// generate floorplan plots
	IO::writeFloorplanGP(*this);

	// generate Corblivar date if solution file is used as output
	if (this->solution_out.is_open()) {
		this->solution_out << corb.CBLsString() << endl;
		this->solution_out.close();
	}

	// thermal-analysis files
	if (valid_solution) {
		// generate power and thermal maps
		IO::writePowerThermalMaps(*this);
		// generate HotSpot files
		IO::writeHotSpotFiles(*this);
	}

	// determine overall runtime
	ftime(&end);
	if (this->logMin()) {
		runtime << "Runtime: " << (1000.0 * (end.time - this->start.time) + (end.millitm - this->start.millitm)) / 1000.0 << " s";
		cout << "Corblivar> " << runtime.str() << endl;
		this->results << runtime.str() << endl;
	}

	// close results file
	this->results.close();

	exit(0);
}

bool FloorPlanner::performRandomLayoutOp(const CorblivarCore& corb, const bool& revertLastOp) {
	int op;
	int die1, die2, tuple1, tuple2, t;

	// revert last op
	if (revertLastOp) {
		op = this->last_op;
	}
	// perform new, random op
	else {
		// see OP_ constants (encoding ``op-codes'') in class CorblivarCore
		// to set op-code ranges
		// recall that randI(x,y) is [x,y)
		this->last_op = op = Math::randI(0, 6);
	}

	// specific op handler
	switch (op) {
		case CorblivarCore::OP_SWAP_BLOCKS_WI_DIE:
			if (!revertLastOp) {
				die1 = Math::randI(0, corb.dies.size());
				// sanity check for dies w/ one or zero tuples
				if (corb.dies[die1]->CBL.size() <= 1) {
					return false;
				}

				tuple1 = Math::randI(0, corb.dies[die1]->CBL.size());
				tuple2 = Math::randI(0, corb.dies[die1]->CBL.size());
				// ensure that tuples are different
				while (tuple1 == tuple2) {
					tuple2 = Math::randI(0, corb.dies[die1]->CBL.size());
				}

				corb.switchBlocksWithinDie(die1, tuple1, tuple2);
			}
			else {
				corb.switchBlocksWithinDie(this->last_op_die1, this->last_op_tuple2, this->last_op_tuple1);
			}

			break;

		case CorblivarCore::OP_SWAP_BLOCKS_ACROSS_DIE:
			if (!revertLastOp) {
				die1 = Math::randI(0, corb.dies.size());
				die2 = Math::randI(0, corb.dies.size());
				// ensure that dies are different
				while (die1 == die2) {
					die2 = Math::randI(0, corb.dies.size());
				}
				// sanity check for empty dies
				if (corb.dies[die1]->CBL.empty() || corb.dies[die2]->CBL.empty()) {
					return false;
				}

				tuple1 = Math::randI(0, corb.dies[die1]->CBL.size());
				tuple2 = Math::randI(0, corb.dies[die2]->CBL.size());

				corb.switchBlocksAcrossDies(die1, die2, tuple1, tuple2);
			}
			else {
				corb.switchBlocksAcrossDies(this->last_op_die2, this->last_op_die1, this->last_op_tuple2, this->last_op_tuple1);
			}

			break;

		case CorblivarCore::OP_MOVE_TUPLE:
			if (!revertLastOp) {
				die1 = Math::randI(0, corb.dies.size());
				die2 = Math::randI(0, corb.dies.size());
				// ensure that dies are different
				while (die1 == die2) {
					die2 = Math::randI(0, corb.dies.size());
				}
				// sanity check for empty (origin) die
				if (corb.dies[die1]->CBL.empty()) {
					return false;
				}

				tuple1 = Math::randI(0, corb.dies[die1]->CBL.size());
				tuple2 = Math::randI(0, corb.dies[die2]->CBL.size());

				corb.moveTupleAcrossDies(die1, die2, tuple1, tuple2);
			}
			else {
				corb.moveTupleAcrossDies(this->last_op_die2, this->last_op_die1, this->last_op_tuple2, this->last_op_tuple1);
			}

			break;

		case CorblivarCore::OP_SWITCH_TUPLE_DIR:
			if (!revertLastOp) {
				die1 = Math::randI(0, corb.dies.size());
				// sanity check for empty dies
				if (corb.dies[die1]->CBL.empty()) {
					return false;
				}

				tuple1 = Math::randI(0, corb.dies[die1]->CBL.size());

				corb.switchTupleDirection(die1, tuple1);
			}
			else {
				corb.switchTupleDirection(this->last_op_die1, this->last_op_tuple1);
			}

			break;

		case CorblivarCore::OP_SWITCH_TUPLE_JUNCTS:
			if (!revertLastOp) {
				die1 = Math::randI(0, corb.dies.size());
				// sanity check for empty dies
				if (corb.dies[die1]->CBL.empty()) {
					return false;
				}

				tuple1 = Math::randI(0, corb.dies[die1]->CBL.size());
				t = corb.dies[die1]->CBL.T[tuple1];

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

				corb.switchTupleJunctions(die1, tuple1, t);
			}
			else {
				corb.switchTupleJunctions(this->last_op_die1, this->last_op_tuple1, this->last_op_juncts);
			}

			break;

		case CorblivarCore::OP_SWITCH_BLOCK_ORIENT:
			if (!revertLastOp) {
				die1 = Math::randI(0, corb.dies.size());
				// sanity check for empty dies
				if (corb.dies[die1]->CBL.empty()) {
					return false;
				}

				tuple1 = Math::randI(0, corb.dies[die1]->CBL.size());

				corb.switchBlockOrientation(die1, tuple1);
			}
			else {
				corb.switchBlockOrientation(this->last_op_die1, this->last_op_tuple1);
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

// adaptive cost model w/ two phases;
// first phase considers only cost for packing into outline
// second phase considers further factors like WL, thermal distr, etc.
FloorPlanner::Cost FloorPlanner::determCost(const double& ratio_feasible_solutions_fixed_outline, const bool& phase_two, const bool& set_max_cost) {
	double cost_total, cost_temp, cost_alignments;
	CostInterconn cost_interconnects;
	Cost cost_area_outline, ret;

	// cost area and outline, returns weighted (and normalized) cost using an adaptive cost model
	// also determine whether layout fits into outline
	cost_area_outline = this->determCostAreaOutline(ratio_feasible_solutions_fixed_outline);

	// consider further cost factors
	if (phase_two) {

		// normalized interconnects cost
		cost_interconnects = this->determCostInterconnects(set_max_cost);

		// TODO cost (failed) alignments
		cost_alignments = 0.0;

		// normalized temperature-distribution cost
		cost_temp = 0.0;
//		cost_temp = this->determCostThermalDistr(set_max_cost);

		// cost function; sum up cost terms
		cost_total =
			this->conf_SA_cost_WL * cost_interconnects.HPWL
			+ this->conf_SA_cost_TSVs * cost_interconnects.TSVs
			+ this->conf_SA_cost_temp * cost_temp
			// area, outline cost is already weighted
			+ cost_area_outline.cost;
		;
	}
	else {
		// invert cost-factor weight since only one factor defines the cost fct
		cost_total = (1.0 / this->conf_SA_cost_area_outline) * cost_area_outline.cost;
	}

#ifdef DBG_LAYOUT
	cout << "DBG_LAYOUT> ";
	cout << "Layout cost: " << cost_total << endl;
#endif

	ret.cost = cost_total;
	ret.fits_fixed_outline = cost_area_outline.fits_fixed_outline;

	return ret;
}

// adaptive cost model: terms for area and AR mismatch are _mutually_
// depending on ratio of feasible solutions (solutions fitting into outline)
FloorPlanner::Cost FloorPlanner::determCostAreaOutline(const double& ratio_feasible_solutions_fixed_outline) {
	double cost_area;
	double cost_outline;
	double max_outline_x;
	double max_outline_y;
	int i;
	vector<double> dies_AR;
	vector<double> dies_area;
	bool layout_fits_in_fixed_outline;
	Cost ret;
	Block *block;

	dies_AR.reserve(this->conf_layer);
	dies_area.reserve(this->conf_layer);

	layout_fits_in_fixed_outline = true;
	// determine outline and area
	for (i = 0; i < this->conf_layer; i++) {

		// determine outline for blocks on all dies separately
		max_outline_x = max_outline_y = 0.0;
		for (auto& b : this->blocks) {
			block = b.second;

			if (block->layer == i) {
				// update max outline coords
				max_outline_x = max(max_outline_x, block->bb.ur.x);
				max_outline_y = max(max_outline_y, block->bb.ur.y);
			}
		}

		// area, represented by blocks' outline; normalized to die outline
		dies_area.push_back((max_outline_x * max_outline_y) / (this->conf_outline_x * this->conf_outline_y));

		// aspect ratio; used to guide optimization towards fixed outline (Chen 2006)
		if (max_outline_y > 0.0) {
			dies_AR.push_back(max_outline_x / max_outline_y);
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

	// cost for AR mismatch (guides into fixed outline, Chen 2006)
	cost_outline = 0.0;
	for (i = 0; i < this->conf_layer; i++) {
		cost_outline = max(cost_outline, pow(dies_AR[i] - this->outline_AR, 2.0));
	}
	// determine cost function value
	cost_outline *= 0.5 * this->conf_SA_cost_area_outline * (1.0 - ratio_feasible_solutions_fixed_outline);

	// cost for area
	cost_area = 0.0;
	// determine max value of (blocks-outline area) / (die-outline area);
	// guides into balanced die occupation and area minimization
	for (i = 0; i < this->conf_layer; i++) {
		cost_area = max(cost_area, dies_area[i]);
	}
	// determine cost function value
	cost_area *= 0.5 * this->conf_SA_cost_area_outline * (1.0 + ratio_feasible_solutions_fixed_outline);

	ret.cost = cost_outline + cost_area;
	ret.fits_fixed_outline = layout_fits_in_fixed_outline;

	return ret;
}

// TODO recode; currently hotspot
FloorPlanner::CostInterconn FloorPlanner::determCostInterconnects(const bool& set_max_cost, const bool& normalize) {
	int i, ii;
	vector<Rect> blocks_to_consider;
	Rect bb;
	bool blocks_above_considered;

	CostInterconn ret;
	ret.HPWL = ret.TSVs = 0.0;
	blocks_to_consider.reserve(this->blocks.size());

	// determine HPWL and TSVs for each net
	for (Net* &cur_net : this->nets) {

		// determine HPWL on each layer separately
		for (i = 0; i < this->conf_layer; i++) {
#ifdef DBG_LAYOUT
			cout << "DBG_LAYOUT> Determine interconnects for net " << cur_net->id << " on layer " << i << " and above" << endl;
#endif

			// consider all related blocks:
			// blocks on this layer and blocks on layer above --- thus we
			// include TSVs in HPWL estimate assuming they are subsequently
			// placed in the related bounding box
			blocks_to_consider.clear();

			// blocks on current layer
			for (Block* &b : cur_net->blocks) {
				if (b->layer == i) {
					blocks_to_consider.push_back(b->bb);
#ifdef DBG_LAYOUT
					cout << "DBG_LAYOUT> 	Consider block " << b->id << " on layer " << i << endl;
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
				for (Block* &b : cur_net->blocks) {
					if (b->layer == ii) {
						blocks_to_consider.push_back(b->bb);
						blocks_above_considered = true;
#ifdef DBG_LAYOUT
						cout << "DBG_LAYOUT> 	Consider block " << b->id << " on layer " << ii << endl;
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
#ifdef DBG_LAYOUT
				cout << "DBG_LAYOUT> 	Ignore single block on uppermost layer" << endl;
#endif
				continue;
			}

			// update TSVs counter if connecting to blocks on some upper layer
			if (blocks_above_considered) {
				ret.TSVs += (ii - i);
#ifdef DBG_LAYOUT
				cout << "DBG_LAYOUT> 	TSVs required: " << (ii - i) << endl;
#endif
			}

			// determine HPWL of related blocks using their bounding box
			bb = Rect::determBoundingBox(blocks_to_consider);
			ret.HPWL += bb.w;
			ret.HPWL += bb.h;
#ifdef DBG_LAYOUT
			cout << "DBG_LAYOUT> 	HPWL of bounding box of blocks to consider: " << (bb.w + bb. h) << endl;
#endif
		}
	}

	// memorize max cost; initial sampling
	if (set_max_cost) {
		this->max_cost_WL = ret.HPWL;
		this->max_cost_TSVs = ret.TSVs;
	}

	// normalize to max value from initial sampling
	if (normalize) {
		ret.HPWL /= this->max_cost_WL;
		ret.TSVs /= this->max_cost_TSVs;
	}

	return ret;
}
