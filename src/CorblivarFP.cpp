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
	double accepted_ops;
	bool annealed;
	bool op_success;
	double cur_cost, best_cost, prev_cost, cost_diff;
	double cur_avg_cost, init_avg_cost;
	double cost_temp_diff;
	unsigned c;
	deque<double> cost_hist;
	double cur_temp, init_temp, temp_diff;
	double r;
	vector<double> init_cost_interconnects;
	bool cur_layout_fits_in_outline;
	int layout_fit_counter;
	double layout_fit_ratio;

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
	for (i = 0; i < innerLoopMax; i++) {
		// perform random layout op
		this->performRandomLayoutOp(chip);
		// generate layout
		chip.generateLayout(this->conf_log);

		// determine and memorize cost of interconnects
		init_cost_interconnects = this->determCostInterconnects();

		// memorize max cost
		this->max_cost_WL = max(this->max_cost_WL, init_cost_interconnects[0]);
		this->max_cost_TSVs = max(this->max_cost_TSVs, init_cost_interconnects[1]);
	}

	// restore initial CBLs
	chip.restoreCBLs();

	// perform some random operations, track cost std dev
	layout_fit_counter = 0;
	for (i = 1; i <= innerLoopMax; i++) {
		// perform random layout op
		this->performRandomLayoutOp(chip);
		// generate layout
		chip.generateLayout(this->conf_log);

		cost_hist.push_back(this->determLayoutCost(cur_layout_fits_in_outline, (double) layout_fit_counter / i));

		// memorize count of solutions fitting into outline
		if (cur_layout_fits_in_outline) {
			layout_fit_counter++;
		}
	}

	// restore initial CBLs
	chip.restoreCBLs();

	// determine initial, normalized cost
	chip.generateLayout(this->conf_log);
	cur_cost = this->determLayoutCost(cur_layout_fits_in_outline, (double) layout_fit_counter / i);

	// determine initial avg cost
	init_avg_cost = 0.0;
	for (c = 0; c < cost_hist.size(); c++) {
		init_avg_cost += cost_hist[c];
	}
	init_avg_cost /= cost_hist.size();

	// init SA parameter: start temp, depends on std dev of costs
	// Huang et al 1986
	init_temp = cur_temp = CorblivarFP::SA_INIT_T_FACTOR * CorblivarFP::stdDev(cost_hist);
	cost_hist.clear();

	if (this->logMed()) {
		cout << "SA> Done" << endl;
		cout << "SA> Start annealing process..." << endl;
	}

	// init loop parameters
	i = 0;
	annealed = false;
	// dummy value >> normalized cost to expect
	best_cost = 100.0;
	layout_fit_ratio = layout_fit_counter = 0.0;

	/// outer loop: annealing -- temperature steps
	while (!annealed && i <= this->conf_SA_loopLimit) {

		if (this->logMed()) {
			cout << "SA> Optimization step: " << i << "/" << this->conf_SA_loopLimit << endl;
		}

		// init loop parameters
		ii = 1;
		cur_avg_cost = 0.0;
		accepted_ops = 0.0;

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
				// memorize cost
				cur_avg_cost += cur_cost;
				// cost difference
				cost_diff = cur_cost - prev_cost;
#ifdef DBG_SA
				cout << "SA> Inner step: " << ii << "/" << innerLoopMax << endl;
				cout << "SA> Cost diff: " << cost_diff << endl;
#endif

				// memorize count of solutions fitting into outline
				if (cur_layout_fits_in_outline) {
					layout_fit_counter++;
				}

				// memorize best solution which fits into outline
				if (cur_cost < best_cost && cur_layout_fits_in_outline) {
					if (this->logMax()) {
						cout << "SA> Currently best (fitting) solution found; cost: " << cur_cost << endl;
					}

					best_cost = cur_cost;
					chip.storeBestCBLs();
				}

				// increase ops count
				accepted_ops++;
				// revert solution w/ worse cost, depending on temperature
				if (cost_diff > 0.0) {
					r = CorblivarFP::randF01();
					if (r > exp(- cost_diff / cur_temp)) {
#ifdef DBG_SA
						cout << "SA> Revert op" << endl;
#endif
						// revert last op
						this->performRandomLayoutOp(chip, true);
						// decrease op count, compensated by unconditional
						// increase above
						accepted_ops--;
						// similarly, decrease cost
						cur_avg_cost -= cur_cost;
					}
				}

				// consider next loop iteration
				ii++;
			}
		}

		// determine ratio of solutions fitting into outline in prev temp step
		layout_fit_ratio = (double) layout_fit_counter / innerLoopMax;
		// reset counter
		layout_fit_counter = 0;

		// determine avg cost for temp step
		cur_avg_cost /= accepted_ops;
		// determine accepted-ops ratio
		accepted_ops /= innerLoopMax;

		if (this->logMed()) {
			cout << "SA> Step done:" << endl;
			cout << "SA>  valid-layouts ratio: " << layout_fit_ratio << endl;
			cout << "SA>  accept-ops ratio: " << accepted_ops << endl;
			cout << "SA>  temp: " << cur_temp << endl;
			cout << "SA>  avg cost: " << cur_avg_cost << endl;
		}

		// reduce temp
		// LV Christian Hochberger "HW Synthese eingebettete Systeme", Kapitel 6
		temp_diff = cur_temp;
		if (accepted_ops > 0.96) {
			cur_temp *= 0.5;
		}
		else if (0.8 < accepted_ops && accepted_ops <= 0.96) {
			cur_temp *= 0.9;
		}
		else if (0.15 < accepted_ops && accepted_ops <= 0.8) {
			cur_temp *= 0.95;
		}
		// accepted_ops <= 0.15
		else {
			cur_temp *= 0.8;
		}
		temp_diff -= cur_temp;

		// determine whether annealed: consider change of cost/temp ratio
		// (Aarts 1986, as given in Shahookar 1991)
		cost_hist.push_back(cur_avg_cost);
		if (cost_hist.size() > 2) {
			// drop cost of previous (> 2) iteration
			cost_hist.pop_front();

			// determine change of cost in prev iteration
			cost_diff = cost_hist[1] - cost_hist[0];

			// determine change of cost/temp ratio
			// weighted w/ temp/init_cost ratio
			cost_temp_diff = abs((cost_diff / temp_diff) * (cur_temp / init_avg_cost));

			// consider as annealed when ratio drops below min level
			annealed = (cost_temp_diff <= this->conf_SA_costTempRatioLowerLimit);

			if (this->logMed()) {
				cout << "SA>  delta(cost/temp) ratio: " << cost_temp_diff << endl;
			}
		}

		// consider next step
		i++;
	}

	// apply best solution, if available, as final solution
	chip.applyBestCBLs(this->conf_log);
	// generate final layout
	chip.generateLayout(this->conf_log);
	// verify if layout fits into outline
	this->determLayoutCost(cur_layout_fits_in_outline, 1.0);

	if (this->logMed()) {
		cout << "SA> Done" << endl;
		cout << endl;
	}

	return cur_layout_fits_in_outline;
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
		this->last_op = op = CorblivarFP::randI(0, 5);
	}

	// specific op handler
	switch (op) {
		case CorblivarLayoutRep::OP_SWAP_TUPLES_WI_DIE:
			if (!revertLastOp) {
				die1 = CorblivarFP::randI(0, chip.dies.size());
				// sanity check for dies w/ one or zero blocks
				if (chip.dies[die1]->CBL.size() <= 1) {
					return false;
				}

				tuple1 = CorblivarFP::randI(0, chip.dies[die1]->CBL.size());
				tuple2 = CorblivarFP::randI(0, chip.dies[die1]->CBL.size());
				// ensure that tuples are different
				while (tuple1 == tuple2) {
					tuple2 = CorblivarFP::randI(0, chip.dies[die1]->CBL.size());
				}

				chip.switchTuplesWithinDie(die1, tuple1, tuple2);
			}
			else {
				chip.switchTuplesWithinDie(this->last_op_die1, this->last_op_tuple2, this->last_op_tuple1);
			}

			break;

		case CorblivarLayoutRep::OP_SWAP_TUPLES_ACROSS_DIE:
			if (!revertLastOp) {
				die1 = CorblivarFP::randI(0, chip.dies.size());
				die2 = CorblivarFP::randI(0, chip.dies.size());
				// ensure that dies are different
				while (die1 == die2) {
					die2 = CorblivarFP::randI(0, chip.dies.size());
				}
				// sanity check for empty dies
				if (chip.dies[die1]->CBL.empty() || chip.dies[die2]->CBL.empty()) {
					return false;
				}

				tuple1 = CorblivarFP::randI(0, chip.dies[die1]->CBL.size());
				tuple2 = CorblivarFP::randI(0, chip.dies[die2]->CBL.size());

				chip.switchTuplesAcrossDies(die1, die2, tuple1, tuple2);
			}
			else {
				chip.switchTuplesAcrossDies(this->last_op_die2, this->last_op_die1, this->last_op_tuple2, this->last_op_tuple1);
			}

			break;

		case CorblivarLayoutRep::OP_MOVE_TUPLE:
			if (!revertLastOp) {
				die1 = CorblivarFP::randI(0, chip.dies.size());
				die2 = CorblivarFP::randI(0, chip.dies.size());
				// ensure that dies are different
				while (die1 == die2) {
					die2 = CorblivarFP::randI(0, chip.dies.size());
				}
				// sanity check for empty (origin) die
				if (chip.dies[die1]->CBL.empty()) {
					return false;
				}

				tuple1 = CorblivarFP::randI(0, chip.dies[die1]->CBL.size());
				tuple2 = CorblivarFP::randI(0, chip.dies[die2]->CBL.size());

				chip.moveTupleAcrossDies(die1, die2, tuple1, tuple2);
			}
			else {
				chip.moveTupleAcrossDies(this->last_op_die2, this->last_op_die1, this->last_op_tuple2, this->last_op_tuple1);
			}

			break;

		case CorblivarLayoutRep::OP_SWITCH_DIR:
			if (!revertLastOp) {
				die1 = CorblivarFP::randI(0, chip.dies.size());
				// sanity check for empty dies
				if (chip.dies[die1]->CBL.empty()) {
					return false;
				}

				tuple1 = CorblivarFP::randI(0, chip.dies[die1]->CBL.size());

				chip.switchTupleDirection(die1, tuple1);
			}
			else {
				chip.switchTupleDirection(this->last_op_die1, this->last_op_tuple1);
			}

			break;

		case CorblivarLayoutRep::OP_SWITCH_JUNCTS:
			if (!revertLastOp) {
				die1 = CorblivarFP::randI(0, chip.dies.size());
				// sanity check for empty dies
				if (chip.dies[die1]->CBL.empty()) {
					return false;
				}

				tuple1 = CorblivarFP::randI(0, chip.dies[die1]->CBL.size());
				t = chip.dies[die1]->CBL[tuple1]->Ti;

				this->last_op_juncts = t;

				if (t == 0) {
					t++;
				}
				else {
					if (CorblivarFP::randB()) {
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
	vector<double> dies_area;

	// TODO Cost Temp
	cost_temp = 0.0;

	// TODO Cost IR
	cost_IR = 0.0;

	// cost interconnects
	cost_interconnects = this->determCostInterconnects();
	// normalize to max value from initial sampling
	cost_interconnects[0] /= this->max_cost_WL;
	cost_interconnects[1] /= this->max_cost_TSVs;

	// TODO Cost (Failed) Alignments
	cost_alignments = 0.0;

	// cost function; cost terms which are independent of particular layer layouts
	cost_total = this->conf_SA_cost_temp * cost_temp
		+ this->conf_SA_cost_WL * cost_interconnects[0]
		+ this->conf_SA_cost_TSVs * cost_interconnects[1]
		+ this->conf_SA_cost_IR * cost_IR
	;

	/// cost outline, area
	// determine max outline coords for blocks on all dies separately
	layout_fits_in_fixed_outline = true;
	non_empty_dies = 0;
	for (i = 0; i < this->conf_layer; i++) {
		max_outline_x = max_outline_y = 0.0;
		for (b = this->blocks.begin(); b != this->blocks.end(); ++b) {
			cur_block = (*b).second;
			// update max outline coords
			if (cur_block->layer == i) {
				max_outline_x = max(max_outline_x, cur_block->bb.ur.x);
				max_outline_y = max(max_outline_y, cur_block->bb.ur.y);
			}
		}

		// determine aspect ratio; used to guide optimization for fixed outline (Chen 2006)
		if (max_outline_x == 0.0 || max_outline_y == 0.0) {
			// dummy value; implies outline cost of 0.0 for this die
			dies_AR.push_back(this->outline_AR);
		}
		else {
			dies_AR.push_back(max_outline_x / max_outline_y);
			non_empty_dies++;
		}
		// normalize outline to max value, i.e., given outline
		max_outline_x /= this->conf_outline_x;
		max_outline_y /= this->conf_outline_y;
		// consider normalized outline for area calculation
		dies_area.push_back(max_outline_x * max_outline_y);
		// memorize whether layout fits into outline
		layout_fits_in_fixed_outline = layout_fits_in_fixed_outline && (max_outline_x <= 1.0 && max_outline_y <= 1.0);
	}

	// cost function; cost terms which are dependent of particular layer layouts,
	// i.e., outline, area
	cost_area_outline = 0.0;
	for (i = 0; i < this->conf_layer; i++) {
		/// adaptive cost model: terms for area and AR mismatch are _mutually_
		/// depending on ratio of feasible solutions (solutions fitting into outline)
		// cost term for area: alpha * ratio * A; 0 <= alpha <= cost_area_outline
		cost_area_outline = cost_area_outline
		+ this->conf_SA_cost_area_outline * ratio_feasible_solutions_fixed_outline * dies_area[i]
		// cost term for aspect ratio mismatch: alpha * (1 - ratio) * (R - R_outline)^2
		+ this->conf_SA_cost_area_outline * (1.0 - ratio_feasible_solutions_fixed_outline) * pow(dies_AR[i] - this->outline_AR, 2.0)
	;
	}
	// determine average of layer-dependent cost factors
	cost_area_outline /= non_empty_dies;

	// add to cost function
	cost_total += cost_area_outline;

#ifdef DBG_SA
	cout << "Layout> ";
	cout << "Layout cost: " << cost_total << endl;
#endif

	return cost_total;
}

// return[0]: HPWL
// return[1]: TSVs
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
			cout << "SA> Determine interconnects for net " << cur_net->id << " on layer " << i << " and above" << endl;
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
					cout << "SA> 	Consider block " << cur_net->blocks[b]->id << " on layer " << i << endl;
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
						cout << "SA> 	Consider block " << cur_net->blocks[b]->id << " on layer " << ii << endl;
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
				cout << "SA> 	Ignore single block on uppermost layer" << endl;
#endif
				continue;
			}

			// update TSVs counter if connecting to blocks on some upper layer
			if (blocks_above_considered) {
				TSVs += (ii - i);
#ifdef DBG_SA
				cout << "SA> 	TSVs required: " << (ii - i) << endl;
#endif
			}

			// determine HPWL of related blocks using their bounding box
			bb = Rect::determBoundingBox(blocks_to_consider);
			HPWL += bb.w;
			HPWL += bb.h;
#ifdef DBG_SA
			cout << "SA> 	HPWL of bounding box of blocks to consider: " << (bb.w + bb. h) << endl;
#endif
		}
	}

	ret.push_back(HPWL);
	ret.push_back(TSVs);

	return ret;
}
