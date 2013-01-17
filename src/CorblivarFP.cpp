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
	double cur_cost, prev_cost;
	double cur_avg_cost;
	deque<double> cost_hist;
	double cost_hist_std_dev;
	double cur_temp, init_temp;
	double r;
	vector<double> init_cost_interconnects;

	/// init SA parameters
	//
	innerLoopMax = this->conf_SA_loopFactor * pow((double) this->blocks.size(), (double) 4/3);
	// assume std deviation of cost as 0.5
	init_temp = cur_temp = CorblivarFP::SA_INIT_T_FACTOR * 0.5;

	// init max cost
	this->max_cost_WL = 0.0;
	this->max_cost_TSVs = 0.0;
	this->max_cost_temp = 0.0;
	this->max_cost_IR = 0.0;
	this->max_cost_alignments = 0.0;

	/// initial solution-space sampling
	/// i.e., outline max cost for various parameters
	//
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
	// determine initial, normalized cost
	cur_cost = this->determLayoutCost();

	// outer loop: annealing -- temperature steps
	i = 0;
	annealed = false;
	while (!annealed) {

		if (this->logMed()) {
			cout << "SA> Optimization step: " << i << endl;
		}

		// inner loop: layout operations
		ii = 0;
		cur_avg_cost = 0.0;
		accepted_ops = 0.0;
		while (ii < innerLoopMax) {

			// perform random layout op
			op_success = this->performRandomLayoutOp(chip);
			if (op_success) {

				prev_cost = cur_cost;

				// generate layout
				chip.generateLayout(this->conf_log);

				// evaluate layout, new cost
				cur_cost = this->determLayoutCost();

				// cost difference
				if (this->logMax()) {
					cout << "SA> Inner step: " << ii << "/" << innerLoopMax << endl;
					cout << "SA> Cost diff: " << cur_cost - prev_cost << endl;
				}

				// revert solution w/ worse cost, depending on temperature
				r = CorblivarFP::randF01();
				if (r > exp(-(cur_cost - prev_cost) / cur_temp)) {
					if (this->logMax()) {
						cout << "SA> Revert op" << endl;
					}
					// revert last op
					this->performRandomLayoutOp(chip, true);
				}
				else {
					if (this->logMax()) {
						cout << "SA> Accept op" << endl;
					}
					accepted_ops++;
				}

				// memorize cost
				cur_avg_cost += cur_cost;
				// consider next loop iteration
				ii++;
			}
		}

		// determine avg cost for temp step
		cur_avg_cost /= innerLoopMax;
		// determine accepted-ops ratio
		accepted_ops /= innerLoopMax;

		if (this->logMed()) {
			cout << "SA> Step done; accepted-operations ratio: " << accepted_ops;
			cout << ", temperature: " << cur_temp << ", average cost: " << cur_avg_cost;
		}

		/// determine whether annealed: consider std deviation
		/// of avg cost of previous 10 iterations
		cost_hist.push_back(cur_avg_cost);
		if (cost_hist.size() > 10) {
			// drop cost of iteration 10 rounds ago
			cost_hist.pop_front();

			// determine std dev
			cost_hist_std_dev = CorblivarFP::stdDev(cost_hist);

			// consider as annealed when std dev drops below min level
			annealed = (cost_hist_std_dev <= this->conf_SA_minStdDevCost);

			if (this->logMed()) {
				cout << ", std dev of avg cost: " << cost_hist_std_dev;
			}
		}

		if (this->logMed()) {
			cout << endl;
		}

		// reduce temp, consider next temp step
		cur_temp *= this->conf_SA_coolingT;
		i++;
	}

	if (this->logMed()) {
		cout << endl;
	}

	return true;
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
				t = CorblivarFP::randI(0, tuple1);

				this->last_op_juncts = chip.switchTupleJunctions(die1, tuple1, t);
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
// optimized solutions, cost will be significantly less than 1
double CorblivarFP::determLayoutCost() {
	double cost_total, cost_temp, cost_IR, cost_alignments;
	vector<double> cost_outline;
	vector<double> cost_interconnects;

	// TODO Cost Temp
	cost_temp = 0.0;

	// TODO Cost IR
	cost_IR = 0.0;

	// cost interconnects
	cost_interconnects = this->determCostInterconnects();
	// normalize to max value from initial sampling
	cost_interconnects[0] /= this->max_cost_WL;
	cost_interconnects[1] /= this->max_cost_TSVs;

	//// cost outline, i.e., max outline coords
	cost_outline = this->determCostOutline();
	// normalize to max value, i.e., given outline
	cost_outline[0] /= this->conf_outline_x;
	cost_outline[1] /= this->conf_outline_y;

	// TODO Cost (Failed) Alignments
	cost_alignments = 0.0;

	cost_total = this->conf_SA_cost_temp * cost_temp
		+ this->conf_SA_cost_WL * cost_interconnects[0]
		+ this->conf_SA_cost_TSVs * cost_interconnects[1]
		+ this->conf_SA_cost_IR * cost_IR
		+ this->conf_SA_cost_outline_x * cost_outline[0]
		+ this->conf_SA_cost_outline_y * cost_outline[1]
		+ this->conf_SA_cost_alignments * cost_alignments
	;

	if (this->logMax()) {
		cout << "Layout> ";
		cout << "Layout cost: " << cost_total << endl;
	}

	return cost_total;
}

vector<double> CorblivarFP::determCostOutline() {
	double max_outline_x = 0.0;
	double max_outline_y = 0.0;
	vector<double> ret;
	Block *cur_block;
	map<int, Block*>::iterator b;

	// consider max outline coords for all blocks on all dies
	for (b = this->blocks.begin(); b != this->blocks.end(); ++b) {
		cur_block = (*b).second;
		// update max outline coords
		max_outline_x = max(max_outline_x, cur_block->bb.ur.x);
		max_outline_y = max(max_outline_y, cur_block->bb.ur.y);
	}

	ret.push_back(max_outline_x);
	ret.push_back(max_outline_y);

	return ret;
}

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
