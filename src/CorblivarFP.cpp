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
	bool annealed;
	double cur_cost;
	vector<double> init_cost_interconnects;

	// init SA parameters
	innerLoopMax = this->conf_SA_loopFactor * pow((double) this->blocks.size(), (double) 4/3);

	// init max cost
	this->max_cost_WL = 0.0;
	this->max_cost_TSVs = 0.0;
	this->max_cost_temp = 0.0;
	this->max_cost_IR = 0.0;
	this->max_cost_alignments = 0.0;

	// initial solution-space sampling
	// i.e., outline max cost for various parameters
	// TODO backup initial CBL
	for (i = 0; i < innerLoopMax; i++) {
		// perform random layout op
		this->SAinnerLoopIter(chip);
		// generate layout
		chip.generateLayout(this->conf_log);

		// determine and memorize cost interconnects
		init_cost_interconnects = this->determCostInterconnects();

		// memorize max cost
		this->max_cost_WL = max(this->max_cost_WL, init_cost_interconnects[0]);
		this->max_cost_TSVs = max(this->max_cost_TSVs, init_cost_interconnects[1]);
	}
	// TODO restore initial CBL

	// outer loop: annealing -- temperature steps
	i = 0;
	annealed = false;
	while (!annealed) {

		if (this->logMed()) {
			cout << "SA> Optimization step: " << i << endl;
		}

		// TODO SA handling
		// inner loop: layout operations
		ii = 0;
		while (ii < innerLoopMax) {

			// perform random layout op
			this->SAinnerLoopIter(chip);
			// generate layout
			chip.generateLayout(this->conf_log);
			// evaluate layout
			cur_cost = this->determLayoutCost();

#ifdef DBG_SA
			cout << "SA> Inner step: " << ii << "/" << innerLoopMax << endl;
#endif

			ii++;
		}

		// TODO logMed: current cost, current temp
		if (this->logMed()) {
			cout << "SA> Step done" << endl;
		}

		i++;

		// TODO determine: use avg cost of prev iterations, if smaller than
		// standard dev?
		annealed = true;
	}

	if (this->logMed()) {
		cout << endl;
	}

	return true;
}

void CorblivarFP::SAinnerLoopRandomOp(CorblivarLayoutRep &chip) {
	int op;

	// perform random layout operation
	op = CorblivarFP::randI(0, 5);
	switch (op) {
		case 0:
			chip.switchRandomTuplesWithinDie();
			break;
		case 1:
			chip.switchRandomTuplesAcrossDies();
			break;
		case 2:
			chip.switchRandomTupleToRandomDie();
			break;
		case 3:
			chip.switchRandomTupleDirection();
			break;
		case 4:
			chip.switchRandomTupleJunctions();
			break;
	}
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

	cost_total = CorblivarFP::COST_FACTOR_TEMP * cost_temp
		+ CorblivarFP::COST_FACTOR_WL * cost_interconnects[0]
		+ CorblivarFP::COST_FACTOR_TSVS * cost_interconnects[1]
		+ CorblivarFP::COST_FACTOR_IR * cost_IR
		+ CorblivarFP::COST_FACTOR_OUTLINE_X * cost_outline[0]
		+ CorblivarFP::COST_FACTOR_OUTLINE_Y * cost_outline[1]
		+ CorblivarFP::COST_FACTOR_ALIGNMENTS * cost_alignments
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
