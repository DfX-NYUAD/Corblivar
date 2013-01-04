/*
 * =====================================================================================
 *
 *    Description:  Corblivar core file (data structures, layout operations)
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

	// init SA parameters
	innerLoopMax = this->conf_SA_loopFactor * pow((double) this->blocks.size(), (double) 4/3);

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

			// generate layout
			chip.generateLayout(*this);
			// evaluate layout
			this->determLayoutCost(chip);

#ifdef DBG_CORB_FP
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

// cost factors should all be normalized to their respective max values; i.e., for
// optimized solutions, cost will be less than 1
double CorblivarFP::determLayoutCost(CorblivarLayoutRep &chip) {
	double cost_total, cost_temp, cost_WL, cost_TSVs, cost_IR, cost_alignments;
	vector<double> cost_outline;

	// TODO Cost Temp
	cost_temp = 0.0;

	// TODO Cost IR
	cost_IR = 0.0;

	// TODO Cost WL
	cost_WL = 0.0;

	// TODO Cost TSVs
	cost_TSVs = 0.0;

	//// Cost outline
	cost_outline = this->determLayoutCostOutline(chip);

	// TODO Cost (Failed) Alignments
	cost_alignments = 0.0;

	cost_total = CorblivarFP::COST_FACTOR_TEMP * cost_temp
		+ CorblivarFP::COST_FACTOR_WL * cost_WL
		+ CorblivarFP::COST_FACTOR_TSVS * cost_TSVs
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

vector<double> CorblivarFP::determLayoutCostOutline(CorblivarLayoutRep &chip) {
	double cost_outline_x = 0.0;
	double cost_outline_y = 0.0;
	double max_outline_x, max_outline_y;
	vector<double> ret;
	int i;
	CorblivarDie *cur_die;
	Block *cur_block;

	// consider dies separately, determine avg cost afterwards
	for (i = 0; i < this->conf_layer; i++) {
		cur_die = chip.dies[i];
		cur_die->resetTuplePointer();

		// init max outline points
		cur_block = cur_die->currentBlock();
		max_outline_x = cur_block->bb.ur.x;
		max_outline_y = cur_block->bb.ur.y;
		// update max outline points
		while (cur_die->incrementTuplePointer()) {
			cur_block = cur_die->currentBlock();
			max_outline_x = max(max_outline_x, cur_block->bb.ur.x);
			max_outline_y = max(max_outline_y, cur_block->bb.ur.y);
		}
		// sum up over all dies
		cost_outline_x += max_outline_x;
		cost_outline_y += max_outline_y;
	}
	// avg for all dies
	cost_outline_x /= this->conf_layer;
	cost_outline_y /= this->conf_layer;

	// normalize to max value, i.e., given outline
	cost_outline_x /= this->conf_outline_x;
	cost_outline_y /= this->conf_outline_y;

	ret.push_back(cost_outline_x);
	ret.push_back(cost_outline_y);

	return ret;
}

void CorblivarLayoutRep::initCorblivar(CorblivarFP &corb) {
	map<int, Block*>::iterator b;
	Block *cur_block;
	Direction cur_dir;
	int i, rand, cur_t;

	if (corb.logMed()) {
		cout << "Layout> ";
		cout << "Initializing Corblivar data for chip on " << corb.conf_layer << " layers..." << endl;
	}

	// init separate data structures for dies
	for (i = 0; i < corb.conf_layer; i++) {
		this->dies.push_back(new CorblivarDie(i));
	}

	// assign each block randomly to one die, generate L and T randomly as well
	for (b = corb.blocks.begin(); b != corb.blocks.end(); ++b) {
		cur_block = (*b).second;

		// consider random die
		rand = CorblivarFP::randI(0, corb.conf_layer);

		// generate direction L
		if (CorblivarFP::randB()) {
			cur_dir = DIRECTION_HOR;
		}
		else {
			cur_dir = DIRECTION_VERT;
		}
		// generate T-junction to be overlapped
		// consider block count as upper bound
		cur_t = CorblivarFP::randI(0, this->dies[rand]->CBL.size());

		// store CBL item
		this->dies[rand]->CBL.push_back(new CBLitem(cur_block, cur_dir, cur_t));
	}

#ifdef DBG_CORB_FP
	unsigned d;
	list<CBLitem*>::iterator CBLi;

	for (d = 0; d < this->dies.size(); d++) {
		cout << "DBG_CORB_FP> ";
		cout << "Init CBL tuples for die " << d << "; " << this->dies[d]->CBL.size() << " tuples:" << endl;
		for (CBLi = this->dies[d]->CBL.begin(); CBLi != this->dies[d]->CBL.end(); ++CBLi) {
			cout << "DBG_CORB_FP> ";
			cout << (* CBLi)->itemString() << endl;
		}
		cout << "DBG_CORB_FP> ";
		cout << endl;
	}
#endif

	if (corb.logMed()) {
		cout << "Layout> ";
		cout << "Done" << endl << endl;
	}
}

void CorblivarLayoutRep::generateLayout(CorblivarFP &corb) {
	unsigned i;
	Block *cur_block;
	bool loop;

	if (corb.logMax()) {
		cout << "Layout> ";
		cout << "Performing layout generation..." << endl;
	}

	// init die pointer
	this->p = this->dies[0];

	// reset die data
	for (i = 0; i < this->dies.size(); i++) {
		// reset progress pointer
		this->dies[i]->resetTuplePointer();
		// reset done flag
		this->dies[i]->done = false;
		// reset placement stacks
		while (!this->dies[i]->Hi.empty()) {
			this->dies[i]->Hi.pop();
		}
		while (!this->dies[i]->Vi.empty()) {
			this->dies[i]->Vi.pop();
		}
	}

	// perform layout generation in loop (until all blocks are placed)
	loop = true;
	while (loop) {
		// handle stalled die / resolve open alignment process by placing current block
		if (this->p->stalled) {
			// place block, increment progress pointer
			cur_block = this->p->placeCurrentBlock();
			// TODO mark current block as placed in AS
			//
			// mark die as not stalled anymore
			this->p->stalled = false;
		}
		// die is not stalled
		else {
			// TODO check for alignment tuples for current block
			//
			if (false) {
			}
			// no alignment tuple assigned for current block
			else {
				// place block, increment progress pointer
				this->p->placeCurrentBlock();
			}
		}

		// die done
		if (this->p->done) {
			// continue loop on yet unfinished die
			for (i = 0; i < this->dies.size(); i++) {
				if (!this->dies[i]->done) {
					this->p = this->dies[i];
					break;
				}
			}
			// all dies handled, stop loop
			if (this->p->done) {
				loop = false;
			}
		}
	}

	if (corb.logMax()) {
		cout << "Layout> ";
		cout << "Done" << endl;
	}
}

Block* CorblivarDie::placeCurrentBlock() {
	Block *cur_block;
	CBLitem *cur_CBLi;
	vector<Block*> relevBlocks;
	unsigned relevBlocksCount, b;
	double x, y;

	cur_CBLi = (* this->pi);
	cur_block = cur_CBLi->Si;

	// assign layer to block
	cur_block->layer = this->id;

	// horizontal placement
	if (cur_CBLi->Li == DIRECTION_HOR) {
		// pop relevant blocks from stack
		relevBlocksCount = min(cur_CBLi->Ti + 1, this->Hi.size());
//cout << "relevBlocksCount=" << relevBlocksCount << endl;
		while (relevBlocksCount > relevBlocks.size()) {
			relevBlocks.push_back(this->Hi.top());
			this->Hi.pop();
		}

		// determine x-coordinate for lower left corner of current block
		x = 0;
		for (b = 0; b < relevBlocks.size(); b++) {
			// determine right front
			x = max(x, relevBlocks[b]->bb.ur.x);
		}

		// determine y-coordinate for lower left corner of current block
		if (this->Hi.empty()) {
			y = 0;
//cout << "Hi empty" << endl;
		}
		else {
			// init min value (lower front)
			if (relevBlocks.empty()) {
				y = 0;
//cout << "relevBlocks empty" << endl;
			}
			else {
//cout << "OK" << endl;
				y = relevBlocks[0]->bb.ll.y;

				// determine min value
				for (b = 0; b < relevBlocks.size(); b++) {
					if (relevBlocks[b]->bb.ll.y != Point::UNDEF) {
						y = min(y, relevBlocks[b]->bb.ll.y);
					}
				}
			}
		}
	}
	// vertical placement
	else {
		// pop relevant blocks from stack
		relevBlocksCount = min(cur_CBLi->Ti + 1, this->Vi.size());
		while (relevBlocksCount > relevBlocks.size()) {
			relevBlocks.push_back(this->Vi.top());
			this->Vi.pop();
		}

		// determine y-coordinate for lower left corner of current block
		y = 0;
		for (b = 0; b < relevBlocks.size(); b++) {
			// determine upper front
			y = max(y, relevBlocks[b]->bb.ur.y);
		}

		// determine x-coordinate for lower left corner of current block
		if (this->Vi.empty()) {
			x = 0;
		}
		else {
			// init min value (left front)
			if (relevBlocks.empty()) {
				x = 0;
			}
			else {
				x = relevBlocks[0]->bb.ll.x;

				// determine min value
				for (b = 0; b < relevBlocks.size(); b++) {
					if (relevBlocks[b]->bb.ll.x != Point::UNDEF) {
						x = min(x, relevBlocks[b]->bb.ll.x);
					}
				}
			}
		}
	}

	// update block coordinates
	cur_block->bb.ll.x = x;
	cur_block->bb.ll.y = y;
	cur_block->bb.ur.x = cur_block->bb.w + x;
	cur_block->bb.ur.y = cur_block->bb.h + y;

	// remember placed block on stacks
	this->Hi.push(cur_block);
	this->Vi.push(cur_block);

#ifdef DBG_CORB_FP
	cout << "DBG_CORB_FP> ";
	cout << "Processing CBL tuple " << cur_CBLi->itemString() << " on die " << this->id << ": ";
	cout << "LL=(" << cur_block->bb.ll.x << ", " << cur_block->bb.ll.y << "), ";
	cout << "UR=(" << cur_block->bb.ur.x << ", " << cur_block->bb.ur.y << ")" << endl;
#endif

	// increment progress pointer, consider next tuple (block)
	this->incrementTuplePointer();

	return cur_block;
}
