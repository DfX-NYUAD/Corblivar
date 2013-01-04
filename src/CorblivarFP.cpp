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

	// init SA parameters
	innerLoopMax = this->conf_SA_loopFactor * pow((double) this->blocks.size(), (double) 4/3);

	// TODO var random operations in the beginning to outline solution space,
	// i.e., outline max cost for var parameters

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
			chip.generateLayout(this->conf_log);
			// evaluate layout
			this->determLayoutCost(chip);

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
	// normalize to max value, i.e., given outline
	cost_outline[0] /= this->conf_outline_x;
	cost_outline[1] /= this->conf_outline_y;

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

	ret.push_back(cost_outline_x);
	ret.push_back(cost_outline_y);

	return ret;
}
