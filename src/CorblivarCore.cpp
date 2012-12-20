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

void CorblivarLayoutRep::initCorblivar(CorblivarFP &corb) {
	map<int, Block*>::iterator b;
	Block *cur_block;
	CBLitem *cur_CBLitem;
	Direction cur_dir;
	int i, rand, cur_t;

	if (corb.logMed()) {
		cout << "Initializing Corblivar data for chip on " << corb.conf_layer << " layers..." << endl;
	}

	// init separate data structures for dies
	for (i = 0; i < corb.conf_layer; i++) {
		this->dies.push_back(new CorblivarDie());
	}

	// assign each block randomly to one die, generate L and T randomly as well
	for (b = corb.blocks.begin(); b != corb.blocks.end(); ++b) {
		cur_block = (*b).second;

		// generate direction L
		if (CorblivarFP::randB()) {
			cur_dir = DIRECTION_HOR;
		}
		else {
			cur_dir = DIRECTION_VERT;
		}
		// generate T-junction to be overlapped
		cur_t = CorblivarFP::randI(0, corb.blocks.size() / 3);

		// init CBL item
		cur_CBLitem = new CBLitem(cur_block, cur_dir, cur_t);

		// assign to random die
		rand = CorblivarFP::randI(0, corb.conf_layer - 1);
		this->dies[rand]->CBL.push_back(cur_CBLitem);
	}

	if (corb.logMed()) {
		cout << "Done" << endl << endl;
	}
}
