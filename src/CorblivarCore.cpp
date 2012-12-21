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
		this->dies.push_back(new CorblivarDie(i));
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
		cur_t = CorblivarFP::randI(0, (corb.blocks.size() / 3) + 1);

		// init CBL item
		cur_CBLitem = new CBLitem(cur_block, cur_dir, cur_t);

		// assign to random die
		rand = CorblivarFP::randI(0, corb.conf_layer);
		this->dies[rand]->CBL.push_back(cur_CBLitem);
	}

#ifdef DBG_CORB_FP
	unsigned d, t;

	for (d = 0; d < this->dies.size(); d++) {
		cout << "CBL tuples for die " << d << "; " << this->dies[d]->CBL.size() << " tuples:" << endl;
		for (t = 0; t < this->dies[d]->CBL.size(); t++) {
			cout << this->dies[d]->CBL[t]->itemString() << endl;
		}
		cout << endl;
	}
#endif

	if (corb.logMed()) {
		cout << "Done" << endl << endl;
	}
}

void CorblivarLayoutRep::generateLayout(CorblivarFP &corb) {
	unsigned i;
	Block *cur_block;
	bool loop;

	if (corb.logMax()) {
		cout << "Performing layout generation..." << endl;
	}

	// init die pointer
	this->p = this->dies[0];
	// init/reset progress pointer
	for (i = 0; i < this->dies.size(); i++) {
		this->dies[i]->pi = 0;
	}

	// perform layout generation in loop (until all blocks are placed)
	loop = true;
	while (loop) {
		// handle stalled die / resolve open alignment process by placing current block
		if (this->p->stalled) {
			// place block
			cur_block = this->p->placeCurrentBlock();
			// TODO mark current block as placed in AS
			//
			// mark die as not stalled anymore
			this->p->stalled = false;
			// increment progress pointer, next block
			this->p->incrementProgressPointer();
		}
		// die is not stalled
		else {
			// TODO check for alignment tuples for current block
			//
			if (false) {
			}
			// no alignment tuple assigned for current block
			else {
				// place block, consider next block
				this->p->placeCurrentBlock();
				this->p->incrementProgressPointer();
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
		cout << "Done" << endl << endl;
	}
}

void CorblivarDie::incrementProgressPointer() {

	if (this->pi == (this->CBL.size() - 1)) {
		this->done = true;
	}
	else {
		this->pi++;
	}
}

Block* CorblivarDie::currentBlock() {
	return this->CBL[this->pi]->Si;
}

Block* CorblivarDie::placeCurrentBlock() {
	Block *cur_block;

	cur_block = this->CBL[this->pi]->Si;

#ifdef DBG_CORB_FP
	cout << "DBG_CORB_FP: Placing block " << cur_block->id << " on die " << this->id << endl;
#endif

	return cur_block;
}
