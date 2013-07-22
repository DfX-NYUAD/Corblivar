/*
 * =====================================================================================
 *
 *    Description:  Corblivar basic data structure: corner block list
 *
 *    Copyright (C) 2013 Johann Knechtel, johann.knechtel@ifte.de, www.ifte.de
 *
 *    This file is part of Corblivar.
 *    
 *    Corblivar is free software: you can redistribute it and/or modify it under the terms
 *    of the GNU General Public License as published by the Free Software Foundation,
 *    either version 3 of the License, or (at your option) any later version.
 *    
 *    Corblivar is distributed in the hope that it will be useful, but WITHOUT ANY
 *    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
 *    PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *    
 *    You should have received a copy of the GNU General Public License along with
 *    Corblivar.  If not, see <http://www.gnu.org/licenses/>.
 *
 * =====================================================================================
 */
#ifndef _CORBLIVAR_CORNERBLOCKLIST
#define _CORBLIVAR_CORNERBLOCKLIST

// library includes
#include "Corblivar.incl.hpp"
// Corblivar includes, if any
#include "Direction.hpp"
#include "Block.hpp"
// forward declarations, if any

class CornerBlockList {
	// debugging code switch (private)
	private:
		static constexpr bool DBG = false;

	// private data, functions
	private:
		// CBL sequences
		vector<Block const*> S;
		vector<Direction> L;
		vector<unsigned> T;

	// constructors, destructors, if any non-implicit
	public:

	// public data, functions
	public:
		friend class CorblivarCore;
		friend class CorblivarDie;

		// POD; wrapper for tuples of separate sequences
		struct Tuple {
			Block const* S;
			Direction L;
			unsigned T;
		};

		// getter / setter
		inline unsigned size() const {

			if (DBG) {
				unsigned ret;
				bool mismatch = false;
				unsigned prev_ret;

				prev_ret = ret = this->S.size();
				ret = min(ret, this->L.size());
				mismatch = (ret != prev_ret);
				prev_ret = ret;
				ret = min(ret, this->T.size());
				mismatch = mismatch || (ret != prev_ret);

				if (mismatch) {
					cout << "DBG_CBL> CBL has sequences size mismatch!" << endl;
					cout << "DBG_CBL> CBL: " << endl;
					cout << this->CBLString() << endl;
				}
			}

			return this->S.size();
		};

		inline unsigned capacity() const {
			return this->S.capacity();
		};

		inline bool empty() const {
			return this->S.empty();
		};

		inline void clear() {
			this->S.clear();
			this->L.clear();
			this->T.clear();
		};

		inline void reserve(unsigned const& elements) {
			this->S.reserve(elements);
			this->L.reserve(elements);
			this->T.reserve(elements);
		};

		inline void insert(Tuple&& tuple) {
			this->S.push_back(tuple.S);
			this->L.push_back(tuple.L);
			this->T.push_back(tuple.T);
		};

		inline string tupleString(unsigned const& tuple) const {
			stringstream ret;

			ret << "tuple " << tuple << " : ";
			ret << "( " << this->S[tuple]->id << " " << static_cast<unsigned>(this->L[tuple]) << " " << this->T[tuple] << " ";
			ret << this->S[tuple]->bb.w << " " << this->S[tuple]->bb.h << " )";

			return ret.str();
		};

		inline string CBLString() const {
			unsigned i;
			stringstream ret;

			for (i = 0; i < this->size(); i++) {
				ret << this->tupleString(i) << "; ";
			}

			return ret.str();
		};
};

#endif
