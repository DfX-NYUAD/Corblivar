/**
 * =====================================================================================
 *
 *    Description:  Corblivar basic data structure: corner block list
 *
 *    Copyright (C) 2013-2016 Johann Knechtel, johann aett jknechtel dot de
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
#include "Block.hpp"
// forward declarations, if any
enum class Direction : unsigned;

/// Corblivar basic data structure: corner block list
class CornerBlockList {
	private:
		/// debugging code switch (private)
		static constexpr bool DBG = false;

	// private data, functions
	private:
		/// CBL sequence as in [Hong00]
		std::vector<Block const*> S;
		/// CBL sequence as in [Hong00]
		std::vector<Direction> L;
		/// CBL sequence as in [Hong00]
		std::vector<unsigned> T;

	// constructors, destructors, if any non-implicit
	public:

	// public data, functions
	public:
		friend class CorblivarCore;
		friend class CorblivarDie;

		/// POD; wrapper for tuples of separate sequences
		struct Tuple {
			Block const* S;
			Direction L;
			unsigned T;
		};

		/// getter
		inline unsigned size() const {

			if (DBG) {
				unsigned ret;
				bool mismatch = false;
				unsigned prev_ret;

				prev_ret = ret = this->S.size();
				ret = std::min<unsigned>(ret, this->L.size());
				mismatch = (ret != prev_ret);
				prev_ret = ret;
				ret = std::min<unsigned>(ret, this->T.size());
				mismatch = mismatch || (ret != prev_ret);

				if (mismatch) {
					std::cout << "DBG_CBL> CBL has sequences size mismatch!" << std::endl;
					std::cout << "DBG_CBL> CBL: " << std::endl;
					std::cout << this->CBLString() << std::endl;
				}
			}

			return this->S.size();
		};

		/// getter
		inline unsigned capacity() const {
			return this->S.capacity();
		};

		/// getter
		inline bool empty() const {
			return this->S.empty();
		};

		/// reset
		inline void clear() {
			this->S.clear();
			this->L.clear();
			this->T.clear();
		};

		/// allocate memory
		inline void reserve(unsigned const& elements) {
			this->S.reserve(elements);
			this->L.reserve(elements);
			this->T.reserve(elements);
		};

		/// insert tuple into CBL
		inline void insert(Tuple&& tuple) {
			this->S.push_back(tuple.S);
			this->L.push_back(tuple.L);
			this->T.push_back(tuple.T);
		};

		/// tuple string
		inline std::string tupleString(unsigned const& tuple) const {
			std::stringstream ret;

			ret << "tuple " << tuple << " : ";
			ret << "( " << this->S[tuple]->id << " " << static_cast<unsigned>(this->L[tuple]) << " " << this->T[tuple] << " ";
			ret << this->S[tuple]->bb.w << " " << this->S[tuple]->bb.h << " )";

			return ret.str();
		};

		/// string for whole CBL
		inline std::string CBLString() const {
			unsigned i;
			std::stringstream ret;

			for (i = 0; i < this->size(); i++) {
				ret << this->tupleString(i) << "; ";
			}

			return ret.str();
		};
};

#endif
