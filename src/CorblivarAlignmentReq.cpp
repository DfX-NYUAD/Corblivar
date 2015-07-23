/*
 * =====================================================================================
 *
 *    Description:  Corblivar alignment requests data
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

// own Corblivar header
#include "CorblivarAlignmentReq.hpp"
// required Corblivar headers
#include "Math.hpp"

CorblivarAlignmentReq::Evaluate CorblivarAlignmentReq::evaluate() const {
	Rect blocks_bb;
	Rect blocks_intersect;
	Evaluate ret;

	// initially, assume zero cost / alignment mismatch
	ret.cost = 0.0;

	// initially, assume the request to be feasible
	this->fulfilled = true;

	// also assume alignment status of blocks themselves to be successful
	this->s_i->alignment = Block::AlignmentStatus::SUCCESS;
	this->s_j->alignment = Block::AlignmentStatus::SUCCESS;

	// for request w/ alignment ranges, we verify the alignment via the
	// blocks' intersection
	if (this->range_x() || this->range_y()) {
		blocks_intersect = Rect::determineIntersection(this->s_i->bb, this->s_j->bb);
	}
	// for requests w/ max distance ranges, we verify the alignment via the
	// blocks' bounding box (considering the blocks' center points)
	if (this->range_max_x() || this->range_max_y()) {
		blocks_bb = Rect::determBoundingBox(this->s_i->bb, this->s_j->bb, true);
	}

	// check partial request, horizontal alignment
	//
	// alignment range
	if (this->range_x()) {

		// consider the spatial mismatch as cost; overlap too small
		if (blocks_intersect.w < this->alignment_x) {

			// missing overlap
			ret.cost += this->alignment_x - blocks_intersect.w;

			// in case blocks don't overlap at all, also consider the
			// blocks' distance as further cost
			if (blocks_intersect.w == 0) {

				if (Rect::rectA_leftOf_rectB(this->s_i->bb, this->s_j->bb, false)) {

					ret.cost += this->s_j->bb.ll.x - this->s_i->bb.ur.x;

					// annotate block-alignment failure
					this->s_i->alignment = Block::AlignmentStatus::FAIL_HOR_TOO_LEFT;
					this->s_j->alignment = Block::AlignmentStatus::FAIL_HOR_TOO_RIGHT;
				}
				else {

					ret.cost += this->s_i->bb.ll.x - this->s_j->bb.ur.x;

					// annotate block-alignment failure
					this->s_i->alignment = Block::AlignmentStatus::FAIL_HOR_TOO_RIGHT;
					this->s_j->alignment = Block::AlignmentStatus::FAIL_HOR_TOO_LEFT;
				}
			}

			// annotate general alignment failure
			this->fulfilled = false;
		}
	}
	// max distance range
	else if (this->range_max_x()) {

		// consider the spatial mismatch as cost; distance too large
		if (blocks_bb.w > this->alignment_x) {

			ret.cost += blocks_bb.w - this->alignment_x;

			// annotate general alignment failure
			this->fulfilled = false;

			// annotate block-alignment failure
			if (this->s_i->bb.ll.x < this->s_j->bb.ll.x) {
				this->s_i->alignment = Block::AlignmentStatus::FAIL_HOR_TOO_LEFT;
				this->s_j->alignment = Block::AlignmentStatus::FAIL_HOR_TOO_RIGHT;
			}
			else {
				this->s_i->alignment = Block::AlignmentStatus::FAIL_HOR_TOO_RIGHT;
				this->s_j->alignment = Block::AlignmentStatus::FAIL_HOR_TOO_LEFT;
			}
		}
	}
	// fixed alignment offset
	else if (this->offset_x()) {

		// check the blocks' offset against the required offset
		if (!Math::doubleComp(this->s_j->bb.ll.x - this->s_i->bb.ll.x, this->alignment_x)) {

			// s_j should be to the right of s_i;
			// consider the spatial mismatch as cost
			if (this->alignment_x >= 0.0) {

				// s_j is to the right of s_i
				if (this->s_j->bb.ll.x > this->s_i->bb.ll.x) {

					// abs required for cases where s_j is too
					// far left, i.e., not sufficiently away
					// from s_i
					ret.cost += std::abs(this->s_j->bb.ll.x - this->s_i->bb.ll.x - this->alignment_x);

					// annotate block-alignment failure;
					// s_j is too far left, s_i too far right
					if ((this->s_j->bb.ll.x - this->s_i->bb.ll.x - this->alignment_x) < 0) {
						this->s_i->alignment = Block::AlignmentStatus::FAIL_HOR_TOO_RIGHT;
						this->s_j->alignment = Block::AlignmentStatus::FAIL_HOR_TOO_LEFT;
					}
					// s_j is too far right, s_i too far left
					else {
						this->s_i->alignment = Block::AlignmentStatus::FAIL_HOR_TOO_LEFT;
						this->s_j->alignment = Block::AlignmentStatus::FAIL_HOR_TOO_RIGHT;
					}
				}
				// s_j is to the left of s_i
				else {
					// cost includes distance b/w (right) s_i,
					// (left) s_j and the failed offset
					ret.cost += this->s_i->bb.ll.x - this->s_j->bb.ll.x + this->alignment_x;

					// annotate block-alignment failure
					this->s_i->alignment = Block::AlignmentStatus::FAIL_HOR_TOO_RIGHT;
					this->s_j->alignment = Block::AlignmentStatus::FAIL_HOR_TOO_LEFT;
				}
			}
			// s_j should be to the left of s_i;
			// consider the spatial mismatch as cost
			else {

				// s_j is to the left of s_i
				if (this->s_j->bb.ll.x < this->s_i->bb.ll.x) {

					// abs required for cases where s_j is too
					// far right, i.e., not sufficiently away
					// from s_i
					ret.cost += std::abs(this->s_i->bb.ll.x - this->s_j->bb.ll.x + this->alignment_x);

					// annotate block-alignment failure;
					// s_j is too far right, s_i too far left
					if ((this->s_i->bb.ll.x - this->s_j->bb.ll.x + this->alignment_x) < 0) {
						this->s_i->alignment = Block::AlignmentStatus::FAIL_HOR_TOO_LEFT;
						this->s_j->alignment = Block::AlignmentStatus::FAIL_HOR_TOO_RIGHT;
					}
					// s_j is too far left, s_i too far right
					else {
						this->s_i->alignment = Block::AlignmentStatus::FAIL_HOR_TOO_RIGHT;
						this->s_j->alignment = Block::AlignmentStatus::FAIL_HOR_TOO_LEFT;
					}
				}
				// s_j is right of s_i
				else {
					// cost includes distance b/w (left) s_i,
					// (right) s_j and the failed (negative) offset
					ret.cost += this->s_j->bb.ll.x - this->s_i->bb.ll.x - this->alignment_x;

					// annotate block-alignment failure
					this->s_i->alignment = Block::AlignmentStatus::FAIL_HOR_TOO_LEFT;
					this->s_j->alignment = Block::AlignmentStatus::FAIL_HOR_TOO_RIGHT;
				}
			}

			// annotate general alignment failure
			this->fulfilled = false;
		}
	}

	// check partial request, vertical alignment
	//
	// alignment range
	if (this->range_y()) {

		// consider the spatial mismatch as cost; overlap too small
		if (blocks_intersect.h < this->alignment_y) {

			// missing overlap
			ret.cost += this->alignment_y - blocks_intersect.h;

			// in case blocks don't overlap at all, also consider the
			// blocks' distance as further cost
			if (blocks_intersect.h == 0) {

				if (Rect::rectA_below_rectB(this->s_i->bb, this->s_j->bb, false)) {

					ret.cost += this->s_j->bb.ll.y - this->s_i->bb.ur.y;

					// annotate block-alignment failure
					this->s_i->alignment = Block::AlignmentStatus::FAIL_VERT_TOO_LOW;
					this->s_j->alignment = Block::AlignmentStatus::FAIL_VERT_TOO_HIGH;
				}
				else {

					ret.cost += this->s_i->bb.ll.y - this->s_j->bb.ur.y;

					// annotate block-alignment failure
					this->s_i->alignment = Block::AlignmentStatus::FAIL_VERT_TOO_HIGH;
					this->s_j->alignment = Block::AlignmentStatus::FAIL_VERT_TOO_LOW;
				}
			}

			// annotate general alignment failure
			this->fulfilled = false;
		}
	}
	// max distance range
	else if (this->range_max_y()) {

		// consider the spatial mismatch as cost; distance too large
		if (blocks_bb.h > this->alignment_y) {

			ret.cost += blocks_bb.h - this->alignment_y;

			// annotate general alignment failure
			this->fulfilled = false;

			// annotate block-alignment failure
			if (this->s_i->bb.ll.y < this->s_j->bb.ll.y) {
				this->s_i->alignment = Block::AlignmentStatus::FAIL_VERT_TOO_LOW;
				this->s_j->alignment = Block::AlignmentStatus::FAIL_VERT_TOO_HIGH;
			}
			else {
				this->s_i->alignment = Block::AlignmentStatus::FAIL_VERT_TOO_HIGH;
				this->s_j->alignment = Block::AlignmentStatus::FAIL_VERT_TOO_LOW;
			}
		}
	}
	// fixed alignment offset
	else if (this->offset_y()) {

		// check the blocks' offset against the required offset
		if (!Math::doubleComp(this->s_j->bb.ll.y - this->s_i->bb.ll.y, this->alignment_y)) {

			// s_j should be above s_i;
			// consider the spatial mismatch as cost
			if (this->alignment_y >= 0.0) {

				// s_j is above s_i
				if (this->s_j->bb.ll.y > this->s_i->bb.ll.y) {

					// abs required for cases where s_j is too
					// far lowerwards, i.e., not sufficiently
					// away from s_i
					ret.cost += std::abs(this->s_j->bb.ll.y - this->s_i->bb.ll.y - this->alignment_y);

					// annotate block-alignment failure;
					// s_j is too far lowerwards, s_i too far upwards
					if ((this->s_j->bb.ll.y - this->s_i->bb.ll.y - this->alignment_y) < 0) {
						this->s_i->alignment = Block::AlignmentStatus::FAIL_VERT_TOO_HIGH;
						this->s_j->alignment = Block::AlignmentStatus::FAIL_VERT_TOO_LOW;
					}
					// s_j is too far upwards, s_i too far
					// lowerwards
					else {
						this->s_i->alignment = Block::AlignmentStatus::FAIL_VERT_TOO_LOW;
						this->s_j->alignment = Block::AlignmentStatus::FAIL_VERT_TOO_HIGH;
					}
				}
				// s_j is below s_i
				else {
					// cost includes distance b/w (upper) s_i,
					// (lower) s_j and the failed offset
					ret.cost += this->s_i->bb.ll.y - this->s_j->bb.ll.y + this->alignment_y;

					// annotate block-alignment failure
					this->s_i->alignment = Block::AlignmentStatus::FAIL_VERT_TOO_HIGH;
					this->s_j->alignment = Block::AlignmentStatus::FAIL_VERT_TOO_LOW;
				}
			}
			// s_j should be below s_i;
			// consider the spatial mismatch as cost
			else {

				// s_j is below s_i
				if (this->s_j->bb.ll.y < this->s_i->bb.ll.y) {

					// abs required for cases where s_j is too
					// far upwards, i.e., not sufficiently
					// away from s_i
					ret.cost += std::abs(this->s_i->bb.ll.y - this->s_j->bb.ll.y + this->alignment_y);

					// annotate block-alignment failure;
					// s_j is too far upwards, s_i too far
					// lowerwards
					if ((this->s_i->bb.ll.y - this->s_j->bb.ll.y + this->alignment_y) < 0) {
						this->s_i->alignment = Block::AlignmentStatus::FAIL_VERT_TOO_LOW;
						this->s_j->alignment = Block::AlignmentStatus::FAIL_VERT_TOO_HIGH;
					}
					// s_j is too far lowerwards, s_i too far
					// upwards
					else {
						this->s_i->alignment = Block::AlignmentStatus::FAIL_VERT_TOO_HIGH;
						this->s_j->alignment = Block::AlignmentStatus::FAIL_VERT_TOO_LOW;
					}
				}
				// s_j is above s_i
				else {
					// cost includes distance b/w (lower) s_i,
					// (upper) s_j and the failed (negative) offset
					ret.cost += this->s_j->bb.ll.y - this->s_i->bb.ll.y - this->alignment_y;

					// annotate block-alignment failure
					this->s_i->alignment = Block::AlignmentStatus::FAIL_VERT_TOO_LOW;
					this->s_j->alignment = Block::AlignmentStatus::FAIL_VERT_TOO_HIGH;
				}
			}

			// annotate general alignment failure
			this->fulfilled = false;
		}
	}

	// memorize the actual mismatch
	ret.actual_mismatch = ret.cost;

	// weight the cost w/ signals count
	ret.cost *= this->signals;

	// dbg logging for alignment
	if (CorblivarAlignmentReq::DBG_EVALUATE) {

		std::cout << "DBG_ALIGNMENT> " << this->tupleString() << std::endl;

		if (this->fulfilled) {
			std::cout << "DBG_ALIGNMENT>  Success" << std::endl;
		}
		else {
			std::cout << "DBG_ALIGNMENT>  Failure" << std::endl;
			std::cout << "DBG_ALIGNMENT>   block " << this->s_i->id << ": " << this->s_i->alignment << std::endl;
			std::cout << "DBG_ALIGNMENT>   block " << this->s_j->id << ": " << this->s_j->alignment << std::endl;
			std::cout << "DBG_ALIGNMENT>   actual mismatch: " << ret.actual_mismatch << std::endl;
			std::cout << "DBG_ALIGNMENT>   weighted cost: " << ret.cost << std::endl;
		}
	}

	return ret;
}

bool CorblivarAlignmentReq::vertical_bus() const {
	return (
		// min overlap in both dimensions
		(this->range_x() && this->range_y()) ||
		// zero-offset fixed alignment in both dimensions
		(this->offset_x() && this->alignment_x == 0 && this->offset_y() && this->alignment_y == 0) ||
		// non-zero offset in both dimensions;
		(this->offset_x() && this->alignment_x != 0.0 && this->offset_y() && this->alignment_y != 0.0
			// but with sufficiently small offset such that blocks will
			// partially intersect
			&& (
				// positive offset: b_j not further offset to the right/top than
				// b_i is wide/high; negative offset: b_j not further offset to
				// the left/bottom than b_j is wide/high
				(this->alignment_x > 0.0) ? this->alignment_x < this->s_i->bb.w : this->alignment_x > -(this->s_j->bb.w) &&
				(this->alignment_y > 0.0) ? this->alignment_y < this->s_i->bb.h : this->alignment_y > -(this->s_j->bb.h)
			   )
		)
	);
}
