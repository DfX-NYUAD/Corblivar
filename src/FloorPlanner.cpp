/*
 * =====================================================================================
 *
 *    Description:  Corblivar floorplanner (SA operations and related handler)
 *
 *         Author:  Johann Knechtel, johann.knechtel@ifte.de
 *        Company:  Institute of Electromechanical and Electronic Design, www.ifte.de
 *
 * =====================================================================================
 */

// own Corblivar header
#include "FloorPlanner.hpp"
// required Corblivar headers
#include "Point.hpp"
#include "Math.hpp"
#include "CorblivarCore.hpp"
#include "Net.hpp"
#include "IO.hpp"

// main handler
bool FloorPlanner::performSA(CorblivarCore& corb) {
	int i, ii;
	int innerLoopMax;
	int accepted_ops;
	double accepted_ops_ratio;
	bool op_success;
	double cur_cost, best_cost, prev_cost, cost_diff, avg_cost, fitting_cost;
	Cost cost;
	vector<double> cost_samples;
	double cur_temp, init_temp;
	double r;
	int layout_fit_counter;
	double layout_fit_ratio;
	bool valid_layout_found;
	int i_valid_layout_found;
	bool best_sol_found;
	bool accept;
	bool SA_phase_two, SA_phase_two_init;

	if (FloorPlanner::DBG_CALLS_SA) {
		cout << "-> FloorPlanner::performSA(" << &corb << ")" << endl;
	}

	// for handling floorplacement benchmarks, i.e., floorplanning w/ very large
	// blocks, we handle this naively by preferring these large blocks in the lower
	// left corner, i.e., perform a sorting of the sequences by block size
	//
	// also, for random layout operations in SA phase one, these blocks are not
	// allowed to be swapped or moved, see performOpSwapBlocks, performOpMoveTuple
	if (this->conf_SA_layout_floorplacement) {
		corb.sortCBLs(this->logMed(), CorblivarCore::SORT_CBLS_BY_BLOCKS_SIZE);
	}

	// init SA: initial sampling; setup parameters, setup temperature schedule
	this->initSA(corb, cost_samples, innerLoopMax, init_temp);

	/// main SA loop
	//
	// init loop parameters
	i = 1;
	cur_temp = init_temp;
	SA_phase_two = SA_phase_two_init = false;
	valid_layout_found = false;
	i_valid_layout_found = Point::UNDEF;
	layout_fit_ratio = 0.0;
	// dummy large value to accept first fitting solution
	best_cost = 100.0 * Math::stdDev(cost_samples);

	/// outer loop: annealing -- temperature steps
	while (i <= this->conf_SA_loopLimit) {

		if (this->logMax()) {
			cout << "SA> Optimization step: " << i << "/" << this->conf_SA_loopLimit << endl;
		}

		// init loop parameters
		ii = 1;
		avg_cost = 0.0;
		accepted_ops = 0;
		layout_fit_counter = 0.0;
		SA_phase_two_init = false;
		best_sol_found = false;

		// init cost for current layout and fitting ratio
		corb.generateLayout();
		cur_cost = this->determCost(layout_fit_ratio, SA_phase_two).cost;

		// inner loop: layout operations
		while (ii <= innerLoopMax) {

			// perform random layout op
			op_success = this->performRandomLayoutOp(corb, SA_phase_two);

			if (op_success) {

				prev_cost = cur_cost;

				// generate layout
				corb.generateLayout();

				// evaluate layout, new cost
				cost = this->determCost(layout_fit_ratio, SA_phase_two);
				cur_cost = cost.cost;
				// cost difference
				cost_diff = cur_cost - prev_cost;

				if (FloorPlanner::DBG_SA) {
					cout << "DBG_SA> Inner step: " << ii << "/" << innerLoopMax << endl;
					cout << "DBG_SA> Cost diff: " << cost_diff << endl;
				}

				// revert solution w/ worse or same cost, depending on temperature
				accept = true;
				if (cost_diff >= 0.0) {
					r = Math::randF(0, 1);
					if (r > exp(- cost_diff / cur_temp)) {

						if (FloorPlanner::DBG_SA) {
							cout << "DBG_SA> Revert op" << endl;
						}
						accept = false;

						// revert last op
						this->performRandomLayoutOp(corb, SA_phase_two, true);
						// reset cost according to reverted CBL
						cur_cost = prev_cost;
					}
				}

				// solution to be accepted, i.e., previously not reverted
				if (accept) {
					// update ops count
					accepted_ops++;
					// sum up cost for subsequent avg determination
					avg_cost += cur_cost;

					if (cost.fits_fixed_outline) {
						// update count of solutions fitting into outline
						layout_fit_counter++;

						// switch to SA phase two when
						// first fitting solution is found
						if (!SA_phase_two) {
							SA_phase_two = SA_phase_two_init = true;
							// also memorize in which
							// iteration we found the first
							// valid layout
							i_valid_layout_found = i;

							// logging
							if (this->logMax()) {
								cout << "SA> " << endl;
							}
							if (this->logMed()) {
								cout << "SA> Phase II: optimizing within outline; switch cost function ..." << endl;
							}
							if (this->logMax()) {
								cout << "SA> " << endl;
							}
						}

						// in order to compare different fitting
						// solutions equally, redetermine cost w/
						// fitting ratio 1.0
						//
						// during switch to phase two, initialize
						// current cost as max cost for further
						// normalization (SA_phase_two_init)
						fitting_cost = this->determCost(1.0, SA_phase_two, SA_phase_two_init).cost;

						// memorize best solution which fits into outline
						if (fitting_cost < best_cost) {

							best_cost = fitting_cost;
							corb.storeBestCBLs();
							valid_layout_found = best_sol_found = true;
						}
					}
				}

				// after phase transition, skip current global iteration
				// in order to consider updated cost function
				if (SA_phase_two_init) {
					break;
				}
				// consider next loop iteration
				else {
					ii++;
				}
			}
		}

		// determine ratio of solutions fitting into outline in prev temp step;
		// note that during the temp step this ratio is fixed in order to avoid
		// sudden changes of related cost terms during few iterations
		layout_fit_ratio = static_cast<double>(layout_fit_counter) / accepted_ops;

		// determine avg cost for temp step
		avg_cost /= accepted_ops;
		// determine accepted-ops ratio
		accepted_ops_ratio = static_cast<double>(accepted_ops) / ii;

		if (this->logMax()) {
			cout << "SA> Step done:" << endl;
			cout << "SA>  new best solution found: " << best_sol_found << endl;
			cout << "SA>  accept-ops ratio: " << accepted_ops_ratio << endl;
			cout << "SA>  valid-layouts ratio: " << layout_fit_ratio << endl;
			cout << "SA>  avg cost: " << avg_cost << endl;
			cout << "SA>  temp: " << cur_temp << endl;
		}

		// log temperature step
		TempStep cur_step;
		cur_step.step = i;
		cur_step.temp = cur_temp;
		cur_step.avg_cost = avg_cost;
		cur_step.new_best_sol_found = best_sol_found;
		this->tempSchedule.push_back(move(cur_step));

		// update SA temperature
		this->updateTemp(cur_temp, i, i_valid_layout_found);

		// consider next outer step
		i++;
	}

	if (this->logMed()) {
		cout << "SA> Done" << endl;
		cout << endl;
	}

	if (FloorPlanner::DBG_CALLS_SA) {
		cout << "<- FloorPlanner::performSA : " << valid_layout_found << endl;
	}

	return valid_layout_found;
}

inline void FloorPlanner::updateTemp(double& cur_temp, int const& iteration, int const& iteration_first_valid_layout) const {
	double loop_factor;
	double prev_temp;
	int phase;
	vector<double> prev_avg_cost;
	double std_dev_avg_cost;
	unsigned i, temp_schedule_size;

	prev_temp = cur_temp;

	// consider reheating in case the SA search has converged in some (possibly local) minima
	//
	// determine std dev of avg cost of some previous temperature steps

	temp_schedule_size = this->tempSchedule.size();

	if (temp_schedule_size >= FloorPlanner::SA_REHEAT_COST_SAMPLES) {

		for (i = 1; i <= FloorPlanner::SA_REHEAT_COST_SAMPLES; i++) {
			prev_avg_cost.push_back(this->tempSchedule[temp_schedule_size - i].avg_cost);
		}

		std_dev_avg_cost = Math::stdDev(prev_avg_cost);
	}
	else {
		std_dev_avg_cost = FloorPlanner::SA_REHEAT_STD_DEV_COST_LIMIT + 1;
	}

	// phase 3; brief reheating due to cost convergence
	if (this->conf_SA_temp_factor_phase3 != 0.0 && std_dev_avg_cost <= FloorPlanner::SA_REHEAT_STD_DEV_COST_LIMIT) {
		cur_temp *= this->conf_SA_temp_factor_phase3;

		phase = 3;
	}
	// phase 1; adaptive cooling (slows down from conf_SA_temp_factor_phase1 to
	// conf_SA_temp_factor_phase1_limit)
	else if (iteration_first_valid_layout == Point::UNDEF) {
		loop_factor = (this->conf_SA_temp_factor_phase1_limit - this->conf_SA_temp_factor_phase1)
			* static_cast<double>(iteration - 1) / (this->conf_SA_loopLimit - 1.0);
		// note that loop_factor is additive in this case; the cooling factor is
		// increased w/ increasing iterations
		cur_temp *= this->conf_SA_temp_factor_phase1 + loop_factor;

		phase = 1;
	}
	// phase 2; reheating and converging (initially reheats and then increases cooling
	// rate faster, i.e., heating factor is decreased w/ increasing iterations to
	// enable convergence)
	else {
		// note that loop_factor must only consider the remaining iteration range
		loop_factor = 1.0 - static_cast<double>(iteration - iteration_first_valid_layout) /
			static_cast<double>(this->conf_SA_loopLimit - iteration_first_valid_layout);
		cur_temp *= this->conf_SA_temp_factor_phase2 * loop_factor;

		phase = 2;
	}

	if (this->logMax()) {
		cout << "SA>  (new) temp-update factor: " << cur_temp / prev_temp << " (phase " << phase << ")" << endl;
	}
}

void FloorPlanner::initSA(CorblivarCore& corb, vector<double>& cost_samples, int& innerLoopMax, double& init_temp) {
	int i;
	int accepted_ops;
	bool op_success;
	double cur_cost, prev_cost, cost_diff;
	Cost cost;

	// reset max cost
	this->max_cost_WL = 0.0;
	this->max_cost_TSVs = 0.0;
	this->max_cost_thermal = 0.0;
	this->max_cost_alignments = 0.0;

	// reset temperature-schedule log
	this->tempSchedule.clear();

	// backup initial CBLs
	corb.backupCBLs();

	// init SA parameter: inner loop ops
	innerLoopMax = pow(static_cast<double>(this->blocks.size()), this->conf_SA_loopFactor);

	/// initial sampling
	//
	if (this->logMed()) {
		cout << "SA> Perform initial solution-space sampling..." << endl;
	}

	// init cost
	corb.generateLayout();
	cur_cost = this->determCost().cost;

	// perform some random operations, for SA temperature = 0.0
	// i.e., consider only solutions w/ improved cost
	// track acceptance ratio and cost (phase one, area and AR mismatch)
	// also trigger cost function to assume no fitting layouts
	i = 1;
	accepted_ops = 0;
	cost_samples.reserve(SA_SAMPLING_LOOP_FACTOR * this->blocks.size());

	while (i <= SA_SAMPLING_LOOP_FACTOR * static_cast<int>(this->blocks.size())) {

		op_success = this->performRandomLayoutOp(corb);

		if (op_success) {

			prev_cost = cur_cost;

			// generate layout
			corb.generateLayout();

			// evaluate layout, new cost
			cost = this->determCost();
			cur_cost = cost.cost;
			// cost difference
			cost_diff = cur_cost - prev_cost;

			// solution w/ worse cost, revert
			if (cost_diff > 0.0) {
				// revert last op
				this->performRandomLayoutOp(corb, false, true);
				// reset cost according to reverted CBL
				cur_cost = prev_cost;
			}
			// accept solution w/ improved cost
			else {
				// update ops count
				accepted_ops++;
			}
			// store cost
			cost_samples.push_back(cur_cost);

			i++;
		}
	}

	// init SA parameter: start temp, depends on std dev of costs [Huan86, see
	// Shahookar91]
	init_temp = Math::stdDev(cost_samples) * this->conf_SA_temp_init_factor;

	if (this->logMed()) {
		cout << "SA> Done; std dev of cost: " << Math::stdDev(cost_samples) << ", initial temperature: " << init_temp << endl;
		cout << "SA> " << endl;
		cout << "SA> Perform simulated annealing process..." << endl;
		cout << "SA> Phase I: packing blocks into outline..." << endl;
		cout << "SA> " << endl;
	}

	// restore initial CBLs
	corb.restoreCBLs();
}

void FloorPlanner::finalize(CorblivarCore& corb, bool const& determ_overall_cost, bool const& handle_corblivar) {
	struct timeb end;
	stringstream runtime;
	bool valid_solution;
	double cost;
	double area, thermal;
	CostInterconn interconn;

	if (FloorPlanner::DBG_CALLS_SA) {
		cout << "-> FloorPlanner::finalize(" << &corb << ", " << determ_overall_cost << ")" << endl;
	}

	// consider CorblivarCore data
	if (handle_corblivar) {
		// apply best solution, if available, as final solution
		valid_solution = corb.applyBestCBLs(this->logMin());
		// generate final layout
		corb.generateLayout();
	}

	// determine final cost, also for non-Corblivar calls
	if (!handle_corblivar || valid_solution) {

		// determine overall cost
		if (determ_overall_cost) {
			cost = this->determCost(1.0, true).cost;
		}

		// determine area cost; invert weight in order to retrieve area ratio
		// (max blocks-outline / die-outline ratio)
		area = (1.0 / FloorPlanner::SA_COST_WEIGHT_AREA_OUTLINE) * this->determWeightedCostAreaOutline(1.0).cost;

		// determine non-normalized WL and TSVs cost
		interconn = this->determCostInterconnects(false, false);

		// determine non-normalized temperature cost
		thermal = this->determCostThermalDistr(false, false);

		// TODO alignment costs
		if (this->logMin()) {

			cout << "SA> Characteristica of final solution:" << endl;

			if (determ_overall_cost) {
				cout << "SA> Final (adapted) cost: " << cost << endl;
			}

			cout << "SA> Max blocks-outline / die-outline ratio: " << area << endl;

			cout << "SA> HPWL: " << interconn.HPWL << endl;
			cout << "SA> TSVs: " << interconn.TSVs << endl;
			cout << "SA>  Deadspace utilization by TSVs: " << interconn.TSVs_area_deadspace_ratio << endl;

			if (this->power_density_file_avail) {
				cout << "SA> Temp cost (no real temp): " << thermal << endl;
			}
			cout << endl;

			if (determ_overall_cost) {
				this->results << "Final (adapted) cost: " << cost << endl;
			}
			this->results << "Max die occupation [\%]: " << area << endl;
			this->results << "HPWL: " << interconn.HPWL << endl;
			this->results << "TSVs: " << interconn.TSVs << endl;
			this->results << "Deadspace utilization by TSVs: " << interconn.TSVs_area_deadspace_ratio << endl;
			if (this->power_density_file_avail) {
				this->results << "Temp cost (no real temp): " << thermal << endl;
			}
		}
	}

	// generate temperature-schedule data
	IO::writeTempSchedule(*this);

	// generate floorplan plots
	IO::writeFloorplanGP(*this);

	// generate Corblivar data if solution file is used as output
	if (handle_corblivar && this->solution_out.is_open()) {
		this->solution_out << corb.CBLsString() << endl;
		this->solution_out.close();
	}

	// thermal-analysis files
	if ((!handle_corblivar || valid_solution) && this->power_density_file_avail) {
		// generate power and thermal maps
		IO::writePowerThermalMaps(*this);
		// generate HotSpot files
		IO::writeHotSpotFiles(*this);
	}

	// determine overall runtime
	ftime(&end);
	if (this->logMin()) {
		runtime << "Runtime: " << (1000.0 * (end.time - this->start.time) + (end.millitm - this->start.millitm)) / 1000.0 << " s";
		cout << "Corblivar> " << runtime.str() << endl;
		this->results << runtime.str() << endl;
	}

	// close results file
	this->results.close();

	if (FloorPlanner::DBG_CALLS_SA) {
		cout << "<- FloorPlanner::finalize" << endl;
	}
}

bool FloorPlanner::performRandomLayoutOp(CorblivarCore& corb, bool const& SA_phase_two, bool const& revertLastOp) {
	int op;
	int die1, die2, tuple1, tuple2, juncts;
	bool ret;

	if (FloorPlanner::DBG_CALLS_SA) {
		cout << "-> FloorPlanner::performRandomLayoutOp(" << &corb << ", " << SA_phase_two << ", " << revertLastOp << ")" << endl;
	}

	// revert last op
	if (revertLastOp) {
		op = this->last_op;
	}
	// perform new, random op
	else {
		// see defined op-codes in class FloorPlanner to set random-number ranges;
		// recall that randI(x,y) is [x,y)
		//
		// for SA phase two, we consider an extended set of operations related to
		// thermal optimiziation; the flag
		// conf_SA_layout_power_guided_block_swapping also considers if thermal
		// optimization is enabled and if a power density file is available
		if (SA_phase_two && this->conf_SA_layout_power_guided_block_swapping) {
			this->last_op = op = Math::randI(1, 7);
		}
		// SA phase one, reduced set of operations
		else {
			this->last_op = op = Math::randI(1, 6);
		}
	}

	die1 = die2 = tuple1 = tuple2 = juncts = -1;

	// specific op handler
	switch (op) {

		case FloorPlanner::OP_SWAP_BLOCKS: // op-code: 1

			ret = this->performOpSwapBlocks(revertLastOp, !SA_phase_two, corb, die1, die2, tuple1, tuple2);

			break;

		case FloorPlanner::OP_SWAP_HOT_COLD_BLOCKS: // op-code: 6

			// relates to SA phase two; swap a hot block from the lower dies
			// w/ a cold block on the upper dies
			ret = this->performOpSwapHotColdBlocks(revertLastOp, corb, die1, die2, tuple1, tuple2);

			break;

		case FloorPlanner::OP_MOVE_TUPLE: // op-code: 2

			ret = this->performOpMoveTuple(revertLastOp, !SA_phase_two, corb, die1, die2, tuple1, tuple2);

			break;

		case FloorPlanner::OP_SWITCH_INSERTION_DIR: // op-code: 3

			ret = this->performOpSwitchInsertionDirection(revertLastOp, corb, die1, tuple1);

			break;

		case FloorPlanner::OP_SWITCH_TUPLE_JUNCTS: // op-code: 4

			ret = this->performOpSwitchTupleJunctions(revertLastOp, corb, die1, tuple1, juncts);

			break;

		case FloorPlanner::OP_ROTATE_BLOCK__SHAPE_BLOCK: // op-code: 5

			ret = this->performOpShapeBlock(revertLastOp, corb, die1, tuple1);

			break;
	}

	// memorize elements of successful op
	if (ret) {
		this->last_op_die1 = die1;
		this->last_op_die2 = die2;
		this->last_op_tuple1 = tuple1;
		this->last_op_tuple2 = tuple2;
		this->last_op_juncts = juncts;
	}

	if (FloorPlanner::DBG_CALLS_SA) {
		cout << "<- FloorPlanner::performRandomLayoutOp : " << ret << endl;
	}

	return ret;
}


bool FloorPlanner::performOpShapeBlock(bool const& revert, CorblivarCore& corb, int& die1, int& tuple1) const {
	Block const* shape_block;
	double col_max_width, row_max_height;
	double gain, loss;

	if (!revert) {
		die1 = Math::randI(0, corb.diesSize());
		// sanity check for empty dies
		if (corb.getDie(die1).getCBL().empty()) {
			return false;
		}

		tuple1 = Math::randI(0, corb.getDie(die1).getCBL().size());
		// related block to be shaped
		shape_block = corb.getDie(die1).getBlock(tuple1);

		// TODO soft blocks: block shaping
		if (shape_block->soft) {
			corb.rotateBlock(die1, tuple1);
		}
		// hard blocks: simple rotation or enhanced rotation: perform block
		// rotation only if layout compaction is achievable; note that enhanced
		// rotation relies on non-compacted, i.e., non-packed layouts, this is
		// ensured during config file parsing
		else {
			// enhanced rotation
			if (this->conf_SA_layout_enhanced_hard_block_rotation) {

				// horizontal block
				if (shape_block->bb.w > shape_block->bb.h) {
					// check blocks in (implicitly constructed) row
					row_max_height = shape_block->bb.h;
					for (Block const& b : this->blocks) {
						if (b.layer != shape_block->layer) {
							continue;
						}
						if (shape_block->bb.ll.y == b.bb.ll.y) {
							row_max_height = max(row_max_height, b.bb.h);
						}
					}
					// gain in horizontal direction by rotation
					gain = shape_block->bb.w - shape_block->bb.h;
					// loss in vertical direction; only if new block height
					// (current width) would be larger than the row currently
					// high is
					loss = shape_block->bb.w - row_max_height;
				}
				// vertical block
				else {
					// check blocks in (implicitly constructed) column
					col_max_width = shape_block->bb.w;
					for (Block const& b : this->blocks) {
						if (b.layer != shape_block->layer) {
							continue;
						}
						if (shape_block->bb.ll.x == b.bb.ll.x) {
							col_max_width = max(col_max_width, b.bb.w);
						}
					}
					// gain in vertical direction by rotation
					gain = shape_block->bb.h - shape_block->bb.w;
					// loss in horizontal direction; only if new block width
					// (current height) would be larger than the column
					// currently wide is
					loss = shape_block->bb.h - col_max_width;
				}

				// perform rotation if no loss or gain > loss
				if (loss < 0.0 || gain > loss) {
					corb.rotateBlock(die1, tuple1);
				}
				else {
					return false;
				}

			}
			// simple rotation, just perform rotation
			else {
				corb.rotateBlock(die1, tuple1);
			}
		}
	}
	// revert last rotation
	else {
		shape_block = corb.getDie(this->last_op_die1).getBlock(this->last_op_tuple1);

		// TODO adapt reverting for block shaping
		if (shape_block->soft) {
			corb.rotateBlock(this->last_op_die1, this->last_op_tuple1);
		}
		else {
			corb.rotateBlock(this->last_op_die1, this->last_op_tuple1);
		}
	}

	return true;
}

bool FloorPlanner::performOpSwitchTupleJunctions(bool const& revert, CorblivarCore& corb, int& die1, int& tuple1, int& juncts) const {
	int new_juncts;

	if (!revert) {
		die1 = Math::randI(0, corb.diesSize());
		// sanity check for empty dies
		if (corb.getDie(die1).getCBL().empty()) {
			return false;
		}

		tuple1 = Math::randI(0, corb.getDie(die1).getCBL().size());
		// juncts is return-by-reference, new_juncts for updating junctions
		new_juncts = juncts = corb.getDie(die1).getJunctions(tuple1);

		// junctions must be geq 0
		if (new_juncts == 0) {
			new_juncts++;
		}
		else {
			if (Math::randB()) {
				new_juncts++;
			}
			else {
				new_juncts--;
			}
		}

		corb.switchTupleJunctions(die1, tuple1, new_juncts);
	}
	else {
		corb.switchTupleJunctions(this->last_op_die1, this->last_op_tuple1, this->last_op_juncts);
	}

	return true;
}

bool FloorPlanner::performOpSwitchInsertionDirection(bool const& revert, CorblivarCore& corb, int& die1, int& tuple1) const {
	if (!revert) {
		die1 = Math::randI(0, corb.diesSize());
		// sanity check for empty dies
		if (corb.getDie(die1).getCBL().empty()) {
			return false;
		}

		tuple1 = Math::randI(0, corb.getDie(die1).getCBL().size());

		corb.switchInsertionDirection(die1, tuple1);
	}
	else {
		corb.switchInsertionDirection(this->last_op_die1, this->last_op_tuple1);
	}

	return true;
}

bool FloorPlanner::performOpMoveTuple(bool const& revert, bool const& SA_phase_one, CorblivarCore& corb, int& die1, int& die2, int& tuple1, int& tuple2) const {
	if (!revert) {
		die1 = Math::randI(0, corb.diesSize());
		die2 = Math::randI(0, corb.diesSize());
		// sanity check for empty (origin) die
		if (corb.getDie(die1).getCBL().empty()) {
			return false;
		}

		tuple1 = Math::randI(0, corb.getDie(die1).getCBL().size());
		tuple2 = Math::randI(0, corb.getDie(die2).getCBL().size());

		// in case of moving w/in same die, ensure that tuples are
		// different
		if (die1 == die2) {
			// this is, however, only possible if at least two
			// tuples are given in that die
			if (corb.getDie(die1).getCBL().size() < 2) {
				return false;
			}
			// determine two different tuples
			while (tuple1 == tuple2) {
				tuple2 = Math::randI(0, corb.getDie(die1).getCBL().size());
			}
		}

		// for SA phase one, floorplacement blocks, i.e., large macros, should not
		// be moved
		if (this->conf_SA_layout_floorplacement && SA_phase_one
				&& (corb.getDie(die1).getBlock(tuple1)->floorplacement || corb.getDie(die2).getBlock(tuple2)->floorplacement)) {
			return false;
		}
		else {
			corb.moveTuples(die1, die2, tuple1, tuple2);
		}
	}
	else {
		corb.moveTuples(this->last_op_die2, this->last_op_die1, this->last_op_tuple2, this->last_op_tuple1);
	}

	return true;
}

bool FloorPlanner::performOpSwapBlocks(bool const& revert, bool const& SA_phase_one, CorblivarCore& corb, int& die1, int& die2, int& tuple1, int& tuple2) const {
	if (!revert) {
		die1 = Math::randI(0, corb.diesSize());
		die2 = Math::randI(0, corb.diesSize());
		// sanity check for empty dies
		if (corb.getDie(die1).getCBL().empty() || corb.getDie(die2).getCBL().empty()) {
			return false;
		}

		tuple1 = Math::randI(0, corb.getDie(die1).getCBL().size());
		tuple2 = Math::randI(0, corb.getDie(die2).getCBL().size());

		// in case of swaps w/in same die, ensure that tuples are different
		if (die1 == die2) {
			// this is, however, only possible if at least two
			// tuples are given in that die
			if (corb.getDie(die1).getCBL().size() < 2) {
				return false;
			}
			// determine two different tuples
			while (tuple1 == tuple2) {
				tuple2 = Math::randI(0, corb.getDie(die1).getCBL().size());
			}
		}

		// for SA phase one, floorplacement blocks, i.e., large macros, should not
		// be swapped
		if (this->conf_SA_layout_floorplacement && SA_phase_one
				&& (corb.getDie(die1).getBlock(tuple1)->floorplacement || corb.getDie(die2).getBlock(tuple2)->floorplacement)) {
			return false;
		}
		else {
			corb.swapBlocks(die1, die2, tuple1, tuple2);
		}
	}
	else {
		corb.swapBlocks(this->last_op_die2, this->last_op_die1, this->last_op_tuple2, this->last_op_tuple1);
	}

	return true;
}

bool FloorPlanner::performOpSwapHotColdBlocks(bool const& revert, CorblivarCore& corb, int& die1, int& die2, int& tuple1, int& tuple2) const {
	int middle_die;
	unsigned tries;

	if (!revert) {
		// determine middle (boundary) die by int division, i.e, results in lower
		// number for uneven layer count, thus range for lower-die selection is
		// biased towards the lower stack, i.e., the hotter stack region
		middle_die = this->conf_layer / 2;
		// random lower die
		die1 = Math::randI(0, middle_die);
		// random upper die
		die2 = Math::randI(middle_die, corb.diesSize());

		// sanity check for empty dies
		if (corb.getDie(die1).getCBL().empty() || corb.getDie(die2).getCBL().empty()) {
			return false;
		}

		// power density boundary, i.e., blocks above this value should be swapped
		double const power_density_boundary = this->blocks_power_density_stats.avg;

		// distinct dies, i.e., swapping w/in dies and thus
		// checking for different tuples is not necessary

		// determine random tuple on lower die
		tries = 0;
		while (tries < corb.getDie(die1).getCBL().size()) {

			tuple1 = Math::randI(0, corb.getDie(die1).getCBL().size());

			// check whether tuple has large power density
			if (corb.getDie(die1).getBlock(tuple1)->power_density > power_density_boundary) {
				break;
			}
			// try next tuple
			else {
				tries++;
			}
		}
		// no large power block on lower die
		if (tries == corb.getDie(die1).getCBL().size() - 1) {
			return false;
		}

		// determine random tuple on upper die
		tries = 0;
		while (tries < corb.getDie(die2).getCBL().size()) {

			tuple2 = Math::randI(0, corb.getDie(die2).getCBL().size());

			// check whether tuple has low power density
			if (corb.getDie(die2).getBlock(tuple2)->power_density < power_density_boundary) {
				break;
			}
			// try next tuple
			else {
				tries++;
			}
		}
		// no low power block on upper die
		if (tries == corb.getDie(die2).getCBL().size() - 1) {
			return false;
		}

		corb.swapBlocks(die1, die2, tuple1, tuple2);
	}
	else {
		corb.swapBlocks(this->last_op_die2, this->last_op_die1, this->last_op_tuple2, this->last_op_tuple1);
	}

	return true;
}

// adaptive cost model w/ two phases;
// first phase considers only cost for packing into outline
// second phase considers further factors like WL, thermal distr, etc.
FloorPlanner::Cost FloorPlanner::determCost(double const& ratio_feasible_solutions_fixed_outline, bool const& SA_phase_two, bool const& set_max_cost) {
	double cost_total, cost_thermal, cost_alignments;
	CostInterconn cost_interconnects;
	Cost cost_area_outline, ret;

	if (FloorPlanner::DBG_CALLS_SA) {
		cout << "-> FloorPlanner::determCost(" << ratio_feasible_solutions_fixed_outline << ", " << SA_phase_two << ", " << set_max_cost << ")" << endl;
	}

	// phase one: consider only cost for packing into outline
	if (!SA_phase_two) {
		// determine area and outline cost
		cost_area_outline = this->determWeightedCostAreaOutline(ratio_feasible_solutions_fixed_outline);
		// invert weight since it's the only cost term
		cost_total = (1.0 / FloorPlanner::SA_COST_WEIGHT_AREA_OUTLINE) * cost_area_outline.cost;
	}
	// phase two: consider further cost factors
	else {

		// area and outline cost, already weigthed w/ global weight factor
		cost_area_outline = this->determWeightedCostAreaOutline(ratio_feasible_solutions_fixed_outline);

		// normalized interconnects cost; only if interconnect opt is on
		//
		if (this->conf_SA_opt_interconnects) {
			cost_interconnects = this->determCostInterconnects(set_max_cost);
		}
		else {
			cost_interconnects.HPWL = 0.0;
			cost_interconnects.TSVs = 0.0;
		}

		// TODO cost (failed) alignments
		cost_alignments = 0.0;

		// normalized temperature-distribution cost; only if thermal opt is on
		//
		if (this->conf_SA_opt_thermal) {
			cost_thermal = this->determCostThermalDistr(set_max_cost);
		}
		else {
			cost_thermal = 0.0;
		}

		// cost function; weight and sum up cost terms
		cost_total =
			FloorPlanner::SA_COST_WEIGHT_OTHERS * (
					this->conf_SA_cost_WL * cost_interconnects.HPWL
					+ this->conf_SA_cost_TSVs * cost_interconnects.TSVs
					+ this->conf_SA_cost_thermal * cost_thermal
				)
			// area, outline cost is already weighted
			+ cost_area_outline.cost;
		;
	}

	if (FloorPlanner::DBG_LAYOUT) {
		cout << "DBG_LAYOUT> ";
		cout << "Layout cost: " << cost_total << endl;
	}

	// return total cost and whether layout fits into outline
	ret.cost = cost_total;
	ret.fits_fixed_outline = cost_area_outline.fits_fixed_outline;

	if (FloorPlanner::DBG_CALLS_SA) {
		cout << "<- FloorPlanner::determCost : " << ret << endl;
	}

	return ret;
}

// adaptive cost model: terms for area and AR mismatch are _mutually_ depending on ratio
// of feasible solutions (solutions fitting into outline), leveraged from Chen et al 2006
// ``Modern floorplanning based on B*-Tree and fast simulated annealing''
FloorPlanner::Cost FloorPlanner::determWeightedCostAreaOutline(double const& ratio_feasible_solutions_fixed_outline) const {
	double cost_area;
	double cost_outline;
	double max_outline_x;
	double max_outline_y;
	int i;
	vector<double> dies_AR;
	vector<double> dies_area;
	bool layout_fits_in_fixed_outline;
	Cost ret;

	if (FloorPlanner::DBG_CALLS_SA) {
		cout << "-> FloorPlanner::determWeightedCostAreaOutline(" << ratio_feasible_solutions_fixed_outline << ")" << endl;
	}

	dies_AR.reserve(this->conf_layer);
	dies_area.reserve(this->conf_layer);

	layout_fits_in_fixed_outline = true;
	// determine outline and area
	for (i = 0; i < this->conf_layer; i++) {

		// determine outline for blocks on all dies separately
		max_outline_x = max_outline_y = 0.0;
		for (Block const& block : this->blocks) {

			if (block.layer == i) {
				// update max outline coords
				max_outline_x = max(max_outline_x, block.bb.ur.x);
				max_outline_y = max(max_outline_y, block.bb.ur.y);
			}
		}

		// area, represented by blocks' outline; normalized to die area
		dies_area.push_back((max_outline_x * max_outline_y) / (this->die_area));

		// aspect ratio; used to guide optimization towards fixed outline
		if (max_outline_y > 0.0) {
			dies_AR.push_back(max_outline_x / max_outline_y);
		}
		// dummy value for empty dies; implies cost of 0.0 for this die, i.e. does
		// not impact cost function
		else {
			dies_AR.push_back(this->die_AR);
		}

		// memorize whether layout fits into outline
		max_outline_x /= this->conf_outline_x;
		max_outline_y /= this->conf_outline_y;
		layout_fits_in_fixed_outline = layout_fits_in_fixed_outline && (max_outline_x <= 1.0 && max_outline_y <= 1.0);
	}

	// cost for AR mismatch, considering max violation guides towards fixed outline
	cost_outline = 0.0;
	for (i = 0; i < this->conf_layer; i++) {
		cost_outline = max(cost_outline, pow(dies_AR[i] - this->die_AR, 2.0));
	}
	// determine cost function value
	cost_outline *= 0.5 * FloorPlanner::SA_COST_WEIGHT_AREA_OUTLINE * (1.0 - ratio_feasible_solutions_fixed_outline);

	// cost for area, considering max value of (blocks-outline area) / (die-outline
	// area) guides towards balanced die occupation and area minimization
	cost_area = 0.0;
	for (i = 0; i < this->conf_layer; i++) {
		cost_area = max(cost_area, dies_area[i]);
	}
	// determine cost function value
	cost_area *= 0.5 * FloorPlanner::SA_COST_WEIGHT_AREA_OUTLINE * (1.0 + ratio_feasible_solutions_fixed_outline);

	ret.cost = cost_outline + cost_area;
	ret.fits_fixed_outline = layout_fits_in_fixed_outline;

	if (FloorPlanner::DBG_CALLS_SA) {
		cout << "<- FloorPlanner::determWeightedCostAreaOutline : " << ret << endl;
	}

	return ret;
}

FloorPlanner::CostInterconn FloorPlanner::determCostInterconnects(bool const& set_max_cost, bool const& normalize) {
	int i, ii;
	vector<Rect const*> blocks_to_consider;
	Rect bb;
	bool blocks_above_considered;
	CostInterconn ret;

	if (FloorPlanner::DBG_CALLS_SA) {
		cout << "-> FloorPlanner::determCostInterconnects(" << set_max_cost << ", " << normalize << ")" << endl;
	}

	ret.HPWL = ret.TSVs = ret.TSVs_area_deadspace_ratio = 0.0;
	blocks_to_consider.reserve(this->blocks.size());

	// set layer boundaries for each net, i.e., determine lowest and uppermost layer
	// of net's blocks
	for (Net& cur_net : this->nets) {
		cur_net.setLayerBoundaries();
	}

	// determine HPWL and TSVs for each net
	for (Net const& cur_net : this->nets) {

		// determine HPWL on each related layer separately
		for (i = cur_net.layer_bottom; i <= cur_net.layer_top; i++) {

			if (FloorPlanner::DBG_LAYOUT) {
				cout << "DBG_LAYOUT> Determine interconnects for net " << cur_net.id << " on layer " << i << " and above" << endl;
			}

			blocks_to_consider.clear();

			// blocks for cur_net on this layer
			for (Block const* b : cur_net.blocks) {
				if (b->layer == i) {
					blocks_to_consider.push_back(move(&b->bb));

					if (FloorPlanner::DBG_LAYOUT) {
						cout << "DBG_LAYOUT> 	Consider block " << b->id << " on layer " << i << endl;
					}
				}
			}
			// ignore cases with no blocks on current layer
			if (blocks_to_consider.empty()) {
				continue;
			}

			// blocks on the layer above; required to assume a reasonable
			// bounding box on current layer w/o placed TSVs
			// the layer above to consider is not necessarily the adjacent
			// one, thus stepwise consider layers until some blocks are found
			blocks_above_considered = false;
			ii = i + 1;
			while (ii <= cur_net.layer_top) {
				for (Block const* b : cur_net.blocks) {
					if (b->layer == ii) {
						blocks_to_consider.push_back(move(&b->bb));
						blocks_above_considered = true;

						if (FloorPlanner::DBG_LAYOUT) {
							cout << "DBG_LAYOUT> 	Consider block " << b->id << " on layer " << ii << endl;
						}
					}
				}

				// loop handler
				if (blocks_above_considered) {
					break;
				}
				else {
					ii++;
				}
			}

			// also consider routes to terminal pins on each layer
			for (Block const* pin :  cur_net.terminals) {
				blocks_to_consider.push_back(move(&pin->bb));

				if (FloorPlanner::DBG_LAYOUT) {
					cout << "DBG_LAYOUT> 	Consider terminal pin " << pin->id << endl;
				}
			}

			// ignore cases where only one block needs to be considered; these
			// cases (single blocks on uppermost layer) are already covered
			// while considering layers below
			if (blocks_to_consider.size() == 1) {

				if (FloorPlanner::DBG_LAYOUT) {
					cout << "DBG_LAYOUT> 	Ignore single block on uppermost layer" << endl;
				}

				continue;
			}

			// update TSVs counter if connecting to blocks on some upper layer
			if (blocks_above_considered) {
				ret.TSVs += (ii - i);

				if (FloorPlanner::DBG_LAYOUT) {
					cout << "DBG_LAYOUT> 	TSVs required: " << (ii - i) << endl;
				}
			}

			// determine HPWL of related blocks using their bounding box
			bb = Rect::determBoundingBox(blocks_to_consider);
			ret.HPWL += bb.w;
			ret.HPWL += bb.h;

			if (FloorPlanner::DBG_LAYOUT) {
				cout << "DBG_LAYOUT> 	HPWL of bounding box of blocks to consider: " << (bb.w + bb. h) << endl;
			}
		}
	}

	// also consider TSV lengths in HPWL; each TSV has to pass the whole Si layer and
	// the bonding layer
	ret.HPWL += ret.TSVs * (FloorPlanner::THICKNESS_SI + FloorPlanner::THICKNESS_BOND);

	// determine by TSVs occupied deadspace amount
	ret.TSVs_area_deadspace_ratio = (ret.TSVs * pow(FloorPlanner::TSV_DIMENSION, 2)) / this->stack_deadspace;

	// memorize max cost; initial sampling
	if (set_max_cost) {
		this->max_cost_WL = ret.HPWL;
		this->max_cost_TSVs = ret.TSVs;
	}

	// normalize to max value from initial sampling
	if (normalize) {
		ret.HPWL /= this->max_cost_WL;
		ret.TSVs /= this->max_cost_TSVs;
	}

	if (FloorPlanner::DBG_CALLS_SA) {
		cout << "<- FloorPlanner::determCostInterconnects : " << ret << endl;
	}

	return ret;
}
