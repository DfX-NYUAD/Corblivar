/*
 * =====================================================================================
 *
 *    Description:  Corblivar floorplanner (SA operations and related handler)
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
#include "FloorPlanner.hpp"
// required Corblivar headers
#include "Point.hpp"
#include "Math.hpp"
#include "CorblivarCore.hpp"
#include "CorblivarAlignmentReq.hpp"
#include "Net.hpp"
#include "IO.hpp"

// memory allocation
constexpr int FloorPlanner::OP_SWAP_BLOCKS;
constexpr int FloorPlanner::OP_MOVE_TUPLE;

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
	double fitting_layouts_ratio;
	bool valid_layout_found;
	int i_valid_layout_found;
	bool best_sol_found;
	bool accept;
	bool SA_phase_two, SA_phase_two_init;
	bool valid_layout;

	if (FloorPlanner::DBG_CALLS_SA) {
		cout << "-> FloorPlanner::performSA(" << &corb << ")" << endl;
	}

	// for handling floorplacement benchmarks, i.e., floorplanning w/ very large
	// blocks, we handle this naively by preferring these large blocks in the lower
	// left corner, i.e., perform a sorting of the sequences by block size
	//
	// also, for random layout operations in SA phase one, these blocks are not
	// allowed to be swapped or moved, see performOpMoveOrSwapBlocks
	if (this->SA_parameters.layout_floorplacement) {
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
	fitting_layouts_ratio = 0.0;
	// dummy large value to accept first fitting solution
	best_cost = 100.0 * Math::stdDev(cost_samples);

	/// outer loop: annealing -- temperature steps
	while (i <= this->SA_parameters.loopLimit) {

		if (this->logMax()) {
			cout << "SA> Optimization step: " << i << "/" << this->SA_parameters.loopLimit << endl;
		}

		// init loop parameters
		ii = 1;
		avg_cost = 0.0;
		accepted_ops = 0;
		layout_fit_counter = 0.0;
		SA_phase_two_init = false;
		best_sol_found = false;

		// init cost for current layout and fitting ratio
		this->generateLayout(corb, this->SA_parameters.opt_alignment && SA_phase_two);
		cur_cost = this->evaluateLayout(corb.getAlignments(), fitting_layouts_ratio, SA_phase_two).total_cost;

		// inner loop: layout operations
		while (ii <= innerLoopMax) {

			// perform random layout op
			op_success = this->performRandomLayoutOp(corb, SA_phase_two);

			if (op_success) {

				prev_cost = cur_cost;

				// generate layout; also memorize whether layout is valid;
				// note that this return value is only effective if
				// CorblivarCore::DBG_VALID_LAYOUT is set
				valid_layout = this->generateLayout(corb, this->SA_parameters.opt_alignment && SA_phase_two);

				// dbg invalid layouts
				if (CorblivarCore::DBG_VALID_LAYOUT && !valid_layout) {

					// generate invalid floorplan for dbg
					IO::writeFloorplanGP(*this, corb.getAlignments(), "invalid_layout");
					// generate related Corblivar solution
					if (this->IO_conf.solution_out.is_open()) {
						this->IO_conf.solution_out << corb.CBLsString() << endl;
						this->IO_conf.solution_out.close();
					}
					// abort further run
					exit(1);
				}

				// evaluate layout, new cost
				cost = this->evaluateLayout(corb.getAlignments(), fitting_layouts_ratio, SA_phase_two);
				cur_cost = cost.total_cost;
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

							// switch phase
							SA_phase_two = SA_phase_two_init = true;

							// re-calculate cost for new
							// phase; assume fitting ratio 1.0
							// for initialization and for
							// effective comparison of further
							// fitting solutions; also
							// initialize all max cost terms
							fitting_cost = this->evaluateLayout(corb.getAlignments(), 1.0, true, true).total_cost;

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
						// not first but any fitting solution; in
						// order to compare different fitting
						// solutions equally, consider cost terms
						// w/ fitting ratio 1.0
						else {
							fitting_cost = cost.total_cost_fitting;
						}

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

		// determine ratio of solutions fitting into outline in current temp step;
		// note that during the next temp step this ratio is fixed in order to
		// avoid sudden changes of related cost terms during few iterations
		if (accepted_ops > 0) {
			fitting_layouts_ratio = static_cast<double>(layout_fit_counter) / accepted_ops;
		}
		else {
			fitting_layouts_ratio = 0.0;
		}

		// determine avg cost for temp step
		if (accepted_ops > 0) {
			avg_cost /= accepted_ops;
		}

		// determine accepted-ops ratio
		accepted_ops_ratio = static_cast<double>(accepted_ops) / ii;

		if (this->logMax()) {
			cout << "SA> Step done:" << endl;
			cout << "SA>  new best solution found: " << best_sol_found << endl;
			cout << "SA>  accept-ops ratio: " << accepted_ops_ratio << endl;
			cout << "SA>  valid-layouts ratio: " << fitting_layouts_ratio << endl;
			cout << "SA>  avg cost: " << avg_cost << endl;
			cout << "SA>  temp: " << cur_temp << endl;
		}

		// log temperature step
		TempStep cur_step;
		cur_step.step = i;
		cur_step.temp = cur_temp;
		cur_step.avg_cost = avg_cost;
		cur_step.new_best_sol_found = best_sol_found;
		cur_step.cost_best_sol = best_cost;
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

void FloorPlanner::updateTemp(double& cur_temp, int const& iteration, int const& iteration_first_valid_layout) const {
	float loop_factor;
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
	if (this->SA_parameters.temp_factor_phase3 != 0.0 && std_dev_avg_cost <= FloorPlanner::SA_REHEAT_STD_DEV_COST_LIMIT) {
		cur_temp *= this->SA_parameters.temp_factor_phase3;

		phase = 3;
	}
	// phase 1; adaptive cooling (slows down from SA_parameters.temp_factor_phase1 to
	// SA_parameters.temp_factor_phase1_limit)
	else if (iteration_first_valid_layout == Point::UNDEF) {
		loop_factor = (this->SA_parameters.temp_factor_phase1_limit - this->SA_parameters.temp_factor_phase1)
			* static_cast<float>(iteration - 1) / (this->SA_parameters.loopLimit - 1.0);
		// note that loop_factor is additive in this case; the cooling factor is
		// increased w/ increasing iterations
		cur_temp *= this->SA_parameters.temp_factor_phase1 + loop_factor;

		phase = 1;
	}
	// phase 2; reheating and converging (initially reheats and then increases cooling
	// rate faster, i.e., heating factor is decreased w/ increasing iterations to
	// enable convergence)
	else {
		// note that loop_factor must only consider the remaining iteration range
		loop_factor = 1.0 - static_cast<float>(iteration - iteration_first_valid_layout) /
			static_cast<float>(this->SA_parameters.loopLimit - iteration_first_valid_layout);
		cur_temp *= this->SA_parameters.temp_factor_phase2 * loop_factor;

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

	// reset max cost
	this->max_cost_WL = 0.0;
	this->max_cost_TSVs = 0;
	this->max_cost_thermal = 0.0;
	this->max_cost_alignments = 0.0;

	// reset temperature-schedule log
	this->tempSchedule.clear();

	// backup initial CBLs
	corb.backupCBLs();

	// init SA parameter: inner loop ops
	innerLoopMax = pow(static_cast<double>(this->blocks.size()), this->SA_parameters.loopFactor);

	/// initial sampling
	//
	if (this->logMed()) {
		cout << "SA> Perform initial solution-space sampling..." << endl;
	}

	// init cost; ignore alignment here
	this->generateLayout(corb, false);
	cur_cost = this->evaluateLayout(corb.getAlignments()).total_cost;

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
			this->generateLayout(corb, false);
			// evaluate layout, new cost
			cur_cost = this->evaluateLayout(corb.getAlignments()).total_cost;
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
	init_temp = Math::stdDev(cost_samples) * this->SA_parameters.temp_init_factor;

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
	double x, y;
	Cost cost;

	if (FloorPlanner::DBG_CALLS_SA) {
		cout << "-> FloorPlanner::finalize(" << &corb << ", " << determ_overall_cost << ", " << handle_corblivar << ")" << endl;
	}

	// consider as regular Corblivar run
	if (handle_corblivar) {
		// apply best solution, if available, as final solution
		valid_solution = corb.applyBestCBLs(this->logMin());
		// generate final layout
		this->generateLayout(corb, this->SA_parameters.opt_alignment);
	}

	// determine final cost, also for non-Corblivar calls
	if (!handle_corblivar || valid_solution) {

		// determine overall blocks outline; reasonable die outline for whole
		// 3D-IC stack
		x = y = 0.0;
		for (Block const& b : this->blocks) {
			x = max(x, b.bb.ur.x);
			y = max(y, b.bb.ur.y);
		}

		// shrink fixed outline considering the final layout
		if (this->IC.outline_shrink) {

			this->resetDieProperties(x, y);
		}

		// determine cost terms and overall cost
		cost = this->evaluateLayout(corb.getAlignments(), 1.0, true, false, true);

		// logging IO_conf.results; consider non-normalized, actual values
		if (this->logMin()) {

			cout << "Corblivar> Characteristica of final solution:" << endl;

			// overall cost only encode a useful number in case the whole
			// optimization run is done, i.e., not for reading in given
			// solution files
			if (determ_overall_cost) {
				cout << "Corblivar> Final (adapted) cost: " << cost.total_cost << endl;
				this->IO_conf.results << "Final (adapted) cost: " << cost.total_cost << endl;
			}

			cout << "Corblivar> Max blocks-outline / die-outline ratio: " << cost.area_actual_value << endl;
			this->IO_conf.results << "Max blocks-outline / die-outline ratio: " << cost.area_actual_value << endl;

			cout << "Corblivar> Overall deadspace [%]: " << 100.0 * (this->IC.stack_deadspace / this->IC.stack_area) << endl;
			this->IO_conf.results << "Overall deadspace [%]: " << 100.0 * (this->IC.stack_deadspace / this->IC.stack_area) << endl;

			cout << "Corblivar> Overall blocks outline (reasonable stack outline):" << endl;
			cout << "Corblivar>  x = " << x << endl;
			cout << "Corblivar>  y = " << y << endl;
			this->IO_conf.results << "Overall blocks outline (reasonable stack outline):" << endl;
			this->IO_conf.results << " x = " << x << endl;
			this->IO_conf.results << " y = " << y << endl;

			cout << "Corblivar> Alignment mismatches [um]: " << cost.alignments_actual_value << endl;
			this->IO_conf.results << "Alignment mismatches [um]: " << cost.alignments_actual_value << endl;

			cout << "Corblivar> HPWL: " << cost.HPWL_actual_value << endl;
			this->IO_conf.results << "HPWL: " << cost.HPWL_actual_value << endl;

			// TODO statistics of hotspot clusters
			//
			// TODO statistics of TSV islands
			cout << "Corblivar> TSVs: " << cost.TSVs_actual_value << endl;
			this->IO_conf.results << "TSVs: " << cost.TSVs_actual_value << endl;

			cout << "Corblivar>  Deadspace utilization by TSVs [%]: " << 100.0 * cost.TSVs_area_deadspace_ratio << endl;
			this->IO_conf.results << " Deadspace utilization by TSVs [%]: " << 100.0 * cost.TSVs_area_deadspace_ratio << endl;

			cout << "Corblivar> Temp cost (estimated max temp for lowest layer [K]): " << cost.thermal_actual_value << endl;
			this->IO_conf.results << "Temp cost (estimated max temp for lowest layer [K]): " << cost.thermal_actual_value << endl;

			cout << endl;
		}
	}

	// generate temperature-schedule data
	IO::writeTempSchedule(*this);

	// generate floorplan plots
	IO::writeFloorplanGP(*this, corb.getAlignments());

	// generate Corblivar data if solution file is used as output
	if (handle_corblivar && this->IO_conf.solution_out.is_open()) {
		this->IO_conf.solution_out << corb.CBLsString() << endl;
		this->IO_conf.solution_out.close();

		// delete file in case no valid solution was generated
		if (!valid_solution) {
			remove(this->IO_conf.solution_file.c_str());
		}
	}

	// thermal-analysis files
	if ((!handle_corblivar || valid_solution) && this->IO_conf.power_density_file_avail) {
		// generate power, thermal and TSV-density maps
		IO::writePowerThermalTSVMaps(*this);
		// generate HotSpot files
		IO::writeHotSpotFiles(*this);
	}

	// determine overall runtime
	ftime(&end);
	if (this->logMin()) {
		runtime << "Runtime: " << (1000.0 * (end.time - this->time_start.time) + (end.millitm - this->time_start.millitm)) / 1000.0 << " s";
		cout << "Corblivar> " << runtime.str() << endl;
		this->IO_conf.results << runtime.str() << endl;
	}

	// close IO_conf.results file
	this->IO_conf.results.close();

	if (FloorPlanner::DBG_CALLS_SA) {
		cout << "<- FloorPlanner::finalize" << endl;
	}
}

bool FloorPlanner::generateLayout(CorblivarCore& corb, bool const& perform_alignment) {
	bool ret;

	// generate layout
	ret = corb.generateLayout(perform_alignment);

	// annotate alignment success/failure in blocks; required for maintaining
	// succeeded alignments during subsequent packing
	if (this->SA_parameters.opt_alignment && this->SA_parameters.layout_packing_iterations > 0) {
		// ignore related cost; use dummy variable
		Cost dummy;
		// also don't derive TSVs; not required here
		this->evaluateAlignments(dummy, corb.getAlignments(), false);
	}

	// perform packing if desired; perform on each die for each
	// dimension separately and subsequently; multiple iterations may
	// provide denser packing configurations
	for (int d = 0; d < this->IC.layers; d++) {

		CorblivarDie& die = corb.editDie(d);

		// sanity check for empty dies
		if (!die.getCBL().empty()) {

			for (int i = 1; i <= this->SA_parameters.layout_packing_iterations; i++) {
				die.performPacking(Direction::HORIZONTAL);
				die.performPacking(Direction::VERTICAL);
			}
		}

		// dbg: sanity check for valid layout
		if (CorblivarCore::DBG_VALID_LAYOUT) {

			// if true, the layout is buggy, i.e., invalid
			if (die.debugLayout()) {
				return false;
			}
		}
	}

	return ret;
}

bool FloorPlanner::performRandomLayoutOp(CorblivarCore& corb, bool const& SA_phase_two, bool const& revertLastOp) {
	int op;
	int die1, die2, tuple1, tuple2, juncts;
	bool ret, swapping_failed_blocks;

	if (FloorPlanner::DBG_CALLS_SA) {
		cout << "-> FloorPlanner::performRandomLayoutOp(" << &corb << ", " << SA_phase_two << ", " << revertLastOp << ")" << endl;
	}

	// init layout operation variables
	die1 = die2 = tuple1 = tuple2 = juncts = -1;

	// revert last op
	if (revertLastOp) {
		op = this->last_op;
	}
	// perform new op
	else {

		// to enable guided block alignment during phase II, we prefer to perform
		// block swapping on particular blocks of failing alignment requests
		swapping_failed_blocks = false;
		if (SA_phase_two && this->SA_parameters.opt_alignment) {

			// try to setup swapping failed blocks
			swapping_failed_blocks = this->prepareBlockSwappingFailedAlignment(corb, die1, tuple1, die2, tuple2);
			this->last_op = op = FloorPlanner::OP_SWAP_BLOCKS;
		}

		// for other regular cases or in case swapping failed blocks was not successful, we proceed with a random
		// operation
		if (!swapping_failed_blocks) {

			// reset layout operation variables
			die1 = die2 = tuple1 = tuple2 = juncts = -1;

			// see defined op-codes in class FloorPlanner to set random-number
			// ranges; recall that randI(x,y) is [x,y)
			this->last_op = op = Math::randI(1, 6);
		}
	}

	// specific op handler
	switch (op) {

		case FloorPlanner::OP_SWAP_BLOCKS: // op-code: 1

			ret = this->performOpMoveOrSwapBlocks(FloorPlanner::OP_SWAP_BLOCKS, revertLastOp, !SA_phase_two, corb, die1, die2, tuple1, tuple2);

			break;

		case FloorPlanner::OP_MOVE_TUPLE: // op-code: 2

			ret = this->performOpMoveOrSwapBlocks(FloorPlanner::OP_MOVE_TUPLE, revertLastOp, !SA_phase_two, corb, die1, die2, tuple1, tuple2);

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

bool FloorPlanner::prepareBlockSwappingFailedAlignment(CorblivarCore const& corb, int& die1, int& tuple1, int& die2, int& tuple2) {
	CorblivarAlignmentReq const* failed_req = nullptr;
	Block const* b1;
	Block const* b1_neighbour = nullptr;

	// determine first failed alignment
	for (unsigned r = 0; r < corb.getAlignments().size(); r++) {

		if (!corb.getAlignments()[r].fulfilled) {
			failed_req = &corb.getAlignments()[r];
			break;
		}
	}

	// handle request; sanity check for found failed request
	if (failed_req != nullptr) {

		// randomly decide for one block; consider the dummy reference block if
		// required
		if (
			// randomly select s_i if it's not the RBOD
			(failed_req->s_i->id != "RBOD" && Math::randB()) ||
			// if s_j is the RBOD, we need to use s_i; assuming that
			// only s_i OR s_j are the RBOD
			failed_req->s_j->id == "RBOD"
		   ) {
			die1 = die2 = failed_req->s_i->layer;
			tuple1 = corb.getDie(die1).getTuple(failed_req->s_i);
			b1 = failed_req->s_i;
		}
		else {
			die1 = die2 = failed_req->s_j->layer;
			tuple1 = corb.getDie(die1).getTuple(failed_req->s_j);
			b1 = failed_req->s_j;
		}

		if (
			// zero-offset fixed alignment in both coordinates
			(failed_req->offset_x() && failed_req->alignment_x == 0 && failed_req->offset_y() && failed_req->alignment_y == 0) ||
			// min overlap in both dimensions
			(failed_req->range_x() && failed_req->range_y())
		   ) {

			// such alignment cannot be fulfilled in one die, i.e., differing
			// dies required
			if (failed_req->s_i->layer == failed_req->s_j->layer) {

				// this is only possible for > 1 layers; sanity check
				if (this->IC.layers == 1) {
					return false;
				}

				while (die1 == die2) {
					die2 = Math::randI(0, this->IC.layers);
				}
			}

			// select block to swap with such that blocks to be aligned are
			// initially at least intersecting blocks
			for (Block const* b2 : corb.getDie(die2).getBlocks()) {

				if (Rect::rectsIntersect(b1->bb, b2->bb)) {

					// however, this should not be the partner block
					// of the alignment request
					if (
						(b1->id == failed_req->s_i->id && b2->id == failed_req->s_j->id) ||
						(b1->id == failed_req->s_j->id && b2->id == failed_req->s_i->id)
					   ) {
						continue;
					}

					b1_neighbour = b2;

					break;
				}
			}
		}

		// failed alignment ranges and (partially) non-zero-offset fixed alignment
		//
		// determine relevant neighbour block to perform swap operation, i.e.,
		// nearest neighbour w.r.t. failure type
		else {

			// also consider randomly to change die2 as well; this is required
			// for alignments which cannot be fulfilled within one die and
			// does not harm for alignments which could be fulfilled within
			// one die (they can then also be fulfilled across dies); note
			// that an explicit check for all the different options of
			// alignments not possible within one die are not performed here
			// but rather a die change is considered randomly
			//
			// note that changing dies is only possible for > 1 layers
			if (Math::randB() && this->IC.layers > 1) {

				while (die1 == die2) {
					die2 = Math::randI(0, this->IC.layers);
				}
			}
		
			for (Block const* b2 : corb.getDie(die2).getBlocks()) {

				switch (b1->alignment) {

					// determine nearest right block
					case Block::AlignmentStatus::FAIL_HOR_TOO_LEFT:

						if (Rect::rectA_leftOf_rectB(b1->bb, b2->bb, true)) {

							if (b1_neighbour == nullptr || b2->bb.ll.x < b1_neighbour->bb.ll.x) {
								b1_neighbour = b2;
							}
						}

						break;

					// determine nearest left block
					case Block::AlignmentStatus::FAIL_HOR_TOO_RIGHT:

						if (Rect::rectA_leftOf_rectB(b2->bb, b1->bb, true)) {

							if (b1_neighbour == nullptr || b2->bb.ur.x > b1_neighbour->bb.ur.x) {
								b1_neighbour = b2;
							}
						}

						break;

					// determine nearest block above
					case Block::AlignmentStatus::FAIL_VERT_TOO_LOW:

						if (Rect::rectA_below_rectB(b1->bb, b2->bb, true)) {

							if (b1_neighbour == nullptr || b2->bb.ll.y < b1_neighbour->bb.ll.y) {
								b1_neighbour = b2;
							}
						}

						break;

					// determine nearest block below
					case Block::AlignmentStatus::FAIL_VERT_TOO_HIGH:

						if (Rect::rectA_below_rectB(b2->bb, b1->bb, true)) {

							if (b1_neighbour == nullptr || b2->bb.ur.y > b1_neighbour->bb.ur.y) {
								b1_neighbour = b2;
							}
						}

						break;

					// dummy case, to catch other (here not occuring)
					// alignment status
					default:
						break;
				}
			}
		}

		// determine related tuple of neigbhour block; == -1 in case the tuple
		// cannot be find; sanity check for undefined neighbour
		if (b1_neighbour != nullptr) {

			tuple2 = corb.getDie(die2).getTuple(b1_neighbour);

			if (FloorPlanner::DBG_ALIGNMENT) {
				cout << "DBG_ALIGNMENT> " << failed_req->tupleString() << " failed so far;" << endl;
				cout << "DBG_ALIGNMENT> considering swapping block " << b1->id << " on layer " << b1->layer;
				cout << " with block " << b1_neighbour->id << " on layer " << b1_neighbour->layer << endl;
			}

			return true;
		}
		else {
			tuple2 = -1;

			return false;
		}
	}
	else {
		return false;
	}
}

bool FloorPlanner::performOpShapeBlock(bool const& revert, CorblivarCore& corb, int& die1, int& tuple1) const {
	Block const* shape_block;

	if (!revert) {

		// randomly select die, if not preassigned
		if (die1 == -1) {
			die1 = Math::randI(0, this->IC.layers);
		}

		// sanity check for empty dies
		if (corb.getDie(die1).getCBL().empty()) {
			return false;
		}

		// randomly select tuple, if not preassigned
		if (tuple1 == -1) {
			tuple1 = Math::randI(0, corb.getDie(die1).getCBL().size());
		}

		// determine related block to be shaped
		shape_block = corb.getDie(die1).getBlock(tuple1);

		// backup current shape
		shape_block->bb_backup = shape_block->bb;

		// soft blocks: enhanced block shaping
		if (shape_block->soft) {
			// enhanced shaping, according to [Chen06]
			if (this->SA_parameters.layout_enhanced_soft_block_shaping) {
				return this->performOpEnhancedSoftBlockShaping(corb, shape_block);
			}
			// simple random shaping
			else {
				shape_block->shapeRandomlyByAR();
			}
		}
		// hard blocks: simple rotation or enhanced rotation (perform block
		// rotation only if layout compaction is achievable); note that this
		// enhanced rotation relies on non-compacted, i.e., non-packed layouts,
		// which is checked during config file parsing
		else {
			// enhanced rotation
			if (this->SA_parameters.layout_enhanced_hard_block_rotation) {
				return this->performOpEnhancedHardBlockRotation(corb, shape_block);
			}
			// simple rotation
			else {
				shape_block->rotate();
			}
		}
	}
	// revert last rotation
	else {
		// revert by restoring backup bb
		corb.getDie(this->last_op_die1).getBlock(this->last_op_tuple1)->bb =
			corb.getDie(this->last_op_die1).getBlock(this->last_op_tuple1)->bb_backup;
	}

	return true;
}

bool FloorPlanner::performOpEnhancedSoftBlockShaping(CorblivarCore const& corb, Block const* shape_block) const {
	int op;
	double boundary_x, boundary_y;
	double width, height;

	// see defined op-codes in class FloorPlanner to set random-number ranges;
	// recall that randI(x,y) is [x,y)
	op = Math::randI(10, 15);

	switch (op) {

		// stretch such that shape_block's right front aligns w/ the right front
		// of the nearest other block
		case FloorPlanner::OP_SHAPE_BLOCK__STRETCH_HORIZONTAL: // op-code: 10

			// dummy value, to be large than right front
			boundary_x = 2.0 * shape_block->bb.ur.x;

			for (Block const* b : corb.getDie(shape_block->layer).getBlocks()) {

				// determine nearest right front of other blocks
				if (b->bb.ur.x > shape_block->bb.ur.x) {
					boundary_x = min(boundary_x, b->bb.ur.x);
				}
			}

			// determine resulting new dimensions
			width = boundary_x - shape_block->bb.ll.x;
			height = shape_block->bb.area / width;

			// apply new dimensions in case the resulting AR is allowed
			return shape_block->shapeByWidthHeight(width, height);

		// shrink such that shape_block's right front aligns w/ the left front of
		// the nearest other block
		case FloorPlanner::OP_SHAPE_BLOCK__SHRINK_HORIZONTAL: // op-code: 12

			boundary_x = 0.0;

			for (Block const* b : corb.getDie(shape_block->layer).getBlocks()) {

				// determine nearest left front of other blocks
				if (b->bb.ll.x < shape_block->bb.ur.x) {
					boundary_x = max(boundary_x, b->bb.ll.x);
				}
			}

			// determine resulting new dimensions
			width = boundary_x - shape_block->bb.ll.x;
			height = shape_block->bb.area / width;

			// apply new dimensions in case the resulting AR is allowed
			return shape_block->shapeByWidthHeight(width, height);

		// stretch such that shape_block's top front aligns w/ the top front of
		// the nearest other block
		case FloorPlanner::OP_SHAPE_BLOCK__STRETCH_VERTICAL: // op-code: 11

			// dummy value, to be large than top front
			boundary_y = 2.0 * shape_block->bb.ur.y;

			for (Block const* b : corb.getDie(shape_block->layer).getBlocks()) {

				// determine nearest top front of other blocks
				if (b->bb.ur.y > shape_block->bb.ur.y) {
					boundary_y = min(boundary_y, b->bb.ur.y);
				}
			}

			// determine resulting new dimensions
			height = boundary_y - shape_block->bb.ll.y;
			width = shape_block->bb.area / height;

			// apply new dimensions in case the resulting AR is allowed
			return shape_block->shapeByWidthHeight(width, height);

		// shrink such that shape_block's top front aligns w/ the bottom front of
		// the nearest other block
		case FloorPlanner::OP_SHAPE_BLOCK__SHRINK_VERTICAL: // op-code: 13

			boundary_y = 0.0;

			for (Block const* b : corb.getDie(shape_block->layer).getBlocks()) {

				// determine nearest bottom front of other blocks
				if (b->bb.ll.y < shape_block->bb.ur.y) {
					boundary_y = max(boundary_y, b->bb.ll.y);
				}
			}

			// determine resulting new dimensions
			height = boundary_y - shape_block->bb.ll.y;
			width = shape_block->bb.area / height;

			// apply new dimensions in case the resulting AR is allowed
			return shape_block->shapeByWidthHeight(width, height);

		case FloorPlanner::OP_SHAPE_BLOCK__RANDOM_AR: // op-code: 14

			shape_block->shapeRandomlyByAR();

			return true;

		// to avoid compiler warnings, non-reachable code due to
		// constrained op value
		default:
			return false;
	}
}

bool FloorPlanner::performOpEnhancedHardBlockRotation(CorblivarCore const& corb, Block const* shape_block) const {
	double col_max_width, row_max_height;
	double gain, loss;

	// horizontal block
	if (shape_block->bb.w > shape_block->bb.h) {

		// check blocks in (implicitly constructed) row
		row_max_height = shape_block->bb.h;

		for (Block const* b : corb.getDie(shape_block->layer).getBlocks()) {

			if (shape_block->bb.ll.y == b->bb.ll.y) {
				row_max_height = max(row_max_height, b->bb.h);
			}
		}

		// gain in horizontal direction by rotation
		gain = shape_block->bb.w - shape_block->bb.h;
		// loss in vertical direction; only if new block
		// height (current width) would be larger than the
		// row's current height
		loss = shape_block->bb.w - row_max_height;
	}
	// vertical block
	else {
		// check blocks in (implicitly constructed) column
		col_max_width = shape_block->bb.w;

		for (Block const* b : corb.getDie(shape_block->layer).getBlocks()) {

			if (shape_block->bb.ll.x == b->bb.ll.x) {
				col_max_width = max(col_max_width, b->bb.w);
			}
		}

		// gain in vertical direction by rotation
		gain = shape_block->bb.h - shape_block->bb.w;
		// loss in horizontal direction; only if new block
		// width (current height) would be larger than the
		// column's current width
		loss = shape_block->bb.h - col_max_width;
	}

	// perform rotation if no loss or gain > loss
	if (loss < 0.0 || gain > loss) {
		shape_block->rotate();

		return true;
	}
	else {
		return false;
	}
}


bool FloorPlanner::performOpSwitchTupleJunctions(bool const& revert, CorblivarCore& corb, int& die1, int& tuple1, int& juncts) const {
	int new_juncts;

	if (!revert) {

		// randomly select die, if not preassigned
		if (die1 == -1) {
			die1 = Math::randI(0, this->IC.layers);
		}

		// sanity check for empty dies
		if (corb.getDie(die1).getCBL().empty()) {
			return false;
		}

		// randomly select tuple, if not preassigned
		if (tuple1 == -1) {
			tuple1 = Math::randI(0, corb.getDie(die1).getCBL().size());
		}

		// juncts is for return-by-reference, new_juncts for updating junctions
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

		// randomly select die, if not preassigned
		if (die1 == -1) {
			die1 = Math::randI(0, this->IC.layers);
		}

		// sanity check for empty dies
		if (corb.getDie(die1).getCBL().empty()) {
			return false;
		}

		// randomly select tuple, if not preassigned
		if (tuple1 == -1) {
			tuple1 = Math::randI(0, corb.getDie(die1).getCBL().size());
		}

		corb.switchInsertionDirection(die1, tuple1);
	}
	else {
		corb.switchInsertionDirection(this->last_op_die1, this->last_op_tuple1);
	}

	return true;
}

bool FloorPlanner::performOpMoveOrSwapBlocks(int const& mode, bool const& revert, bool const& SA_phase_one, CorblivarCore& corb, int& die1, int& die2, int& tuple1, int& tuple2) const {

	if (!revert) {

		// randomly select die, if not preassigned
		if (die1 == -1) {
			die1 = Math::randI(0, this->IC.layers);
		}
		if (die2 == -1) {
			die2 = Math::randI(0, this->IC.layers);
		}

		if (mode == FloorPlanner::OP_MOVE_TUPLE) {
			// sanity check for empty (origin) die
			if (corb.getDie(die1).getCBL().empty()) {
				return false;
			}
		}
		else if (mode == FloorPlanner::OP_SWAP_BLOCKS) {
			// sanity check for empty dies
			if (corb.getDie(die1).getCBL().empty() || corb.getDie(die2).getCBL().empty()) {
				return false;
			}
		}

		// randomly select tuple, if not preassigned
		if (tuple1 == -1) {
			tuple1 = Math::randI(0, corb.getDie(die1).getCBL().size());
		}
		if (tuple2 == -1) {
			tuple2 = Math::randI(0, corb.getDie(die2).getCBL().size());
		}

		// in case of swapping/moving w/in same die, ensure that tuples are
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

		// for power-aware block handling, ensure that blocks w/ lower power
		// density remain in lower layer
		if (this->SA_parameters.layout_power_aware_block_handling) {
			if (die1 < die2
					&& (corb.getDie(die1).getBlock(tuple1)->power_density < corb.getDie(die2).getBlock(tuple2)->power_density)) {
				return false;
			}
			else if (die2 < die1
					&& (corb.getDie(die2).getBlock(tuple2)->power_density < corb.getDie(die1).getBlock(tuple1)->power_density)) {
				return false;
			}
		}

		// for SA phase one, floorplacement blocks, i.e., large macros, should not
		// be moved/swapped
		if (this->SA_parameters.layout_floorplacement && SA_phase_one
				&& (corb.getDie(die1).getBlock(tuple1)->floorplacement || corb.getDie(die2).getBlock(tuple2)->floorplacement)) {
			return false;
		}

		// perform move/swap; applies only to valid candidates
		if (mode == FloorPlanner::OP_MOVE_TUPLE) {
			corb.moveTuples(die1, die2, tuple1, tuple2);
		}
		else if (mode == FloorPlanner::OP_SWAP_BLOCKS) {
			corb.swapBlocks(die1, die2, tuple1, tuple2);
		}
	}
	else {
		if (mode == FloorPlanner::OP_MOVE_TUPLE) {
			corb.moveTuples(this->last_op_die2, this->last_op_die1, this->last_op_tuple2, this->last_op_tuple1);
		}
		else if (mode == FloorPlanner::OP_SWAP_BLOCKS) {
			corb.swapBlocks(this->last_op_die1, this->last_op_die2, this->last_op_tuple1, this->last_op_tuple2);
		}
	}

	return true;
}

// adaptive cost model w/ two phases: first phase considers only cost for packing into
// outline, second phase considers further factors like WL, thermal distr, etc.
FloorPlanner::Cost FloorPlanner::evaluateLayout(vector<CorblivarAlignmentReq> const& alignments, double const& fitting_layouts_ratio, bool const& SA_phase_two, bool const& set_max_cost, bool const& finalize) {
	Cost cost;

	if (FloorPlanner::DBG_CALLS_SA) {
		cout << "-> FloorPlanner::evaluateLayout(" << &alignments << ", " << fitting_layouts_ratio << ", " << SA_phase_two << ", " << set_max_cost << ", " << finalize << ")" << endl;
	}

	// phase one: consider only cost for packing into outline
	if (!SA_phase_two) {

		// area and outline cost, already weighted w/ global weight factor
		this->evaluateAreaOutline(cost, fitting_layouts_ratio);

		// determine total cost; invert weight of area and outline cost since it's
		// the only cost term
		cost.total_cost = (1.0 / FloorPlanner::SA_COST_WEIGHT_AREA_OUTLINE) * cost.area_outline;
	}
	// phase two: consider further cost factors
	else {
		// reset TSVs
		this->TSVs.clear();

		// area and outline cost, already weighted w/ global weight factor
		this->evaluateAreaOutline(cost, fitting_layouts_ratio);

		// determine interconnects cost; if interconnect opt is on or for finalize
		// calls
		if (this->SA_parameters.opt_interconnects) {
			this->evaluateInterconnects(cost, set_max_cost);
		}
		// for finalize calls and when no cost was previously determined, we need
		// to initialize the max_cost
		else if (finalize) {
			this->evaluateInterconnects(cost, true);
		}
		else {
			cost.HPWL = cost.HPWL_actual_value = 0.0;
			cost.TSVs = cost.TSVs_actual_value = 0;
			cost.TSVs_area_deadspace_ratio = 0.0;
		}

		// cost for failed alignments, i.e., alignment mismatches; also annotates
		// failed request, this provides feedback for further alignment
		// optimization
		if (this->SA_parameters.opt_alignment) {
			this->evaluateAlignments(cost, alignments, true, set_max_cost);
		}
		// for finalize calls and when no cost was previously determined, we need
		// to initialize the max_cost
		else if (finalize) {
			this->evaluateAlignments(cost, alignments, true, true);
		}
		else {
			cost.alignments = cost.alignments_actual_value = 0.0;
		}

		// temperature-distribution cost; if thermal opt is on or for finalize
		// run; note that vertical buses impact heat conduction via TSVs, thus the
		// block alignment / bus planning is analysed before thermal distribution
		if (this->SA_parameters.opt_thermal) {
			this->evaluateThermalDistr(cost, set_max_cost);
		}
		// for finalize calls and when no cost was previously determined, we need
		// to initialize the max_cost
		else if (finalize) {
			this->evaluateThermalDistr(cost, true);
		}
		else {
			cost.thermal = cost.thermal_actual_value = 0.0;
		}

		// for finalize calls, re-determine interconnects, for proper hotspot
		// cluster and TSV islands; the best solution's thermal distribution is
		// probably significantly different from the previous temporary solution,
		// thus a re-determination is required for proper results
		if (finalize) {
			this->evaluateInterconnects(cost, false);
		}

		// determine total cost; weight and sum up cost terms
		cost.total_cost =
			FloorPlanner::SA_COST_WEIGHT_OTHERS * (
					this->SA_parameters.cost_WL * cost.HPWL
					+ this->SA_parameters.cost_TSVs * cost.TSVs
					+ this->SA_parameters.cost_alignment * cost.alignments
					+ this->SA_parameters.cost_thermal * cost.thermal
				)
			// area, outline cost is already weighted
			+ cost.area_outline;

		// determine total cost assuming a fitting ratio of 1.0
		cost.total_cost_fitting =
			FloorPlanner::SA_COST_WEIGHT_OTHERS * (
					this->SA_parameters.cost_WL * cost.HPWL
					+ this->SA_parameters.cost_TSVs * cost.TSVs
					+ this->SA_parameters.cost_alignment * cost.alignments
					+ this->SA_parameters.cost_thermal * cost.thermal
				)
			// consider only area term for ratio 1.0, see evaluateAreaOutline
			+ cost.area_actual_value * FloorPlanner::SA_COST_WEIGHT_AREA_OUTLINE;
	}

	if (FloorPlanner::DBG_LAYOUT) {
		cout << "DBG_LAYOUT> Total cost: " << cost.total_cost << endl;
		cout << "DBG_LAYOUT>  HPWL cost: " << cost.HPWL << endl;
		cout << "DBG_LAYOUT>  TSVs cost: " << cost.TSVs << endl;
		cout << "DBG_LAYOUT>  Alignments cost: " << cost.alignments << endl;
		cout << "DBG_LAYOUT>  Thermal cost: " << cost.thermal << endl;
	}

	if (FloorPlanner::DBG_CALLS_SA) {
		cout << "<- FloorPlanner::evaluateLayout : " << cost << endl;
	}

	return cost;
}

void FloorPlanner::evaluateThermalDistr(Cost& cost, bool const& set_max_cost) {

	// generate power maps based on layout and blocks' power densities
	this->thermalAnalyzer.generatePowerMaps(this->IC.layers, this->blocks,
			this->getOutline(), this->power_blurring_parameters);

	// adapt power maps to account for TSVs' impact
	this->thermalAnalyzer.adaptPowerMaps(this->IC.layers, this->TSVs, this->nets, this->power_blurring_parameters);

	// perform actual thermal analysis
	this->thermalAnalyzer.performPowerBlurring(this->thermal_analysis, this->IC.layers,
			this->power_blurring_parameters);

	// memorize max cost; initial sampling
	if (set_max_cost) {
		this->max_cost_thermal = this->thermal_analysis.cost_temp;
	}

	// store normalized temp cost
	cost.thermal = this->thermal_analysis.cost_temp / this->max_cost_thermal;
	// store actual temp value
	cost.thermal_actual_value = this->thermal_analysis.max_temp;
};

// adaptive cost model: terms for area and AR mismatch are _mutually_ depending on ratio
// of feasible solutions (solutions fitting into outline), leveraged from Chen et al 2006
// ``Modern floorplanning based on B*-Tree and fast simulated annealing''
void FloorPlanner::evaluateAreaOutline(FloorPlanner::Cost& cost, double const& fitting_layouts_ratio) const {
	double cost_area;
	double cost_outline;
	double max_outline_x;
	double max_outline_y;
	int i;
	vector<double> dies_AR;
	vector<double> dies_area;
	bool layout_fits_in_fixed_outline;

	if (FloorPlanner::DBG_CALLS_SA) {
		cout << "-> FloorPlanner::evaluateAreaOutline(" << fitting_layouts_ratio << ")" << endl;
	}

	dies_AR.reserve(this->IC.layers);
	dies_area.reserve(this->IC.layers);

	layout_fits_in_fixed_outline = true;
	// determine outline and area
	for (i = 0; i < this->IC.layers; i++) {

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
		dies_area.push_back((max_outline_x * max_outline_y) / (this->IC.die_area));

		// aspect ratio; used to guide optimization towards fixed outline
		if (max_outline_y > 0.0) {
			dies_AR.push_back(max_outline_x / max_outline_y);
		}
		// dummy value for empty dies; implies cost of 0.0 for this die, i.e. does
		// not impact cost function
		else {
			dies_AR.push_back(this->IC.die_AR);
		}

		// memorize whether layout fits into outline
		max_outline_x /= this->IC.outline_x;
		max_outline_y /= this->IC.outline_y;
		layout_fits_in_fixed_outline = layout_fits_in_fixed_outline && (max_outline_x <= 1.0 && max_outline_y <= 1.0);
	}

	// cost for AR mismatch, considering max violation guides towards fixed outline
	cost_outline = 0.0;
	for (i = 0; i < this->IC.layers; i++) {
		cost_outline = max(cost_outline, pow(dies_AR[i] - this->IC.die_AR, 2.0));
	}
	// store actual value
	cost.outline_actual_value = cost_outline;
	// determine cost function value
	cost_outline *= 0.5 * FloorPlanner::SA_COST_WEIGHT_AREA_OUTLINE * (1.0 - fitting_layouts_ratio);

	// cost for area, considering max value of (blocks-outline area) / (die-outline
	// area) guides towards balanced die occupation and area minimization
	cost_area = 0.0;
	for (i = 0; i < this->IC.layers; i++) {
		cost_area = max(cost_area, dies_area[i]);
	}
	// store actual value
	cost.area_actual_value = cost_area;
	// determine cost function value
	cost_area *= 0.5 * FloorPlanner::SA_COST_WEIGHT_AREA_OUTLINE * (1.0 + fitting_layouts_ratio);

	cost.area_outline = cost_outline + cost_area;
	cost.fits_fixed_outline = layout_fits_in_fixed_outline;

	if (FloorPlanner::DBG_CALLS_SA) {
		cout << "<- FloorPlanner::evaluateAreaOutline" << endl;
	}
}


// Obtain hotspots (i.e., locally connected regions surrounding local maximum
// temperatures) from the (previous!) thermal analysis run. The determination of
// hotspots/blobs is based on Lindeberg's grey-level blob detection algorithm.
//
// Here, a ``chicken-egg'' problem arises: the clustered TSVs impact the thermal analysis,
// but for clustering TSVs we require the result of the thermal analysis. Thus, the
// determination of hotspots, which are the source for clustering TSVs into islands, are
// based on the previous thermal analysis run; with the assumption that one layout
// operation does not alter the thermal profile _significantly_ this appears a valid
// compromise. (The most precise however time-consuming approach would be to 1) perform
// the thermal analysis w/o TSVs, 2) cluster TSVs according to the thermal-analysis
// results, and 3) perform the thermal analysis again, w/ consideration of TSVs.
//
// TODO put determined TSV islands into FloorPlanner's vector<TSV_Group> TSVs;
void FloorPlanner::clusterSignalTSVs(vector< list<SegmentedNet> > &nets_seg, double temp_offset) {
	int x, y;
	unsigned i;
	list<SegmentedNet>::iterator it_net_seg;
	list<ThermalAnalyzer::ThermalMapBin*> thermal_map_list;
	list<ThermalAnalyzer::ThermalMapBin*> relev_neighbors;
	list<ThermalAnalyzer::ThermalMapBin*>::iterator it1;
	list<ThermalAnalyzer::ThermalMapBin*>::iterator it2;
	list<int> neighbor_regions;
	list<int>::iterator it3;
	ThermalAnalyzer::ThermalMapBin *cur_bin;
	int hotspot_region_id;
	map<int, HotspotRegion>::iterator it4;
	HotspotRegion *cur_region;
	bool bin_handled;

	if (FloorPlanner::DBG_CALLS_SA) {
		cout << "-> FloorPlanner::clusterSignalTSVs(" << &nets_seg << ")" << endl;
	}

	// sanity check for available thermal-analysis result (which is not the case in
	// the very first run of SA Phase II where interconnects and thermal profile are
	// (to be by definition) evaluated in that particular order
	if (this->thermal_analysis.thermal_map == nullptr) {
		return;
	}

	// reset hotspot regions
	this->hotspot_regions.clear();

	// reset bin-hotspot associations
	for (x = 0; x < ThermalAnalyzer::THERMAL_MAP_DIM; x++) {
		for (y = 0; y < ThermalAnalyzer::THERMAL_MAP_DIM; y++) {
			(*this->thermal_analysis.thermal_map)[x][y].hotspot_region_id = ThermalAnalyzer::HOTSPOT_UNDEFINED;
		}
	}

	// TODO use for monitoring clustering process
	//
	// reset cluster flag
	for (Net& cur_net : this->nets) {
		cur_net.clustered = false;
	}

	// sort the nets' bounding boxes by their area
	for (i = 0; i < nets_seg.size(); i++) {

		nets_seg[i].sort(
			// lambda expression
			[&](SegmentedNet sn1, SegmentedNet sn2) {
				return sn1.bb.area > sn2.bb.area;
			}
		);
	}

	// dbg, display all nets to consider for clustering
	if (FloorPlanner::DBG_CLUSTERING) {

		for (i = 0; i < nets_seg.size(); i++) {

			cout << "DBG_CLUSTERING> nets to consider for clustering on layer " << i << ":" << endl;

			for (it_net_seg = nets_seg[i].begin(); it_net_seg != nets_seg[i].end(); ++it_net_seg) {
				cout << "DBG_CLUSTERING>  net id: " << it_net_seg->net.id << endl;
				cout << "DBG_CLUSTERING>   bb area: " << it_net_seg->bb.area << endl;
			}
		}
	}
	
	// parse the extended 2D grid into an list (to be sorted below); data structure
	// for blob detection
	for (x = 0; x < ThermalAnalyzer::THERMAL_MAP_DIM; x++) {
		for (y = 0; y < ThermalAnalyzer::THERMAL_MAP_DIM; y++) {

			// ignore bins w/ temperature values near the offset
			if (Math::doubleComp(temp_offset, (*this->thermal_analysis.thermal_map)[x][y].temp)) {
				continue;
			}

			thermal_map_list.push_back(&(*this->thermal_analysis.thermal_map)[x][y]);
		}
	}

	// sort list by temperature values
	thermal_map_list.sort(
		// lambda expression
		[&](ThermalAnalyzer::ThermalMapBin* b1, ThermalAnalyzer::ThermalMapBin* b2) {
			return (b1->temp > b2->temp)
				// in cases with equal temperature, randomly shuffle bins
				// such that chances for neighboring bins w/ same
				// temperatures are mitigated
				|| ((b1->temp == b2->temp) && Math::randB());
		}
	);

	if (FloorPlanner::DBG_CLUSTERING) {
		cout << "DBG_CLUSTERING> bin w/ global max temperature [x][y]: " << thermal_map_list.front()->x << ", " << thermal_map_list.front()->y << endl;
		cout << "DBG_CLUSTERING>  temp: " << thermal_map_list.front()->temp << endl;
		for (it1 = thermal_map_list.front()->neighbors.begin(); it1 != thermal_map_list.front()->neighbors.end(); ++it1) {
			cout << "DBG_CLUSTERING>  neighbor bin [x][y]: " << (*it1)->x << ", " << (*it1)->y << endl;
		}
	}

	// group the thermal-map list into hotspot regions; perform actual blob detection
	hotspot_region_id = 0;
	for (it1 = thermal_map_list.begin(); it1 != thermal_map_list.end(); ++it1) {

		cur_bin = (*it1);

		// determine all neighboring bins w/ higher temperature
		relev_neighbors.clear();
		for (it2 = cur_bin->neighbors.begin(); it2 != cur_bin->neighbors.end(); ++it2) {

			if ((*it2)->temp > cur_bin->temp) {
				relev_neighbors.push_back(*it2);
			}
		}

		// if no such neighbor exits, then the current bin is a local maximum and
		// will be the seed for a new hotspot/blob
		if (relev_neighbors.empty()) {

			// initialize new hotspot
			this->hotspot_regions.insert( pair<int, HotspotRegion>(
					// region id is the key for the regions map
					hotspot_region_id,
					// actual hotspot initialization
					{
						// peak temp
						cur_bin->temp,
						// base-level temp; currently undefined
						-1.0,
						// temperature gradient; currently
						// undefined
						-1.0,
						// allocate list of associated bins
						list<ThermalAnalyzer::ThermalMapBin*>(),
						// memorize hotspot as still growing
						true,
						// region id
						hotspot_region_id,
						// region score; currently undefined
						-1.0
						})
				);

			// memorize bin as first bin of new hotspot
			this->hotspot_regions.find(hotspot_region_id)->second.bins.push_back(cur_bin);

			// mark bin as associated to this new hotspot
			cur_bin->hotspot_region_id = hotspot_region_id;

			// increment hotspot region counter/id
			hotspot_region_id++;
		}

		// some neighbor bins w/ higher temperatures exit
		else {
			bin_handled = false;

			// if any of these neighbors is a background bin, then this bin is
			// also a background bin
			for (it2 = relev_neighbors.begin(); it2 != relev_neighbors.end(); ++it2) {

				if ((*it2)->hotspot_region_id == ThermalAnalyzer::HOTSPOT_BACKGROUND) {

					cur_bin->hotspot_region_id = ThermalAnalyzer::HOTSPOT_BACKGROUND;
					bin_handled = true;

					break;
				}
			}

			// none of the neighbors are background bins; proceed with check
			// if the neighbors belong to one or to different hotspots
			if (!bin_handled) {

				neighbor_regions.clear();

				for (it2 = relev_neighbors.begin(); it2 != relev_neighbors.end(); ++it2) {

// not required, since not happened during
// debugging
//					// ignore so far undefined bins
//					//
//					if ((*it2)->hotspot_region_id == ThermalAnalyzer::HOTSPOT_UNDEFINED) {
//
//						if (FloorPlanner::DBG_CLUSTERING) {
//							cout << "DBG_CLUSTERING> blob-detection error; undefined bin triggered" << endl;
//						}
//
//						continue;
//					}

					neighbor_regions.push_back((*it2)->hotspot_region_id);
				}

				if (FloorPlanner::DBG_CLUSTERING) {

					if (neighbor_regions.empty()) {
						cout << "DBG_CLUSTERING> blob-detection error; no valid neighbor bin found" << endl;
					}
				}

				// memorize only unique regions
				neighbor_regions.sort();
				neighbor_regions.unique();

				// all neighbors belong to one specific hotspot
				if (neighbor_regions.size() == 1) {

					cur_region = &this->hotspot_regions.find(neighbor_regions.front())->second;

					// if the hotspot region is still allowed to grow,
					// associated this bin with it, and mark bin as
					// associated
					if (cur_region->still_growing) {

						cur_region->bins.push_back(cur_bin);
						cur_bin->hotspot_region_id = cur_region->region_id;
					}
					// if the region is not allowed to grow anymore,
					// mark the bin as background bin
					else {
						cur_bin->hotspot_region_id = ThermalAnalyzer::HOTSPOT_BACKGROUND;
					}
				}
				// neighbors belong to different hotspots
				else {
					// the bin has to be background since it defines
					// the base level for different hotspots
					cur_bin->hotspot_region_id = ThermalAnalyzer::HOTSPOT_BACKGROUND;

					// the different hotspots have reached their base
					// level w/ this bin; mark them as not growing
					// anymore and memorize the base-level temp
					for (it3 = neighbor_regions.begin(); it3 != neighbor_regions.end(); ++it3) {

						this->hotspot_regions.find(*it3)->second.still_growing = false;
						this->hotspot_regions.find(*it3)->second.base_temp = cur_bin->temp;

						// the determination of temp gradient and
						// score could be also conducted here, but
						// is postponed since a post-processing of
						// all regions is required anyway
					}
				}
			}
		}
	}

	// post-processing hotspot regions
	for (it4 = this->hotspot_regions.begin(); it4 != this->hotspot_regions.end(); ++it4) {

		cur_region = &(*it4).second;

		// some regions may be still marked as growing; mark such regions as not
		// growing anymore
		if (cur_region->still_growing) {

			cur_region->still_growing = false;

			// also approximate base temp, using the minimal temperature of
			// all bins of the region; note that the actual base temp is
			// slightly lower since the base-level bin is not included in the
			// region
			cur_region->base_temp = (*cur_region->bins.begin())->temp;
			for (it1 = cur_region->bins.begin(); it1 != cur_region->bins.end(); ++it1) {

				cur_region->base_temp = min(cur_region->base_temp, (*it1)->temp);
			}
		}

		// using the base temp, determine gradient
		cur_region->temp_gradient = cur_region->peak_temp - cur_region->base_temp;

		// using the base temp, determine score; the score is defined by its temp
		// gradient over the bin count, i.e., a measure of how ``compact'' the
		// local maxima is spread
		cur_region->region_score = cur_region->temp_gradient / cur_region->bins.size();
	}

	if (FloorPlanner::DBG_CLUSTERING) {
		int bins_hotspot = 0;
		int bins_background = 0;
		int bins_undefined = 0;

		cout << "DBG_CLUSTERING> hotspot regions:" << endl;

		for (it4 = this->hotspot_regions.begin(); it4 != this->hotspot_regions.end(); ++it4) {
			cout << "DBG_CLUSTERING>  region id: " << (*it4).second.region_id << endl;
			cout << "DBG_CLUSTERING>   peak temp: " << (*it4).second.peak_temp << endl;
			cout << "DBG_CLUSTERING>   base temp: " << (*it4).second.base_temp << endl;
			cout << "DBG_CLUSTERING>   temp gradient: " << (*it4).second.temp_gradient << endl;
			cout << "DBG_CLUSTERING>   region score: " << (*it4).second.region_score << endl;
			cout << "DBG_CLUSTERING>   bins count: " << (*it4).second.bins.size() << endl;
			cout << "DBG_CLUSTERING>   still growing: " << (*it4).second.still_growing << endl;
		}

		cout << "DBG_CLUSTERING> adapted thermal-map:" << endl;

		for (x = 0; x < ThermalAnalyzer::THERMAL_MAP_DIM; x++) {
			for (y = 0; y < ThermalAnalyzer::THERMAL_MAP_DIM; y++) {

				cur_bin = &(*this->thermal_analysis.thermal_map)[x][y];

				if (cur_bin->hotspot_region_id == ThermalAnalyzer::HOTSPOT_BACKGROUND) {
					bins_background++;
				}
				else if (cur_bin->hotspot_region_id == ThermalAnalyzer::HOTSPOT_UNDEFINED) {
					bins_undefined++;
				}
				else {
					bins_hotspot++;
				}
			}
		}

		cout << "DBG_CLUSTERING>  bins w/ hotspot assigned: " << bins_hotspot << endl;
		cout << "DBG_CLUSTERING>  background bins: " << bins_background << endl;
		cout << "DBG_CLUSTERING>  undefined bins: " << bins_undefined << endl;
	}

	if (FloorPlanner::DBG_CALLS_SA) {
		cout << "<- FloorPlanner::clusterSignalTSVs" << endl;
	}
}

void FloorPlanner::evaluateInterconnects(FloorPlanner::Cost& cost, bool const& set_max_cost) {
	int i;
	vector<Rect const*> blocks_to_consider;
	vector< list<SegmentedNet> > nets_seg;
	Rect bb, prev_bb;
	double prev_TSVs;

	if (FloorPlanner::DBG_CALLS_SA) {
		cout << "-> FloorPlanner::evaluateInterconnects(" << set_max_cost << ")" << endl;
	}

	// init cost terms
	cost.HPWL = cost.HPWL_actual_value = 0.0;
	cost.TSVs = cost.TSVs_actual_value = 0;
	cost.TSVs_area_deadspace_ratio = 0.0;

	// allocate vector for blocks to be considered
	blocks_to_consider.reserve(this->blocks.size());
	// allocate vector for nets' segments
	for (i = 0; i < this->IC.layers; i++) {
		nets_seg.emplace_back(list<SegmentedNet>());
	}

	// determine HPWL and TSVs for each net
	for (Net& cur_net : this->nets) {

		// set layer boundaries, i.e., determine lowest and uppermost layer of
		// net's blocks
		cur_net.setLayerBoundaries();

		if (Net::DBG) {
			cout << "DBG_NET> Determine interconnects for net " << cur_net.id << endl;
		}

		// trivial HPWL estimation, considering one global bounding box; required
		// to compare w/ other 3D floorplanning tools
		if (FloorPlanner::SA_COST_INTERCONNECTS_TRIVIAL_HPWL) {

			// resets blocks to be considered for each cur_net
			blocks_to_consider.clear();

			// blocks for cur_net on all layer
			for (Block const* b : cur_net.blocks) {
				blocks_to_consider.push_back(&b->bb);
			}

			// also consider routes to terminal pins
			for (Pin const* pin :  cur_net.terminals) {
				blocks_to_consider.push_back(&pin->bb);
			}

			// determine HPWL of related blocks using their bounding box;
			// consider center points of blocks instead their whole outline
			bb = Rect::determBoundingBox(blocks_to_consider, true);
			cost.HPWL += bb.w;
			cost.HPWL += bb.h;

			if (Net::DBG) {
				cout << "DBG_NET> 		HPWL of bounding box of blocks to consider: " << (bb.w + bb. h) << endl;
			}
		}
		// more detailed estimate; consider HPWL on each layer separately using
		// layer-related bounding boxes
		else {
			// reset previous bb
			prev_bb.area = 0.0;

			// determine HPWL on each related layer separately
			for (i = cur_net.layer_bottom; i <= cur_net.layer_top; i++) {

				// determine HPWL using the net's bounding box on the
				// current layer
				bb = cur_net.determBoundingBox(i);
				cost.HPWL += bb.w;
				cost.HPWL += bb.h;

				// memorize bounding boxes for nets connecting further up
				// (i.e., requiring a TSV); to be used later on for
				// clustering
				if (i != cur_net.layer_top) {

					// determBoundingBox may also return empty
					// bounding boxes; here, for not considering the
					// uppermost layer_top, only for nets w/o blocks
					// on the currently considered layer. Then, we
					// need to consider the non-empty box from one of
					// the layers below in order to provide a net's bb
					// for clustering
					if (bb.area == 0.0) {
						bb = prev_bb;
					}
					nets_seg[i].push_back({cur_net, bb});
				}

				// memorize current non-empty bb as previous bb for next
				// iteration
				if (bb.area != 0.0) {
					prev_bb = bb;
				}

				if (Net::DBG) {
					cout << "DBG_NET> 		HPWL of bounding box of blocks (in current and possibly upper layers) to consider: " << (bb.w + bb. h) << endl;
				}
			}
		}

		if (Net::DBG) {
			prev_TSVs = cost.TSVs;
		}

		// determine TSV count
		cost.TSVs += cur_net.layer_top - cur_net.layer_bottom;
		// also consider that terminal pins require TSV connections to the
		// lowermost die
		if (!cur_net.terminals.empty()) {
			cost.TSVs += cur_net.layer_bottom;
		}

		if (Net::DBG) {
			cout << "DBG_NET>  TSVs required: " << cost.TSVs - prev_TSVs << endl;
		}
	}

	// perform clustering of signal TSVs into TSV islands
	if (!FloorPlanner::SA_COST_INTERCONNECTS_TRIVIAL_HPWL && this->SA_parameters.layout_signal_TSV_clustering) {
		this->clusterSignalTSVs(nets_seg, this->power_blurring_parameters.temp_offset);
	}

	// also consider TSV lengths in HPWL; each TSV has to pass the whole Si layer and
	// the bonding layer
	if (!FloorPlanner::SA_COST_INTERCONNECTS_TRIVIAL_HPWL) {
		cost.HPWL += cost.TSVs * (this->IC.die_thickness + this->IC.bond_thickness);
	}

	// determine by TSVs occupied deadspace amount
	cost.TSVs_area_deadspace_ratio = (cost.TSVs * pow(this->IC.TSV_pitch, 2)) / this->IC.stack_deadspace;

	// memorize max cost; initial sampling
	if (set_max_cost) {
		this->max_cost_WL = cost.HPWL;
		this->max_cost_TSVs = cost.TSVs;
	}

	// store actual values
	cost.HPWL_actual_value = cost.HPWL;
	cost.TSVs_actual_value = cost.TSVs;
	// normalized values; refer to max value from initial sampling
	//
	cost.HPWL /= this->max_cost_WL;
	// sanity check for zero TSVs; applies to 2D floorplanning
	if (this->max_cost_TSVs != 0) {
		cost.TSVs /= this->max_cost_TSVs;
	}

	if (FloorPlanner::DBG_CALLS_SA) {
		cout << "<- FloorPlanner::evaluateInterconnects" << endl;
	}
}

// costs are derived from spatial mismatch b/w blocks' alignment and intended alignment;
// note that this function also marks requests as failed or successful
void FloorPlanner::evaluateAlignments(Cost& cost, vector<CorblivarAlignmentReq> const& alignments, bool const& derive_TSVs, bool const& set_max_cost) {
	Rect blocks_intersect;
	Rect blocks_bb;
	double TSVs_row_col;

	if (FloorPlanner::DBG_CALLS_SA) {
		cout << "-> FloorPlanner::evaluateAlignments(" << &cost << ", " << &alignments << ", " << derive_TSVs << ", " << set_max_cost << ")" << endl;
	}

	cost.alignments = cost.alignments_actual_value = 0.0;

	// evaluate all alignment requests
	for (CorblivarAlignmentReq const& req : alignments) {

		// initially, assume the request to be feasible
		req.fulfilled = true;
		// also assume alignment status of blocks themselves to be successful
		req.s_i->alignment = Block::AlignmentStatus::SUCCESS;
		req.s_j->alignment = Block::AlignmentStatus::SUCCESS;

		// for request w/ alignment ranges, we verify the alignment via the
		// blocks' intersection
		if (req.range_x() || req.range_y()) {
			blocks_intersect = Rect::determineIntersection(req.s_i->bb, req.s_j->bb);
		}
		// for requests w/ max distance ranges, we verify the alignment via the
		// blocks' bounding box (considering the blocks' center points)
		if (req.range_max_x() || req.range_max_y()) {
			blocks_bb = Rect::determBoundingBox(req.s_i->bb, req.s_j->bb, true);
		}

		// check partial request, horizontal alignment
		//
		// alignment range
		if (req.range_x()) {

			// consider the spatial mismatch as cost; overlap too small
			if (blocks_intersect.w < req.alignment_x) {

				// missing overlap
				cost.alignments += req.alignment_x - blocks_intersect.w;

				// in case blocks don't overlap at all, also consider the
				// blocks' distance as further cost
				if (blocks_intersect.w == 0) {

					if (Rect::rectA_leftOf_rectB(req.s_i->bb, req.s_j->bb, false)) {

						cost.alignments += req.s_j->bb.ll.x - req.s_i->bb.ur.x;

						// annotate block-alignment failure
						req.s_i->alignment = Block::AlignmentStatus::FAIL_HOR_TOO_LEFT;
						req.s_j->alignment = Block::AlignmentStatus::FAIL_HOR_TOO_RIGHT;
					}
					else {

						cost.alignments += req.s_i->bb.ll.x - req.s_j->bb.ur.x;

						// annotate block-alignment failure
						req.s_i->alignment = Block::AlignmentStatus::FAIL_HOR_TOO_RIGHT;
						req.s_j->alignment = Block::AlignmentStatus::FAIL_HOR_TOO_LEFT;
					}
				}

				// annotate general alignment failure
				req.fulfilled = false;
			}
		}
		// max distance range
		else if (req.range_max_x()) {

			// consider the spatial mismatch as cost; distance too large
			if (blocks_bb.w > req.alignment_x) {

				cost.alignments += blocks_bb.w - req.alignment_x;

				// annotate general alignment failure
				req.fulfilled = false;

				// annotate block-alignment failure
				if (req.s_i->bb.ll.x < req.s_j->bb.ll.x) {
					req.s_i->alignment = Block::AlignmentStatus::FAIL_HOR_TOO_LEFT;
					req.s_j->alignment = Block::AlignmentStatus::FAIL_HOR_TOO_RIGHT;
				}
				else {
					req.s_i->alignment = Block::AlignmentStatus::FAIL_HOR_TOO_RIGHT;
					req.s_j->alignment = Block::AlignmentStatus::FAIL_HOR_TOO_LEFT;
				}
			}
		}
		// fixed alignment offset
		else if (req.offset_x()) {

			// check the blocks' offset against the required offset
			if (!Math::doubleComp(req.s_j->bb.ll.x - req.s_i->bb.ll.x, req.alignment_x)) {

				// s_j should be to the right of s_i;
				// consider the spatial mismatch as cost
				if (req.alignment_x >= 0.0) {

					// s_j is to the right of s_i
					if (req.s_j->bb.ll.x > req.s_i->bb.ll.x) {

						// abs required for cases where s_j is too
						// far left, i.e., not sufficiently away
						// from s_i
						cost.alignments += abs(req.s_j->bb.ll.x - req.s_i->bb.ll.x - req.alignment_x);

						// annotate block-alignment failure;
						// s_j is too far left, s_i too far right
						if ((req.s_j->bb.ll.x - req.s_i->bb.ll.x - req.alignment_x) < 0) {
							req.s_i->alignment = Block::AlignmentStatus::FAIL_HOR_TOO_RIGHT;
							req.s_j->alignment = Block::AlignmentStatus::FAIL_HOR_TOO_LEFT;
						}
						// s_j is too far right, s_i too far left
						else {
							req.s_i->alignment = Block::AlignmentStatus::FAIL_HOR_TOO_LEFT;
							req.s_j->alignment = Block::AlignmentStatus::FAIL_HOR_TOO_RIGHT;
						}
					}
					// s_j is to the left of s_i
					else {
						// cost includes distance b/w (right) s_i,
						// (left) s_j and the failed offset
						cost.alignments += req.s_i->bb.ll.x - req.s_j->bb.ll.x + req.alignment_x;

						// annotate block-alignment failure
						req.s_i->alignment = Block::AlignmentStatus::FAIL_HOR_TOO_RIGHT;
						req.s_j->alignment = Block::AlignmentStatus::FAIL_HOR_TOO_LEFT;
					}
				}
				// s_j should be to the left of s_i;
				// consider the spatial mismatch as cost
				else {

					// s_j is to the left of s_i
					if (req.s_j->bb.ll.x < req.s_i->bb.ll.x) {

						// abs required for cases where s_j is too
						// far right, i.e., not sufficiently away
						// from s_i
						cost.alignments += abs(req.s_i->bb.ll.x - req.s_j->bb.ll.x + req.alignment_x);

						// annotate block-alignment failure;
						// s_j is too far right, s_i too far left
						if ((req.s_i->bb.ll.x - req.s_j->bb.ll.x + req.alignment_x) < 0) {
							req.s_i->alignment = Block::AlignmentStatus::FAIL_HOR_TOO_LEFT;
							req.s_j->alignment = Block::AlignmentStatus::FAIL_HOR_TOO_RIGHT;
						}
						// s_j is too far left, s_i too far right
						else {
							req.s_i->alignment = Block::AlignmentStatus::FAIL_HOR_TOO_RIGHT;
							req.s_j->alignment = Block::AlignmentStatus::FAIL_HOR_TOO_LEFT;
						}
					}
					// s_j is right of s_i
					else {
						// cost includes distance b/w (left) s_i,
						// (right) s_j and the failed (negative) offset
						cost.alignments += req.s_j->bb.ll.x - req.s_i->bb.ll.x - req.alignment_x;

						// annotate block-alignment failure
						req.s_i->alignment = Block::AlignmentStatus::FAIL_HOR_TOO_LEFT;
						req.s_j->alignment = Block::AlignmentStatus::FAIL_HOR_TOO_RIGHT;
					}
				}

				// annotate general alignment failure
				req.fulfilled = false;
			}
		}

		// check partial request, vertical alignment
		//
		// alignment range
		if (req.range_y()) {

			// consider the spatial mismatch as cost; overlap too small
			if (blocks_intersect.h < req.alignment_y) {

				// missing overlap
				cost.alignments += req.alignment_y - blocks_intersect.h;

				// in case blocks don't overlap at all, also consider the
				// blocks' distance as further cost
				if (blocks_intersect.h == 0) {

					if (Rect::rectA_below_rectB(req.s_i->bb, req.s_j->bb, false)) {

						cost.alignments += req.s_j->bb.ll.y - req.s_i->bb.ur.y;

						// annotate block-alignment failure
						req.s_i->alignment = Block::AlignmentStatus::FAIL_VERT_TOO_LOW;
						req.s_j->alignment = Block::AlignmentStatus::FAIL_VERT_TOO_HIGH;
					}
					else {

						cost.alignments += req.s_i->bb.ll.y - req.s_j->bb.ur.y;

						// annotate block-alignment failure
						req.s_i->alignment = Block::AlignmentStatus::FAIL_VERT_TOO_HIGH;
						req.s_j->alignment = Block::AlignmentStatus::FAIL_VERT_TOO_LOW;
					}
				}

				// annotate general alignment failure
				req.fulfilled = false;
			}
		}
		// max distance range
		else if (req.range_max_y()) {

			// consider the spatial mismatch as cost; distance too large
			if (blocks_bb.h > req.alignment_y) {

				cost.alignments += blocks_bb.h - req.alignment_y;

				// annotate general alignment failure
				req.fulfilled = false;

				// annotate block-alignment failure
				if (req.s_i->bb.ll.y < req.s_j->bb.ll.y) {
					req.s_i->alignment = Block::AlignmentStatus::FAIL_VERT_TOO_LOW;
					req.s_j->alignment = Block::AlignmentStatus::FAIL_VERT_TOO_HIGH;
				}
				else {
					req.s_i->alignment = Block::AlignmentStatus::FAIL_VERT_TOO_HIGH;
					req.s_j->alignment = Block::AlignmentStatus::FAIL_VERT_TOO_LOW;
				}
			}
		}
		// fixed alignment offset
		else if (req.offset_y()) {

			// check the blocks' offset against the required offset
			if (!Math::doubleComp(req.s_j->bb.ll.y - req.s_i->bb.ll.y, req.alignment_y)) {

				// s_j should be above s_i;
				// consider the spatial mismatch as cost
				if (req.alignment_y >= 0.0) {

					// s_j is above s_i
					if (req.s_j->bb.ll.y > req.s_i->bb.ll.y) {

						// abs required for cases where s_j is too
						// far lowerwards, i.e., not sufficiently
						// away from s_i
						cost.alignments += abs(req.s_j->bb.ll.y - req.s_i->bb.ll.y - req.alignment_y);

						// annotate block-alignment failure;
						// s_j is too far lowerwards, s_i too far upwards
						if ((req.s_j->bb.ll.y - req.s_i->bb.ll.y - req.alignment_y) < 0) {
							req.s_i->alignment = Block::AlignmentStatus::FAIL_VERT_TOO_HIGH;
							req.s_j->alignment = Block::AlignmentStatus::FAIL_VERT_TOO_LOW;
						}
						// s_j is too far upwards, s_i too far
						// lowerwards
						else {
							req.s_i->alignment = Block::AlignmentStatus::FAIL_VERT_TOO_LOW;
							req.s_j->alignment = Block::AlignmentStatus::FAIL_VERT_TOO_HIGH;
						}
					}
					// s_j is below s_i
					else {
						// cost includes distance b/w (upper) s_i,
						// (lower) s_j and the failed offset
						cost.alignments += req.s_i->bb.ll.y - req.s_j->bb.ll.y + req.alignment_y;

						// annotate block-alignment failure
						req.s_i->alignment = Block::AlignmentStatus::FAIL_VERT_TOO_HIGH;
						req.s_j->alignment = Block::AlignmentStatus::FAIL_VERT_TOO_LOW;
					}
				}
				// s_j should be below s_i;
				// consider the spatial mismatch as cost
				else {

					// s_j is below s_i
					if (req.s_j->bb.ll.y < req.s_i->bb.ll.y) {

						// abs required for cases where s_j is too
						// far upwards, i.e., not sufficiently
						// away from s_i
						cost.alignments += abs(req.s_i->bb.ll.y - req.s_j->bb.ll.y + req.alignment_y);

						// annotate block-alignment failure;
						// s_j is too far upwards, s_i too far
						// lowerwards
						if ((req.s_i->bb.ll.y - req.s_j->bb.ll.y + req.alignment_y) < 0) {
							req.s_i->alignment = Block::AlignmentStatus::FAIL_VERT_TOO_LOW;
							req.s_j->alignment = Block::AlignmentStatus::FAIL_VERT_TOO_HIGH;
						}
						// s_j is too far lowerwards, s_i too far
						// upwards
						else {
							req.s_i->alignment = Block::AlignmentStatus::FAIL_VERT_TOO_HIGH;
							req.s_j->alignment = Block::AlignmentStatus::FAIL_VERT_TOO_LOW;
						}
					}
					// s_j is above s_i
					else {
						// cost includes distance b/w (lower) s_i,
						// (upper) s_j and the failed (negative) offset
						cost.alignments += req.s_j->bb.ll.y - req.s_i->bb.ll.y - req.alignment_y;

						// annotate block-alignment failure
						req.s_i->alignment = Block::AlignmentStatus::FAIL_VERT_TOO_LOW;
						req.s_j->alignment = Block::AlignmentStatus::FAIL_VERT_TOO_HIGH;
					}
				}

				// annotate general alignment failure
				req.fulfilled = false;
			}
		}

		// dbg logging for alignment
		if (FloorPlanner::DBG_ALIGNMENT) {

			cout << "DBG_ALIGNMENT> " << req.tupleString() << endl;

			if (req.fulfilled) {
				cout << "DBG_ALIGNMENT>  Success" << endl;
			}
			else {
				cout << "DBG_ALIGNMENT>  Failure" << endl;
				cout << "DBG_ALIGNMENT>   block " << req.s_i->id << ": " << req.s_i->alignment << endl;
				cout << "DBG_ALIGNMENT>   block " << req.s_j->id << ": " << req.s_j->alignment << endl;
			}
		}

		// derive TSVs for vertical buses if desired; only consider fulfilled
		// alignments
		if (derive_TSVs && req.fulfilled) {

			// consider valid block intersections independent of defined
			// alignment; this way, all vertical buses arising from different
			// alignment requests will be considered 
			blocks_intersect = Rect::determineIntersection(req.s_i->bb, req.s_j->bb);
			if (blocks_intersect.area != 0.0) {

				// consider TSVs in all affected layers
				for (int layer = min(req.s_i->layer, req.s_j->layer); layer < max(req.s_i->layer, req.s_j->layer); layer++) {

					// init new bus
					TSV_Group vert_bus = TSV_Group("bus_" + req.s_i->id + "_" + req.s_j->id, req.signals, layer);

					// define bus outline; consider required area for
					// given amount of TSVs
					//
					// note that the following code does _not_consider
					// a sanity check where the required area for TSVs
					// is larger than the intersection; since TSVs are
					// assumed to be embedded into blocks later on
					// anyway, such over-usage of block area is not
					// critical

					// init TSV group with actual intersection; this
					// way, the lower-left corners of the TSV group
					// and the actual intersection match
					vert_bus.bb = blocks_intersect;

					// minimal side of TSV-group rectangle is
					// intersection's width
					if (blocks_intersect.w < blocks_intersect.h) {

						// determine maximal amount of TSVs to be
						// put in smaller side of TSV-group
						// rectangle
						TSVs_row_col = floor(blocks_intersect.w / this->IC.TSV_pitch);

						// define smaller side of actual TSV-group
						// rectangle
						vert_bus.bb.w = TSVs_row_col * this->IC.TSV_pitch;
						vert_bus.bb.ur.x = vert_bus.bb.ll.x + vert_bus.bb.w;

						// define larger side of actual TSV-group
						// rectangle; ceil accounts for additional
						// row of TSVs if they are not completely
						// fitting, i.e., not filling a rectangle
						vert_bus.bb.h = ceil(vert_bus.TSVs_count / TSVs_row_col) * this->IC.TSV_pitch;
						vert_bus.bb.ur.y = vert_bus.bb.ll.y + vert_bus.bb.h;
					}
					// minimal side of TSV-group rectangle is
					// intersection's height
					else {
						// determine maximal amount of TSVs to be
						// put in smaller side of TSV-group
						// rectangle
						TSVs_row_col = floor(blocks_intersect.h / this->IC.TSV_pitch);

						// define smaller side of actual TSV-group
						// rectangle
						vert_bus.bb.h = TSVs_row_col * this->IC.TSV_pitch;
						vert_bus.bb.ur.y = vert_bus.bb.ll.y + vert_bus.bb.h;

						// define larger side of actual TSV-group
						// rectangle; ceil accounts for additional
						// column of TSVs if they are not completely
						// fitting, i.e., not filling a rectangle
						vert_bus.bb.w = ceil(vert_bus.TSVs_count / TSVs_row_col) * this->IC.TSV_pitch;
						vert_bus.bb.ur.x = vert_bus.bb.ll.x + vert_bus.bb.w;
					}

					// dbg logging for TSV generation
					if (FloorPlanner::DBG_TSVS) {

						cout << "DBG_TSVs> TSV group" << endl;
						cout << "DBG_TSVs>  " << vert_bus.id << endl;
						cout << "DBG_TSVs>  (" << vert_bus.bb.ll.x << "," << vert_bus.bb.ll.y << ")";
						cout << "(" << vert_bus.bb.ur.x << "," << vert_bus.bb.ur.y << ")" << endl;
					}

					// store bus
					this->TSVs.push_back(move(vert_bus));
				}
			}
		}
	}

	// memorize max cost; initial sampling
	if (set_max_cost) {
		this->max_cost_alignments = cost.alignments;
	}

	// store actual value
	cost.alignments_actual_value = cost.alignments;
	// normalize value; refers to max value from initial sampling
	cost.alignments /= this->max_cost_alignments;

	if (FloorPlanner::DBG_CALLS_SA) {
		cout << "<- FloorPlanner::evaluateAlignments : " << cost.alignments << endl;
	}
}
