/*
 * =====================================================================================
 *
 *    Description:  Chip parameters
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
#ifndef _CORBLIVAR_CHIP
#define _CORBLIVAR_CHIP

class Chip {
	public:

		// 3D IC die layer parameters
		// 100um thick dies; own value
		static constexpr double THICKNESS_SI = 100.0e-6;
		// 2um active Si layer; [Sridhar10]
		static constexpr double THICKNESS_SI_ACTIVE = 2.0e-06;
		// passive Si layer, results in 98um
		static constexpr double THICKNESS_SI_PASSIVE = THICKNESS_SI - THICKNESS_SI_ACTIVE;
		// 12um BEOL; [Sridhar10]
		static constexpr double THICKNESS_BEOL = 12.0e-06;
		// 20um BCB bond; [Sridhar10]
		static constexpr double THICKNESS_BOND = 20.0e-06;
		//
		// TSV properties; own values
		// 5um dimension
		static constexpr double TSV_DIMENSION = 5.0e-06;
		// 10um pitch
		static constexpr double TSV_PITCH = 10.0e-06;
		// TSV density; for groups of 2^i (i >= 2) TSVs; derived for i = 2, i.e.,
		// a square w/ 4 TSVs
		static constexpr double TSV_DENSITY = (4.0 * TSV_DIMENSION * TSV_DIMENSION) / ((TSV_DIMENSION + TSV_PITCH) * (TSV_DIMENSION + TSV_PITCH));
		// Cu-Si area ratio for TSV groups; A_Cu / A_Si = A_Cu / (A_group - A_Cu)
		// = 1 / ( (A_group / A_Cu) - 1 )
		static constexpr double TSV_GROUP_CU_SI_AREA_RATIO = 1.0 / (
			((TSV_DIMENSION + TSV_PITCH) * (TSV_DIMENSION + TSV_PITCH)) / (4.0 * TSV_DIMENSION * TSV_DIMENSION) - 1.0);
};

#endif
