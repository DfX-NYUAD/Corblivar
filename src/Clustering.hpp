/*
 * =====================================================================================
 *
 *    Description:  Corblivar signal-TSV clustering
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
#ifndef _CORBLIVAR_CLUSTERING
#define _CORBLIVAR_CLUSTERING

// library includes
#include "Corblivar.incl.hpp"
// Corblivar includes, if any
#include "ThermalAnalyzer.hpp"
#include "Block.hpp"
// forward declarations, if any

class Clustering {
	// debugging code switches
	private:
		static constexpr bool DBG = false;
		static constexpr bool DBG_HOTSPOT = false;
		static constexpr bool DBG_CLUSTERING = false;
	public:
		static constexpr bool DBG_HOTSPOT_PLOT = false;

	// constructors, destructors, if any non-implicit
	public:

	// public data, functions
	public:
		// POD wrapping nets' segments
		struct Segments {
			Net const* net;
			Rect bb;
		};
		// POD wrapping net clusters
		struct Cluster {
			std::list<Net const*> nets;
			Rect bb;
			int hotspot_id;
		};
		// POD wrapping hotspot regions
		struct Hotspot {
			double peak_temp;
			double base_temp;
			double temp_gradient;
			std::list<ThermalAnalyzer::ThermalMapBin*> bins;
			bool still_growing;
			int id;
			double score;
			Rect bb;
		};

		// hotspots container
		std::map<double, Hotspot, std::greater<double>> hotspots;

		// clustering helper
		void clusterSignalTSVs(std::vector<Net> &nets,
				std::vector< std::list<Segments> > &nets_segments,
				std::vector<TSV_Island> &TSVs,
				double const& TSV_pitch,
				ThermalAnalyzer::ThermalAnalysisResult &thermal_analysis);

	// private data, functions
	private:

		// hotspot determination
		void determineHotspots(ThermalAnalyzer::ThermalAnalysisResult &thermal_analysis);

		// normalization scale for hotspot score
		static constexpr double SCORE_NORMALIZATION = 1.0e6;

		// cluster container
		std::vector< std::list<Cluster> > clusters;
};

#endif
