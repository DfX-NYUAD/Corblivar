/**
 * =====================================================================================
 *
 *    Description:  Corblivar signal-TSV clustering
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
#ifndef _CORBLIVAR_CLUSTERING
#define _CORBLIVAR_CLUSTERING

// library includes
#include "Corblivar.incl.hpp"
// Corblivar includes, if any
#include "ThermalAnalyzer.hpp"
// forward declarations, if any

/// Corblivar signal-TSV clustering
class Clustering {
	private:
		/// debugging code switches
		static constexpr bool DBG = false;
		/// debugging code switches
		static constexpr bool DBG_HOTSPOT = false;
		/// debugging code switches
		static constexpr bool DBG_CLUSTERING = false;
		/// debugging code switches
		static constexpr bool DBG_CLUSTERING_FINAL= false;

	public:
		/// debugging code switches
		static constexpr bool DBG_HOTSPOT_PLOT = false;

	// constructors, destructors, if any non-implicit
	public:

	// public data, functions
	public:
		/// POD wrapping nets' segments
		struct Segments {
			Net* net;
			Rect bb;
		};
		/// POD wrapping net clusters
		struct Cluster {
			std::list<Net*> nets;
			Rect bb;
			unsigned hotspot_id;
		};
		/// POD wrapping hotspot regions
		struct Hotspot {
			double peak_temp;
			double base_temp;
			double temp_gradient;
			std::vector<ThermalAnalyzer::ThermalMapBin*> bins;
			bool still_growing;
			unsigned id;
			double score;
			Rect bb;
		};

		/// Hotspots container
		std::vector<Hotspot> hotspots;

		/// Clustering helper
		void clusterSignalTSVs(std::vector<Net> &nets,
				std::vector< std::vector<Segments> > &nets_segments,
				std::vector<TSV_Island> &TSVs,
				std::vector<Block> const& blocks,
				double const& outline_x,
				double const& outline_y,
				double const& TSV_pitch,
				unsigned const& upper_limit_TSVs,
				ThermalAnalyzer::ThermalAnalysisResult &thermal_analysis);

	// private data, functions
	private:

		/// Hotspot determination
		void determineHotspots(ThermalAnalyzer::ThermalAnalysisResult &thermal_analysis);

		/// Normalization scale for hotspot score
		static constexpr double SCORE_NORMALIZATION = 1.0e6;

		/// Cluster container
		std::vector< std::list<Cluster> > clusters;
};

#endif
