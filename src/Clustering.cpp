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

// own Corblivar header
#include "Clustering.hpp"
// required Corblivar headers
#include "ThermalAnalyzer.hpp"
#include "Net.hpp"

// For clustering, a ``chicken-egg'' problem arises: the clustered TSVs impact the thermal
// analysis, but for clustering TSVs we require the result of the thermal analysis. Thus,
// the determination of hotspots, which are the source for clustering TSVs into islands,
// are based on the previous thermal analysis run---with the assumption that one layout
// operation does not alter the thermal profile _significantly_ this appears a valid
// compromise. (The most precise, however time-consuming, approach would be to 1) perform
// the thermal analysis w/o TSVs, 2) cluster TSVs according to the thermal-analysis
// results, and 3) perform the thermal analysis again, w/ consideration of TSVs.
//
// TODO put determined TSV islands into FloorPlanner's vector<TSV_Group> TSVs;
void Clustering::clusterSignalTSVs(vector<Net> &nets, vector< list<Net::Segments> > &nets_segments, ThermalAnalyzer::ThermalAnalysisResult &thermal_analysis) {
	int x, y;
	unsigned i;
	list<Net::Segments>::iterator it_net_seg;

	if (Clustering::DBG) {
		cout << "-> Clustering::clusterSignalTSVs(" << &nets << ", " << &nets_segments << ", " << &thermal_analysis << ")" << endl;
	}

	// sanity check for available thermal-analysis result; note that these results are
	// for example _not_ available during the very first run of SA Phase II where
	// interconnects (and thus this function) are evaluated before the thermal profile
	if (thermal_analysis.thermal_map == nullptr) {
		return;
	}

	// reset hotspot regions
	this->hotspot_regions.clear();

	// reset hotspot associations in the thermal map
	for (x = 0; x < ThermalAnalyzer::THERMAL_MAP_DIM; x++) {
		for (y = 0; y < ThermalAnalyzer::THERMAL_MAP_DIM; y++) {
			(*thermal_analysis.thermal_map)[x][y].hotspot_region_id = ThermalAnalyzer::HOTSPOT_UNDEFINED;
		}
	}

	// determine hotspots according to (previous!) thermal-analysis run
	this->determineHotspots(thermal_analysis);

	// reset cluster flag
	// TODO use for monitoring clustering process
	for (Net& cur_net : nets) {
		cur_net.clustered = false;
	}

	// sort the nets' bounding boxes by their area
	for (i = 0; i < nets_segments.size(); i++) {

		nets_segments[i].sort(
			// lambda expression
			[&](Net::Segments sn1, Net::Segments sn2) {
				return sn1.bb.area > sn2.bb.area;
			}
		);
	}

	// dbg, display all nets to consider for clustering
	if (Clustering::DBG_CLUSTERING) {

		for (i = 0; i < nets_segments.size(); i++) {

			cout << "DBG_CLUSTERING> nets to consider for clustering on layer " << i << ":" << endl;

			for (it_net_seg = nets_segments[i].begin(); it_net_seg != nets_segments[i].end(); ++it_net_seg) {
				cout << "DBG_CLUSTERING>  net id: " << it_net_seg->net.id << endl;
				cout << "DBG_CLUSTERING>   bb area: " << it_net_seg->bb.area << endl;
			}
		}
	}

	if (Clustering::DBG) {
		cout << "<- Clustering::clusterSignalTSVs" << endl;
	}
}

// Obtain hotspots (i.e., locally connected regions surrounding local maximum
// temperatures) from the thermal analysis run. The determination of hotspots/blobs is
// based on Lindeberg's grey-level blob detection algorithm.
void Clustering::determineHotspots(ThermalAnalyzer::ThermalAnalysisResult &thermal_analysis) {
	int x, y;
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

	// parse the thermal grid into an list (to be sorted below); data structure for
	// blob detection
	for (x = 0; x < ThermalAnalyzer::THERMAL_MAP_DIM; x++) {
		for (y = 0; y < ThermalAnalyzer::THERMAL_MAP_DIM; y++) {

			// ignore bins w/ temperature values near the offset
			if (Math::doubleComp(thermal_analysis.temp_offset, (*thermal_analysis.thermal_map)[x][y].temp)) {
				continue;
			}

			thermal_map_list.push_back(&(*thermal_analysis.thermal_map)[x][y]);
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

	if (Clustering::DBG_HOTSPOT) {
		cout << "DBG_HOTSPOT> bin w/ global max temperature [x][y]: " << thermal_map_list.front()->x << ", " << thermal_map_list.front()->y << endl;
		cout << "DBG_HOTSPOT>  temp: " << thermal_map_list.front()->temp << endl;
		for (it1 = thermal_map_list.front()->neighbors.begin(); it1 != thermal_map_list.front()->neighbors.end(); ++it1) {
			cout << "DBG_HOTSPOT>  neighbor bin [x][y]: " << (*it1)->x << ", " << (*it1)->y << endl;
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
//						if (Clustering::DBG_HOTSPOT) {
//							cout << "DBG_HOTSPOT> blob-detection error; undefined bin triggered" << endl;
//						}
//
//						continue;
//					}

					neighbor_regions.push_back((*it2)->hotspot_region_id);
				}

				if (Clustering::DBG_HOTSPOT) {

					if (neighbor_regions.empty()) {
						cout << "DBG_HOTSPOT> blob-detection error; no valid neighbor bin found" << endl;
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

	if (Clustering::DBG_HOTSPOT) {
		int bins_hotspot = 0;
		int bins_background = 0;
		int bins_undefined = 0;

		cout << "DBG_HOTSPOT> hotspot regions:" << endl;

		for (it4 = this->hotspot_regions.begin(); it4 != this->hotspot_regions.end(); ++it4) {
			cout << "DBG_HOTSPOT>  region id: " << (*it4).second.region_id << endl;
			cout << "DBG_HOTSPOT>   peak temp: " << (*it4).second.peak_temp << endl;
			cout << "DBG_HOTSPOT>   base temp: " << (*it4).second.base_temp << endl;
			cout << "DBG_HOTSPOT>   temp gradient: " << (*it4).second.temp_gradient << endl;
			cout << "DBG_HOTSPOT>   region score: " << (*it4).second.region_score << endl;
			cout << "DBG_HOTSPOT>   bins count: " << (*it4).second.bins.size() << endl;
			cout << "DBG_HOTSPOT>   still growing: " << (*it4).second.still_growing << endl;
		}

		cout << "DBG_HOTSPOT> adapted thermal-map:" << endl;

		for (x = 0; x < ThermalAnalyzer::THERMAL_MAP_DIM; x++) {
			for (y = 0; y < ThermalAnalyzer::THERMAL_MAP_DIM; y++) {

				cur_bin = &(*thermal_analysis.thermal_map)[x][y];

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

		cout << "DBG_HOTSPOT>  bins w/ hotspot assigned: " << bins_hotspot << endl;
		cout << "DBG_HOTSPOT>  background bins: " << bins_background << endl;
		cout << "DBG_HOTSPOT>  undefined bins: " << bins_undefined << endl;
	}
}
