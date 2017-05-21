/*
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

// own Corblivar header
#include "Clustering.hpp"
// required Corblivar headers
#include "Net.hpp"

/// For clustering, a ``chicken-egg'' problem arises: the clustered TSVs impact the thermal
/// analysis, but for clustering TSVs we require the result of the thermal analysis. Thus,
/// the determination of hotspots, which are the source for clustering TSVs into islands,
/// are based on the previous thermal analysis run---with the assumption that one layout
/// operation does not alter the thermal profile _significantly_ this appears a valid
/// compromise. (The most precise, however time-consuming, approach would be to 1) perform
/// the thermal analysis w/o TSVs, 2) cluster TSVs according to the thermal-analysis
/// results, and 3) perform the thermal analysis again, w/ consideration of TSVs.)
// TODO according to valgrind/callgrind, the efforts for thermal analysis are around 8%, whereas the efforts for determineHotspots are 30%; thus, we could also allow for the
// additional efforts for another run of thermal analysis
void Clustering::clusterSignalTSVs(std::vector<Net> &nets, std::vector< std::vector<Segments> > &nets_segments, std::vector<TSV_Island> &TSVs, double const& TSV_pitch, unsigned const& upper_limit_TSVs, ThermalAnalyzer::ThermalAnalysisResult &thermal_analysis) {
	unsigned i, j;
	std::vector<Segments>::iterator it_seg;
	std::list<Net*>::iterator it_net;
	Rect intersection, cluster;
	bool all_clustered;
	std::list<Cluster>::iterator it_cluster;

	if (Clustering::DBG) {
		std::cout << "-> Clustering::clusterSignalTSVs(" << &nets << ", " << &nets_segments << ", " << &thermal_analysis << ")" << std::endl;
	}

	// reset previous hotspots and re-determine hotspots according to (previous!)
	// thermal-analysis run
	this->determineHotspots(thermal_analysis);

	// reset previous cluster
	this->clusters.clear();

	// perform layer-wise clustering
	for (i = 0; i < nets_segments.size(); i++) {

		// sort the nets' bounding boxes by their area
		std::sort(nets_segments[i].begin(), nets_segments[i].end(),
			// lambda expression
			[](Segments const& sn1, Segments const& sn2) {
				return
				// std::sort requires a _strict_ ordering, thus we have to make sure that same elements returns false
				// http://stackoverflow.com/a/1541909
				// also helps to make comparison short-cutting it early
				(sn1.net != sn2.net) && (
					sn1.bb.area > sn2.bb.area
				);
			}
		);

		// dbg, display all nets to consider for clustering
		if (Clustering::DBG_CLUSTERING) {

			std::cout << "DBG_CLUSTERING> nets to consider for clustering on layer " << i << ":" << std::endl;

			for (it_seg = nets_segments[i].begin(); it_seg != nets_segments[i].end(); ++it_seg) {
				std::cout << "DBG_CLUSTERING>  net id: " << it_seg->net->id << std::endl;
				std::cout << "DBG_CLUSTERING>   bb area: " << it_seg->bb.area << std::endl;
			}

			std::cout << "DBG_CLUSTERING>" << std::endl;
		}

		// reset cluster flags of nets to consider on this layer
		for (it_seg = nets_segments[i].begin(); it_seg != nets_segments[i].end(); ++it_seg) {
			(*it_seg).net->clustered = false;
		}

		// allocate cluster list
		this->clusters.emplace_back(std::list<Cluster>());

		// iteratively merge net segments into clusters; try at most so many times
		// like nets are to considered on this layer
		for (j = 1; j <= nets_segments[i].size(); j++) {

			if (Clustering::DBG_CLUSTERING) {
				std::cout << "DBG_CLUSTERING> clustering of net segments; clustering iteration " << j << std::endl;
			}

			// reset cluster
			cluster.area = 0.0;
			// reset clustering flag
			all_clustered = true;

			for (it_seg = nets_segments[i].begin(); it_seg != nets_segments[i].end(); ++it_seg) {

				// ignore already clustered segments
				if ((*it_seg).net->clustered) {
					continue;
				}
				else {
					// memorize that at least one net is not clustered
					// yet
					all_clustered = false;

					// init new cluster if not done yet
					if (cluster.area == 0.0) {

						if (Clustering::DBG_CLUSTERING) {
							std::cout << "DBG_CLUSTERING> init new cluster..." << std::endl;
							std::cout << "DBG_CLUSTERING>  initial net: " << (*it_seg).net->id << std::endl;
						}

						// actual init
						this->clusters[i].push_back({
								// init list of nets with
								// this initial net
								std::list<Net*>(1, (*it_seg).net),
								// init enclosing bb with
								// this initial net
								(*it_seg).bb,
								// dummy hotspot id, since
								// cluster is not
								// associated with any
								// hotspot yet
								0
							});

						// memorize initial cluster
						cluster = (*it_seg).bb;

						// also mark initial net as clustered now
						(*it_seg).net->clustered = true;

						// try to merge with any hotspot;
						// considering the most critical ones
						// first, done via iteration of
						// score-sorted map
						//
						// note that this step is implicitly
						// ignored when thermal optimization and
						// thus thermal analysis are deactivated
						//
						for (Hotspot const& cur_hotspot : this->hotspots) {

							intersection = Rect::determineIntersection(cluster, cur_hotspot.bb);

							// this hotspot overlaps the
							// initial net; consider their
							// intersection for further
							// clustering
							if (intersection.area != 0.0) {

								cluster = intersection;

								if (Clustering::DBG_CLUSTERING) {
									std::cout << "DBG_CLUSTERING>  considering hotspot ";
									std::cout << cur_hotspot.id << " for this cluster" << std::endl;
								}

								//also memorize hotspot id
								//in cluster itself
								this->clusters[i].back().hotspot_id = cur_hotspot.id;

								break;
							}
						}
					}
					// cluster is already initialized; try to merge
					// (further) segments into current cluster; only
					// until upper limit of TSVs per cluster is not
					// reached yet
					else if (this->clusters[i].back().nets.size() < upper_limit_TSVs) {

						// determine intersection of cluster w/
						// current segment
						intersection = Rect::determineIntersection(cluster, (*it_seg).bb);

						// ignore merges which would results in
						// empty (i.e., non-overlapping) segments
						if (intersection.area == 0.0) {

							if (Clustering::DBG_CLUSTERING) {
								std::cout << "DBG_CLUSTERING>  ignore net " << (*it_seg).net->id << " for this cluster" << std::endl;
							}

							continue;
						}
						// else update cluster
						else {
							this->clusters[i].back().nets.push_back((*it_seg).net);
							this->clusters[i].back().bb = intersection;

							// also update cluster-region
							// monitor variable
							cluster = intersection;

							// also mark net as clustered now
							(*it_seg).net->clustered = true;

							if (Clustering::DBG_CLUSTERING) {
								std::cout << "DBG_CLUSTERING>  add net " << (*it_seg).net->id << " to this cluster" << std::endl;
							}
						}
					}
				}
			}

			if (Clustering::DBG_CLUSTERING) {
				std::cout << "DBG_CLUSTERING>" << std::endl;
			}

			// break merge loop in case all nets have been already clustered
			if (all_clustered) {
				break;
			}
		}

		// dbg, display all cluster
		if (Clustering::DBG_CLUSTERING_FINAL) {

			std::cout << "DBG_CLUSTERING> final set of clusters on layer " << i << ":" << std::endl;
			std::cout << "DBG_CLUSTERING>" << std::endl;

			for (it_cluster = this->clusters[i].begin(); it_cluster != this->clusters[i].end(); ++it_cluster) {

				std::cout << "DBG_CLUSTERING>  cluster bb:";
				std::cout << " (" << (*it_cluster).bb.ll.x << ",";
				std::cout << (*it_cluster).bb.ll.y << "),";
				std::cout << " (" << (*it_cluster).bb.ur.x << ",";
				std::cout << (*it_cluster).bb.ur.y << ")" << std::endl;

				std::cout << "DBG_CLUSTERING>  associated hotspot:" << (*it_cluster).hotspot_id << std::endl;

				for (it_net = (*it_cluster).nets.begin(); it_net != (*it_cluster).nets.end(); ++it_net) {
					std::cout << "DBG_CLUSTERING>   net id: " << (*it_net)->id << std::endl;
				}

				std::cout << "DBG_CLUSTERING>" << std::endl;
			}

			std::cout << "DBG_CLUSTERING>" << std::endl;
		}

		// derive TSV islands from clusters and store into global TSV container;
		// they will will be handled and plotted in the TSV-density maps
		//
		// also link TSVs (blocks) to the respective nets; this is required for
		// more accurate wirelength estimation
		//
		for (it_cluster = this->clusters[i].begin(); it_cluster != this->clusters[i].end(); ++it_cluster) {

			TSV_Island TSVi = TSV_Island(
					// cluster id
					"net_cluster_" + std::to_string((*it_cluster).nets.size()),
					// signal / TSV count
					(*it_cluster).nets.size(),
					// TSV pitch; required for proper scaling
					// of TSV island
					TSV_pitch,
					// cluster bb; reference point for
					// placement of TSV island;
					//
					// note that proper sizing is done via
					// TSV_Island() constructor!
					(*it_cluster).bb,
					// layer assignment
					i
				);

			// perform greedy shifting in case new island overlaps with any
			// previous one
			//
			TSV_Island::greedyShifting(TSVi, TSVs);

			// store in global TSVs container
			TSVs.push_back(TSVi);

			// link TSV block to each associated net
			//
			// since the container "TSVs" will be reallocated subsequently, we
			// cannot use pointers to its elements; for simplicity (to avoid
			// later on traversal and search of matching TSV islands for nets)
			// we simply copy TSVs islands into nets
			//
			for (it_net = (*it_cluster).nets.begin(); it_net != (*it_cluster).nets.end(); ++it_net) {
				(*it_net)->TSVs.push_back(TSVi);
			}
		}
	}

	if (Clustering::DBG) {
		std::cout << "<- Clustering::clusterSignalTSVs" << std::endl;
	}
}

/// Obtain hotspots (i.e., locally connected regions surrounding local maximum
/// temperatures) from the thermal analysis run. The determination of hotspots/blobs is
/// based on Lindeberg's grey-level blob detection algorithm.
void Clustering::determineHotspots(ThermalAnalyzer::ThermalAnalysisResult &thermal_analysis) {
	unsigned x, y;
	std::vector<ThermalAnalyzer::ThermalMapBin*> thermal_map;
	std::vector<ThermalAnalyzer::ThermalMapBin*>::iterator it1;
	std::list<ThermalAnalyzer::ThermalMapBin*> relev_neighbors;
	std::list<ThermalAnalyzer::ThermalMapBin*>::iterator it2;
	ThermalAnalyzer::ThermalMapBin *cur_bin;
	unsigned hotspot_id;
	std::unordered_map<unsigned, Hotspot> hotspots;
	std::unordered_map<unsigned, Hotspot>::iterator it4;
	Hotspot *cur_hotspot;
	bool bin_handled;

	// sanity check for available thermal-analysis result; note that these results are
	// for example _not_ available during the very first run of SA Phase II where
	// interconnects (and thus this function) are evaluated before the thermal profile
	if (thermal_analysis.thermal_map == nullptr) {
		return;
	}

	// reset hotspot regions
	this->hotspots.clear();

	// reset hotspot associations in the thermal map
	for (x = 0; x < ThermalAnalyzer::THERMAL_MAP_DIM; x++) {
		for (y = 0; y < ThermalAnalyzer::THERMAL_MAP_DIM; y++) {
			(*thermal_analysis.thermal_map)[x][y].hotspot_id = ThermalAnalyzer::HOTSPOT_UNDEFINED;
		}
	}

	// parse the thermal grid into an list (to be sorted below); data structure for
	// blob detection
	for (x = 0; x < ThermalAnalyzer::THERMAL_MAP_DIM; x++) {
		for (y = 0; y < ThermalAnalyzer::THERMAL_MAP_DIM; y++) {

			// ignore bins w/ temperature values near the offset
			if (Math::looseDoubleComp(thermal_analysis.temp_offset, (*thermal_analysis.thermal_map)[x][y].temp)) {
				continue;
			}

			thermal_map.push_back(&(*thermal_analysis.thermal_map)[x][y]);
		}
	}

	// sort list by temperature values
	std::sort(thermal_map.begin(), thermal_map.end(),
		// lambda expression
		[](ThermalAnalyzer::ThermalMapBin const* b1, ThermalAnalyzer::ThermalMapBin const* b2) {
			return
			// std::sort requires a _strict_ ordering, thus we have to make sure that same elements returns false
			// http://stackoverflow.com/a/1541909
			// also helps to make comparison short-cutting it early
			(b1 != b2) &&
			(b1->temp > b2->temp);
		}
	);

	if (Clustering::DBG_HOTSPOT) {
		std::cout << "DBG_HOTSPOT> bin w/ global max temperature [x][y]: " << thermal_map.front()->x << ", " << thermal_map.front()->y << std::endl;
		std::cout << "DBG_HOTSPOT>  temp: " << thermal_map.front()->temp << std::endl;
		for (it2 = thermal_map.front()->neighbors.begin(); it2 != thermal_map.front()->neighbors.end(); ++it2) {
			std::cout << "DBG_HOTSPOT>  neighbor bin [x][y]: " << (*it2)->x << ", " << (*it2)->y << std::endl;
		}
	}

	// group the thermal-map list into hotspot regions; perform actual blob detection
	hotspot_id = ThermalAnalyzer::HOTSPOT_FIRST_ID;
	for (it1 = thermal_map.begin(); it1 != thermal_map.end(); ++it1) {

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
			hotspots.insert( std::pair<unsigned, Hotspot>(
					// id is the (temporary) key for the map, used for
					// easier data access during blob detection
					hotspot_id,
					// actual hotspot initialization
					{
						// peak temp
						cur_bin->temp,
						// base-level temp; currently undefined
						-1.0,
						// temperature gradient; currently
						// undefined
						-1.0,
						// allocate vector of associated bins;
						// initialize with cur_bin as first bin of
						// new hotspot
						std::vector<ThermalAnalyzer::ThermalMapBin*>(1, cur_bin),
						// memorize hotspot as still growing
						true,
						// id
						hotspot_id,
						// score; currently undefined
						-1.0,
						// enclosing bb; initialize with cur_bin
						cur_bin->bb
						})
				);

			// mark bin as associated to this new hotspot
			cur_bin->hotspot_id = hotspot_id;

			// increment hotspot counter/id
			hotspot_id++;
		}

		// some neighbor bins w/ higher temperatures exit
		else {
			bin_handled = false;

			// if any of these neighbors is a background bin, then this bin is
			// also a background bin
			for (it2 = relev_neighbors.begin(); it2 != relev_neighbors.end(); ++it2) {

				if ((*it2)->hotspot_id == ThermalAnalyzer::HOTSPOT_BACKGROUND) {

					cur_bin->hotspot_id = ThermalAnalyzer::HOTSPOT_BACKGROUND;
					bin_handled = true;

					break;
				}
			}

			// none of the neighbors are background bins; proceed with check
			// if the neighbors belong to one or to different hotspots
			if (!bin_handled) {

				if (Clustering::DBG_HOTSPOT) {

					if (relev_neighbors.empty()) {
						std::cout << "DBG_HOTSPOT> blob-detection error; no valid neighbor bin found" << std::endl;
					}
					else {
						for (it2 = relev_neighbors.begin(); it2 != relev_neighbors.end(); ++it2) {

							if ((*it2)->hotspot_id == ThermalAnalyzer::HOTSPOT_UNDEFINED) {

								std::cout << "DBG_HOTSPOT> blob-detection error; undefined bin triggered" << std::endl;
								continue;
							}
						}
					}
				}

				// memorize only unique bins; initially sort the neighbors
				relev_neighbors.sort(
						// lambda expression
						[](ThermalAnalyzer::ThermalMapBin const* b1, ThermalAnalyzer::ThermalMapBin const* b2) {

							return
							// std::sort requires a _strict_ ordering, thus we have to make sure that same elements returns false
							// http://stackoverflow.com/a/1541909
							// also helps to make comparison short-cutting it early
							(b1 != b2) &&
							(b1->hotspot_id < b2->hotspot_id);
						}
					);
				// now, remove consecutive duplicates
				relev_neighbors.unique();

				// all neighbors belong to one specific hotspot
				if (relev_neighbors.size() == 1) {

					cur_hotspot = &hotspots.find(relev_neighbors.front()->hotspot_id)->second;

					// if the hotspot is allowed to grow, associated
					// this bin with it, and mark bin as well
					if (cur_hotspot->still_growing) {

						cur_hotspot->bins.push_back(cur_bin);
						cur_bin->hotspot_id = cur_hotspot->id;
					}
					// if the hotspot is not allowed to grow anymore,
					// mark the bin as background bin
					else {
						cur_bin->hotspot_id = ThermalAnalyzer::HOTSPOT_BACKGROUND;
					}
				}
				// neighbors belong to different hotspots
				else {
					// the bin has to be background since it defines
					// the base level for different hotspots
					cur_bin->hotspot_id = ThermalAnalyzer::HOTSPOT_BACKGROUND;

					// the different hotspots have reached their base
					// level w/ this bin; mark them as not growing
					// anymore and memorize the base-level temp
					for (it2 = relev_neighbors.begin(); it2 != relev_neighbors.end(); ++it2) {

						hotspots.find((*it2)->hotspot_id)->second.still_growing = false;
						hotspots.find((*it2)->hotspot_id)->second.base_temp = cur_bin->temp;

						// the determination of temp gradient and
						// score could be also conducted here, but
						// is postponed since a post-processing of
						// all hotspot regions is required anyway
					}
				}
			}
		}
	}

	// post-processing hotspot regions, also re-order them according to their score
	for (it4 = hotspots.begin(); it4 != hotspots.end(); ++it4) {

		cur_hotspot = &(*it4).second;

		// some regions may be still marked as growing; mark such regions as not
		// growing anymore
		if (cur_hotspot->still_growing) {

			cur_hotspot->still_growing = false;

			// also approximate base temp, using the minimal temperature of
			// all bins of the hotspot; note that the actual base temp is
			// slightly lower since the base-level bin is not included in the
			// hotspot itself
			cur_hotspot->base_temp = (*cur_hotspot->bins.begin())->temp;
			for (it1 = cur_hotspot->bins.begin(); it1 != cur_hotspot->bins.end(); ++it1) {

				cur_hotspot->base_temp = std::min(cur_hotspot->base_temp, (*it1)->temp);
			}
		}

		// using the base temp, determine gradient
		cur_hotspot->temp_gradient = cur_hotspot->peak_temp - cur_hotspot->base_temp;

		// determine hotspot score; the score is defined by its peak temp and temp
		// gradient, i.e., measures how ``critical'' the local thermal maxima is
		cur_hotspot->score = cur_hotspot->temp_gradient * std::pow(cur_hotspot->peak_temp, 2.0);
		// apply normalization such that scores are roughly in the range of
		// [0..10]
		cur_hotspot->score /= Clustering::SCORE_NORMALIZATION;

		// determine the (all bins enclosing) bb; this is used to simplify checks
		// of nets overlapping hotspot regions, but also reduces spatial accuracy
		for (it1 = cur_hotspot->bins.begin(); it1 != cur_hotspot->bins.end(); ++it1) {

			cur_hotspot->bb = Rect::determBoundingBox(cur_hotspot->bb, (*it1)->bb);
		}

		// enlarge final bb by 2x, which should increase chances for the
		// subsequent clustering to match net bounding box with these cluster bbs
		cur_hotspot->bb.ll.x -= (cur_hotspot->bb.w / 2.0);
		cur_hotspot->bb.ur.x += (cur_hotspot->bb.w / 2.0);
		cur_hotspot->bb.ll.y -= (cur_hotspot->bb.h / 2.0);
		cur_hotspot->bb.ur.y += (cur_hotspot->bb.h / 2.0);
		cur_hotspot->bb.w = cur_hotspot->bb.ur.x - cur_hotspot->bb.ll.x;
		cur_hotspot->bb.h = cur_hotspot->bb.ur.y - cur_hotspot->bb.ll.y;
		cur_hotspot->bb.area = cur_hotspot->bb.w * cur_hotspot->bb.h;

		// put hotspot into  global map, which shall be sorted by the hotspot scores
		this->hotspots.push_back(std::move(*cur_hotspot));
	}

	// finally, sort global map according to hotspot scores
	//
	std::sort(this->hotspots.begin(), this->hotspots.end(),
			// lambda expression
			[](Hotspot const& hs1, Hotspot const& hs2) {

				// std::sort requires a _strict_ ordering, thus we have to make sure that same elements returns false
				// http://stackoverflow.com/a/1541909
				// also helps to make comparison short-cutting it early
				return (hs1.id != hs2.id) && (
						// the higher the cost the better; sort in desceding order
						(hs1.score > hs2.score)
					);
			}
		 );

	if (Clustering::DBG_HOTSPOT) {
		int bins_hotspot = 0;
		int bins_background = 0;
		int bins_undefined = 0;

		std::cout << "DBG_HOTSPOT> hotspots :" << std::endl;

		for (Hotspot const& cur_hotspot : this->hotspots) {
			std::cout << "DBG_HOTSPOT>  id: " << cur_hotspot.id << std::endl;
			std::cout << "DBG_HOTSPOT>   bb: (" << cur_hotspot.bb.ll.x << "," << cur_hotspot.bb.ll.y;
				std::cout <<  "),(" << cur_hotspot.bb.ur.x << "," << cur_hotspot.bb.ur.y << ")" << std::endl;
			std::cout << "DBG_HOTSPOT>   peak temp: " << cur_hotspot.peak_temp << std::endl;
			std::cout << "DBG_HOTSPOT>   base temp: " << cur_hotspot.base_temp << std::endl;
			std::cout << "DBG_HOTSPOT>   temp gradient: " << cur_hotspot.temp_gradient << std::endl;
			std::cout << "DBG_HOTSPOT>   score: " << cur_hotspot.score << std::endl;
			std::cout << "DBG_HOTSPOT>   bins count: " << cur_hotspot.bins.size() << std::endl;
			std::cout << "DBG_HOTSPOT>   still growing: " << cur_hotspot.still_growing << std::endl;
		}

		std::cout << "DBG_HOTSPOT> adapted thermal-map:" << std::endl;

		for (x = 0; x < ThermalAnalyzer::THERMAL_MAP_DIM; x++) {
			for (y = 0; y < ThermalAnalyzer::THERMAL_MAP_DIM; y++) {

				cur_bin = &(*thermal_analysis.thermal_map)[x][y];

				if (cur_bin->hotspot_id == ThermalAnalyzer::HOTSPOT_BACKGROUND) {
					bins_background++;
				}
				else if (cur_bin->hotspot_id == ThermalAnalyzer::HOTSPOT_UNDEFINED) {
					bins_undefined++;
				}
				else {
					bins_hotspot++;
				}
			}
		}

		std::cout << "DBG_HOTSPOT>  bins w/ hotspot assigned: " << bins_hotspot << std::endl;
		std::cout << "DBG_HOTSPOT>  background bins: " << bins_background << std::endl;
		std::cout << "DBG_HOTSPOT>  undefined bins: " << bins_undefined << std::endl;
	}
}
