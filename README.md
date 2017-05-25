# Corblivar			{#mainpage}

Corblivar is a simulated-annealing-based floorplanning suite for 3D ICs, with special emphasis on planning of large-scale interconnects, timing-driven voltage assignment, and
analysis and mitigation of thermal side-channel leakage.

## Licence
Copyright (C) 2013-2017 Johann Knechtel, johann aett jknechtel dot de

https://github.com/jknechtel/Corblivar

This file is part of Corblivar.

Corblivar is free software: you can redistribute it and/or modify it under the terms
of the GNU General Public License as published by the Free Software Foundation,
either version 3 of the License, or (at your option) any later version.

Corblivar is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
Corblivar.  If not, see <http://www.gnu.org/licenses/>.

## Citation
If you find this tool useful, and apply it for your research and publications, please
cite the following:
- In general: https://github.com/IFTE-EDA/Corblivar
- Related to thermal side-channel leakage: J. Knechtel, O. Sinanoglu, "On Mitigation of Side-Channel Attacks in 3D ICs: Decorrelating Thermal Patterns from Power and Activity", in Proc. Design Automation Conference, 2017, DOI: http://dx.doi.org/10.1145/3061639.3062293 (PDF: https://wp.nyu.edu/johann/knechtel17_tsc_dac/)
- Related to planning of large-scale interconnects, fast thermal management, and 3D floorplanning in general: J. Knechtel, E. F. Y. Young, J. Lienig, "Planning Massive Interconnects in 3D Chips", in IEEE Transactions on Computer-Aided Design of Integrated Circuits and Systems, 34(11):1808-1821, 2015, DOI: http://dx.doi.org/10.1109/TCAD.2015.2432141 (PDF: http://www.ifte.de/mitarbeiter/lienig/TCAD_Nov2015_pp1808_1821.pdf)
- Related to planning of large-scale interconnects, fast thermal management, and 3D floorplanning in general: J. Knechtel, E. F. Y. Young, J. Lienig, "Structural Planning of 3D-IC Interconnects by Block Alignment", in Proc. Asia South Pacific Design Automation Conference, pp. 53-60, 2014, DOI: http://dx.doi.org/10.1109/ASPDAC.2014.6742866 (PDF: http://www.ifte.de/mitarbeiter/lienig/aspdac2014_pp53_60.pdf)

## Compile & Run
**To compile and run Corblivar, you need the following**
- C++ compiler (clang++ was used; for clang++ at least version 3.1 is required)
- libboost-dev
- gnuplot
- octave
- perl
- cairosvg
- a (slightly) modified copy of the HotSpot thermal analyzer; the code should be provided
along with Corblivar or can be retrieved from https://github.com/jknechtel/HotSpot. Note
that this code has to be compiled separately.  The provided and modified copy is currently
based on HotSpot 6.0, and is compiled and tested with SuperLU 5.2.1
(http://crd-legacy.lbl.gov/~xiaoye/SuperLU/#superlu)

## Usage
**To use Corblivar, the following procedure should be followed**

### 1) Configuration of HotSpot
**see exp/hotspot&ast;.config**

Most relevant are the specs for the heat sink and heat spreader; they should be adapted to
reflect largest chip dimensions under consideration.

Also note that the helper script exp/HotSpot.sh relies on HotSpot being in ~/code/HotSpot;
if your local setup differs, adapt the script accordingly.

### 2) Configuration of Corblivar
**see exp/Corblivar.conf and exp/Technology.conf, or other examples in exp/configs/**

Technology parameters like die dimensions and TSV sizes are configured in
exp/Technology.conf

In exp/Corblivar.conf,
the section "SA -- Layout generation options" can be used to configure the optimization
heuristic. Reasonable values depend on the experiments under consideration.

The sections "SA -- Loop parameters" and "SA -- Temperature schedule parameters" control
the runtime behaviour of the optimization. These values can impact the success rate,
especially for ``complex'' experiments with many block-alignment requests and/or dense
packing. The latter section, however, is considered to be applicable for different
experiments---the adaptive SA-optimization schedule draws the cooling parameters somewhat
robust since local minima in the solution space can be escaped easily by iterative
temperature increases (cooling phase 3).

The section "SA -- Factors for second-phase cost function" controls the various
optimization modules; the related values should be adapted to reflect the desired
cost function. Note that zero values deactivate the respective optimization
completely and thus allows to save runtime.

The section "Power blurring (thermal analysis) -- Default thermal-mask parameters" can be
left as is; the related parametrization is done via separate scripts, as described in the
next step.

### 3) Parametrization of Power-Blurring Thermal Analysis
**see thermal_analysis_octave/ and doc/therma_analysis_octave.pdf**

As indicated in 2), the thermal-mask parameters are determined separately. The related
Octave scripts should be run whenever the 3D-IC setup changes notably, i.e., when the
number of layers, the outline, the heatsink, and/or the (magnitude of) power consumption
of the benchmarks changes.

To configure the Octave scripts, see thermal_analysis_octave/parameters.m Note that
given default parameters should be applicable for most GSRC-benchmarks-based experiments.

To run the Octave scripts, either change directory to thermal_analysis_octave/ and start
scripts from there (octave optimization.m BENCH CORBLIVAR.CONF), or copy the scripts from
thermal_analysis_octave/ to separate working directories; see below and/or exp/run9.sh for
further details.

It's important to note that parallel runs of different of the Octave scripts have to be
avoided; they will result in runtime errors and undermine parametrization!

The Octave scripts work roughly like this: first, generate an initial floorplan solution,
used as a baseline reference; second, run HotSpot on this solution (note that specific
values for heterogeneous TSV densities are already considered here); third, match the
power-blurring temperature map to the HotSpot map via a local search; fourth, output the
related power-blurring parameters for the best match, which describes the HotSpot estimate
most closely. For further details, see documentation_Octave.pdf.

Note that Corblivar models the thermal impact of both regular signal TSVs and vertical
buses, i.e., large TSV groups. Regular signal TSVs may be clustered into vertical buses as
well, when the layout-generation option "Clustering of signal TSVs" is activated.
Vertical buses are assumed to have tightest possible packing of multiple TSVs (100% TSV
density) for the whole bus region, even if fewer TSVs would suffice for signal
transmission.

### 4) Running Corblivar
**see exp/run&ast;.sh or directly start ./Corblivar**

To run Corblivar, one can start the binary directly, for example from the exp/ folder as

	../Corblivar BENCH CORBLIVAR.CONF benches/

The other option is to call Corblivar in a batch mode, as outlined in the scripts
exp/run&ast;.sh

Note that for generation of plotted data, one has to call the script exp/gp.sh afterwards
in the related working directory.

### 5) Optional: Running Correlation_TSC, Postprocessing_TSC, and Variation_TSC after running Corblivar
**see exp/&ast;TSC.sh or directly start the above binaries after running Corblivar**

A new feature introduced in v1.5.0 regards the analysis and optimization (i.e., reduction) of the thermal side-channel (TSC). This feature serves to dissolve the correlation
between power and thermal maps. It is implemented in Corblivar itself and in the three auxiliary binaries. All auxilarly binaries should be run on a previously obtained solution
which may or may not have been optimized with respect to thermal leakage; see the sections on "thermal-related leakage mitigation" in the experimental configuration files.

Variation_TSC: initialy, this binary reads in a Corblivar solution. Then, it **iteratively samples** all the blocks' power densities as Gaussian distribution, calls HotSpot for
detailed thermal analysis, reads in the results, and computes the Pearson correlation between the power and temperature values over all dies. Finally, the average correlation
values over all sampling iterations are reported.

Postprocessing_TSC: initialy, this binary reads in a Corblivar solution. Then, it **iteratively samples** all the blocks' power densities as Gaussian distribution, calls HotSpot
for detailed thermal analysis, reads in the results, and computes the Pearson correlation between the power and temperature values over all dies. Next, the average correlation
values over all sampling iterations are reported. Based on these results, additional TSVs are inserted in a post-processing fashion, to locally reduce correlation. TSV are inserted
for those locations where the average correlation was the highest (with some threshold) over the previous sampling run. Then, **the sampling process is repeated**. Overall, this
two-fold sampling and post-processing loop is repeated until no further reduction in the average correlation can be achieved.

Correlation_TSC: this binary simply reads in a Corblivar solution and the related HotSpot results, and it calculates the Pearson correlation (over all power and temperature values) and the spatial entropies (over the power maps) for all dies.

The configuration files are currently only provided for 2 dies; see exp/configs/2dies/TSC

The parameters to run the above binaries are the same as with the main Corblivar binary; see exp/&ast;TSC.sh

# Comments
**The further comments elaborate on the folders and scripts of Corblivar**

Various experiments can be started using exp/run&ast;.sh; these scripts are not a complete
set for running all experiments but rather a guideline for different setups.

The folder exp/benches/ includes MCNC (some are not working, i.e., have issues with their
content), GSRC, and IBM-HB+ benchmarks, all in the GSRC format

The folder thermal_analysis_octave/ includes Octave scripts for the parameterization of
the power-blurring-based thermal analysis; they can be also included e.g. in run&ast;.sh
scripts.  Note that these scripts will produce temporary output data in
thermal_analysis_octave/, i.e., you might want to copy thermal_analysis_octave/ into
separate working directories for parallel execution of different experiments, to avoid
mixed up data. See for example exp/run9.sh

The script exp/gp.sh delegates to gnuplot for generating various output plots, e.g.,
thermal map and floorplan, after running Corbilvar.

The script exp/HotSpot.sh calls a (slightly modified) version of BU's 3D HotSpot program; the
related code should be provided along with Corblivar. Note that the script contains a path
to the HotSpot binary which has to be adapted for your local setup.

The script exp/extract_numeric_results.sh and the spreadsheet exp/evaluate_via_charts.ods
can be used to evaluate experimental batches, generated via some run script in exp/run&ast;.sh

The file exp/Corblivar.conf is a template for the config file required by Corblivar,
further examples can be found in exp/configs/.

# Changelog

## 1.6.0
*May 2017: new feature: system-level static timing analysis; various updates and bugfixes*
- TODO

## 1.5.1
*May 2017: new cost critiera: level shifter; updates config/experimental files for "regular", "VA_&ast;" and "voltage\_assignment"*

## 1.5.0
*April 2017: new feature: analysis and mitigation of thermal side-channel (TSC); further updates and fixes*
- new feature described in more details above, in Sec 5)
- added related three new binaries (src_aux): Correlation_TSC, Postprocessing_TSC, and Variation_TSC
- various updates and fixes related to voltage assignment
- minor updates logging and plotting

## 1.4.4
*November 2016: updates related to HotSpot to 6.0; various other updates and fixes*
- experimental config files: added hotspot&ast;.config to exp/; previously kept separately in
HotSpot directory/repository
- update experimental setup (HotSpot): consider now also secondary path, only possible
				       thanks to updated HotSpot 6.0
- update HotSpot data generation: nets' power consumption is now also modeled, by
dummy blocks covering all nets' bounding boxes in each respective BEOL layers, with evenly
				       distributed sum of nets' power
- update power maps: also plot original (unpadded, unscaled) power density map
- update TSV handling: add dummy TSVs in regular grid-like fashion wherever a minimum TSV
				       density (new technology parameter) would otherwise be violated
- fix greedy shifting TSV island: add sanity check for shifting current island itself
- fix scaling of single-TSV islands
- fix calculation of heat capacities for TSV islands
- fix read in of solution file: parse rotated/shaped blocks correctly
- fix merging of adjacent voltage volumes
- fix layout generation: random operation on blocks exceeding outline, independent of SA phase
- various cleanups

## 1.4.3
*July 2016: updates and fixes; added Doxygen documentation*
- benchmarks: added parser and some functions for GATech-style benchmarks
- benchmarks and experimental scripts: updates for IBM-HB+ benchmarks
- config files: added parameters for adaptive die shrinking and trivial HPWL
- layout evaluation: updates/fixes for interconnects, overall dies and voltage assignment

## 1.4.2
*November 2015: updates and fixes for layout operations / floorplanner and voltage assignment; further general updates/fixes*
- benchmarks: GSRC benchmarks now also available with soft blocks, e.g., n100_soft; added
power densities for (randomly) selected IBM-HB+ benchmarks
- layout operations / floorplanner: fix thermal-aware block moving, to avoid excessive deadspace; consider blocks with
largest net preferably for minimizing timing and WL; consider blocks which exceed outline
preferably to improve chances for fixed-outline-fitting solutions; fix area-outline cost;
shrink die outlines whenever possible, which improves deadspace and runtime
- voltage assignment: reduce timing threshold/constraint whenever a better solution is
found; this way, notable runtime can be saved since voltage assignment is only conducted
in case timing is not violated; quality is also improved this way; also, ignore compound
modules spreading more than two dies, to limit solution space
- power evaluation: dynamic power consumption of interconnects, i.e., TSVs and wires, is
captured now as well
- HotSpot thermal data is now prepared such that it will be plotted directly from HotSpot
data using gnuplot, not via (buggy) perl script from HotSpot
- clustering: size of thermal hotspots not considered anymore; this was misleading in case
very small but very hot hotspots are found along with very large but very ``cool''
hotspots
- updates/fixes greedy shifting of TSV islands
- fix read-in of Corblivar solutions; dimensions of soft blocks were previously ignored
- div updates/fixes for improving performance and refactoring classes
- updates config scripts and helper scripts

## 1.4.1
*August 2015: major updates and fixes for voltage assignment, mainly related to memory/runtime efforts*
- voltage assignment: consider only one best-cost candidate, notably reduces memory
consumption but maintains proper solution-space exploration
- voltage assignment: perform only for solutions without delay violations, reduces runtime
notably
- voltage assignment: drop non-essential copy operations
- introduce numerical block id, along the previous string id; notably reduces
computational efforts for voltage assignment
- iterative updates of power saving and corners for compound modules; improves
computational effort for voltage assignment notably
- updates cost handling for floorplanner
- fix memory leakage related to TSV islands
- updates technology and Corblviar config files for different experiments
- div refactoring and cleanups

## 1.4.0
*July 2015: new feature: delay-aware voltage assignment, minor other updates and fixes*
- added determination of delays, using Elmore delay for net and TSV delay, and module
delays based on voltage assignment
- added voltage-assignment handler: determines possible arrangements of voltage domains and
selects the best-cost solutions
- added related contiguity analysis for modules/blocks
- related update floorplanning flow
- related update Technology.conf and Corblivar.conf; added new experiments and config scripts
- fix memory leakage related to greedy shifting of TSV islands
- div refactoring and cleanups

## 1.3.1
*May 2015: minor updates and fixes*
- fixes related to handling alignment with RBOD, i.e., alignments for pre-fixed blocks
- updates for TSV handling; put single TSVs (in net's bounding boxes' center) in case
clustering is not applied

## 1.3.0
*May 2015: considerable updates and fixes (interconnects handling, clustering, layout operations (HotSpot data, etc), new feature: routing-congestion estimation*
- added estimation of routing congestion; considering all wires, also connecting to TSVs
- various fixes regarding clustering and related interconnects handling
- improved accuracy for determination of interconnects; for TSVs, regular wires and
massive buses
- improved layout packing; added dedicated handling for outer blocks
- improved handling TSV islands, greedy shifting to avoid overlaps
- various fixes regarding griding of HotSpot input-data; HotSpot errors due to conversion
errors are now settled
- updates and fixes handling of strictly aligned massive interconnects
- updates and fixes for handling for intermediately failed alignments
- updates and fixes layout operations
- various updates and cleanups for logging
- various updates and cleanups for benchmarks and experimental setups

## 1.2.0
*October 13, 2014: major updates (alignment encoding and handling, consideration of interconnects' HPWL); new features (clustering of signal TSVs and related hotspot determination), and various fixes and cleanups*
- added handling for TSV blocks
- added feature; clustering of signal TSVs into vertical buses, related features like blob-detection-based hotspot determination and handling of vertical buses during thermal analysis
- update alignment encoding; added global type (strict, flexible) and signals count
- update alignment handling; added dedicated swap operations for failed alignments
- update massive-interconnects handling; consider interconnects' weighted HPWL during optimization (for fair comparison and better final results)
- updates folder structure
- added Technology.conf; separate file for technology parameters
- updates and cleanups classes, code structure, and namespace
- updates experimental setups and configs
- updates and fixes octave scripts for thermal analysis
- updates and fixes HotSpot file generation
- various further fixes and updates

## 1.1.1
*May 7, 2014: updates, consideration of heterogeneous TSV densities*
- dropped deprecated handling of different masks
- dropped dummy TSV handling
- added handler for TSV densities considering both signal TSVs and vertical buses
- Octave script now considers parameter for scaling down power in TSV regions
- various minor updates and fixes

## 1.1.0
*Nov 13, 2013: new feature, consideration of heterogeneous TSV densities*
- adapted power blurring for using different masks
- added plotting of TSV-density maps
- adapted HotSpot file handler
- added dummy TSV handler
- Octave script now considers determination of different masks
- various minor updates and fixes

## 1.0.4
*Aug 21, 2013: fixes and updates, thermal analysis*

## 1.0.3
*Aug 1, 2013: fix, compiling error for 64-bit libaries*

## 1.0.2
*Jul 29, 2013: update, enable fixed-position block alignment*

## 1.0.1
*Jul 29, 2013: bugfixes and updates*
- update HotSpot BU to v 1.2
- fixes calculation of thermal-related material properties
- new class Chip contains all chip-related settings

## 1.0.0
*Jul 22, 2013: initial public release*
