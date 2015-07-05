Licence
=======

Copyright (C) 2015 Johann Knechtel, johann.knechtel@ifte.de, www.ifte.de

https://github.com/jknechtel/Corblivar

This file is part of Corblivar.

Corblivar is a simulated-annealing-based floorplanning suite for 3D ICs, with special
emphasis on structural planning of massive interconnects by block alignment.

Corblivar is free software: you can redistribute it and/or modify it under the terms
of the GNU General Public License as published by the Free Software Foundation,
either version 3 of the License, or (at your option) any later version.

Corblivar is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
Corblivar.  If not, see <http://www.gnu.org/licenses/>.

Citation
========
If you find this tool useful, and apply it for your research and publications, please
cite our papers:
- Knechtel, J.; Young, E. F. Y. & Lienig, J. "Planning Massive Interconnects in 3D Chips" IEEE Trans. Comput.-Aided Des. Integr. Circuits Sys., 2015, DOI: http://dx.doi.org/10.1109/TCAD.2015.2432141
- Knechtel, J.; Young, E. F. Y. & Lienig, J. "Structural Planning of 3D-IC Interconnects by Block Alignment" Proc. Asia South Pacific Des. Autom. Conf., pp. 53-60, 2014

Compile & Run
=============
**To compile and run Corblivar, you need the following tool**
- C++ compiler (clang++ was used; for clang++ at least version 3.1 is required)
- libboost
- gnuplot
- octave
- perl
- cairosvg
- a modified copy of the BU's HotSpot 3D-IC thermal analyzer; the code should be provided
along with Corblivar or can be retrieved from https://github.com/jknechtel/HotSpot. Note
that this code has to be compiled separately.

Usage
=====
**To use Corblivar, the following procedure should be followed**

1) Configuration of HotSpot
---------------------------
**see ../HotSpot/hotspot.config**

Relevant are the specs for the heat sink and heat spreader; they should be adapted to
reflect largest chip dimensions under consideration.

Also note that the helper script exp/HotSpot.sh relies on HotSpot being in ~/code/HotSpot;
if your local setup differs, adapt the script accordingly.

2) Configuration of Corblivar
-----------------------------
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

3) Parametrization of Power-Blurring Thermal Analysis
-----------------------------------------------------
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

4) Running Corblivar
--------------------
**see exp/run*.sh or directly start ./Corblivar**

To run Corblivar, one can start the binary directly, for example from the exp/ folder as

	../Corblivar BENCH CORBLIVAR.CONF benches/

The other option is to call Corblivar in a batch mode, as outlined in the scripts
exp/run*.sh

Note that for generation of plotted data, one has to call the script exp/gp.sh afterwards
in the related working directory.

Comments
========
**The further comments below are for better understanding of the Corblivar tool and its structure**

Various experiments can be started using exp/run*.sh; these scripts are not a complete
set for running all experiments but rather a guideline for different setups.

The folder exp/benches/ includes MCNC (some are not working, i.e., have issues with their
content), GSRC, and IBM-HB+ benchmarks, all in the GSRC format

The folder thermal_analysis_octave/ includes Octave scripts for the parameterization of
the power-blurring-based thermal analysis; they can be also included e.g. in run*.sh
scripts.  Note that these scripts will produce temporary output data in
thermal_analysis_octave/, i.e., you might want to copy thermal_analysis_octave/ into
separate working directories for parallel execution of different experiments, to avoid
mixed up data. See for example exp/run9.sh

The script exp/gp.sh delegates to gnuplot for generating various output plots, e.g.,
thermal map and floorplan, after running Corbilvar.

The script HotSpot.sh calls a (slightly modified) version of BU's 3D HotSpot program; the
related code should be provided along with Corblivar.

The file exp/Corblivar.conf is a template for the config file required by Corblivar,
further examples can be found in exp/configs/.

Settings in some scripts may rely on the folder ~/code ; putting ~/code/Corblivar along
with ~/code/HotSpot is the default setup which should be working in any case.

Changelog
=========

1.3.1
-----
*May 2015, commit f770ba1d2e56f9df6f3f0dab0d9e2b020e111928*
**minor updates and fixes**
- fixes related to handling aligmnet with RBOD, i.e., alignments for pre-fixed blocks
- updates for TSV handling; put single TSVs (in net's bounding boxes' center) in case
clustering is not applied

1.3.0
-----
*May 2015, commit 10098b4fe61b8f23e8d777c1039ef75b9523e024*
**considerable updates and fixes (interconnects handling, clustering, layout operations,
HotSpot data, etc), new feature: routing-congestion estimation**
- added estimation of routing congestion; considering all wires, also connecting to TSVs
- various fixes regarding clustering and related interconnects handling
- improved accuracy for determination of interconnects; for TSVs, regular wires and
massive busses
- improved layout packing; added dedicated handling for outer blocks
- improved handling TSV islands, greedy shifting to avoid overlaps
- various fixes regarding gridding of HotSpot input-data; HotSpot errors due to conversion
errors are now settled
- updates and fixes handling of strictly aligned massive interconnects
- updates and fixes for handling for intermediately failed alignmentsA
- updates and fixes layout operations
- various updates and cleanups for logging
- various updates and cleanups for benchmarks and experimental setups

1.2.0
-----
*October 13, 2014, commit 3f770ef4302729f6aa253ecdb65d2ffbb6c2ffbb*
**major updates (alignment encoding and handling, consideration of interconnects' HPWL),
new features (clustering of signal TSVs and related hotspot determination), and various fixes and cleanups**
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

1.1.1
-----
*May 7, 2014, commit bacb85a62a4b779cb94286ee3a1e946c19e234a1*
**updates, consideration of heterogeneous TSV densities**
- dropped deprecated handling of different masks
- dropped dummy TSV handling
- added handler for TSV densities considering both signal TSVs and vertical buses
- Octave script now considers parameter for scaling down power in TSV regions
- various minor updates and fixes

1.1.0
-----
*Nov 13, 2013, commit 1de2426d361595bbec4542e425c8eb74fecaf544*
**new feature, consideration of heterogeneous TSV densities**
- adapted power blurring for using different masks
- added plotting of TSV-density maps
- adapted HotSpot file handler
- added dummy TSV handler
- Octave script now considers determination of different masks
- various minor updates and fixes

1.0.4
-----
*Aug 21, 2013, commit 44f573ed8f31654e6c07d6cc391cf528233f30bf*
**fixes and updates, thermal analysis**

1.0.3
-----
*Aug 1, 2013, commit 157d3c989ac20799d0e4efce8acf6b244a68a480*
**fix, compiling error for 64-bit libaries**

1.0.2
-----
*Jul 29, 2013, commit 80029340de6e0d9fc97c705828671cbc4a26057c*
**update, enable fixed-position block alignment**

1.0.1
-----
*Jul 29, 2013, commit f583c3b77b4c67a7b11aacc117f0d436b5408d84*
**bugfixes and updates**
- update HotSpot BU to v 1.2
- fixes calculation of thermal-related material properties
- new class Chip contains all chip-related settings

1.0.0
-----
*Jul 22, 2013, commit 286b7917b05be13d3cbcdb4b837422baa00888ad*
**initial public release**
