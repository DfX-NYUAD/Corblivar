
%% Description:  Octave program for optimizing the Error between the thermal analysis of HotSpot and Corblivar via parameter variation 

%% Copyright (C) 2013 Timm Amstein, timm.amstein@gmail.com, Johann Knechtel, johann.knechtel@ifte.de, www.ifte.

%% This file is part of Corblivar.

%%    Corblivar is free software: you can redistribute it and/or modify
%%    it under the terms of the GNU General Public License as published by the Free
%%    Software Foundation, either version 3 of the License, or (at your option) any later
%%    version.
%%    
%%    Corblivar is distributed in the hope that it will be useful,
%%    but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
%%    FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
%%    details.
%%    
%%    You should have received a copy of the GNU General
%%    Public License along with Corblivar.  If not, see <http://www.gnu.org/licenses/>.


%%% To execute this program you need the package "octave" which includes the gnu octave language for numerical computing
  
%% For execution use terminal or Octave GUI like "qtoctave" and type:		octave optimization.m  
% After that Octave should initialize and start the program instantaneously


%%% Start optimization
%% clear octave core and storage
	
clear; clc;
   
% read command line parameters 

args = argv();

% print instructions

helpConf = sprintf('\n\nPlease make sure that you edited the "Corblivar.conf" file for your chip before you perform the optimization!!\n\nAlso, the first two command-line parameters are required and the following two are optional; required: 1) the benchmarks name, 2) the config file path; optional: 3) the (non-standard) directory containing the Corblivar binary, 4) the (homogeneous) TSV density to be assumed\n\n');
	
printf(helpConf); 

% define benchmark from command line

bench = args{1}

% read path for config file from command line

conf.path = args{2};

% define directories

[dir,conf] = directories(args,conf);

% read TSV density from command line

if size(args,1) == 4

	conf.TSV_density = args{4};

else

	conf.TSV_density = '0';

end

%% use parameters function to import all general initialization parameters 

[p] = parameters();

% create folder for saving optimization history
	
savefold = sprintf('%s/%s_TSV_dens_%s__thermal_analysis_fitting', conf.dir, bench, conf.TSV_density)
savefold_rm = sprintf('rm -rf %s',savefold);

system(savefold_rm);
mkdir(savefold);

%% start timers for measuring the elapsed time for the whole process 

tic

%% read and copy "Corblivar.conf" file into octave text format

[conf, p] = confText(conf, p);

%% read "Technology.conf" file; parse layers

[conf, p] = techText(conf, p);
p.opt.layers.value

% use config function to write new "Corblivar.conf" file with initial values

config(p,conf);

%%% execute initial Corblivar floorplanning

% path for executing Corblivar without a solution file

pathCbl = sprintf('%s/Corblivar %s %s %s/benches/',dir.bin, bench, conf.path ,dir.exp);		% %s will be replaced by the parameter standing behind the string

% execute only when no solution file exists
solution_file = sprintf('%s.solution', bench);

if exist(solution_file, 'file') == 0

	system (pathCbl);										% system function uses terminal and bash notation

end
 
%% execute Corblivar with given floorplan and initial parameters
% path for execution of Corblivar with given solution file 
	
pathCblsol = sprintf('%s/Corblivar %s %s %s/benches/ %s.solution',
		dir.bin, bench, conf.path, dir.exp, bench);

pathCblsolnt = sprintf('%s/Corblivar %s %s %s/benches/ %s.solution %s >/dev/null',
		dir.bin, bench, conf.path, dir.exp, bench, conf.TSV_density);
	
% path for executing 3D-HotSpot

pathHS = sprintf('%s/HotSpot.sh %s %d',dir.exp, bench, p.opt.layers.value);

%% execute 3D-HotSpot analysis
  
system (pathCblsol);										% system function uses terminal and bash notation
system (pathHS);

%% plot thermal and power maps with given shell script
   	
pathGP = sprintf('%s/gp.sh',dir.exp);
system (pathGP);

%% analyse HS data

[HS] = HotSpotData(bench, p);

%define offset for config file, derived from min temp in HS analysis

p.conf.minHS.value = HS(1).minHS;

%% start the eval function to assess error between HotSpot and Corblivar solution vectors
   
[eval,Cbl] = evalCorb(p,bench,HS);

%% initialize history for sample parameters and errors

hist = [p.conf.I.value p.conf.If.value p.conf.Mb.value p.conf.PDPZ.value p.conf.PDTR.value eval.Error eval.Error_sq eval.value p.opt.accuracy]; 

%% define first parameters as the optimal parameters and start history for optimal parameters and errors

opt.I = p.conf.I.value;
opt.If = p.conf.If.value;
opt.Mb = p.conf.Mb.value;
opt.PDPZ = p.conf.PDPZ.value;
opt.PDTR = p.conf.PDTR.value;
opt.Error = eval.Error;
opt.Error_sq = eval.Error_sq;
opt.Eval = eval.value;

%% safe optimal values 

opthist = [opt.I opt.If opt.Mb opt.PDPZ opt.PDTR opt.Error opt.Error_sq opt.Eval p.opt.accuracy];

 
%%% start for-loop for randomized sampling 
%% generators use normal curve of distribution with optimal parameters as mu and sigma

iter = 0;	% iteration counter (whole loop)
j = 0;		% counter for sigma adjustment (phase 2) 
x = 0;		% iteration counter (phase 2)
ad = 0;		% accuracy determined
dc = 0;		% determination counter

while x < p.opt.iterations

	% impulse factor I

	do 
		p.conf.I.value = opt.I + randn * p.conf.I.sigma; % randomly generated I with mean value optI and variance sigma_I
		
	until	p.conf.I.value > 0				% parameter must be positive
		
	% impulse scaling factor If

	do 
		p.conf.If.value = opt.If + randn * p.conf.If.sigma;

	until p.conf.If.value > 0

	% mask boundary Mb
	
	count = 0;

	do
		p.conf.Mb.value = opt.Mb + randn * p.conf.Mb.sigma;

		count++;

		if count > 100						% if means are too widely apart and sigma was refined it's possible by chance that I becomes so small that Mb can't get smaller

			p.conf.I.value = p.conf.I.value * 1.05;		% forces I to increase

			count = 0;
		endif				

	until (p.conf.Mb.value > 0) && (p.conf.Mb.value < I)		% Mb hast two preconditions: it has to be positive and smaller than I

	% power density scaling factor in padding zone PDPZ

	do
		p.conf.PDPZ.value = opt.PDPZ + randn * p.conf.PDPZ.sigma;

	until	(p.conf.PDPZ.value > 1.0) && (p.conf.PDPZ.value < p.conf.PDPZ.max_val)	% power density < 1 is not reasonable; also consider max value

	% power density scaling factor for TSV Regions PDTR
	
	do 
		p.conf.PDTR.value = opt.PDTR + randn * p.conf.PDTR.sigma;

	until	p.conf.PDTR.value > 0.0 && p.conf.PDTR.value <= 1 %% set max as 1

	%% end of random generation of parameters
	
	%% rewrite new parameters into "Corblivar.conf" file
		
 	config(p,conf);	
	
	%%% execute Corblivar with given floorplan and without returning something to the terminal
	
	system (pathCblsolnt);

 	%% reevaluate the error between HotSpot and Corblivar
	
	[eval,Cbl] = evalCorb(p,bench,HS);

 	%% update history 
	
  	hist = [hist ; p.conf.I.value p.conf.If.value p.conf.Mb.value p.conf.PDPZ.value p.conf.PDTR.value eval.Error eval.Error_sq eval.value p.opt.accuracy]; 

	iter = iter + 1;  % output number of iter of for loop

	
	%%% Decision
	%% decide if Error is smaller than optimal value
	% if yes, update all optimal parameters	

	if ad							% if minimal accuracy is determined (Phase 2)			
		%if (eval.value < opt.Eval) && eval.valid 	% if new evaluation is better than old optimal value and valid then replace it
		%
		% NOTE validity ignored so that any solution better than the initial
		% parameters can be found in case w/ thermal profiles with large gradients
		%
		if (eval.value < opt.Eval)
				
			printf("new optimal solution\n")

			% ! it is necessary to reduce accuracy again to evaluate on first validity range that couldn't be met through random sampling

			do

				p.opt.accuracy = p.opt.accuracy * p.opt.sigma_update;
				
				[eval,Cbl] = evalCorb(p,bench,HS);

			until eval.valid == 0

			
			% update optimal parameters

			opt.Error = eval.Error;
			opt.Error_sq = eval.Error_sq;
			opt.I = p.conf.I.value;
			opt.If = p.conf.If.value;
			opt.Mb = p.conf.Mb.value;
			opt.PDPZ = p.conf.PDPZ.value;
			opt.PDTR = p.conf.PDTR.value;
			opt.Eval = eval.value

			%% update optimal history

			opthist = [opthist; opt.I opt.If opt.Mb opt.PDPZ opt.PDTR opt.Error opt.Error_sq opt.Eval p.opt.accuracy];
		
		endif 
	
		%% use counter to adjust the sigma of the randomizer after predefined stepsize	
		x++;
		Phase_2 = x 						% counter phase 2 (when ad)
	   	j++;							% counter for adjusting sigma
	
		%% update sigmas

		if j == p.opt.step;					% stepsize can be edited in the parameters function

			p.conf.I.sigma = p.conf.I.sigma * p.opt.sigma_update;
			p.conf.If.sigma = p.conf.If.sigma * p.opt.sigma_update;
			p.conf.Mb.sigma = p.conf.Mb.sigma * p.opt.sigma_update;
			p.conf.PDPZ.sigma = p.conf.PDPZ.sigma * p.opt.sigma_update;
			p.conf.PDTR.sigma = p.conf.PDTR.sigma * p.opt.sigma_update;

			% output sigmas
	
			p.conf.I.sigma
			p.conf.If.sigma
			p.conf.Mb.sigma
			p.conf.PDPZ.sigma
			p.conf.PDTR.sigma

			j = 0;						% set back counter
		end

	else							% the level of accuracy isn't defined jet (Phase 1)	
		
		if eval.valid						% valid solution inside range was found

			new_valid_sol = 1;
			printf("new valid solution\n")
				
			% reduce the validity range as much as possible

			do

				p.opt.accuracy = p.opt.accuracy * p.opt.sigma_update;
				p.opt.accuracy
				
				[eval,Cbl] = evalCorb(p,bench,HS);

			until eval.valid == 0

			dc = 1;						% reset the determination counter
				
			% update optimal parameters
 
			opt.Error = eval.Error;
			opt.Error_sq = eval.Error_sq;
			opt.I = p.conf.I.value;
			opt.If = p.conf.If.value;
			opt.Mb = p.conf.Mb.value;
			opt.PDPZ = p.conf.PDPZ.value;
			opt.PDTR = p.conf.PDTR.value;
			opt.Eval = eval.value

			%% update optimal history

			opthist = [opthist; opt.I opt.If opt.Mb opt.PDPZ opt.PDTR opt.Error opt.Error_sq opt.Eval p.opt.accuracy];

		
		else							% no valid solution was found
			dc++;
			Phase_1 = dc				
				
			if dc > p.opt.iterations / 5		% this should suffice for defining an appropriate accuracy
					
				ad = 1;				% set accuracy defined		
				p.opt.accuracy			% print accuracy
			end
		end			
	end
end

%%% end of optimization loop

%%% plotting the results

%% load optmial parameters

p.conf.I.value = opt.I;
p.conf.If.value = opt.If;
p.conf.Mb.value = opt.Mb;
p.conf.PDPZ.value = opt.PDPZ;
p.conf.PDTR.value = opt.PDTR;

%% rewrite "Corblivar.conf" file 

config(p,conf);  	

%% execute Corblivar with given floorplan and optimal parameters

system (pathCblsol);

%% generate the thermal map and power maps

system (pathGP);

%% evaluate the optimal solution

[eval,Cbl] = evalCorb(p,bench,HS);

%% print results as colourmaps 
 
surf(Cbl.thermal) , colorbar;
print('Cbl.pdf','-dpdf');

imagesc(eval.matError) , colorbar;
print('matError.pdf','-dpdf');

imagesc(eval.matrix), colorbar;
print('evalMat.pdf','-dpdf');

imagesc(Cbl.maxPoints) , colorbar;
print('Cbl_maxPoints.pdf','-dpdf');

%% save maximum values and Error in "eval.txt" file

results.txt = sprintf('%s/%s_thermal_analysis_fitting_ranges.txt', savefold,bench);

save results.txt HS Cbl eval;

%% save history of sampling in a reloadable "hist.data" file

helpHist = sprintf('This reloadable(in Octave) matrix is a history of the parameters impulse faktor,impulse-scaling factor, mask boundary and the power-density scaling factor together with the evaluated error');

save (sprintf('%s/%s_thermal_analysis_fitting_hist.data',savefold,bench), "helpHist", "hist");

%% save history of optimal Parameters in a reloadable file

helpOptHist = sprintf('This reloadable(in Octave) matrix is a history of the parameters impulse faktor,impulse-scaling factor, mask boundary and the power-density scaling factor together with the evaluated error');
 
save (sprintf('%s/%s_thermal_analysis_fitting_opt_hist.data',savefold,bench), "helpOptHist", "opthist");

% copy sampling files of Corblivar and HotSpot

file_list = ls;

list_size = size(file_list,1);

solution_file = sprintf('%s.solution',bench);	

for sample = 1:list_size

	if 	strncmp('optimization.m',file_list(sample,:),14) || ...
		strncmp('config.m',file_list(sample,:),8) ||...
		strncmp('confText.m',file_list(sample,:),10) ||...
		strncmp('techText.m',file_list(sample,:),10) ||...
		strncmp('directories.m',file_list(sample,:),13) ||...
		strncmp('evalCorb.m',file_list(sample,:),10) ||...
		strncmp('HotSpotData.m',file_list(sample,:),13) ||...
		strncmp('extrema.m',file_list(sample,:),9) ||...
		strncmp('parameters.m',file_list(sample,:),12)				% leave all optimization files where they are
			
	elseif 	strncmp(solution_file,file_list(sample,:),size(solution_file,2))

		copyfile(file_list(sample,:), savefold,'f');
	else

		copyfile(file_list(sample,:), savefold,'f');		
	endif
end

toc

%% clean working dir

clean = sprintf('%s/clean.sh',dir.exp);		% uses shell script clean.sh in exp-folder

system (clean);

%%% end of program
