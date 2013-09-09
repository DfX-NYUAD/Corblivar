
 %% Description:  Octave program for optimizing the Error between the thermal analysis of HotSpot and Corblivar via parameter variation 

 %% Copyright (C) 2013 Timm Amstein, Johann Knechtel, johann.knechtel@ifte.de, www.ifte.

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
  %% clean experiments folder
	
	clear; clc;
   
   % read command line parameters 

	args = argv();

  %% memorize working directory

   	dir = pwd()
	
  %% memorize experiments folder

	% check if bin folder defined
	
	check = 0;
	
	if size(args,1) == 3
		
		check = isdir(args{3});
		
		if check == 1		
		
			bin = args{3}		
		end
	end

	if size(args,1) == 4
		
		check = isdir(args{4});
		
		if check == 1		
		
			bin = args{4}		
		end
	end

	if size(args,1) == 5
		
		check = isdir(args{5});
		
		if check == 1		
		
			bin = args{5}		
		end
	end

	if check == 0

		ind_bin = rindex(dir, "/");

		bin = dir(1,1:(ind_bin-1))

	end

	exp = sprintf('%s/exp',bin)

  %% clean working dir

	clean = sprintf('%s/clean.sh',exp);

	system (clean);

  %% use parameters function to import all general initialization parameters 

	[sigma_I,sigma_If,sigma_Mb,sigma_PDPZ,I,If,Mb,PDPZ,minHS,step, iter, sigma_update,max_PDPZ, TSV_dens, TSV_dens_step] = parameters();

  %% get imput from user
   % the needed benchmark and the number of iterations must be chosen by the user

   % print recommendation for editing the .conf-file

	helpConf = sprintf('\nThese scripts rely on being in a subfolder of the particular Corblivar experiment to be parameterized!!\n\nPlease make sure that you edited the "Corblivar.conf" file for your chip before you perform the optimization!!\n\nAlso, two command-line parameters are required and 3 are optional: 1) the benchmarks name, 2) the related config-files name, 3) Choose the case for TSV density analysis \n\n for using the defined values of the parameters function use no command or type 0\n use 1 for the optimization without TSV insertion \n use 2 for the optimization with 0 percent and 100 percent TSVs \n use 3 to define specific step size in percent as a 4th command line parameter\n\n for defining a specific binary directory use directory as additonal parameter\n\n');
	
	printf(helpConf); 

   % if ordered, redefine TSV density step size 

	if size(args,1) < 3
		
		X = 0;
	else
		
		X = args{3};
	end

	if X == '0'
	
	elseif X == '1'
			TSV_dens = 0
 	elseif X == '2'
			TSV_dens_step = 100
	elseif X == '3'
			TSV_dens_step = args{4};
			TSV_dens_step = str2num(TSV_dens_step)
	endif

   % define benchmark from command line

	bench = args{1}

  %% open the "Corblivar.conf" file for reading

	%   conf = 'Corblivar.conf';

  % read path for config file from command line

	conf = args{2}

  % save Config directory

	ind_conf = rindex(conf, "/");
	
	if ind_conf == 0

		dir_conf = exp		
		conf = sprintf('%s/%s', exp, conf)
	
	else	

		dir_conf = conf(1,1:ind_conf)
	
	end

  %% create Folder for saving optimization history
	
	savefold = sprintf('%s/%s_thermal_analysis_fitting',dir_conf,bench)
	savefold_rm = sprintf('rm -rf %s',savefold);

	system(savefold_rm);
	mkdir(savefold);

   % initial parameter

	init = 'y';

  %% start timers for measuring the elapsed time for the whole process 

   	tic

 %%% independently read the "Corblivar.conf" file and create and string array confText for further internal use

   	confText = fopen(conf,'rt');	% Octave opens Text temporarily in own format

  %% initiate line counter and get read first line of Configtext
   % already read lines will be erased from temporary Text

   	line = 1;
   	text = fgetl(confText);
 
  %% initialize variables for distinction of cases and check-string to import the layers parameter  

   	h = 10000;
   	k = 0;
   	checkLayers = '# Layers';

  %% start reading process

   while k < 1

	line++;						% line counter
	temp = fgetl (confText);			% get next unread line of text 
		
		if 	strncmp(checkLayers, temp,8)	% search for checkLayers string

			h = line;				% safe line (there is the definition)

		endif	

		if line == (h+2)			% get line two lines below the safed one (there is the numerical parameter)

			layers = temp;				% extract layers parameter
			layers = str2num(layers);		% convert extracted string into number

		endif

		if temp == -1				% no more lines to read --> stop while-loop

			k = 1;
	
		break
		endif

	text = [text ; temp];				% update the string array of for confText and safe line beneath the last safed one

   endwhile

  %% save temporary string array as confText

   	confText = text;

    %% use config function to write new "Corblivar.conf" file with initial values

   	config(confText,I,If,Mb,PDPZ,minHS, conf);


%%% start loop for different TSV densities [%]
   

minHS_100 = -1;

while TSV_dens >= 0

	TSV_dens	% show TSV-density

 	%%% execute initial Corblivar floorplanning

 	if init == 'y'

		% path for executing Corblivar without a solution file

		pathCbl = sprintf('%s/Corblivar %s %s %s/benches/',bin, bench, conf,exp);					% %s will be replaced by the parameter standing behind the string
   
		system (pathCbl);	% system function uses terminal and bash notation


 	endif
	
	% set all sigmas back to initial values
%	[sigma_I,sigma_If,sigma_Mb,sigma_PDPZ] = parameters();

	% reset sigmas and initial parameter values
	[sigma_I,sigma_If,sigma_Mb,sigma_PDPZ,I,If,Mb,PDPZ] = parameters();

	% rewrite the config.file

	config(confText,I,If,Mb,PDPZ,minHS,conf);
 
	%%% execute HotSpot thermal analysis

  	%% execute Corblivar with given floorplan and initial parameters
  
	% path for execution of Corblivar with given solution file 
	
	pathCblsol = sprintf('%s/ThermalAnalyzerFitting %s %s %s/benches/ %s.solution %d',bin, bench, conf, exp, bench, TSV_dens); 
	
	system (pathCblsol);

	% path for executing 3D-HotSpot
	pathHS = sprintf('%s/HotSpot.sh %s %d',exp, bench, layers);

   	system (pathHS);


	%% plot thermal and power maps with given shell script
   	
	pathGP = sprintf('%s/gp.sh',exp);
	system (pathGP);


  	%%% start the eval function to assess error between HotSpot and Corblivar solution vectors
   
	[maxHS, minHS, maxCbl,minCbl, Error] = evalCorb(bench);

	if TSV_dens == 100

		minHS_100 = minHS;
	end

   	%% initialization of history for sample parameters and errors

    	hist = [I If Mb PDPZ Error]; 

   	%% set first parameters as the optimal parameters and start history for optimal parameters and errors

    	optI = I;
    	optIf = If;
    	optMb = Mb;
    	optPDPZ = PDPZ;
    	optError = Error;

    	optHist = [optI optIf optMb optPDPZ optError];


 	%%% start for-loop for randomized sampling 
  	%% generators use normal curve of distribution with optimal parameters as mu and sigma

  	j = 0;

   	for iter = 1:iter
		
		% impulse factor I
		I = 0;
		
		while I <= 0				% parameter must be positive

			I = optI + randn * sigma_I;	% randomly generated I with mean value optI and variance sigma_I
		end
		
		% impulse scaling factor If
		If = 0;
	
		while If <= 0
	
			If = optIf + randn * sigma_If;
		end

		% mask boundary Mb
	
		i = 0;
		a = 0;
		b = 0;
		count = 0;
	
		while i < 2				% Mb hast two preconditions: it has to be positive and smaller than I
		
			Mb = optMb + randn * sigma_Mb;
			
			a = Mb > 0;
			b = Mb < I;
		
			i = a + b;
		
			count++;			
		
			if count > 100			% if means are too widely apart and sigma was refined it's possible by chance that I becomes so small that Mb can't get smaller

				I = I * 1.05;		% forces I to increase
				count = 0;

			endif				

		end

		% power density scaling factor in padding zone

		PDPZ = 0.99;
	
		while PDPZ < 1.0 || PDPZ > max_PDPZ	% power density < 1 is not reasonable; also consider max value
	
			PDPZ = optPDPZ + randn * sigma_PDPZ;
		end

	
		%% end of random generation of parameters
	
		%% rewrite new parameters into "Corblivar.conf" file

		if minHS_100 != -1
			
			minHS = minHS_100;

		end
		
	 	config(confText, I,If,Mb,PDPZ,minHS,conf);	
	
	
		%%% execute Corblivar with given floorplan and without returning something to the terminal
	
		pathCblsolnt = sprintf('%s/ThermalAnalyzerFitting %s %s %s/benches/ %s.solution %d >/dev/null',bin, bench, conf,exp, bench,TSV_dens);
	
  		system (pathCblsolnt);
	
	 	%% reevaluate the error between HotSpot and Corblivar
	
	  	[maxHS, minHS, maxCbl, minCbl, Error] = evalCorb(bench);

	 	%% update history 
	
	  	hist = [hist ; I If Mb PDPZ Error];


		%%% Decision
		 %% decide if Error is smaller than optimal value
		  % if yes, update all optimal parameters	

		if Error < optError
	
			optError = Error 
			optI = I
			optIf = If
			optMb = Mb
			optPDPZ = PDPZ

			%% update optimal history

			optHist = [optHist; optI optIf optMb optPDPZ optError];
		
		endif

   		iter  % output number of iterations of for loop 


   		%% use counter to adjust the sigma of the randomizer after predefined stepsize 

   		j++;

		if j==step;				% stepsize can be edited in the parameters function
	
			sigma_I = sigma_I * sigma_update;
			sigma_If = sigma_If * sigma_update;
			sigma_Mb = sigma_Mb * sigma_update;
			sigma_PDPZ = sigma_PDPZ * sigma_update;

			% output sigmas
			sigma_I
			sigma_If
			sigma_Mb
			sigma_PDPZ

			j = 0;					% set back counter
	
		end

	end

 	%%% end of optimization loop

 	%%% plotting the results

  	%% load optmial parameters

   	I = optI;
   	If = optIf;
   	Mb = optMb;
   	PDPZ = optPDPZ;

	% create fitting history for all TSV_densities

	if minHS_100 != -1
			
		minHS = minHS_100;

	end

	if init == 'y'

		fitting_hist = [TSV_dens optI optIf optMb optPDPZ minHS];

	else
	
		fitting_hist = [fitting_hist ; TSV_dens optI optIf optMb optPDPZ minHS];

	endif


  	%% rewrite "Corblivar.conf" file 

   	config(confText, I,If,Mb,PDPZ,minHS,conf);


  	%% execute Corblivar with given floorplan and optimal parameters

   	system (pathCblsol);


  	%% generate the thermal map and power maps

   	system (pathGP);

  	%% evaluate the optimal solution

   	[maxHS, minHS, maxCbl, minCbl, Error, matError] = evalCorb(bench);

  	%% save maximum values and Error in "eval.txt" file

	savefold_TSV_dens = sprintf('%s/TSV_density_%d/',savefold,TSV_dens);
	
	mkdir(savefold_TSV_dens);
	
   	save (sprintf('%s/%s_thermal_analysis_fitting_ranges.txt', savefold_TSV_dens,bench), "maxHS", "minHS", "maxCbl", "minCbl", "Error");

  	%% save history of sampling in a reloadable "hist.data" file

   	helpHist = sprintf('This reloadable(in Octave) matrix is a history of the parameters impulse faktor,impulse-scaling factor, mask boundary and the power-density scaling factor together with the evaluated error');

   	save (sprintf('%s/%s_thermal_analysis_fitting_hist.data',savefold_TSV_dens,bench), "helpHist", "hist");

  	%% save history of optimal Parameters in a reloadable file

   	helpOptHist = sprintf('This reloadable(in Octave) matrix is a history of the parameters impulse faktor,impulse-scaling factor, mask boundary and the power-density scaling factor together with the evaluated error');
 
   	save (sprintf('%s/%s_thermal_analysis_fitting_opt_hist.data',savefold_TSV_dens,bench), "helpOptHist", "optHist");

	% copy sampling files of Corblivar and HotSpot

	file_list = ls;

	list_size = size(file_list,1);

	solution_file = sprintf('%s.solution',bench);	

	for sample = 1:list_size

		if strncmp('optimization.m',file_list(sample,:),14) || strncmp('config.m',file_list(sample,:),8) || strncmp('evalCorb.m',file_list(sample,:),10) || strncmp('parameters.m',file_list(sample,:),12)
			
		elseif strncmp(solution_file,file_list(sample,:),size(solution_file,2))

			copyfile(file_list(sample,:), savefold,'f');

		else

			copyfile(file_list(sample,:), savefold_TSV_dens,'f');
		
		endif
	end

	% set initial parameter to false
	
	init = 'n';

	% use counter to increase TSV density by TSV density step size
 
	TSV_dens = TSV_dens - TSV_dens_step;

end

  % size of fitting hist matrix
  fit_hist_size = size(fitting_hist);
  
  fitting_hist = flipud(fitting_hist);
  % create X and Y values for plotting optimal values

  plot_TSV_dens = fitting_hist(:,1);
  plot_para = fitting_hist(:,2:fit_hist_size(2));

  % create relative values for plotting of relative change

  ref_para = plot_para(1,:);
  rel_para = ones(1,(fit_hist_size(2)-1)) ;

for s = 2:fit_hist_size(1)

	rel_para = [rel_para; plot_para(s,:) ./ ref_para];

end

  % save fitting history and relative parameters

  save (sprintf('%s/%s_fitting_hist', savefold,bench), "fitting_hist","rel_para");

  % plot fitting history

  figure;
  semilogy(plot_TSV_dens,plot_para);
  xlabel('TSV density');
  legend('I','If','Mb','PDPZ','minHS');
  print(sprintf('%s/fitting_hist.pdf',savefold),'-dpdf');

  % plot relative change

  figure;
  plot(plot_TSV_dens, rel_para);
  xlabel('TSV density');
  legend('I','If','Mb','PDPZ','minHS');
  print(sprintf('%s/rel_fitting_hist.pdf',savefold),'-dpdf');

  %% output elapsed time fo optimization process

  toc

  %% clean working dir

  system (clean);

 %%% end of program
