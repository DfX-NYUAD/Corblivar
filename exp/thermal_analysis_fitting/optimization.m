
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

   clear; clc;


  %% use parameters function to import all general initialization parameters 

   [step, iter, sigma_update, minHS, I, sigma_I, If, sigma_If, Mb, sigma_Mb, PDPZ, sigma_PDPZ, max_PDPZ] = parameters();

  %% memorize working directory

   dir = pwd()

  %% get imput from user
   % the needed benchmark and the number of iterations must be chosen by the user

   % print recommendation for editing the .conf-file

   helpConf = sprintf('\nPlease make sure that you edited the "Corblivar.conf" file for your chip before you perform the optimization!!\n\n');
   printf(helpConf); 

  %% ask user for benchmark

   inputBench = sprintf('Please pick a benchmark! \n benchmark = ');
%   bench = input(inputBench,'s');
args = argv();
   bench = args{1};

%  %% ask user for number of iterations the optimization should perfom
%
%   inputIter = sprintf('\nHow many iterations shell to optimization circle perform?\nChoose a number between 100 (fast) and 1000 (best results)\n\n iterations = ');
%   iter = input(inputIter); 

  %% ask user for initial run
%  inputInit = sprintf('Perform an initial run of Corblivar and HotSpot to gain data for optimization? \n Run [y/n] = ');
%  init = input(inputInit, 's');
  init = 'y';


  %% start timers for measuring the elapsed time for the Floorplanning and HotSpot analysis and for the whole process 

   tic
   tic


 %%% independently read the "Corblivar.conf" file and create and string array confText for further internal use

   cd ..

  %% open the "Corblivar.conf" file for reading

%   conf = 'Corblivar.conf';
   conf = args{2};
   confText = fopen(conf,'rt');	% Octave opens Text temporarily in own format

  %% initiate line counter and get read first line of text
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

  

   cd (dir)

  %% use config function to write new "Corblivar.conf" file with initial values

   config(confText,I,If,Mb,PDPZ,minHS);


  %% prepare paths for shell scrips

   pathCbl = sprintf('~/code/Corblivar/Corblivar %s %s benches/', bench, conf);					% %s will be replaced by the parameter standing behind the string
   pathCblsol = sprintf('~/code/Corblivar/Corblivar %s %s benches/ %s.solution', bench, conf, bench);
   pathCblsolnt = sprintf('~/code/Corblivar/Corblivar %s %s benches/ %s.solution >/dev/null', bench, conf, bench);
   pathHS = sprintf('./HotSpot.sh %s %d',bench, layers);



 %%% execute Corblivar floorplanning

 if init == 'y'

   system (pathCbl);	% system function uses terminal and bash notation


 %%% execute HotSpot thermal analysis

   system (pathHS);

 else

  %% execute Corblivar with given floorplan and initial parameters
   system (pathCblsol);

 endif

   %% plot thermal and power maps
   system ('./gp.sh');


  %% print the elapsed time for floorplanning and HotSpot analysis

   toc


   cd (dir)


  %%% start the eval function to assess error between HotSpot and Corblivar solution vectors

   [maxHS, minHS, maxCbl,minCbl, Error] = evalCorb(bench, dir);


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

	I = 0;
	while I <= 0				% parameter must be positive
		I = optI + randn * sigma_I;	% randomly generated I with mean value optI and variance sigma_I
	end
	
	If = 0;
	while If <= 0
		If = optIf + randn * sigma_If;
	end


	% Mb
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

	PDPZ = 0.99;
	while PDPZ < 1.0 || PDPZ > max_PDPZ	% power density < 1 is not reasonable; also consider max value
		PDPZ = optPDPZ + randn * sigma_PDPZ;
	end

	%% end of random generation of parameters
	
	%% rewrite new parameters into "Corblivar.conf" file

	 config(confText, I,If,Mb,PDPZ,minHS);	
	
	
	%%% execute Corblivar with given floorplan and without returning something to the terminal

	  system (pathCblsolnt);

	
	  cd (dir)	
	
	 %% reevaluate the error between HotSpot and Corblivar
	
	  [maxHS, minHS, maxCbl, minCbl, Error] = evalCorb(bench, dir);


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


  %% rewrite "Corblivar.conf" file 

   config(confText, I,If,Mb,PDPZ,minHS);


  %% execute Corblivar with given floorplan and optimal parameters

   system (pathCblsol);


  %% generate the thermal map and power maps

   system ('./gp.sh');


   cd (dir)

  %% evaluate the optimal solution

   [maxHS, minHS, maxCbl, minCbl, Error, matError] = evalCorb(bench, dir);


   cd ..

  %% save maximum values and Error in "eval.txt" file

   save (sprintf('%s_thermal_analysis_fitting_ranges.txt', bench), "maxHS", "minHS", "maxCbl", "minCbl", "Error");

  %% save history of sampling in a reloadable "hist.data" file

   helpHist = sprintf('This reloadable(in Octave) matrix is a history of the parameters impulse faktor,impulse-scaling factor, mask boundary and the power-density scaling factor together with the evaluated error');

   save (sprintf('%s_thermal_analysis_fitting_hist.data', bench), "helpHist", "hist");

  %% save history of optimal Parameters in a reloadable file

   helpOptHist = sprintf('This reloadable(in Octave) matrix is a history of the parameters impulse faktor,impulse-scaling factor, mask boundary and the power-density scaling factor together with the evaluated error');
 
   save (sprintf('%s_thermal_analysis_fitting_opt_hist.data', bench), "helpOptHist", "optHist");


  %% output elapsed time fo optimization process

   toc


 %%% end of program
