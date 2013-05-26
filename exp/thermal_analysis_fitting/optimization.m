
 %%% Description:  Octave program for optimizing the Error between the thermal analysis of HotSpot and Corblivar via parameter variation 

 %%% Author:  Timm Amstein for Johann Knechtel, johann.knechtel@ifte.de
 %%% Company:  Institute of Electromechanical and Electronic Design, www.ifte.de



 %%% To execute this program you need the package "octave" which includes the gnu octave language for numerical computing
  
  %% For execution use terminal or Octave GUI like "qtoctave" and type:		octave optimization.m  
   % After that Octave should initialize and start the program instantaneously

 %%% Start optimization
  %% clear octave core and storage

   clear; clc;


  %% use parameters function to import all general initialization parameters 

   [step, j, h , minHS, I, sigma_I, If, sigma_If, Mb, sigma_Mb, PDPZ, sigma_PDPZ] = parameters();


  %% get imput from user
   % the needed benchmark and the number of iterations must be chosen by the user

   % print recommendation for editing the .conf-file

   helpConf = sprintf('Please make sure that you edited the "Corblivar.conf" file for your chip before you perform the optimization!!\n\n');
   printf(helpConf); 

  %% ask user for benchmark

   inputBench = sprintf('Please pick a benchmark! \nYou are able to choose between:\n  n100\n  n200\n  n300\n\n benchmark = ');
   bench = input(inputBench,'s');

  %% ask user for number of iterations the optimization should perfom

   inputIter = sprintf('\nHow many iterations shell to optimization circle perform?\nChoose a number between 100 (fast) and 1000 (best results)\n\n iterations = ');
   iter = input(inputIter); 


  %% start timers for measuring the elapsed time for the Floorplanning and HotSpot analysis and for the whole process 

   tic
   tic


 %%% independently read the "Corblivar.conf" file and create and string array confText for further internal use

   cd ..

  %% open the "Corblivar.conf" file for reading

   fid = 'Corblivar.conf';
   confText = fopen(fid,'rt');	% Octave opens Text temporarily in own format

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

  

   cd opt

  %% use config function to write new "Corblivar.conf" file with initial values

   config(confText,I,If,Mb,PDPZ,minHS);


  %% prepare paths for shell scrips

   pathCbl = sprintf('../Corblivar %s Corblivar.conf bench/',bench);					% %s will be replaced by the parameter standing behind the string
   pathCblsol = sprintf('../Corblivar %s Corblivar.conf bench/ %s.solution',bench,bench);
   pathCblsolnt = sprintf('../Corblivar %s Corblivar.conf bench/ %s.solution >/dev/null',bench,bench);
   pathHS = sprintf('./HotSpot.sh %s %d',bench, layers);



 %%% execute Corblivar floorplanning

   system (pathCbl);	% system function uses terminal and bash notation


 %%% execute HotSpot thermal analysis

   system (pathHS);


  %% print the elapsed time for floorplanning and HotSpot analysis

   toc


   cd opt


  %%% start the eval function to assess error between HotSpot and Corblivar solution vectors

   [maxHS, minHS, maxCbl,minCbl, Error] = eval(bench);


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

   for iter = 1:iter

	I = optI + randn * sigma_I;			% randomly generated I with mean value opt I and standard deviation sigma_I

		while I < 0				% parameter must be positive
		I = optI + randn * sigma_I;
		end
	
	If = optIf + randn * sigma_If;

		while If < 0
		If = optIf + randn * sigma_If;
		end

	Mb = optMb + randn * sigma_Mb;

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

				I = I + 0.1;		% forces I to increase
				count = 0;

			endif				

		end

	PDPZ = optPDPZ + randn * sigma_PDPZ;

		while PDPZ < 0
		PDPZ = optPDPZ + randn * sigma_PDPZ;
		end

	%% end of random generation of parameters
	
	%% rewrite new parameters into "Corblivar.conf" file

	 config(confText, I,If,Mb,PDPZ,minHS);	
	
	
	%%% execute Corblivar with given floorplan and without returning something to the terminal

	  system (pathCblsolnt);

	
	  cd opt	
	
	 %% reevaluate the error between HotSpot and Corblivar
	
	  [maxHS, minHS, maxCbl, minCbl, Error] = eval(bench);


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

	
	%% adjust the standart deviation of the random generators after predefined stepsize   

	if j==step;				% stepsize can be edited in the parameters function
	
	sigma_I = sigma_I * 0.75;		% std dev will be diminished: 25%
 	sigma_If = sigma_If * 0.75;
 	sigma_Mb = sigma_Mb * 0.75;
 	sigma_PDPZ = sigma_PDPZ * 0.75;

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


   cd opt

  %% evaluate the optimal solution

   [maxHS, minHS, maxCbl, minCbl, Error, matError] = eval(bench);


   cd ..

  %% save maximum values and Error in "eval.txt" file

   save eval.txt maxHS minHS maxCbl minCbl Error


  %% save history of sampling in a reloadable "hist.data" file

   helpHist = sprintf('This reloadable(in Octave) matrix is a history of the parameters impulse faktor,impulse-scaling factor, mask boundary and the power-density scaling factor together with the evaluated error');

   save hist.data helpHist hist

  %% save history of optimal Parameters in a reloadable file

   helpOptHist = sprintf('This reloadable(in Octave) matrix is a history of the parameters impulse faktor,impulse-scaling factor, mask boundary and the power-density scaling factor together with the evaluated error');
 
   save optHist.data helpOptHist optHist


  %% output elapsed time fo optimization process

   toc


 %%% end of program
