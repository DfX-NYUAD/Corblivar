function [maxHS, minHS, maxCbl,minCbl, Error, matError] = eval(bench, dir)

  %% Description:  Integrated Octave function for the evaluation of the thermal analysis of Corblivar

  %% Author:  Timm Amstein for Johann Knechtel, johann.knechtel@ifte.de
  %% Company:  Institute of Electromechanical and Electronic Design, www.ifte.de

   % function needs 'bench' parameter
   % [...] defines possible output variables of eval-function  

  %% initialization of Octave function

   % for correct execution make sure that Corblivar generated the files 'X_HotSpot.steady.grid.layer_1' and 'X_1_thermal.data' when X stands for the used benchmark 


  %% implementation of sample data

   % define path to HotSpot data
 
   pathHS = sprintf('%s_HotSpot.steady.grid.layer_1', bench);

   % define path to Corblivar data

   pathCbl = sprintf('%s_1_thermal.data', bench);


   % load Hotspot datavector

   cd ..

   dataHS = load(pathHS);

   HS = dataHS(:,2);

   % load Corblivar datavector

   dataCbl = load(pathCbl);

   Cbl = dataCbl(:,3);

   cd (dir)


   % save length of Cbl vector

   m = length(Cbl);

   % find maximum values of HotSpot and Corblivar 

   maxHS = max(HS);

   maxCbl = max(Cbl);

   minHS = min(HS);

   minCbl = min(Cbl);

  %% converting HS and Cbl vector into fitting matrices
   % necessary because of differences between reading directions of HotSpot and Corblivar
 
   n = 65;

   A = HS((n-1)*0+1:(n-1)*1);
   A = [A HS((n-1)*0+1:(n-1)*1)];

	for iter = 1:(n-2)

		A = [A HS((n-1)*iter+1:(n-1)*(iter+1))];

	end
   

   A = [A ; A(64,:)];

   A = A';
   

   Cbl = flipud(Cbl);

   B = Cbl(n*0+1:n*1);

	for iter = 1:(n-1)

		B = [B, Cbl(n*iter+1:n*(iter+1))];

	end

   B = fliplr(B);


   % redefine HotSpot data as a matrix

   HS = A;

   % redefine Corblivar data a a matrix

   Cbl = B;


  %% visual test of resulting matrices

    %imagesc(A) , colorbar ;
    %print('HS.eps','-deps');
    %imagesc(B) , colorbar;
    %print('Cbl.eps','-deps');


  %% start interpretation

   % create error matrix as difference between HotSpot and Corblivar data

   matError = HS - Cbl;


 %%% evaluation of the error
   % to emphazise the importance of big differences the values of the error matrix are squared
   % Error can be interpreted as a weighted average
   % ! because the character of the Octave sum-function the double sum is necessary

   Error = 1/m*sum(sum(matError.^2));

 %%% end of evaluation process

end

