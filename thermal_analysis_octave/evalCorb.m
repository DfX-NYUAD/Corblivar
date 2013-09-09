function [maxHS, minHS, maxCbl,minCbl, Error, matError] = evalCorb(bench)

  %% Description:  Integrated Octave function for the evaluation of the thermal analysis of Corblivar

  %% Copyright (C) 2013 Timm Amstein, Johann Knechtel, johann.knechtel@ifte.de, www.ifte.

  %% This file is part of Corblivar.

  %%    Corblivar is free software: you can redistribute it and/or modify
  %%    it under the terms of the GNU General Public License as published by
  %%    the Free Software Foundation, either version 3 of the License, or (at your option)
  %%    any later version.
  %%    
  %%    Corblivar is distributed in the hope that it will be useful,
  %%    but WITHOUT ANY WARRANTY; without even the implied warranty of
  %%    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public
  %%    License for more details.
  %%    
  %%    You should have received a copy of the GNU General
  %%    Public License along with Corblivar.  If not, see <http://www.gnu.org/licenses/>.

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

   dataHS = load(pathHS);

   HS = dataHS(:,2);

   % load Corblivar datavector

   dataCbl = load(pathCbl);

   Cbl = dataCbl(:,3);

   % find maximum values of HotSpot and Corblivar 

   maxHS = max(HS);

   maxCbl = max(Cbl);

   minHS = min(HS);

   minCbl = min(Cbl);

  %% converting HS and Cbl vector into fitting matrices
   % necessary because of differences between reading directions of HotSpot and Corblivar

   % matrix dimension + 1

   n = 65;

   A = HS((n-1)*0+1:(n-1)*1);
   A = [A HS((n-1)*0+1:(n-1)*1)];

	for iter = 1:(n-2)

		A = [A HS((n-1)*iter+1:(n-1)*(iter+1))];

	end
   

   A = [A ; A(n-1,:)];

   A = A';
   

   Cbl = flipud(Cbl);

   B = Cbl(n*0+1:n*1);

	for iter = 1:(n-1)

		B = [B, Cbl(n*iter+1:n*(iter+1))];

	end

   B = fliplr(B);


   % redefine HotSpot data as a matrix

   % drop last column, first row (dummy data)

      A(:,[n]) = [];
      A([1],:) = [];

   HS = A;

   %   disp(HS);
   %   disp(length(HS));

   % redefine Corblivar data a a matrix

   % drop last column, first row (dummy data)

      B(:,[n]) = [];
      B([1],:) = [];

   Cbl = B;

   %   disp(Cbl);
   %   disp(length(Cbl));

  %% visual test of resulting matrices

   %    imagesc(HS) , colorbar ;
   %    print('HS.eps','-deps');
   %    imagesc(Cbl) , colorbar;
   %    print('Cbl.eps','-deps');


  %% start interpretation

   % create error matrix as difference between HotSpot and Corblivar data

   matError = HS - Cbl;

   %   disp(length(matError));

   %    imagesc(matError) , colorbar;
   %    print('matError.eps','-deps');


 %%% evaluation of the error
   % to emphazise the importance of big differences the values of the error matrix are squared
   % Error can be interpreted as a weighted average
   % ! because the character of the Octave sum-function the double sum is necessary

   % avg squared error
   Error = 1/((n-1)^2) * sum(sum(matError.^2));

 %%% end of evaluation process

end

