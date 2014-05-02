function [eval,Cbl] = evalCorb(p,bench,HS)

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


%% implementation of sample data
% define path to Corblivar data

pathCbl = sprintf('%s_1_thermal.data', bench);

% load Corblivar datavector

dataCbl = load(pathCbl);

Cbl.thermal = dataCbl(:,3);

% find maximum values of Corblivar 

Cbl.max_value = max(Cbl.thermal);

Cbl.min_value = min(Cbl.thermal);


%% converting Cbl vector into fitting matrix
% necessary because of differences between reading directions of HotSpot and Corblivar
% matrix dimension + 1
   
  n = 65;

% flip vector upside-down

Cbl.thermal = flipud(Cbl.thermal);

% rearrange as matrix

B = Cbl.thermal(n*0+1:n*1);

for iter = 1:(n-1)

	B = [B, Cbl.thermal(n*iter+1:n*(iter+1))];

end

% flip left-right

B = fliplr(B);

% drop last column, first row (dummy data)

B(:,[n]) = [];
B([1],:) = [];

Cbl.thermal = B;


%% find maximum points in Cbl

[Cbl.maxPoints,Cbl.max_list] = extrema(Cbl.thermal,0,1);

  
%% start interpretation
 % create error matrix as difference between HotSpot and Corblivar data

eval.matError = Cbl.thermal - HS(1).tempMap; 			% defines real difference between Corblivar and HotSpot thermal data


%% average squared error
% ! because the character of the Octave sum-function the double sum is necessary

eval.Error = 1/((n-1)^2) * sum(sum(abs(eval.matError)));
eval.Error_sq = 1/((n-1)^2) * sum(sum(abs(eval.matError.^2)));


%%% create evaluation matrix

eval(1).matrix = zeros(size(eval.matError));			% initialize as matrix of zeros

%% start evaluation of error

for l = 1: size(eval(1).matrix,1)

	for m = 1: size(eval(1).matrix,2) 

		if eval(1).matError(l,m) > p.opt.accuracy	% if error is bigger than current level of accuracy 

			eval(1).matrix(l,m) = 6;
		endif

		if eval(1).matError(l,m) < -p.opt.accuracy	% if error is less than current level of accuracy

			eval(1).matrix(l,m) = 4;
		endif

		if eval(1).matError(l,m) > -p.opt.accuracy && eval(1).matError(l,m) < p.opt.accuracy 	% if error is in valid range 

			eval(1).matrix(l,m) = 0;

		endif		
	end
end
 
%% sum over whole evaluation

eval.value = sum(sum(eval(1).matrix));

%% end of evaluation process


%% validity check
% initialize valid parameter

eval.valid = 1;

% check if HS maximum values are valid range	

for u = 1:size(HS(1).max_list,1)
		
	a = HS(1).max_list(u,1);
	b = HS(1).max_list(u,2);		

	if eval(1).matrix(a,b) > 0
			
		eval.valid = 0;
			
		break;
	end	
end 

% check if Cbl maximum values are in valid range	

for v = 1:size(Cbl.max_list,1)
		
	a = Cbl.max_list(v,1);
	b = Cbl.max_list(v,2);	

	if eval(1).matrix(a,b) > 0
		
		eval.valid = 0;
			
		break;
	end
end 

% check if HS minimum values are in valid range	

for w = 1:size(HS(1).min_list,1)
		
	a = HS(1).min_list(w,1);
	b = HS(1).min_list(w,2);		

	if eval(1).matrix(a,b) > 0
			
		eval.valid = 0;
			
		break;
	end
end 

%% end of validity check

end

