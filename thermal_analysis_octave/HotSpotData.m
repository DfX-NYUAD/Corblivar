function [HS] = HotSpotData(bench,p)

%% Description:  Integrated Octave function for integration and evaluation of HotSpot-data

%% Copyright (C) 2013 Timm Amstein,timm.amstein@gmail.com, Johann Knechtel, johann.knechtel@ifte.de, www.ifte.

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

%% initialize bools and counter

i = 1;
k = 1; 

%% loop over all the layers

for v = 1:p.opt.layers.value

	%k;									% for plotting value of k  

	% define path to HS thermal maps

	pathHS = sprintf('%s_HotSpot.steady.grid.layer_%d', bench,k);

	% load Hotspot datavector

	dataHS = load(pathHS);

	% drop first row (positions)

	HS_temp = dataHS(:,2);


	%% save HS data and also absolut minimum and maximum points

	if i == 1 
		
		HS_data = HS_temp;
	
	 	HS(v).maxHS = max(HS_data);
		HS(v).minHS = min(HS_data);
	
		i = 0;
	else
		
		HS_data = [ HS_data HS_temp ];

	 	HS(v).maxHS = max(HS_data(:,v));
		HS(v).minHS = min(HS_data(:,v));
	end


	%% rearrange data vector as 2D matrix matching with Corblivar thermal map

	n = 65;

	A = HS_data((n-1)*0+1:(n-1)*1,v);

	A = [A HS_data((n-1)*0+1:(n-1)*1,v)];

		for iterations = 1:(n-2)

			A = [A HS_data((n-1)*iterations+1:(n-1)*(iterations+1),v)];
		end
	   
	A = [A ; A(n-1,:)];

	A = A';

	% drop last column, first row (dummy data)

	A(:,[n]) = [];
	A([1],:) = [];
	
	% save temperature map

	HS(v).tempMap = A;

	% save temperature difference (room temperature 293 K)

	HS(v).deltaTemp = A - 293;

	% plot HS temperature map as 3D image

	surf(HS(v).tempMap),colorbar;
	print(sprintf('HS_%d',v),'-dpdf');


	%% find maximum points in HS
	
	[HS(v).maxPoints,HS(v).max_list] = extrema(HS(v).tempMap,0,1);
	
	% plot map of maxima

	imagesc(HS(v).maxPoints) , colorbar;
	print(sprintf('HS_%d_maxPoints.pdf',v),'-dpdf');

	% find minimum points in HS

	[HS(v).minPoints,HS(v).min_list] = extrema(HS(v).tempMap,HS(v).minHS,0);
	
	% plot map of minima

	imagesc(HS(v).minPoints) , colorbar;
	print(sprintf('HS_%d_minPoints.pdf',v),'-dpdf');

	% skip next layers (only analyse active layers)

	k = k + 4;
end
end
