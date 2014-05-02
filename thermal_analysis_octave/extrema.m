function [points,extList] = extrema(tempMap,extPoint,z)

%% Description:  Integrated Octave function to search for extrema in a 2D array

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

%% ! define z == 0 for minima and z == 1 for maxima

% copy 2D Map

A = tempMap;

%% create a frame 
% zeros for maximum view
% maximum temperature for minimum view 
 

if z == 0
	A = [(ones(size(A,1),1)*extPoint) A (ones(size(A,1),1)*extPoint)];
	A = [(ones(1,size(A,2))*extPoint); A ; (ones(1,size(A,2))*extPoint)];
else
	A = [zeros(size(A,1),1) A ones(size(A,1),1)];
	A = [zeros(1,size(A,2)); A ; ones(1,size(A,2))];
end

%% start checking process

% parameter needed for first extrema

set = 0;

% save size of 2D map

size_A = size(A);

% create an array of zeros with same size

tempMap = zeros(size_A);

%% check for extrema by comparing each point with all its neighbours

for l = 2:(size_A(1)-1)
		
	for m = 2:(size_A(2)-1) 	   
			
		if z == 0	% minima, therefor comparison is always neighbours >= point

			if (A(l-1,m-1) >= A(l,m));
			if (A(l,m-1) >= A(l,m));
			if (A(l+1,m-1) >= A(l,m));
			if (A(l-1,m) >= A(l,m));
			if (A(l+1,m) >= A(l,m));
			if (A(l-1,m+1) >= A(l,m));
			if (A(l,m+1) >= A(l,m));
			if (A(l+1,m+1) >= A(l,m));
				
				t = [l-1 m-1];				% conversion of point because of the frame
					
				if set == 0
					
					extList = t;
					set = 1;
				else
					
					extList = [extList ; t];	% create list of extrema
				endif
		
				tempMap(l,m) = 1;			% create temporary map where position of the extrema are shown
			end
			end
			end
			end
			end
			end
			end
			end

		else		% maxima, therefor comparison is always neighbours <= point
	
			if (A(l-1,m-1) >= A(l,m));
			if (A(l,m-1) >= A(l,m));
			if (A(l+1,m-1) >= A(l,m));
			if (A(l-1,m) >= A(l,m));
			if (A(l+1,m) >= A(l,m));
			if (A(l-1,m+1) >= A(l,m));
			if (A(l,m+1) >= A(l,m));
			if (A(l+1,m+1) >= A(l,m));
				
				t = [l-1 m-1];
					
				if set == 0
					
					extList = t;
					set = 1;

				else

					extList = [extList ; t];	
				endif
	
				tempMap(l,m) = 1;
			end
			end
			end
			end
			end
			end
			end
			end
		end		
	end
end 

%% save position map without frame
	
points = tempMap(2:size(tempMap,1)-1,2:size(tempMap,2)-1);

end
