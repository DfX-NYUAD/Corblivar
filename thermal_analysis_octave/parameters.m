function [p] = parameters() 

%% Description:  Integrated Octave function providing parameters for the optimization of the thermal analysis of Corblivar

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


%%% define general parameters for optimization
	
p.opt.layers.check = '# Layers';	% check string for comparing with the Corblivar.conf file
  
p.opt.accuracy = 50;			% starting point for valid solution space		


%% define after how many iterations the sigma of the normal distribution curve of the random generator will be refined
%% total iterations, i.e., tries for parameter fitting

p.opt.iterations = 100; 

% if the generator should not be refined set denominator <= 1
  
p.opt.step = p.opt.iterations / 10;


%% refinement factor of sigma
 
p.opt.sigma_update = 0.8;


%% define the initial minimum of the HotSpot analysis (only needed for the first writing of the config-file
% set to 293 K which corresponds to 20°C (room temperature)
   
p.conf.minHS.value = 293;
p.conf.minHS.check = '# temperature offset';



%%% Power blurring (thermal analysis) -- Thermal mask parameters
%% define initial impulse factor I, for the dominant mask (lowest layer)
%% define initial sigma for the random generator of I
 
p.conf.I.value = 1;
   
p.conf.I.sigma = 1;

p.conf.I.check = '# Impulse factor I';

  
%% define initial impulse-scaling factor If, I(layer) = I / (layer^If)
%% define initial sigma for the random generator of If
 
p.conf.If.value = 10;
 
p.conf.If.sigma = 5;

p.conf.If.check = '# Impulse-scaling factor If';


%% define initial mask-boundary /value/ b, gauss2D(x=y) = b at mask boundaries x=y, relates to dominant mask
%% define initial sigma for the random generator of mask boundary

p.conf.Mb.value = 0.1;

p.conf.Mb.sigma = 1;

p.conf.Mb.check = '# Mask-boundary';



%%% Power blurring -- Power maps parameters
%% define initial Power-density scaling factor in padding zone
%% define initial sigma for the random generator of power-density scaling factor in padding zone

p.conf.PDPZ.value = 1.1;

p.conf.PDPZ.sigma = 0.5;

p.conf.PDPZ.max_val = 2.0;

p.conf.PDPZ.check = '# Power-density scaling factor in padding zone';


%% %% define initial Power-density scaling factor in TSV regions
%% define initial sigma for the random generator of power-density scaling factor in TSV regions

p.conf.PDTR.value = 1;

p.conf.PDTR.sigma = 0.5;

p.conf.PDTR.check = '# Power-density down-scaling factor for TSV regions';

end
