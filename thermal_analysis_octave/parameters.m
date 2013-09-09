function [sigma_I,sigma_If,sigma_Mb,sigma_PDPZ,I,If,Mb,PDPZ,minHS, step, iterations, sigma_update,max_PDPZ, TSV_dens, TSV_dens_step] = parameters() 

  %% Description:  Integrated Octave function providing parameters for the optimization of the thermal analysis of Corblivar

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

 %%% define general parameters for optimization

  %%  define TSV Density on chip
   %  define starting value for optimization in [%]

   TSV_dens = 100;

   %  define step size for TSV density increase after every optimization loop in [%]
   % for case without TSV use step size > 100
   % for case with 0% and 100% TSV use step size = 100
   % for further analysis of different densities use step size < 100

   TSV_dens_step = 10;


  %% define after how many iterations the sigma of the normal distribution curve of the random generator will be refined
   % if the generator should not be refined set step > number of iterations
  
   step = 10;

  %% total iterations, i.e., tries for parameter fitting

   iterations = 100; 

  %% refinement factor of sigma
 
   sigma_update = 0.9;

  %% define the initial minimum of the HotSpot analysis (only needed for the first writing of the config-file
   % set to 293 K which corresponds to 20°C (room temperature)
   
   minHS = 293;

 %%% Power blurring (thermal analysis) -- Thermal mask parameters
  %% define initial impulse factor I, for the dominant mask (lowest layer)
  %% define initial sigma for the random generator of I
 
   I = 1;
   
   sigma_I = 0.5;

  %% define initial impulse-scaling factor If, I(layer) = I / (layer^If)
  %% define initial sigma for the random generator of If
 
   If = 10;
 
   sigma_If = 5;

  %% define initial mask-boundary /value/ b, gauss2D(x=y) = b at mask boundaries x=y, relates to dominant mask
  %% define initial sigma for the random generator of mask boundary

   Mb = 0.1;

   sigma_Mb = 0.25;

 %%% Power blurring -- Power maps parameters
  %% define initial Power-density scaling factor in padding zone
  %% define initial sigma for the random generator of power-density scaling factor

   PDPZ = 1.1;
 
   sigma_PDPZ = 0.2;

   max_PDPZ = 1.5;

end
