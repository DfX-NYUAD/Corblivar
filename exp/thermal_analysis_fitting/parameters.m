function [step, iterations, sigma_update, minHS, I, sigma_I, If, sigma_If, Mb, sigma_Mb, PDPZ, sigma_PDPZ, max_PDPZ] = parameters() 

  %% Description:  Integrated Octave function providing parameters for the optimization of the thermal analysis of Corblivar

  %% Author:  Timm Amstein for Johann Knechtel, johann.knechtel@ifte.de
  %% Company:  Institute of Electromechanical and Electronic Design, www.ifte.de

 %%% define general parameters for optimization

  %% define after how many iterations the sigma of the normal distribution curve of the random generator will be refined
   % if the generator should not be refined set step > number of iterations
  
   step = 5;

  %% total iterations, i.e., tries for parameter fitting

   iterations = 50;

  %% refinement factor of sigma
 
   sigma_update = 0.8;

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
