function C = config(p, conf)

%% Description:  Integrated Octave function for updating the Corblivar.conf file

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

 
%%% convert all generated numerical parameters into strings to replace the old ones
%% the size of the strings must have the same size as the whole string array 

% converting impulse factor I

I = num2str(p.conf.I.value); 		% convert value to string
sI = size(I); 				% get size of string
sI = conf.sct(2)-sI(2);			% get difference between size of parameter-string and confText size 
I = [I blanks(sI)];			% add difference with blanks

% converting impulse scaling factor
 
If = num2str(p.conf.If.value);
sIf = size(If);
sIf = conf.sct(2)-sIf(2);
If = [If blanks(sIf)];

% converting mask boundary Mb

Mb = num2str(p.conf.Mb.value);
sMb = size(Mb);
sMb = conf.sct(2)-sMb(2);
Mb = [Mb blanks(sMb)];

% converting power density in padding zone PDPZ

PDPZ = num2str(p.conf.PDPZ.value);
sPDPZ = size(PDPZ);
sPDPZ = conf.sct(2)-sPDPZ(2);
PDPZ = [PDPZ blanks(sPDPZ)];

% converting power density in TSV regions PDTR

PDTR = num2str(p.conf.PDTR.value);
sPDTR = size(PDTR);
sPDTR = conf.sct(2)-sPDTR(2);
PDTR = [PDTR blanks(sPDTR)];

% converting minimum HotSpot value minHS

minHS = num2str(p.conf.minHS.value);
sminHS = size(minHS);
sminHS = conf.sct(2)-sminHS(2);
minHS = [minHS blanks(sminHS)];


%%% write new Corblivar.conf file
%% open a textfile for writing and simultaneously erase old file

fid = fopen(conf.path, "w");
  
% initialize counter

j = 1;

% for loop for writing down the Text line by line 
% look for checkstrings and replace new strings before writing 


for j = 1:conf.sct(1)

	temp = conf.text(j,:);	% loads one line temporarily

	if 	strncmp(p.conf.I.check, temp,18)
	conf.text(j+2,:) = I;
	endif
	
	if	strncmp(p.conf.If.check, temp,27)
	conf.text(j+2,:) = If;
	endif
	
	if	strncmp(p.conf.Mb.check, temp,15)
	conf.text(j+2,:) = Mb;		
	endif
	
	if	strncmp(p.conf.PDPZ.check, temp,46)	
	conf.text(j+2,:) = PDPZ;
	endif

	if	strncmp(p.conf.minHS.check, temp,20)	
	conf.text(j+2,:) = minHS;
	endif
	
	if	strncmp(p.conf.PDTR.check, temp,46)	
	conf.text(j+2,:) = PDTR;
	endif

  fprintf(fid,'%s\n',temp);	% writes down the line

  j++;

  end

  fclose(fid);			% Corblivar.conf file will only be edited after this command, everything before happens inside Octave

  % also copy new config file to separte config file w/ TSVs; same parameters in both
  % files required in order to neglect impact of TSVs (i.e., different masks) while actual
  % determination of masks
  copyfile(conf.path, conf.path_TSVs);

 %% check parameter for output
  % Octave functions don't work without output 

  C = 1;				

end
