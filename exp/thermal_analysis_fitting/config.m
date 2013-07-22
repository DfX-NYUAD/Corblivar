function C = config(confText, I,If,Mb,PDPZ,minHS)

  %% Description:  Integrated Octave function for updating the Corblivar.conf file

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

 

  %% initialize strings for searching in string array confText to allocate positions of parameters

   cd ..

   checkI = '# Impulse factor I';
   checkIf = '# Impulse-scaling factor If';
   checkMb = '# Mask-boundary';
   checkPDPZ = '# Power-density scaling factor';
   checkminHS = '# temperature offset';

  %% get size of the string array confText

   scT = size(confText);
   sizeConf = scT(1);

 %%% convert all numerical parameters into strings that can replace the old ones
  %% this has to be consistent with the saved string array 

   I = num2str(I); 	% convert to string
   sI = size(I); 	% get size of string
   sI = scT(2)-sI(2);	% difference between size of parameter-string and confText size 
   I = [I blanks(sI)];	% add difference with blanks

   If = num2str(If);
   sIf = size(If);
   sIf = scT(2)-sIf(2);
   If = [If blanks(sIf)];

   Mb = num2str(Mb);
   sMb = size(Mb);
   sMb = scT(2)-sMb(2);
   Mb = [Mb blanks(sMb)];

   PDPZ = num2str(PDPZ);
   sPDPZ = size(PDPZ);
   sPDPZ = scT(2)-sPDPZ(2);
   PDPZ = [PDPZ blanks(sPDPZ)];

   minHS = num2str(minHS);
   sminHS = size(minHS);
   sminHS = scT(2)-sminHS(2);
   minHS = [minHS blanks(sminHS)];

 %%% write new Corblivar.conf file
  %% open a textfile for writing and simultaneously erase old file

args = argv();
   conf = args{2};
   fid = fopen(conf, "w");
  
  % initialize counter

  j = 1;

 %% for loop for writing down the Text line by line 
  % look for checkstrings and replace new strings before writing 


  for j = 1:sizeConf

	temp = confText(j,:);	% loads one line temporarily

	if 	strncmp(checkI, temp,18)
	confText(j+2,:) = I;
	endif
	
	if	strncmp(checkIf, temp,27)
	confText(j+2,:) = If;
	endif
	
	if	strncmp(checkMb, temp,15)
	confText(j+2,:) = Mb;		
	endif
	
	if	strncmp(checkPDPZ, temp,30)	
	confText(j+2,:) = PDPZ;
	endif

	if	strncmp(checkminHS, temp,20)	
	confText(j+2,:) = minHS;
	endif

  fprintf(fid,'%s\n',temp);	% writes down the line

  j++;

  end

  fclose(fid);			% text file is only edited after this command everything before happens in Octave

 %% check parameter for output
  % Octave functions don't work without output 

  C = 1;				

end
