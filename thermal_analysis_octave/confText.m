function [conf, p]= confText(conf, p)

%% Description:  Integrated Octave function for reading the "Corblivar.conf" file and creating a string array confText for further internal use

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


conf.text = fopen(conf.path,'rt');	% Octave opens Text temporarily in own format

%% initiate line counter and get read first line of Configtext

line = 1;
text = fgetl(conf.text);	% fgetl puts out first line of text and erases the same in the original

%% initialize variables for distinction of cases and check-string to import the layers parameter  

h = 10000;
k = 0;

%% start reading process

while k < 1

	line++;							% line counter
	temp = fgetl (conf.text);				% get next unread line of text 
		
	if 	strncmp(p.opt.layers.check, temp,8)	% search for checkLayers string

		h = line;					% safe line (there is the definition and the name of parameter in config file)
	endif	

	if line == (h+2)					% get line two lines below the safed one (there is the numerical parameter)

		layers = temp;					% extract layers parameter
		p.opt.layers.value = str2num(layers);	% convert extracted string into number
	endif

	if temp == -1						% no more lines to read --> stop while-loop

		k = 1;
		
		break
	endif

	text = [text ; temp];					% update the string array for confText and safe the temporary line beneath the last safed one

endwhile

%% save temporary string array as confText

conf.text = text;

%% get size of the string array

conf.sct = size(conf.text);

end
