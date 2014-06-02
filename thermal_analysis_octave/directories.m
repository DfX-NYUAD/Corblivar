function [dir,conf] = directories(args,conf)

%% Description:  Integrated Octave function for defining all directories to work with

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


%% memorize working directory
% place where all the temporary files will be safed

dir.work = pwd();
	

%% memorize directory of Corblivar binary
% check if bin folder was exclusively defined

% initialize checking parameter
 
check = 0;					

% a directory dir.bin was defined exclusively 

if size(args,1) >= 3
		
	check = isdir(args{3});
		
	if check == 1		
	
		dir.bin = args{3};
	end
end

% if not dir.bin is parent of dir.work

if check == 0

	ind_bin = rindex(dir.work, "/");

	dir.bin = dir.work(1,1:(ind_bin-1));
end

% save directory for experiments folder of Corblivar

dir.exp = sprintf('%s/exp',dir.bin);

%% save Config directory
% check path for config file

ind_conf = rindex(conf.path, "/");

% defines place where all the results should be safed

% if directory of Corblivar.conf isn't defined explicitly its in the experiments folder
 
if ind_conf == 0

	conf.dir = dir.exp;
	conf.path = sprintf('%s/%s', dir.exp, conf.path);
	
else	

	conf.dir = conf(1,1:ind_conf);
	
end

end
