%%%%% Load from .csv file data in the form [x1,x2,...,xN,time] %%%%%
function [data, time] = loadData(filename,Nt,timeAtBeginning)

if(~exist('timeAtBeginning','var'))
    timeAtBeginning = 0;
end

if(isempty(filename))
    [filename,~] = uigetfile('*.csv');
end
fileID = fopen(filename,'r');

delimiter = ',';
startRow = 2;

first = fgetl(fileID);
if(~exist('Nt','var'))
    [~,Nt] = sscanf(first,"%s");
end
formatSpec = '%f';
for i=2:Nt
    formatSpec = strcat(formatSpec,'%f');
end
formatSpec = strcat(formatSpec,'%[^\n\r]');

dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'TextType',...
    'string', 'ReturnOnError', false, 'EndOfLine', '\r\n');

data = [];
if timeAtBeginning == 0
    time = dataArray{end-1};
    data = dataArray{1};
end
for i=2:(Nt-1)
    data = [data dataArray{i}];
end
if timeAtBeginning == 1
    time = dataArray{1};
    data = [data dataArray{Nt}];
end

fclose(fileID);

end