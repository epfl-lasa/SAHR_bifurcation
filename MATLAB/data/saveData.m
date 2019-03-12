%%%%% Save data to .csv file as [data1,data2,...,dataN,time] %%%%%
function [] = saveData(filename, data, time)

fileID = fopen(filename,'w');
for j=1:size(data,2)
    s = strcat('x',num2str(j),',');
    fprintf(fileID, '%s ', s);
end
fprintf(fileID, '%s\n', 'time');

for i=1:size(data,1)
    for j=1:size(data,2)
        fprintf(fileID, '%f, ', data(i,j));
    end
    if (i ~= size(data,1))
        fprintf(fileID, '%f\n', time(i));
    else
        fprintf(fileID, '%f', time(i));
    end
end

fclose(fileID);

end