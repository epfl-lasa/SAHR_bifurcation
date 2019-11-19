A = fileread('defs.txt');
id1 = strfind(A,'(');
id2 = strfind(A,')');
for i=length(id1):-1:1
A = strrep(A, A(id1(i):id2(i)), []);
end
fileID2 = fopen('defs2.txt','w');
fwrite(fileID2, A);