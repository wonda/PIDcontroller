fid = fopen('output.txt');
C = textscan(fid, '%f%f%f%f%f%f')
fclose(fid);
x = [1:170];
plot(x, C{4}, x, C{5}, x, C{6})
grid on
%plot(C{2})
%plot(C{3})