f1 = fopen('kd.txt');
f2 = fopen('kp90.txt');
C1 = textscan(f1, '%f%f')
C2 = textscan(f2, '%f%f')
fclose(f1); fclose(f2);
plot(C1{1}, C1{2})
%plot(C1{1}, C1{2}, C2{1}, C2{2})
grid on
%plot(C{2})
%plot(C{3})