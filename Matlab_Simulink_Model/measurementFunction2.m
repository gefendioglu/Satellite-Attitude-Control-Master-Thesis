function y = measurementFunction2(x)

Hgyro = [eye(3,3) zeros(3,4)];
Hstr = [zeros(4,3) eye(4,4)];
y =[Hgyro;Hstr]*x;

end