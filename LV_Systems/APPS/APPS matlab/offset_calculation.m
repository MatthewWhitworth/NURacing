%   Vcc
%    |
%    Ra
%    |  <- Vmax
%    R
%    |  <- Vmin
%    Rb
%    |
%   GND

Vcc = 5;
Vmax = 4.5;
Vmin = 1.0;
R = 2.1;

Va = Vcc - Vmax;
Vb = Vmin;

syms Ra Rb
eqn1 = Ra*(Vcc-Va) - Rb*Va == R*Va;
eqn2 = Rb*(Vcc-Vb) - Ra*Vb == R*Vb;

[A,B] = equationsToMatrix([eqn1, eqn2], [Ra, Rb]);
X = linsolve(A,B);
RA = double(X(1,1));
RB = double(X(2,1));

gain = Vcc/(Vmax - Vmin);