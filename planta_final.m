% Matlab File (this line only on github)
% Parâmetros do sistema
Pot= 100; % potencia resistencia
Vol =0.35*0.25*0.15; % Volume de Ar
rho = 1.169; % Densidade (kg/m^3)
Car = 1005; % Capacidade térmica do ar (J/(kg*K))
DeltaT = 25; % Mudança de temperatura (°C)
Rel = 100/(8.33^2); % Resistência elétrica (ohms)
Delta_t = 0.1; % Intervalo de tempo (s)
Rf =  0.04142502; % Resistência térmica (°C/W)
Ti = 50; % Temperatura interna inicial (°C)
Te = 25; % Temperatura externa (°C)
Req=Rf+Te;
A = 2*40*0.0325*0.015; % Área de superfície (m^2)
h = Pot*0.85/(A*DeltaT) ; % Coeficiente de transferência de calor por convecção (W/(m^2*K))

syms s 

U= Req*Rel*s^2-1;

Y=Rf*Vol*rho*Car*s;

G=Y/U;
simplify(G)
pretty(G)
%pidtool(G)
