// Definizione della variabile complessa s
s = %s;

// Parametri PID
Kp = 1;    // Guadagno proporzionale
Ki = 1.5;    // Guadagno integrale
Kd = 0.5;  // Guadagno derivativo

// Definizione del controllore PID: C(s) = Kp + Ki/s + Kd*s
C = Kp + Ki/s + Kd*s;

// Sistema da controllare: G(s) = 1 / (1 + s)
G = 1 / (1 + s);

// Funzione di trasferimento in retroazione unitaria: GR = (C*G)/(1 + C*G)
GR = (C * G) / (1 + C * G);

// Definizione del sistema continuo
SIS_GR = syslin('c', GR);

// Tempo di simulazione
t = 0:0.01:25;

// Risposta al gradino
U = csim('step', t, SIS_GR);

// Tracciamento del grafico
plot(t, U)
xtitle("Risposta al gradino del sistema con PID","Tempo (s)", "U(t)")
