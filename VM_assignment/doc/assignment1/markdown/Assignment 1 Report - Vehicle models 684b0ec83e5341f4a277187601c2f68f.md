# Assignment 1 Report - Vehicle models

Created: December 7, 2023 12:54 PM
Class: PA 046
Type: Assignment
Reviewed: No

# Introduzione

In questo documento vengono mostrati e spiegati i risultati ottenuti dalle implementazione dei seguenti modelli:

- [**Kinematic**](https://www.notion.so/Assignment-1-Report-Vehicle-models-684b0ec83e5341f4a277187601c2f68f?pvs=21)
- [**Dynamic**](https://www.notion.so/Assignment-1-Report-Vehicle-models-684b0ec83e5341f4a277187601c2f68f?pvs=21)
- [**Dynamic w/ Road allignment**](https://www.notion.so/Assignment-1-Report-Vehicle-models-684b0ec83e5341f4a277187601c2f68f?pvs=21)

Lo scopo è quello di mettere a confronto i modelli in termini di **Velocità**, per i modelli Kinematic e Dynamic; in termini di ********Path******** per i modelli Dynamic e Dynamic w/ Road allignment.

## Modello fisico di simulazione

Tutti i modelli utilizzano lo stesso modello fisico del veicolo e segnale di input di sterzo, definiti come seguono:

- Modello fisico e geometrico del veicolo:
    
    ```matlab
    %% Vehicle parameteres
    %  Physical parameteres
    mass = 2164;        % Vehicle mass in kg
    %  Forces parameteres
    inertia = 4373;     % Inertia in kg*m^2
    %  Geometry parameteres
    l_f = 1.3384;       % Distance from the center of mass to front axle
    l_r = 1.6456;       % Distance from the center of mass to rear axle
    %  Constant parameteres
    C_f = 1.0745e5;     % Front cornering stiffness coefficient
    C_r = 1.9032e5;     % Rear cornering stiffness coefficient
    ```
    
- Input di sterzo (sinusoidale):
    
    ```matlab
    %% Simulation parameters
    %  Movement parameteres
    v = 30 * 1000 / 3600;   % Vehicle speed in m/s
    
    %  Signal parameters
    freq = 0.5;             % Frequency of sinusoidal steering input in Hz
    amp_steering = 30;      % Amplitude of sinusoidal steering input in rad
    
    % code ... %
    
    %% Support anonymous fun
    %  Calculate the slip angle in relation of the time (t)
    slip = @(t) deg2rad(30 * sin(2 * pi * freq * t));
    ```
    
    Il valore di `v = 30 * 1000 / 3600;   % Vehicle speed in m/s` viene manipolato in base alle tipologie di simulazioni che vengono eseguite, ovvero con valori di v in km/h di:
    
    - 50;
    - 70.

# Kinematic

## Descrizione del modello

> Il modello Kinematic non tiene conto delle forze fisiche agenti sul veicolo, ma solo del suo moto.
> 

## Risultati ottenuti

### 50 Km/h

![Grafico della posizione globale del veicolo per la velocità data (50km/h).](Assignment%201%20Report%20-%20Vehicle%20models%20684b0ec83e5341f4a277187601c2f68f/50_pos.jpg)

Grafico della posizione globale del veicolo per la velocità data (50km/h).

![Grafici generali del modello per la velocità data (50km/h).](Assignment%201%20Report%20-%20Vehicle%20models%20684b0ec83e5341f4a277187601c2f68f/50_all.jpg)

Grafici generali del modello per la velocità data (50km/h).

### 70 Km/h

![Grafico della posizione globale del veicolo per la velocità data (70km/h).](Assignment%201%20Report%20-%20Vehicle%20models%20684b0ec83e5341f4a277187601c2f68f/70_pos.jpg)

Grafico della posizione globale del veicolo per la velocità data (70km/h).

![Grafici generali del modello per la velocità data (70km/h).](Assignment%201%20Report%20-%20Vehicle%20models%20684b0ec83e5341f4a277187601c2f68f/70_all.jpg)

Grafici generali del modello per la velocità data (70km/h).

## Motivazione dei risultati

Quello che si può notare dal grafico delle posizioni è che all’aumentare della velocità v, si perde la posizione corretta del veicolo; perché il modello, non tenendo in considerazioni le forze fisiche che agiscono sul veicolo, in questo caso il movimento laterale in y; all’aumentare della velocità queste forze aumentano e quindi diventano non trascurabili per il calcolo della posizione globale sull’alle y.

# Dynamic

## Descrizione del modello

> Il modello Dynamic tiene in considerazione il movimento laterale (sull’asse y) del veicolo e nel sistema $\dot{x} = A(V_x) + B(V_x)u$, dove viene applicata la legge di Newton sull’asse y.
> 

## Risultati ottenuti

### 50 Km/h

![Grafico della posizione globale del veicolo per la velocità data (50km/h).](Assignment%201%20Report%20-%20Vehicle%20models%20684b0ec83e5341f4a277187601c2f68f/50_pos%201.jpg)

Grafico della posizione globale del veicolo per la velocità data (50km/h).

![Grafici generali del modello per la velocità data (50km/h).](Assignment%201%20Report%20-%20Vehicle%20models%20684b0ec83e5341f4a277187601c2f68f/50_all%201.jpg)

Grafici generali del modello per la velocità data (50km/h).

### 70 Km/h

![Grafico della posizione globale del veicolo per la velocità data (70km/h).](Assignment%201%20Report%20-%20Vehicle%20models%20684b0ec83e5341f4a277187601c2f68f/70_pos%201.jpg)

Grafico della posizione globale del veicolo per la velocità data (70km/h).

![Grafici generali del modello per la velocità data (70km/h).](Assignment%201%20Report%20-%20Vehicle%20models%20684b0ec83e5341f4a277187601c2f68f/70_all%201.jpg)

Grafici generali del modello per la velocità data (70km/h).

## Motivazione dei risultati

Il Dynamic all’aumentare della velocità, riesce tenere la posizione sull’asse y corretta, infatti da 50 Km/h a 70 Km/h non soffre come il Kinematic, questo perché nel sistema $\dot{x} = A(V_x)x+B(V_x)u$ contiene le leggi orarie sull’asse y e la geometrica del veicolo.

# Dynamic vs Kinematic, why?

La motivazione del perché il dynamic non soffre alle alte velocità in confronto al Kinematic la si trova nelle formule per il calcolo della posizione globale del veicolo.

- Posizioni X,Y nel Kinematic
    
    $$
    \dot{X} = V \cos(\psi + \beta)\\
    \dot{Y} = V \sin(\psi + \beta)
    
    $$
    
- Posizioni X,Y nel Dynamic
    
    $$
    \dot{X} = V_x \cos(\psi) + V_y \sin(\psi) \\
    \dot{Y} = V_y \cos(\psi) + V_x \sin(\psi)
    
    $$
    
    Ricordiamo l’assunzione che $V_y = \dot{y}$
    

Dalle due formule per calcolare $\dot{X}$ e $\dot{Y}$ si può notare che il Kinematic non tiene in considerazione le leggi orarie (legge di Newton) delle forze sull’asse y. 
Invece il Dynamic, dal sistema $\dot{x} = A(V_x)x+B(V_x)u$ mette a sistema le forze applicate sul veicolo e la geometria del veicolo; mitigando i problemi ad alta velocità che ha il Kinematic, tenendo in considerazione forze che diventano non trascurabili.
Una rapida osservazione che possiamo notare, senza analizzare le formule matematiche, sono i valori che assumono sugli assi y, nel Dynamic siano molto inferiori rispetto al Kinematic, dovuto appunto al tenere in considerazione le forze laterali applicate al veicolo.

# Dynamic w/ Road allignment

## Descrizione del modello

> Estensione del modello Dynamic, aggiungendo un raggio di curvatura da seguire, questo modificherà il calcolo della posizione globale del veicolo basandosi sull’errore della posizione dalla path desiderata da seguire.
> 

### Modifica del modello di simulazione

Aggiunta del raggio di curvatura R

```matlab
% code ... %

R = 100;     % Path

% code ... %

psi_des_dot = v_x/R;
```

## Risultati ottenuti

### R = 100

![Grafico della posizione globale e path desiderato dal raggio di curvatura dato (100)](Assignment%201%20Report%20-%20Vehicle%20models%20684b0ec83e5341f4a277187601c2f68f/r100.jpg)

Grafico della posizione globale e path desiderato dal raggio di curvatura dato (100)

![Grafici generali dato dal raggio di curvatura dato (100)](Assignment%201%20Report%20-%20Vehicle%20models%20684b0ec83e5341f4a277187601c2f68f/r100_all.jpg)

Grafici generali dato dal raggio di curvatura dato (100)

### R = $\infty$

![Grafico della posizione globale e path desiderato dal raggio di curvatura dato ($\infty$)](Assignment%201%20Report%20-%20Vehicle%20models%20684b0ec83e5341f4a277187601c2f68f/rinf.jpg)

Grafico della posizione globale e path desiderato dal raggio di curvatura dato ($\infty$)

![Grafici generali dato dal raggio di curvatura dato ($\infty$)](Assignment%201%20Report%20-%20Vehicle%20models%20684b0ec83e5341f4a277187601c2f68f/rinf_all.jpg)

Grafici generali dato dal raggio di curvatura dato ($\infty$)

## Motivazione dei risultati

Possiamo notare che dai grafici, che il modello cerca di ottenere la posizione, computando gli errori derivanti dal sistema $\dot{x} = A(V_x)x+B(V_x)u+B_d(V_x)d$.
Questo permette di correggere la posizione del veicolo e avere una posizione globale più vicina a quella desiderata.
Il problema è che non riusciamo a seguire il path corretto, perché non abbiamo un sistema di controllo che ci permette di cambiare il moto e la direzione del veicolo e quindi correggere la traiettoria reale del veicolo

# Dynamic vs w/ Road allignment

Mettendo a confronto i grafici del Dynamic e del Road allignment, possiamo notare che il Road allignment, su path R = $\infty$, ovvero path rettilineo; è più preciso del suo precedente. Questo perché come analizzato in precedenza, avendo un calcolo della posizione globale basato sugli errori $e_1$ ed $e_2$, ovviamente dalla formula $\dot{x} = A(V_x)x+B(V_x)u+B_d(V_x)d$ rimangono le leggi orarie sull’asse y, ma vengono aggiunti gli errori che permettono di avere un calcolo più preciso delle cordinate globali, avendo un path di riferimento da seguire

# Istruzioni per l’esecuzione

Kinematic:

- Spostarsi nella cartella `kinematic/` ed eseguire con MatLab il file `kinematic.m`

Dynamic:

- Spostarsi nella cartella `dynamic/default/` ed eseguire con MatLab il file `dynamic.m`

Road allignment:

- Spostarsi nella cartella `dynamic/road_allignment` ed eseguire con MatLab il file `road_allignment.m`