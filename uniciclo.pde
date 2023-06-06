/*
Si consideri lo sketch KalmanUniciclo.pde visto a lezione, nel quale si implementa il controllo di un uniciclo
basato sulla stima fornita da un filtro di Kalman esteso. Modificare tale sketch tenendo conto delle seguenti specifiche:

    Ciascun landmark può essere acceso/spento premendo un tasto della tastiera: per esempio, 
    premendo il tasto 1, se il landmark 1 è acceso, viene spento. Premendo di nuovo il tasto 1, lo si riaccende. 
    Lo stesso discorso vale per gli altri landmark (con il tasto 2 si accende/spegne il landmark 2 e così via). 
    Per semplicità, per non dover usare altri tasti oltre quelli numerici, si assuma che il numero massimo di landmark
    non possa essere maggiore di 9. Solo i landmark accesi possono essere utilizzati dal robot: gli altri è come se non esistessero.
    Un landmark è visibile solo quando è acceso e cade dentro una regione di visibiltà che è un settore circolare 
    di raggio rMax e angolo betaMax centrato nell'uniciclo e simmetrico rispetto alla direzione di avanzamento 
    dell'uniciclo stesso (vedere la regione verde nella figura riportata sotto). 
    Il robot può utilizzare solo le misure provenienti dai landmark visibili. La misura associata a un landmark visibile
    è l'angolo (detto di bearing) con cui il robot vede questo landmark rispetto al suo orientamento 
    (e non più quindi la distanza del landmark dal robot come accadeva nello sketch KalmanUniciclo.pde). 
    Tale angolo va considerato positivo quando il landmark è sulla sinistra del robot e negativo quando è sulla destra 
    (vale 0 se il landmark sta proprio nella direzione verso cui è orientato l'uniciclo). 
    Tale misura è caratterizzata da un errore gaussiano a media nulla e deviazione standard che va presa pari a 10 gradi.

Lo sketch KalmanUniciclo.pde va modificato, oltre che per tenere conto del nuovo tipo di misure disponibili, 
anche per introdurre le seguenti funzionalità:

    La regione di visibilità deve essere modificabile in tempo reale da tastiera: in particolare premendo i tasti 'r' e 'R' 
    deve essere possibile diminuire e, rispettivamente, aumentare il range massimo rMax del settore circolare. 
    Analogamente, con i tasti 'b' e 'B', deve essere possibile variare l'angolo di bearing massimo betaMax che definisce 
    l'apertura del settore circolare (nel senso che l'angolo di apertura di tale settore è 2*betaMax). 
    Prevedere un valore minimo positivo per rangeMax (per esempio 50 pixel) e dei valori limite per betaMax che deve sempre 
    essere compreso tra 0 e 180 gradi (per esempio 10 < betaMax < 140 gradi). La regione di visibilità deve essere disegnata.
    I landmark spenti, quelli accesi ma non visibili e quelli visibili vanno disegnati con un colore (o livello di grigio) 
    diverso per descriverne lo stato (quindi tre possibili colori o livelli di grigio in tutto).
    Lasciare attiva la funzionalità già presente nello sketch KalmanUniciclo.pde di modificare con le frecce il tempo tStep 
    tra una misura e l'altra. Come avviene in KalmanUniciclo.pde, quando in un passo k non è disponibile nessuna misura 
    (questo avviene non solo quando non è passato un tempo tStep dall'ultima misura ma anche quando non ci sono landmark visibili),
    il filtro di Kalman usa solo la predizione basata sui passi encoder delle ruote.
*/

// Coordinate attuali uniciclo
int sizeX = 1250;//1000;
int sizeY =  950;//550;

// Coordinate attuali uniciclo
float x = 0;
float y = 0;
float theta = 0;

// Coordinate desiderate uniciclo
float xDes = 0;
float yDes = 0;

// Caratteristiche fisiche uniciclo
float r = 8; // raggio ruote in pixel
float d = 25; // distanza tra le ruote in pixel
float w = 5; // spessore ruota in pixel
float R = 1.2*sqrt(pow(r,2)+pow(d/2+w/2,2)); // raggio robot

float dt = (float) 1/60; // tempo di campionamento

float e_p = 0; // errore posizionamento
float v1 = 0; // velocità lineare robot
float kv1 = 1; // costante legge proporzionale controllo v1
float v2 = 0; // velocità rotazionale robot
float kv2 = 2;  // costante legge proporzionale controllo v2
int nGiri = 0; // conta quanti giri su se stesso ha fatto il robot
float thetaDes; // orientamento desiderato (per raggiungere il target)
float t0,tempo; // tempo inizio e attuale missione
float tempoUltimaMisura; // tempo in cui si è effettuata l'ultima misura
long nStime = 0; // numero progressivo totale delle stime fatte

float omegaR = 0; // velocità angolare ruota destra
float omegaL = 0; // velocità angolare ruota sinistra
float uR, uL, uRe, uLe; // spostamenti ruote destra e sinistra veri e presunti (comandati)

// Variabili relative alla stima e al filtro di Kalman esteso
float sigmaX0 = 10; // deviazione standard dell'errore di stima del valore iniziale di x0
float sigmaY0 = 10; // deviazione standard dell'errore di stima del valore iniziale di y0
float sigmaTheta0 = 10*PI/180; // deviazione standard (in rad) dell'errore di stima del valore iniziale di theta0
float xHat = x + Gaussian(0,sigmaX0); // stima di x inizializzata al valore vero + una perturbazione gaussiana a media 0 e deviazione standard sigmaX0
float yHat = y + Gaussian(0,sigmaY0); // stima di y inizializzata al valore vero + una perturbazione gaussiana a media 0 e deviazione standard sigmaY0
float thetaHat = theta + Gaussian(0,sigmaTheta0); // stima di theta inizializzata al valore vero + una perturbazione gaussiana a media 0 e deviazione standard sigmaTheta0 rad
float xHatMeno,yHatMeno,thetaHatMeno; // stime a priori di x,y,theta
float KR = 0.01; // coefficiente varianza errore odometrico ruota destra
float KL = KR; // coefficiente varianza errore odometrico ruota sinistra
float stDevR,stDevL; // deviazione standard errore odometrico ruota destra e sinistra
//float [][] Landmark = {{0,sizeY/2-20}};  // Coordinate landmark (un solo landmark)
//float [][] Landmark = {{0,-sizeY/2+20},{0,sizeY/2-20}}; // Coordinate landmark (due landmark)
float [][] Landmark = {{-sizeX/2+20,-sizeY/2+20},{sizeX/2-20,-sizeY/2+20},{0,sizeY/2-20}};  // Coordinate landmark (3 landmark)
int nL = Landmark.length; // numero totale di landmark

// Seguono le matrici utilizzate dal filtro di Kalman esteso (EKF)
float[][] F = {{1, 0, 0},{0, 1, 0},{0, 0, 1}}; // matrice giacobiana F=df/dx (alcuni elementi delle prime due righe vanno aggiustati durante l'esecuzione)
float[][] P = {{pow(sigmaX0,2), 0, 0},{0, pow(sigmaY0,2), 0},{0, 0, pow(sigmaTheta0,2)}}; // matrice di covarianza P inizializzata in base all'incertezza iniziale
float[][] Pmeno = new float[3][3]; // matrice di covarianza a priori P-
float[][] W = {{1, 0},{0,1},{1/d,-1/d}}; //  matrice giacobiana W = df/dw (gli elementi delle prime due righe vanno aggiustati durante l'esecuzione) 
float[][] Q = {{1, 0},{0,1}}; // matrice di covarianza del rumore di misura odometrico w (gli elementi sulla diagonale vanno aggiustati durante l'esecuzione)

float DeltaX,DeltaY,DeltaXY; // variabili di supporto
float sigmaLandmark = 10*PI/180; // deviazione standard errore di misura di distanza dai landmark (gradi)
float[][] correzione = new float[3][1]; // termine correttivo stima
float tStep = 0; // tempo (in ms) tra una misura e la successiva (impostabile da tastiera)


float rMax = 1000; //valore raggio arco in pixel
float betaMax = 35; //valore angolo arco in gradi
int nV = 0; // Numero landmark visibili;
int[] ldi = new int[nL]; //landmark attivo o meno
int[] visibile = new int[nL]; //landmark visibili
float Angle;
float dist;



void setup() 
{
  size(1250,950);
  tempoUltimaMisura = 0; // Inizializzo a zero il tempo in cui è stata effettuata l'ultima misura
}

void draw() 
{ 
  background(0);
  pushMatrix();
  translate(sizeX/2,sizeY/2);
 
// Controllo la visibilità
  for(int i=0; i<nL; i++){
    dist = sqrt(pow(Landmark[i][0]-x, 2) + pow(Landmark[i][1]-y, 2));
    Angle = atan2(Landmark[i][1]-y, Landmark[i][0]-x)-theta;
  
    if(abs(atan2(sin(Angle),cos(Angle)))<betaMax*PI/180 && dist <= rMax/2 && ldi[i]==1){
      visibile[i] = 1;
    }else{
      visibile[i] = 0;
    }
  }
  nV = visibile.length;

// Dichiarazioni dipendenti dal numero di landmark visibili
  float[][] H = new float[nV][3]; // matrice giacobiana H = dh/dx
  float[][] K = new float[3][nV]; // guadagno di Kalman
  float[][] Rs = idMat(nV,pow(sigmaLandmark,2)); // matrice di covarianza errore misura dai landmark
  float[][] innovazione = new float[nV][1]; // innovazione EKF
  float[] misureLandmark = new float[nV]; // vettore con le misure vere di distanza dagli nL landmark
  float[] misureAtteseLandmark = new float[nV]; // vettore con le misure attese di distanza dagli nL landmark



  if (keyPressed)
  {
    if (keyCode == UP) // aumento di 1 il tempo tra una misura e la successiva
    {
      tStep += 1;
    }
    if (keyCode == DOWN)  // decremento di 1 il tempo tra una misura e la successiva
    {
      tStep = max(0,tStep-1);
    }
    if (keyCode == RIGHT) // moltiplico per due il tempo tra una lettura e la successiva
    {
      tStep = tStep*2;
    }
    if (keyCode == LEFT) // divido per due il tempo tra una lettura e la successiva
    {
      tStep = tStep/2;
    }
    if(key == '1')
    {
      if(ldi[0] >= 1) //se premo 1 accendo/spengo il landmark 1 
      {
        ldi[0] = 0;
      }else{
        ldi[0] = 1;
      }
    }   
    if(key == '2')
    {
      if(ldi[1] >= 1) //se premo 2 accendo/spengo il landmark 2
      {
        ldi[1] = 0;
      }else{
        ldi[1] = 1;
      }
    }
    if(key == '3')
    {
      if(ldi[2] >= 1) //se premo 3 accendo/spengo il landmark 3
      {
        ldi[2] = 0;
      }else{
        ldi[2] = 1;
      }
    }
    if(key == 'r')
    {
      if(rMax<= 50)
      {
        rMax = 50;
      }else
      {
        rMax -= 10;
      }  
    }
    if(key == 'R')
    {
      rMax += 10;
    }
    if(key == 'b')
    {
      if(betaMax <= 10){
        betaMax = 10;
      }else
      {  
       betaMax -= 2;
      }
    }
    if(key == 'B')
    {
      if(betaMax >= 140){
        betaMax = 140;
      }else
      {  
       betaMax += 2;
      }
    }
  }
  
  keyPressed = false;
  
  if (mousePressed) // assegno target
  {
    xDes = mouseX - sizeX/2;
    yDes = sizeY/2 - mouseY;
    t0 = millis(); // inizio conteggio del tempo di missione
  }
  
  
    
// Calcolo errore e controllo basandomi sul valore stimato (xHat,yHat,thetaHat) delle variabili dell'uniciclo
  e_p = sqrt(pow(xDes-xHat,2)+pow(yDes-yHat,2));
  if (e_p > 1) // mi muovo solo se l'errore è maggiore di una certa quantità
  {
    tempo = (millis()-t0)/1000;  // tempo missione in secondi

    // assegno velocità secondo legge proporzionale (in termini delle quantità stimate!)
    v1 = -kv1*((xHat-xDes)*cos(thetaHat) + (yHat-yDes)*sin(thetaHat));

    // Calcolo l'angolo verso il target: scelgo il multiplo di 2PI 
    // più vicino all'orientamento corrente ***stimato*** del robot
    thetaDes = atan2(yDes-yHat,xDes-xHat) + nGiri*2*PI;
    if (abs(thetaDes+2*PI-thetaHat) < abs(thetaDes-thetaHat))
    {
      thetaDes = thetaDes+2*PI;
      nGiri += 1;
    }
    else
    {
      if (abs(thetaDes-2*PI-thetaHat) < abs(thetaDes-thetaHat))
      {
        thetaDes = thetaDes-2*PI;
        nGiri += -1;
      }
    }
    
   // assegno velocità angolare secondo legge proporzionale sempre in funzione delle quantità stimate   
    v2 = kv2*(thetaDes-thetaHat);
  }
  else // se penso di essere vicino al target mi fermo
  {
    v1 = 0;
    v2 = 0;
  }
  
  // Calcolo i movimenti da impartire alle ruote in base alle v1 e v2 trovate
  omegaR = (v1+v2*d/2)/r;
  omegaL = (v1-v2*d/2)/r;
  uRe = r*omegaR*dt; // spostamento comandato alla ruota destra (da considerarsi anche come informazione odometrica disponibile sul suo spostamento in (t,t+dt))
  uLe = r*omegaL*dt; // spostamento comandato alla ruota sinistra (da considerarsi anche come informazione odometrica disponibile sul suo spostamento in (t,t+dt))
  // Perturbo i due movimenti: gli spostamenti reali delle ruote non saranno mai esattamente uguali a quelli comandati 
  stDevR = sqrt(KR*abs(uRe));
  stDevL = sqrt(KL*abs(uLe));  
  uR = uRe + Gaussian(0,stDevR); // Spostamento vero ruota destra
  uL = uLe + Gaussian(0,stDevL); // Spostamento vero ruota sinistra
  
  // Dinamica effettiva dell'uniciclo
  x = x + ((uR+uL)/2)*cos(theta);
  y = y + ((uR+uL)/2)*sin(theta);
  theta = theta + (uR-uL)/d;

  // STIMA FILTRO KALMAN ESTESO: PASSO di PREDIZIONE
  xHatMeno = xHat + ((uRe+uLe)/2)*cos(thetaHat);
  yHatMeno = yHat + ((uRe+uLe)/2)*sin(thetaHat);
  thetaHatMeno = thetaHat + (uRe-uLe)/d;

  //Aggiorno la giacobiana F (solo gli elementi variabili)
  F[0][2] = -(uRe+uLe)*sin(thetaHat)/2;
  F[1][2] = (uRe+uLe)*cos(thetaHat)/2;
  
  // Aggiorno W (solo gli elementi variabili)
  W[0][0] = .5*cos(thetaHat);
  W[0][1] = .5*cos(thetaHat);
  W[1][0] = .5*sin(thetaHat);
  W[1][1] = .5*sin(thetaHat);

  //Aggiorno Q (solo gli elementi variabili)
  Q[0][0] = KR*abs(uRe);
  Q[1][1] = KL*abs(uLe);
  
  // Calcolo matrice covarianza a priori
  Pmeno = mSum(mProd(mProd(F,P),trasposta(F)),mProd(mProd(W,Q),trasposta(W))); // Pmeno = F*P*F' + W*Q*W'

 // STIMA FILTRO DI KALMAN ESTESO: PASSO di CORREZIONE
  if (millis()-tempoUltimaMisura >= tStep && nV != 0) // attuo la correzione solo se ho le misure (che arrivano ogni tStep ms)
  {
    tempoUltimaMisura = millis(); // memorizzo il tempo in cui ho fatto l'ultima misura
    nStime++; // incremento il contatore delle stime fatte
    
    // per ogni landmark calcolo misura vera, attesa, la riga della 
    // matrice giacobiana H e l'innovazione corrispondente
    for (int i=0,j=0; i<nL; i++)
    {
      if(visibile[i] == 1){
        misureLandmark[j] = atan2(Landmark[i][1]-y, Landmark[i][0]-x)-theta + Gaussian(0,sigmaLandmark);
        misureAtteseLandmark[j] = atan2(Landmark[i][1]-yHatMeno, Landmark[i][0]-xHatMeno)-thetaHatMeno;
        DeltaX = Landmark[i][0]-xHatMeno;
        DeltaY = Landmark[i][1]-yHatMeno;
        DeltaXY = DeltaY/DeltaX;
        H[j][0] = DeltaY/(pow(DeltaX,2)*(1+pow(DeltaXY,2)));
        H[j][1] = -1/(DeltaX*(1+pow(DeltaXY,2)));
        H[j][2] = -1;
        innovazione[j][0] = atan2(sin(misureLandmark[j]),cos(misureLandmark[j]))-atan2(sin(misureAtteseLandmark[j]),cos(misureAtteseLandmark[j]));
        j++;
      }
    }
    
    // Calcolo guadagno Kalman e aggiorno covarianza
    K = mProd(mProd(Pmeno,trasposta(H)),invMat(mSum(mProd(mProd(H,Pmeno),trasposta(H)),Rs)));
    P = mProd(mSum(idMat(3,1),mProd(idMat(3,-1),mProd(K,H))),Pmeno);

    // Correggo la stima    
    correzione = mProd(K,innovazione);
    xHat = xHatMeno + correzione[0][0];
    yHat = yHatMeno + correzione[1][0];
    thetaHat = thetaHatMeno + correzione[2][0];    
  }

  else  // se non ho misure non correggo nulla
  {
    xHat = xHatMeno;
    yHat = yHatMeno;
    thetaHat = thetaHatMeno;
    P = Pmeno;
  }
// FINE EKF

// Disegno il robot vero e quello stimato
  robot(x,y,theta,1); // l'argomento 1 fa un robot rosso (robot reale)
  robot(xHat,yHat,thetaHat,0); // l'argomento 0 un robot giallo (robot nella posa stimata)

// Disegno i landmark con dei triangoli bianchi col contorno rosso e l'identificativo del landmark all'interno
  stroke(255,0,0);
  strokeWeight(2);  
  for (int indLandmark=0; indLandmark<nL;indLandmark++)
  {
    if(ldi[indLandmark] == 1)
    {
      if(visibile[indLandmark]==1){
        fill(255,255,255);
      }else{
        fill(150,150,150);
      }
    }else{
      fill(52,52,52);
    }
    triangle(Landmark[indLandmark][0]-15,-Landmark[indLandmark][1]+15,Landmark[indLandmark][0]+15,-Landmark[indLandmark][1]+15,Landmark[indLandmark][0],-Landmark[indLandmark][1]-15);
    textSize(10);
    fill(0,0,0);
    text("L",Landmark[indLandmark][0]-5,-Landmark[indLandmark][1]+8);
    text(indLandmark+1,Landmark[indLandmark][0]+1,-Landmark[indLandmark][1]+8);
  }
  stroke(0);
  strokeWeight(1);
  
  popMatrix();

  textSize(20);
  fill(0,0,255);
  text("v1 (pixel/s) = ",10,20); 
  text(v1,200,20);
  text("v2 (gradi/s) = ",10,50); 
  text(v2*180/PI,200,50);
  
  fill(255,0,0);  
  text("x = ",10,160); 
  text(x,80,160);
  text("y = ",10,190); 
  text(y,80,190);
  text("theta = ",10,220); 
  text(theta*180/PI,100,220);  

  fill(255,255,255);
  text("tempo = ",10,110); 
  text(tempo,120,110);  

  fill(0,0,255);
  text("omegaR (gradi/s) = ",650,20); 
  text(omegaL*180/PI,900,20);
  text("omegaL (gradi/s) = ",650,50); 
  text(omegaL*180/PI,900,50);
  
  fill(255,0,0);
  text("xDes = ",700,100); 
  text(xDes,800,100);
  text("yDes = ",700,130); 
  text(yDes,800,130);

  fill(255,255,0);  
  text("xHat = ",10,280); 
  text(xHat,120,280);
  text("yHat = ",10,310); 
  text(yHat,120,310);
  text("thetaHat = ",10,340); 
  text(thetaHat*180/PI,160,340);  

  fill(255,255,255);
  text("nStime = ",10,390); 
  text(nStime,120,390);  

  fill(255,255,255);
  text("tStep (ms) = ",10,420); 
  text(tStep,150,420);  

  fill(255,255,0);  
  text("P = ",10,460); 
  text(P[0][0],10,490); text(P[0][1],100,490); text(P[0][2],190,490);
  text(P[1][0],10,520); text(P[1][1],100,520); text(P[1][2],190,520); 
  text(P[2][0],10,550); text(P[2][1],100,550); text(P[2][2],190,550);   
  
  fill(0, 255, 255);
  text("rMax = ", 10, 580);
  text(rMax, 70, 580);
  text("betaMax = ", 10, 610);
  text(betaMax, 95, 610);
  text("vi1 = ", 10, 630);
  text(visibile[0], 95, 630);
  text("vi2 = ", 10, 650);
  text(visibile[1], 95, 650);
  text("vi3 = ", 10, 670);
  text(visibile[2], 95, 670);
  
  text("angolo 1: ", 10, 690);
  text(atan2(sin(atan2(Landmark[0][1]-y, Landmark[0][0]-x)-theta),cos(atan2(Landmark[0][1]-y, Landmark[0][0]-x)-theta))*180/PI, 100, 690);
  text("Angolo 2: ", 10, 710);
  text(atan2(sin(atan2(Landmark[1][1]-y, Landmark[1][0]-x)-theta),cos(atan2(Landmark[1][1]-y, Landmark[1][0]-x)-theta))*180/PI, 100, 710);
  text("Angolo 3: ", 10, 730);
  text(atan2(sin(atan2(Landmark[2][1]-y, Landmark[2][0]-x)-theta),cos(atan2(Landmark[2][1]-y, Landmark[2][0]-x)-theta))*180/PI, 100, 730);
}

void robot(float x, float y, float theta, int colore)
{
  pushMatrix();
  translate(x,-y);
  rotate(-theta);
  if (colore==1)
  {
    fill(0,100,1);
    arc(0, 0, rMax, rMax, -betaMax * PI/180 , betaMax * PI/180, PIE);
    fill(255,0,0);
  }
  else
  {
    fill(255,255,0);
  }
  
  ellipse(0,0,2*R,2*R); // il robot
  fill(0,0,255);  
  rect(-r,-d/2-w/2,2*r,w);
  rect(-r,d/2-w/2,2*r,w);
  fill(255);
  ellipse(.8*R,0,.2*R,.2*R);
  ellipse(-.8*R,0,.2*R,.2*R);  
  fill(0,255,0);
  triangle(-.1*R,.3*R,-.1*R,-.3*R,.5*R,0);
  
  popMatrix();
  stroke(0,0,0);
}





/******************************************************
/******************************************************
  DA QUI IN POI CI SONO FUNZIONI DI SERVIZIO: 
  1) CALCOLO ERRORE GAUSSIANO
  2) ALGEBRA MATRICIALE
/******************************************************
/******************************************************/

float Gaussian(float media, float stDev) // Restituisce variabile N(media,stDev^2) approssimata sfruttando il teorema del limite centrale
{
  float somma = 0;
  for (int k=0; k<27; k+=1) // 27 in modo che sqrt(3/27)=1/3
  {
    somma = somma + random(-stDev/3,stDev/3);
  }
  return media + somma;
}

float[][] mProd(float[][] A,float[][] B) // Calcola prodotto di due matrici A e B (si assume, senza controllarlo, che numero colonne A = numero righe B!)
{
  int nA = A.length;
  int nAB = A[0].length;
  int nB = B[0].length;
  
  float[][] C = new float[nA][nB]; 

  for (int i=0; i < nA; i++) 
  {
    for (int j=0; j < nB; j++) 
    {  
      for (int k=0; k < nAB; k++) 
      {
        C[i][j] += A[i][k] * B[k][j];
      }
    }
  }
  return C;
}

float[][] mSum(float[][] A,float[][] B) // Calcola la somma di due matrici A e B (si assume, senza controllarlo, che A e B abbiano stesse dimensioni!)
{
  int nA = A.length;
  int nB = A[0].length;
  
  float[][] C = new float[nA][nB]; 

  for (int i=0; i < nA; i++) 
  {
    for (int j=0; j < nB; j++) 
    {  
      C[i][j] = A[i][j] + B[i][j];
    }
  }
  return C;
}

float[][] trasposta(float[][] A) // Calcola la trasposta di una matrice A
{
  int nR = A.length;
  int nC = A[0].length; 
  
  float[][] C = new float[nC][nR]; 

  for (int i=0; i < nC; i++) 
  {
    for (int j=0; j < nR; j++) 
    {  
      C[i][j] = A[j][i];
    }
  }
  return C;
}


float[][] minore(float[][] A, int i, int j) // Determina il minore (i,j) di una matrice A (si assume, senza controllarlo, che A sia quadrata!)
{
  int nA = A.length;
  float[][] C = new float[nA-1][nA-1];
  
  for (int iM = 0; iM < i; iM++)
  {
    for (int jM = 0; jM < j; jM++)
    {
      C[iM][jM] = A[iM][jM];
    } 
    for (int jM = j; jM < nA-1; jM++)
    {
      C[iM][jM] = A[iM][jM+1];
    } 
  }
  for (int iM = i; iM < nA-1; iM++)
  {
    for (int jM = 0; jM < j; jM++)
    {
      C[iM][jM] = A[iM+1][jM];
    } 
    for (int jM = j; jM < nA-1; jM++)
    {
      C[iM][jM] = A[iM+1][jM+1];
    } 
  }
  return C;
}


float det(float[][] A) // Calcola il determinante di A (si assume, senza controllarlo, che A sia quadrata!)
{
  int nA = A.length;
  float determinante = 0;
  
  if (nA == 1)
  {
    determinante = A[0][0];
  }
  else
  {
    for (int j=0; j < nA; j++) 
    {
      determinante = determinante + A[0][j]*pow(-1,j)*det(minore(A,0,j));
    }
  }
  return determinante;
}


float[][] invMat(float[][] A) // Calcola l'inversa di una matrice A (si assume, senza controllarlo, che A sia quadrata!)
{
  int nA = A.length;
  float[][] C = new float[nA][nA];
  float detA = det(A);

/*
  if (abs(detA)<0.001) // Per evitare casi singolari con determinanti vicini a 0
  {
    if (detA>0)
    {
      detA = 0.001;
    }
    else
    {
      detA = -0.001;
    }
  }
*/  
  
  if (nA == 1)
  {
    C[0][0] = 1/detA;
  }
  else
  {
    for (int i=0; i < nA; i++) 
    {
      for (int j=0; j < nA; j++) 
      {
        C[j][i] = pow(-1,i+j)*det(minore(A,i,j))/detA;
      }
    }
  }
  return C;
}

float[][] idMat(int nA, float sigma) // Assegna una matrice identità di ordine nA moltiplicata per una costante sigma
{
  float[][] I = new float[nA][nA]; 

  for (int i=0; i < nA; i++) 
  {
    for (int j=0; j < nA; j++) 
    {  
      I[i][j] = 0;
    }
    I[i][i] = sigma;
  }
  return I;
}
