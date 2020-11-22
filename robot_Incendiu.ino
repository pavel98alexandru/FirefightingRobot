#include <AFMotor.h>
#include <SoftwareSerial.h>
#include "Timer.h"


SoftwareSerial BlueTooth(A0, A0); //Serial-ul necesar conexiunii bluetooth
AF_DCMotor motor1(1); //motorul stang
AF_DCMotor motor2(2); //motorul drept
AF_DCMotor pompa(3); //pompa de apa

boolean flagStart = true;
const int buzzer = 9;
int comanda = 6;
int comandaAnterioara = 6;
boolean flagFoc = false;
boolean flagAlarma = false;

void setup() {
  BlueTooth.setTimeout(50);
  BlueTooth.begin(9600);
  Serial.begin(9600);
  motor1.setSpeed(255);
  motor1.run(RELEASE);
  motor2.setSpeed(255);
  motor2.run(RELEASE);
  pompa.setSpeed(150);
  pompa.run(RELEASE);
  pinMode(A0, INPUT);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  pinMode(buzzer, OUTPUT);
  digitalWrite(buzzer, HIGH);
}

void controlRC() //controlul robotului de la telecomanda
{
  switch (comanda)
  {
    case 0: //stanga
      Serial.println("Stanga");
      if (comandaAnterioara == 1)
      {
        //cazul in care roobtul mergea la dreapta, acum opreste
        motor1.run(RELEASE);
        motor2.run(RELEASE);
        comandaAnterioara = 6;
      }
      else
      {
        motor1.run(BACKWARD);
        motor2.run(FORWARD);
        comandaAnterioara = comanda;
      }
      comanda = 6;
      break;
    case 1: //dreapta
    Serial.println("Dreapta");
      if (comandaAnterioara == 0)
      {
        //cazul in care roobtul mergea la stanga, acum opreste
        motor1.run(RELEASE);
        motor2.run(RELEASE);
        comandaAnterioara = 6;
      }
      else
      {
        motor1.run(FORWARD);
        motor2.run(BACKWARD);
        comandaAnterioara = comanda;
      }
      comanda = 6;
      break;
    case 2://inainte
      if (comandaAnterioara == 3)
      {
        //cazul in care roobtul mergea inapoi, acum opreste
        motor1.run(RELEASE);
        motor2.run(RELEASE);
        comandaAnterioara = 6;
      }
      else
      {
        motor1.run(BACKWARD);
        motor2.run(BACKWARD);
        comandaAnterioara = comanda;
      }
      comanda = 6;
      break;
    case 3: //inapoi
      if (comandaAnterioara == 2)
      {
        //cazul in care roobtul mergea inainte, acum opreste
        motor1.run(RELEASE);
        motor2.run(RELEASE);
        comandaAnterioara = 6;
      }
      else
      {
        motor1.run(FORWARD);
        motor2.run(FORWARD);
        comandaAnterioara = comanda;
      }
      comanda = 6;
      break;
    case 4: //alarma off
      flagAlarma = false;
      comanda = comandaAnterioara;
      comandaAnterioara = 6;
      break;
    case 5: //alarma on
      flagAlarma = true;
      comanda = comandaAnterioara;
      comandaAnterioara = 6;
      break;

  }
}

void alarma() //alarma de incendiu, activeaza buzzer-ul si led-urile legate la pinii A1 si A2
{
  analogWrite(buzzer, 450);
  digitalWrite(A1, HIGH);
  delay(500);
  analogWrite(buzzer, 250);
  digitalWrite(A1, LOW);
  digitalWrite(A2, HIGH);
  delay(500);
  digitalWrite(A2, LOW);
  digitalWrite(buzzer, HIGH);
}

void focStanga()
{
  int timer=0;//necesar pentru a nu ne rotii mai mult de marginea opusa
  motor1.setSpeed(255);
  motor1.run(RELEASE);
  motor2.setSpeed(255);
  motor2.run(RELEASE);
  while (timer<=10)
  { 
    //ne rotim pana cand focul e aliniat cu senzorul de pe A4
    motor1.run(BACKWARD);
    motor2.run(FORWARD);
    delay(100);
    motor1.run(RELEASE);
    motor2.run(RELEASE);
    if (analogRead(A4) <= 35)
    {
       //focul e aliniat
      focCentru();
      break;
    }
    timer++;
  }
}

void focCentru()
{
  motor1.run(BACKWARD);
    motor2.run(BACKWARD);
    delay(50);
    motor1.run(RELEASE);
    motor2.run(RELEASE);
  while (analogRead(A4) < 400)
  {
    pompa.run(FORWARD);
    delay(2000);
    pompa.run(RELEASE);
    pompa.setSpeed(200);
    pompa.run(FORWARD);
    delay(2000);
    pompa.run(RELEASE);
    pompa.setSpeed(255);
    pompa.run(FORWARD);
    delay(2000);
    pompa.setSpeed(150);
  }
  pompa.run(RELEASE);
}

void focDreapta()
{
  int counter=0; //necesar pentru a nu ne rotii mai mult de marginea opusa
  motor1.setSpeed(255);
  motor1.run(RELEASE);
  motor2.setSpeed(255);
  motor2.run(RELEASE);
  while (counter<=10)
  {
    //ne rotim pana cand focul e aliniat cu senzorul de pe A4
    motor1.run(FORWARD);
    motor2.run(BACKWARD);
    delay(100);
    motor1.run(RELEASE);
    motor2.run(RELEASE);
    if (analogRead(A4) <= 35)
    {
      //focul e aliniat
      focCentru();
      break;
    }
    counter++;
  }
}
\
void autoPilot() //program de autopilotare in cazul in care robotul a detectat foc
{
  for (int i = 0; i < 4; i++) //alarma de incendiu e activata
  {
    alarma();
  }

  
  int senzor1 = analogRead(A3);
  int senzor2 = analogRead(A4);
  int senzor3 = analogRead(A5);
  int closer;   //cautam senzorul cel mai apropiat de foc, cu prioritate pe senzorul din mijloc (de pe pinul A4)
  
  if (senzor2 < 40)
  {
    closer = senzor2;
  }
  else
  {
    closer = min(min(senzor1, senzor2), senzor3);
  }
  if (closer == senzor1) //rotim masina incat focul sa fie aliniat cu senzorul din mijloc
  {
    focStanga();
  }
  if (closer == senzor2) // senzorul central e aliniat cu focul, stingem incendiul
  {
    focCentru();
  }
  if (closer == senzor3) //rotim masina incat focul sa fie aliniat cu senzorul din mijloc
  {
    focDreapta();
  }
  flagFoc = false;
}

void loop() {
  if (flagAlarma) //daca s-a apasat butonul de alarama din aplicatie sirenele se aprind/sting
  {
    alarma();
  }
  else //ne asigurama ca sirenele sunt stinse si buzzer-ul e inactiv
  {
    digitalWrite(buzzer, HIGH);
    digitalWrite(A1, LOW);
    digitalWrite(A2, LOW);
  }
  if (flagStart) //sectiunea de pornire a robotului, ruleaza o singura data la inceput
  {
    flagStart = false;
    for (int i = 0; i < 10; i++)
    {
      analogWrite(buzzer, i * 50);
      digitalWrite(A1, HIGH);
      delay(100);
      digitalWrite(A1, LOW);
      digitalWrite(A2, HIGH);
      delay(100);
      digitalWrite(A2, LOW);
      digitalWrite(buzzer, HIGH);
    }
  }
  if (analogRead(A3) < 30 || analogRead(A4) < 30 || analogRead(A5) < 30) //daca detectam flacara aproape de unul dintre cei 2 senzori, setam flagFoc la true
  {
    motor1.run(RELEASE);
    motor2.run(RELEASE);
    flagFoc = true;
  }
  else
  {
    flagFoc = false;
  }
  if (BlueTooth.available() && !flagFoc) //daca serialul Bluetooth-ului e disponibil SI FOCUL NU A FOST DETECTAT controlam robotul
  {
    comanda = BlueTooth.read();
    controlRC();
  }
  if (flagFoc) //daca focul a fost detectat, robotul intra in modul autopilot
  {
    motor1.run(BACKWARD);
    motor2.run(BACKWARD);
    delay(100);
    motor1.run(RELEASE);
    motor2.run(RELEASE);
    autoPilot();
  }
}
