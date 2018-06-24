/* FUGA 177
   Modified by Sathyajith
   05/09/2017
*/

int pwm_right = 5;
int pwm_left  = 10;
int  s1, s2, s3, s4, s5, s6;
char lastreading = 'l';
float basespeed = 55, Kp = 23, Kd = 3.5;
unsigned int leftpulse, rightpulse;
float error = 0 , terror ;
float PP, II, DD, correction;
float Ki = 0.15;
int s = 3;

void LMN()
{
  digitalWrite(9, LOW);
  digitalWrite(8, LOW);
}

void LMS()
{
  digitalWrite(9, HIGH);
  digitalWrite(8, HIGH);
}
void LMB()
{
  digitalWrite(8, HIGH);
  digitalWrite(9, LOW);
}
void LMF()
{
  digitalWrite(9, HIGH);
  digitalWrite(8, LOW);
}
void RMN()
{
  digitalWrite(7, LOW);
  digitalWrite(6, LOW);
}

void RMS()
{
  digitalWrite(7, HIGH);
  digitalWrite(6, HIGH);
}
void RMB()
{
  digitalWrite(7, LOW);
  digitalWrite(6, HIGH);
}
void RMF()
{
  digitalWrite(7, HIGH);
  digitalWrite(6, LOW);
}

void sensors_input()
{
  int sensor1 = digitalRead(14);
  int sensor2 = digitalRead(15);
  int sensor3 = digitalRead(16);
  int sensor4 = digitalRead(19);
  int sensor5 = digitalRead(20);
  int sensor6 = digitalRead(21);

  if (sensor1 == 1)// sensor 1 left side
  { s1 = 1;
     //L
  }
  else {
    s1 = 0;

  }
  if (sensor2 == 1)// sensor 2
  {
    s2 = 1;
    lastreading = 'r';
  }
  else
  {
    s2 = 0;
  }

  if (sensor3 == 1)// sensor 3
  {
    s3 = 1;
  }
  else
  {
    s3 = 0;
  }

  if (sensor4 == 1)// sensor 4
  {
    s4 = 1;
  }
  else
  {
    s4 = 0;
  }

  if (sensor5 == 1)// sensor 5
  {
    s5 = 1;
  }
  else
  {
    s5 = 0;
  }

  if (sensor6 == 1)// sensor 6
  {
    s6 = 1;
    lastreading = 'l'; //R
  }
  else
  {
    s6 = 0;
  }
}

void CalcError()
{
  int sum;
  terror = error;       //(terror for previous error)
  error = ((s1 * 1) + (s2 * 2) + (s3 * 3) + (s4 * 4) + (s5 * 5) + (s6 * 6));

  sum = (s1 + s2 + s3 + s4 + s5 + s6);
  if (sum == 0)
  { sum = 1;
  }
  else
  {
    error = error / sum;
    error = error - 3.5 ;
  }
}

void stopp()
{ if (((s1 + s2 + s3 + s4 + s5 + s6) == 6))
  { LMF();
    RMF();
    analogWrite(pwm_right, 60);
    analogWrite(pwm_left, 60);
    delay(100);
    sensors_input();
    if (((s1 + s2 + s3 + s4 + s5 + s6) == 6))
    {
      while (s > 1) {
        RMS();
        LMS();
        analogWrite(pwm_right, 60);
        analogWrite(pwm_left, 60);
      }
    }
  }
  else
  {

    RMB();
    LMF();
    analogWrite(pwm_right, 60);
    analogWrite(pwm_left, 60);

    delay(60);
  }
}

void Lreading()
{
  if (lastreading == 'l') //checks if the last sensor to the activated was right
  {


    RMB();   //turn right at full speed
    LMF();
    analogWrite(pwm_right, 60);
    analogWrite(pwm_left, 60);


  }

  else if (lastreading == 'r') //checks if the last sensor to the activated was left
  {


    RMF();   //turn right at full speed
    LMB();
    analogWrite(pwm_right, 60);
    analogWrite(pwm_left, 60);

  }
}


void junction()
{

  if (((s1 + s2 + s3 + s4) >= 3) && ((s5 + s6) <= 1))
  {
    RMF();
    LMF();
    analogWrite(pwm_right, 60);
    analogWrite(pwm_left, 60);
    delay(200);

    if ((s1 + s2 + s3 + s4 + s5 + s6) == 0) {
      while (s4 == 0)
      {
        sensors_input();
        RMF();
        LMB();
        analogWrite(pwm_right, 60);
        analogWrite(pwm_left, 60);
      }
    }
    else {
     
      pid();
    }
  }
  if ( ((s1 + s2) <= 1) && ((s3 + s4 + s5 + s6) >= 3))
  {
    RMF();
    LMF();
    analogWrite(pwm_right, 60);
    analogWrite(pwm_left, 60);
    delay(200);

    if ((s1 + s2 + s3 + s4 + s5 + s6) == 0)
    {
      while (s2 == 1)
      {
        sensors_input();
        RMB();
        LMF();
        analogWrite(pwm_right, 60);
        analogWrite(pwm_left, 60);
      }
    }
    else {
      pid();
    }
  }
}

void pid()

{
  sensors_input();
//  junction();
  CalcError();
  if ((s1 + s2 + s3 + s4 + s5 + s6) == 0) //   //robot away from the road
  {
    Lreading();

  }

  else   //robot on line
  { PP = error * Kp;
    II += error;
    II = II * Ki;
    DD = (error - terror) * Kd;
    correction = PP + II + DD;
    rightpulse =  basespeed - correction;
    leftpulse = basespeed + correction;
    LMF();
    RMF();
    if (leftpulse > 100)
      leftpulse = 100;
    if (rightpulse > 100)
      rightpulse = 100;
    analogWrite(pwm_left, leftpulse);
    analogWrite(pwm_right, rightpulse);


  }
  if ((s1 + s2 + s3 + s4 + s5 + s6) == 6)
  {
    stopp();
  }
}


void setup() {

  pinMode(14, INPUT);
  pinMode(15, INPUT);
  pinMode(16, INPUT);
  pinMode(19, INPUT);
  pinMode(20, INPUT);
  pinMode(21, INPUT);

  pinMode(9, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(6, OUTPUT);
}


void loop()
{




  RMF();
  LMF();
  analogWrite(pwm_right, 60);
  analogWrite(pwm_left, 60);
  delay(600);
  while (s > 1)
  {
    pid();
  }


}

