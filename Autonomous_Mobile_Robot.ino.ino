#include <Astar.h>
#include <Vector.h>
#include <SPI.h>
#include <MFRC522.h>

#define trigger1 10
#define echo1 9

#define trigger2 12
#define echo2 11

#define ROW 5
#define COL 5

#define RST_PIN         5           
#define SS_PIN          53 
// SDA - BLUE - 53
// SCK - MAROON - 52
// MOSI - YELLOW - 51
// MISO - ORANGE - 50
// RST - GREEN - 5

#define S0 40   // ORANGE
#define S1 41   // YELLOW
#define S2 42   // GREEN
#define S3 43   // BLUE
#define sOut 44 // GRAY

int red, green, blue;

MFRC522 mfrc522(SS_PIN, RST_PIN);   // Create MFRC522 instance

// card UID's
String cardUID[ROW][COL] = {
                        { "CA FA 03 CB", "3B 06 44 0C", "8B 12 42 0C", "2B A8 51 0C", "4A 85 48 E0" },
                        { "DB 0C 45 0C", "BB 23 4E 0C", "9B 0D 46 0C", "8B DF 49 0C", "2A EA 08 CB" },
                        { "9A CB 0E CB", "FB 36 53 0C", "EB 2E 50 0C", "EE 57 12 1F", "EA 2A 04 CB" },
                        { "4B 09 47 0C", "2B 71 4F 0C", "AB A8 95 0B", "DA 72 80 5B", "5B B3 47 0C" },
                        { "DA DE 49 E0", "3A 36 0A CB", "DA 33 40 E0", "40 6F 4B A7", "BB 79 44 0C" }
                      };

Astar astar(ROW, COL);

// Motor A pins
int enA = 3;
int in1 = 23;
int in2 = 25;

// Motor B pins
int enB = 2;
int in3 = 22;
int in4 = 24;

// Motor PWM
int pwm = 255;

// IR Sensor pins
int s1 = 26;
int s2 = 27;
int s3 = 28;
int s4 = 29;
int s5 = 30;
int distanceSensor = 31;

Vector A;
Vector B;
Vector Storage;

uint8_t buf[10]= {};
MFRC522::Uid id;
MFRC522::Uid id2;
bool is_card_present = false;
uint8_t control = 0x00;

void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  SPI.begin();                                                  // Init SPI bus
  mfrc522.PCD_Init(); 

  // Initialize Motors
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  pinMode(13,  OUTPUT);

  // Initialize IR Sensors
  pinMode(s1, INPUT);
  pinMode(s2, INPUT);
  pinMode(s3, INPUT);
  pinMode(s4, INPUT);
  pinMode(s5, INPUT);

  // Initialize Distance Sensor
  pinMode(distanceSensor, INPUT);

  pinMode(trigger1, OUTPUT);
  pinMode(echo1, INPUT);

  pinMode(trigger2, OUTPUT);
  pinMode(echo2, INPUT);

  // Setting color sensor output
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);

  // Setting color sensor input
  pinMode(sOut, INPUT);

  // color sensor frequency scaling
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);

  A = getCardCoordinates("3B 06 44 0C");
  B = getCardCoordinates("2B A8 51 0C");
  Storage = getCardCoordinates("DA DE 49 E0");

  Serial.println("Setup complete!");
}

bool check = false;
void loop() 
{
  // put your main code here, to run repeatedly:
  
  while(!Destination(Storage)) {}

  delay(5000);
  
  while(!Destination(B)) {}
}

void Am()
{
  if(Serial.available() > 0)
  {
    Vector Goal;
    char ch = Serial.read();

    if(ch == 'A')
      Goal = A;
    else if(ch == 'B')
      Goal = B;

    // Go to Storage
    while(!Destination(Storage)) {}

    // Deliver 
    while(!Serial.available() > 0)
    {
      
    }

    Stop();

    while(!Destination(Goal)) {}
  }
}

bool Destination(Vector Goal)
{
  analogWrite(enA, pwm);
  analogWrite(enB, pwm);
  
  while(!tagPresent())
  {
    ForwardRobot();
  }

  String UID = GetHex(id.uidByte, id.size);

  Stop();

  Vector Start = getCardCoordinates(UID);
  Vector End   = Goal;

  bool found = astar.FindPath(Start, End);

  if(found)
  {
    String prev = "";
    for(int i=0; i<astar.path.Count; i++)
    {
        String card = "_";

        bool flag = false;
        while(!flag)
        {
          bool obstacle = hasObstacle(trigger1, echo1);

          if(obstacle)
          {
            while(!tagPresent())
            {
              BackwardRobot();
            }
            
            Stop();

            int X = astar.path[i]->coordinates.X;
            int Y = astar.path[i]->coordinates.Y;
            
            Node* node = &astar.nodes[Y][X];
            node->isWalkable = false;

            return false;
          }
          else if(tagPresent())
          {
            card = GetHex(id.uidByte, id.size);
          
            if(card == prev)
            {
              ForwardRobot();
            }
            else
            {
              Stop();
              flag = true;
            }
          }
          else
          {
            ForwardRobot();
          }
        }

        if(card == cardUID[astar.path[i]->coordinates.Y][astar.path[i]->coordinates.X])
        {
          if(card == cardUID[Goal.Y][Goal.X])
          {
            Stop();

            return true;
          }
          else
          {
              prev = card;
              Vector current = getCardCoordinates(card);
              Vector next = astar.path[i + 1]->coordinates;

              if(getSensorColor() == getCurrentColor(current,next))
              {
                analogWrite(enA, pwm);
                analogWrite(enB, pwm);
              }
              else
              {
                while(1)
                {
                  analogWrite(enA, 100);
                  analogWrite(enB, 100);
                
                  if(digitalRead(s3) == 0)
                  {
                    Stop();

                    if(getSensorColor() == getCurrentColor(current,next))
                    {
                      analogWrite(enA, pwm);
                      analogWrite(enB, pwm);
                      break;
                    }
                    else
                    {
                      TurnRight();

                      delay(50);

                      Stop();

                      delay(50);
                    }
                  }
                  else
                  {
                    TurnRight();

                    delay(50);

                    Stop();

                    delay(50);
                  }
                }
              }
          
              Stop();

              delay(1000);
          }
        }
        else
        {
          return false;
        }
    }
    
    return true;
  }

  astar.ResetMap();
  return false;
}

//*****************************************************************************************//

int getSensorColor()
{
    digitalWrite(S2, LOW);
    digitalWrite(S3, LOW);

    red = pulseIn(sOut, LOW);
    //delay(20);

    digitalWrite(S2, HIGH);
    digitalWrite(S3, HIGH);

    green = pulseIn(sOut, LOW);
    //delay(20);

    digitalWrite(S2, LOW);
    digitalWrite(S3, HIGH);

    blue = pulseIn(sOut, LOW);
    //delay(20);
    
    red = map(red, 39, 225, 255, 0);
    green = map(green, 43, 261, 255, 0);
    blue = map(blue, 41, 196, 255, 0);

    red = constrain(red, 0, 255);
    green = constrain(green, 0, 255);
    blue = constrain(blue, 0, 255);

    Serial.print(red);
    Serial.print(" ");
    Serial.print(green);
    Serial.print(" ");
    Serial.print(blue);
    Serial.println();
    Serial.println();  

    if(red >= 250 && green >= 250 && blue >= 250)
    {
      Serial.println("White");
      return 0;
    }
    else if(red < 200 && green < 200 && blue < 200)
    {
      Serial.println("Black");
      return -1;
    }
    else if(red > green && red > blue)
    {
      Serial.println("RED");
      return 1;
    }
    else if(green > red && green > blue)
    {
      Serial.println("GREEN");
      return 2;
    }
    else if(blue > red && blue > green)
    {
      Serial.println("BLUE");
      return 3;
    }
      
    return -2;    
}

//*****************************************************************************************//

int getCurrentColor(Vector current, Vector next)
{
  // UP
  if(current.X == next.X && current.Y > next.Y)
  {
    return 3;
  }

  // DOWN
  if(current.X == next.X && current.Y < next.Y)
  {
    return 1;
  }

  // LEFT
  if(current.X > next.X && current.Y == next.Y)
  {
    return 2;
  }

  // RIGHT
  if(current.X < next.X && current.Y == next.Y)
  {
    return 0;
  }
}

//*****************************************************************************************//

String GetHex(uint8_t *data, uint8_t length) // gets 8-bit data in hex
{
     char tmp[16];
     String str = "";
     for (int i=0; i<length; i++) 
     { 
        sprintf(tmp, "%.2X",data[i]); 

        if(i == length - 1)
          str += String(tmp);
        else
          str += String(tmp) + " ";
     }
     return str;
}

//*****************************************************************************************//

void cpid(MFRC522::Uid *id)
{
  memset(id, 0, sizeof(MFRC522::Uid));
  memcpy(id->uidByte, mfrc522.uid.uidByte, mfrc522.uid.size);
  id->size = mfrc522.uid.size;
  id->sak = mfrc522.uid.sak;
}

//*****************************************************************************************//

bool tagPresent()
{
  MFRC522::MIFARE_Key key;
  for (byte i = 0; i < 6; i++) key.keyByte[i] = 0xFF;
  MFRC522::StatusCode status;
    
  if ( !mfrc522.PICC_IsNewCardPresent()) 
  {
    return false;
  }
    
  if ( !mfrc522.PICC_ReadCardSerial()) 
  {
    return false;
  }

    bool result = true;
    uint8_t buf_len=4;
    cpid(&id);

    while(true)
    {
      control = 0;
      for(int i=0; i<3; i++)
      {
        if(!mfrc522.PICC_IsNewCardPresent())
        {
          if(mfrc522.PICC_ReadCardSerial())
          {
            //Serial.print('a');
            control |= 0x16;
          }
          if(mfrc522.PICC_ReadCardSerial())
          {
            //Serial.print('b');
            control |= 0x16;
          }
            //Serial.print('c');
            control += 0x1;
        }
        //Serial.print('d');
        control += 0x4;
      }
    
      //Serial.println(control);
      if(control == 13 || control == 14)
      {
        //card is still there
        return true;
      }
      else 
      {
        return false;
      }
    }
}

//*****************************************************************************************//

Vector getCardCoordinates(String UID)
{
  Vector coordinates;
  for(int row=0; row<ROW; row++)
  {
    for(int col =0; col<COL; col++)
    {
      if(cardUID[row][col] == UID)
      {
        coordinates.X = col;
        coordinates.Y = row;

        return coordinates;
      }
    }
  }
}

//*****************************************************************************************//

void ForwardRobot()
{ 
  int L  = digitalRead(s1);
  int AL = digitalRead(s2);
  int M  = digitalRead(s3);
  int AR = digitalRead(s4);
  int R  = digitalRead(s5);
  
  if(L == 1 && AL == 1 && M == 1 && AR == 1 && R == 0)
  {
    TurnRight();
  }
  if(L == 1 && AL == 1 && M == 1 && AR == 0 && R == 1)
  {
    TurnRight();
  }
  if(L == 1 && AL == 1 && M == 1 && AR == 0 && R == 0)
  {
    TurnRight();
  }
  if(L == 1 && AL == 1 && M == 0 && AR == 0 && R == 1)
  {
    TurnRight();
  }
  if(L == 0 && AL == 1 && M == 1 && AR == 1 && R == 1)
  {
    TurnLeft();
  }
  if(L == 1 && AL == 0 && M == 1 && AR == 1 && R == 1)
  {
    TurnLeft();
  }
  if(L == 0 && AL == 0 && M == 1 && AR == 1 && R == 1)
  {
    TurnLeft();
  }
  if(L == 1 && AL == 0 && M == 0 && AR == 1 && R == 1)
  {
    TurnLeft();
  }
  if(L == 0 && AL == 0 && M == 0 && AR == 0 && R == 0)
  {
    Forward();
  }
  if(L == 1 && AL == 1 && M == 0 && AR == 0 && R == 0)
  {
    Forward();
  }
  if(L == 0 && AL == 0 && M == 0 && AR == 1 && R == 1)
  {
    Forward();
  }
  if(L == 1 && AL == 1 && M == 1 && AR == 1 && R == 1)
  {
    Forward();
  }
  if(L == 1 && AL == 1 && M == 0 && AR == 1 && R == 1)
  {
    Forward();
  }
  if(L == 0 && AL == 0 && M == 0 && AR == 0 && R == 0)
  {
    Forward();
  }
  if(L == 1 && AL == 1 && M == 0 && AR == 0 && R == 0)
  {
    Forward();
  }
  if(L == 0 && AL == 0 && M == 0 && AR == 1 && R == 1)
  {
    Forward();
  }
  if(L == 1 && AL == 1 && M == 1 && AR == 1 && R == 1)
  {
    Forward();
  }
}

void BackwardRobot()
{
  int L  = digitalRead(s1);
  int AL = digitalRead(s2);
  int M  = digitalRead(s3);
  int AR = digitalRead(s4);
  int R  = digitalRead(s5);
  
  if(L == 1 && AL == 1 && M == 1 && AR == 1 && R == 0)
  {
    TurnRight();
  }
  if(L == 1 && AL == 1 && M == 1 && AR == 0 && R == 1)
  {
    TurnRight();
  }
  if(L == 1 && AL == 1 && M == 1 && AR == 0 && R == 0)
  {
    TurnRight();
  }
  if(L == 1 && AL == 1 && M == 0 && AR == 0 && R == 1)
  {
    TurnRight();
  }
  if(L == 0 && AL == 1 && M == 1 && AR == 1 && R == 1)
  {
    TurnLeft();
  }
  if(L == 1 && AL == 0 && M == 1 && AR == 1 && R == 1)
  {
    TurnLeft();
  }
  if(L == 0 && AL == 0 && M == 1 && AR == 1 && R == 1)
  {
    TurnLeft();
  }
  if(L == 1 && AL == 0 && M == 0 && AR == 1 && R == 1)
  {
    TurnLeft();
  }
  if(L == 0 && AL == 0 && M == 0 && AR == 0 && R == 0)
  {
    Backward();
  }
  if(L == 1 && AL == 1 && M == 0 && AR == 0 && R == 0)
  {
    Backward();
  }
  if(L == 0 && AL == 0 && M == 0 && AR == 1 && R == 1)
  {
    Backward();
  }
  if(L == 1 && AL == 1 && M == 1 && AR == 1 && R == 1)
  {
    Backward();
  }
  if(L == 1 && AL == 1 && M == 0 && AR == 1 && R == 1)
  {
    Backward();
  }
  if(L == 0 && AL == 0 && M == 0 && AR == 0 && R == 0)
  {
    Backward();
  }
  if(L == 1 && AL == 1 && M == 0 && AR == 0 && R == 0)
  {
    Backward();
  }
  if(L == 0 && AL == 0 && M == 0 && AR == 1 && R == 1)
  {
    Backward();
  }
  if(L == 1 && AL == 1 && M == 1 && AR == 1 && R == 1)
  {
    Backward();
  }
}

void Forward()
{
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void Backward()
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void TurnRight()
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void TurnLeft()
{
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void Stop()
{
  Serial.println("Stop");
  
  analogWrite(enA, pwm);
  analogWrite(enB, pwm);
  
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

bool hasObstacle(int trigger, int echo)
{
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);

  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);

  digitalWrite(trigger, LOW);
  float duration = pulseIn(echo, HIGH);

  // if an obstacle is detected within 10cm distance, return true otherwise false
  return (duration / 58.2 <= 10) ? true : false;
}
