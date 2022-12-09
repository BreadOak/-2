// Inverted Pendulum JSK //

//////// OLED ///////////
#define OLED_DC 5
#define OLED_CLK 8
#define OLED_MOSI 7
#define OLED_RESET 6

#define PWM 9
#define IN1 10
#define IN2 11

#define ENCODER_A 4
#define ENCODER_B 2

#define KEY_Memu 3
#define KEY_S 15
#define KEY_Minus 16
#define KEY_Plus 17
#define KEY_X 18

#define PI 3.141592

#include <MsTimer2.h>
#include <SSD1306.h>
#include <PinChangeInt.h> // This is the library file
#include <DATASCOPE.h>    // This is the library file
DATASCOPE data;

SSD1306 oled(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, 0);

unsigned char Send_Count = 1;

float Sensor, pwm;
float rad, deg_show;

float max_vol = 12;       // [V] 
float bit_res = pow(2,8); // PWM resolution 0~255 8bit

float ENCODER = 0;
float Enc_res = 1040.0;

float UpPos = 785;
float Target = 0;

float CartPos = 0;
float x = 0, x_speed = 0, angle = 0, angle_speed = 0;
float last_Encoder = 0, last_angle = 0, last_x_speed = 0, last_angle_speed = 0;

float k1 = , k2 = , k3 = , k4 = ; // LQR gain
float st = 0.005;                                                   // Sampling time:5[ms]

/**************************************************************************
  Function : ADC sensing
**************************************************************************/
u16 Get_Adc_Average(u8 ch, u8 times)
{
  int temp_val = 0;
  char  t;
  for (t = 0; t < times; t++)
  {
    temp_val += analogRead(ch);
  }
  return temp_val / times;    // Average
}

/**************************************************************************
  Function : ADC -> Degree
**************************************************************************/
float ADC2deg(float sensor)
{
  deg_show = (UpPos - sensor) * 360 / 1024.0;
  if (deg_show < 0)
  {
    deg_show = 360.0 - abs(deg_show);
  }
  return deg_show;
}

/**************************************************************************
Function : ADC -> rad
**************************************************************************/
float angle_count(float sensor)
{
  // Fill in the Missing Code
  static float deg;
  deg = ;
  angle = ;
  // Fill in the Missing Code
  
  return angle;
}

/**************************************************************************
  Function : LQR Controller
**************************************************************************/
float LQR_Control(int Encoder, float Sensor)
{
  float u;

  x = (Encoder/Enc_res)*2.0*PI*0.018; // (r = 0.018)
  x_speed = (((Encoder-last_Encoder)/Enc_res)*2.0*PI*0.018)/st;
  x_speed = 0.02*x_speed + 0.98*last_x_speed; // 1st order LPF
  last_Encoder = Encoder;
  last_x_speed = x_speed;

  angle = angle_count(Sensor);
  angle_speed = (angle - last_angle) / st;
  angle_speed = 0.01*angle_speed + 0.99*last_angle_speed; // 1st order LPF
  last_angle = angle;
  last_angle_speed = angle_speed;
  
  // Fill in the Missing Code
  u = ;

  return u;
}

/**************************************************************************
  Function ：Control core code
**************************************************************************/
void control()
{
  Sensor = float(Get_Adc_Average(5, 10));         // Mean filtering
  deg_show = ADC2deg(Sensor);                     // Pole angle [deg]
  CartPos = (ENCODER/Enc_res)*2.0*PI*0.018*1000;  // Cart position [mm]

  // Fill in the Missing Code
  pwm = ; 
  
  // Saturation
  if (pwm > 245)  pwm =  245;           // PWM parameter limit : 255
  if (pwm < -245) pwm = -245;           // PWM parameter limit : 255
  
  Set_PWM(-pwm);
}

void Set_PWM(int pwm)
{
  if (pwm < 0)    digitalWrite(IN1, HIGH),      digitalWrite(IN2, LOW);
  else            digitalWrite(IN1, LOW),       digitalWrite(IN2, HIGH);
  analogWrite(PWM, abs(pwm) + 10);
}

////////////////////////////////////////////////////////////////////////////////

void DataScope(void)
{
  int i;
  data.DataScope_Get_Channel_Data(Sensor, 1);   // Fist data
  data.DataScope_Get_Channel_Data(ENCODER, 2); // Second data
  Send_Count = data.DataScope_Data_Generate(2);
  for ( i = 0 ; i < Send_Count; i++)
  {
    Serial.write(DataScope_OutPut_Buffer[i]);
  }
  delay(20);
}

uint32_t oled_pow(uint8_t m, uint8_t n)
{
  uint32_t result = 1;
  while (n--)result *= m;
  return result;
}

void OLED_ShowNumber(uint8_t x, uint8_t y, uint32_t num, uint8_t len)
{
  u8 t, temp;
  u8 enshow = 0;
  for (t = 0; t < len; t++)
  {
    temp = (num / oled_pow(10, len - t - 1)) % 10;
    oled.drawchar(x + 6 * t, y, temp + '0');
  }
}

void setup()
{
  int fff = 1;
  TCCR1B = (TCCR1B & 0xF8) | fff;  // Set frequency to 31.374KHZ.
  oled.ssd1306_init(SSD1306_SWITCHCAPVCC);
  oled.clear();   // clears the screen and buffer
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(PWM, OUTPUT);
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 0);
  digitalWrite(PWM, 0);
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
  Serial.begin(115200);
  delay(200);
  MsTimer2::set(5, control);       // Use Timer2 to set 5[ms] timing interrupt
  MsTimer2::start();               // Interrupt enablement
  attachInterrupt(0, READ_ENCODER_A, CHANGE);
  attachInterrupt(1, READ_ENCODER_B, CHANGE);
}

/**************************************************************************
  Function : Main loop
**************************************************************************/
void loop()
{
  //===== Line 1 Display Encoder Data =====//
  oled.drawstring(00, 4, "CartPos:");
  OLED_ShowNumber(60, 4, abs(CartPos), 5);

  //===== Line 2 Display angular displacement sensor values & starting values =====//
  oled.drawstring(00, 5, "PoleAngle:");
  OLED_ShowNumber(60, 5, abs(deg_show), 5);

  //===== Line 3 Display angular displacement ADC values =====//
  oled.drawstring(00, 6, "ADC:");
  OLED_ShowNumber(60, 6, abs(Sensor), 5);

  //===== Refresh =====//
  oled.display();
  DataScope();
}

/**************************************************************************
  Function : External interrupt reads encoder data with 4x frequency function.
**************************************************************************/
void READ_ENCODER_A()
{
  if (digitalRead(ENCODER_A) == HIGH)
  {
    if (digitalRead(ENCODER_B) == LOW)   ENCODER++;
    else                                 ENCODER--;
  }
  else
  {
    if (digitalRead(ENCODER_B) == LOW)   ENCODER--;
    else                                 ENCODER++;
  }
}
/**************************************************************************
Function : External interrupt reads encoder data with 4x frequency function.
**************************************************************************/
void READ_ENCODER_B()
{
  if (digitalRead(ENCODER_B) == LOW)
  {
    if (digitalRead(ENCODER_A) == LOW)   ENCODER++;
    else                                 ENCODER--;
  }
  else
  {
    if (digitalRead(ENCODER_A) == LOW)   ENCODER--;
    else                                 ENCODER++;
  }
}
