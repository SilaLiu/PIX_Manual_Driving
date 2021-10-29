#include <avr/wdt.h>
#include <can.h>
#include <mcp2515.h> ///Lib from github: https://github.com/autowp/arduino-mcp2515.git
#include <SPI.h>
#include <string.h>

#define D_pin         3   //换D挡
#define R_pin         4   //换R挡
#define L_Light       5   //左转向灯
#define R_Light       6   //右转向灯
#define Light         7   //大灯
#define Brake         8   //手刹
#define Enable        9   //使能信号
#define Steer_pin     A0  //转向
#define Throttle_pin  A1  //油门
#define Brake_pin     A2  //刹车

int gear_value      = 0;
int steer_value     = 0;
int throttle_value  = 0;
int brake_value     = 0;
int Eig_byte_value  = 128;
int throttle_offset = 210;//210
int brake_offset    = 210;
int sycsd           = 0;

struct can_frame canMsg1;

MCP2515 mcp2515(10);

////////////////////////////////////////////////////////////////////////////////////////////
//to HEX
char buffer [4] = {'0','0','0','0'}; //用于存放转换好的十六进制字符串，可根据需要定义长度
char * inttohex(int aa)
{
    sprintf(buffer, "%x", aa);
    return (buffer);
}

//throttle sensor
int throttle(){
  int throttle_raw = analogRead(Throttle_pin);
  int throttle_value = 0;
  
  float throttle_factor = 600.0/(741.0 - throttle_offset);
  
  throttle_value = int((throttle_raw - throttle_offset) * throttle_factor); //183 is offset of throttle
  if (throttle_value <= 1)
    {throttle_value = 0;} 
   else if (throttle_value > 600)
    {throttle_value = 600;}
   
  if(throttle_value - sycsd > 1)
  {
    throttle_value = sycsd + 1;
    }
   else if(sycsd - throttle_value > 1)
   {
      throttle_value = sycsd -1;
    }
    sycsd = throttle_value;
    
  return throttle_value;
}


//brake sensor
int brake(){
  int brake_raw = analogRead(Brake_pin);
  int brake_value = 0;

  float brake_factor = 1024.0/(738.0 - brake_offset);
  
  brake_value = int((brake_raw - brake_offset) * brake_factor); //183 is offset of throttle
  if (brake_value <= 1)
    {brake_value = 0;} 
  else if (brake_value > 1023)
    {brake_value = 1023;}

  return brake_value;
}

////steering sensor
//int steer(){
//  int steer_raw = analogRead(Steer_pin);
//  int steer_can_value = 2 * steer_raw - 1024;
//  
//  return steer_can_value;
//}

//shift gear sensor
int shift(){
  //D:1 N:2 R:3
  int gear    = 2;
  int gear_D  = digitalRead(D_pin);
  int gear_R  = digitalRead(R_pin);

  // D gear
  if (gear_D == LOW)
    {gear = 1;}
  // R gear
  else if (gear_R == LOW )
    {gear = 3;}
  // N gear or no gear
  else 
    {gear = 2;}
  
  return gear;
}

int Eighth_byte()
{
  int Eight_byte_value=0;
  int L_Light_value       = digitalRead(L_Light);
  int R_Light_value       = digitalRead(R_Light);
  int Light_value         = digitalRead(Light);
  int Brake_value         = digitalRead(Brake);
  int Enable_value        = digitalRead(Enable);
  if(L_Light_value == LOW)
  {
    Eight_byte_value += 1;
   }
   if(R_Light_value == LOW)
  {
    Eight_byte_value += 4;
   }
   if(Light_value == LOW)
  {
    Eight_byte_value += 16;
   }
   if(Brake_value == LOW)
  {
    Eight_byte_value += 8;
   }
   if(Enable_value == HIGH)
   {
    Eight_byte_value += 128;
   }
   return Eight_byte_value;
}
//show in serial

void showinfo( int gear, int steer, int throttle, int brake, int eig){
      Serial.print("Gear is: ");      Serial.print(gear);     Serial.print(" ### ");
      Serial.print("Steering is: ");  Serial.print(steer);    Serial.print(" ### ");
      Serial.print("Throttle is: ");  Serial.print(throttle); Serial.print(" ### ");
      Serial.print("Brake is: ");     Serial.print(brake);    Serial.print(" ### ");
      Serial.print("eight is: ");     Serial.print(eig);      Serial.println(" ### ");
}

//////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  delay(100);
  pinMode(D_pin, INPUT_PULLUP);
  pinMode(R_pin, INPUT_PULLUP);
  pinMode(L_Light, INPUT_PULLUP);
  pinMode(R_Light, INPUT_PULLUP);
  pinMode(Light, INPUT_PULLUP);
  pinMode(Brake, INPUT_PULLUP);
  pinMode(Enable, INPUT_PULLUP);
  
  while (!Serial);
  Serial.begin(115200);
  
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS);
  mcp2515.setNormalMode();
  
  Serial.println("Example: Write to CAN");

  throttle_offset = analogRead(A1);
  brake_offset = analogRead(A2);

  wdt_enable(WDTO_2S);
}

void loop() {
 // buffer[4] = {0,0,0,0};
  
  canMsg1.can_id  = 0x383;
  canMsg1.can_dlc = 8;
  canMsg1.data[0] = 0x00;
  canMsg1.data[1] = 0x00;
  canMsg1.data[2] = 0x00;
  canMsg1.data[3] = 0x00;
  canMsg1.data[4] = 0x00;
  canMsg1.data[5] = 0x00;
  canMsg1.data[6] = 0x00;
  canMsg1.data[7] = 0x80;

  //gear, D=1, N=2, R=3 
  gear_value = shift(); //读取档位值
  
  //Steering, from -1024 to 1023
  steer_value = 750; //读取转向模拟值

  //throttle, from 0 to 600
  throttle_value = throttle();  //读取油门模拟值

  //brake, from 0 to 1023
  brake_value = brake();      //读取刹车模拟值

  //第8字节
  Eig_byte_value=Eighth_byte();

  if(brake_value > 300)
  {
      sycsd = 0;
      throttle_value = 0;
    }
  //show in serial 打印读取到的档位、转向、油门、刹车等值 
  showinfo(gear_value, steer_value, throttle_value, brake_value,Eig_byte_value); //bool, gear, steer, throttle, brake
  
  canMsg1.data[0] = throttle_value&0xff;  //油门模拟值低字节
  canMsg1.data[1] = throttle_value>>8;    //油门模拟值高字节
  canMsg1.data[2] = brake_value&0xff;     //刹车模拟值低字节
  canMsg1.data[3] = brake_value>>8;       //刹车模拟值高字节
  canMsg1.data[4] = steer_value&0xff;     //转向模拟值低字节
  canMsg1.data[5] = steer_value>>8;       //转向模拟值高字节
  if (gear_value == 1)//档位值
  {
      canMsg1.data[6] = 0x01;
   }
   else if (gear_value == 2)
   {
      canMsg1.data[6] = 0x02;
    }
   else if (gear_value == 3)
    {
      canMsg1.data[6] = 0x03;
    }
  canMsg1.data[7] = Eig_byte_value&0xff;
  //CAN sends message to car  
  mcp2515.sendMessage(&canMsg1);  //发送给SPI
  Serial.println("Messages sent");
  Serial.println(canMsg1.data[2]);
  Serial.println(canMsg1.data[3]);

  delay(20);
  wdt_reset(); //喂狗操作，使看门狗定时器复位
}
