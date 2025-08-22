#include <JY901_dfs.h>
#include <JY901.h>
#include <string.h>
// #include <LiquidCrystal_I2C.h> // Library for LCD

// 電機控制引腳定義
// int m[4][3] = {{12, 34, 35}, {8, 37, 36}, {9, 43, 42}, {5, 29, 39}};
int m[4][3] = { {8, 37, 36}, {5, 39, 29},{12, 34, 35}, {9, 42, 43}};

// LiquidCrystal_I2C lcd(0x27,20,4);

// 搖桿引腳定義
#define x1 A15
#define y1 A14
#define x2 A11
#define y2 A10

// 搖桿校準值
int x1a = 500, x1d = 500, x2a = 500, x2d = 500, y1a = 500, y1d = 500, y2a = 500, y2d = 500;
int x1_center, x2_center, y1_center, y2_center;

// 死區閾值
const int DEADZONE = 10;

// JY901角度變量
int yaw, targetyaw;

// PID转向设置
int maxTurnSpeed = 10; // 1~+oo, 一次打满杆旋转的角度 摇杆
int maxTurnRate = 30;  // 1~1000,一秒内最多旋转多少次 摇杆
float notLineCurvature = 1; //0~+oo,非线性PID修正(⚠️非摇杆)硬度，数字越大越硬

// 車移動速度限制：X Y R映射限制
int maxXspeed = 100;
int maxYspeed = 100;
int maxRotationspeed = 100;
int totalMaxspeed = 100;

// 電機控制函數
void mo(int a, int b, int c)
{
    digitalWrite(m[a][1], b);
    digitalWrite(m[a][2], 1 - b);
    analogWrite(m[a][0], (c * 2.55));
}

// 搖桿校準函數
void jo_st()
{
    // 計算搖桿中心值
    Serial.println("搖桿中心檢測");
    while(!digitalRead(A1)){}
    x1_center = (analogRead(x1));
    x2_center = (analogRead(x2));
    y1_center = (analogRead(y1));
    y2_center = (analogRead(y1));
    Serial.println("搖桿邊界檢測");
    int tim = millis();
    while (1)
    {
        x1a = min(analogRead(x1), x1a);
        x1d = max(analogRead(x1), x1d);
        x2a = min(analogRead(x2), x2a);
        x2d = max(analogRead(x2), x2d);
        y1a = min(analogRead(y1), y1a);
        y1d = max(analogRead(y1), y1d);
        y2a = min(analogRead(y2), y2a);
        y2d = max(analogRead(y2), y2d);

        if (millis() - tim >= 4000)
            break;
    }

}

// 應用死區函數
int applyDeadzone(int value, int center)
{
    if (abs(value - center) < DEADZONE)
    {
        return center;
    }
    return value;
}

// 映射搖桿值到速度值
int mapJoystickToSpeed(int value, int min_val, int max_val, int center)
{
    if (value > center)
    {
        return map(value, center, max_val, 0, 100);
    }
    else
    {
        return map(value, center, min_val, 0, -100);
    }
}

// 麥克納姆輪運動計算函數
void calculateMecanumWheels(int x, int y, int r, int *wheels)
{
    x = x  / (totalMaxspeed / 100.0) * maxXspeed / 100;
    y = y  / (totalMaxspeed / 100.0) * maxYspeed / 100;
    r = r  / (totalMaxspeed / 100.0) * maxRotationspeed / 100;
    
    // 計算四個輪子的速度
    wheels[0] = y - x - r; // 前左轮
    wheels[1] = y + x + r; // 前右轮
    wheels[2] = y + x - r; // 後左轮
    wheels[3] = y - x + r; // 後右轮
    
    // 限制速度在-100到100之間
    for (int i = 0; i < 4; i++)
    {
        wheels[i] = constrain(wheels[i], -100, 100);
    }
}

int last_rotation = 0;

void setup()
{
    Serial.begin(115200);
    // lcd.init();
    // lcd.backlight();
    // lcd.clear();
    // lcd.setCursor(0,0);
    // lcd.print("dercfe");
    // 初始化電機引腳
    for (int o = 0; o < 4; ++o)
    {
        for (int i = 0; i < 3; ++i)
        {
            pinMode(m[o][i], OUTPUT);
        }
    }
    Serial.println("開始執行");
    
    // 搖桿校準
    jo_st();
    
    Serial.println("搖桿校準完成");
    Serial.print("x1: "); Serial.print(x1a); Serial.print(" - "); Serial.print(x1_center); Serial.print(" - "); Serial.println(x1d);
    Serial.print("x2: "); Serial.print(x2a); Serial.print(" - "); Serial.print(x2_center); Serial.print(" - "); Serial.println(x2d);
    Serial.print("y1: "); Serial.print(y1a); Serial.print(" - "); Serial.print(y1_center); Serial.print(" - "); Serial.println(y1d);
    Serial.print("y2: "); Serial.print(y2a); Serial.print(" - "); Serial.print(y2_center); Serial.print(" - "); Serial.println(y2d);
    Serial3.begin(9600);
	JY901.attach(Serial3);
    JY901.receiveSerialData();
    yaw = -JY901.getYaw() + 180;
    targetyaw = -JY901.getYaw() + 180;
}
/*
// 角度轉0-360
int pos_angle(int angle)
{
    return (angle >= 0) ? (angle % 360) : ((360 - angle) % 360);
    }
*/

void loop()
{
    // 讀取yaw並修改變量
    JY901.receiveSerialData();
    yaw = -JY901.getYaw() + 180;
    // 讀取搖桿值並應用死區
    int x1_val = applyDeadzone(analogRead(x1), x1_center);
    int y1_val = applyDeadzone(analogRead(y1), y1_center);
    int x2_val = applyDeadzone(analogRead(x2), x2_center);
    int y2_val = applyDeadzone(analogRead(y2), y2_center);

    // 映射搖桿值到速度值
    int x_speed = mapJoystickToSpeed(x1_val, x1a, x1d, x1_center);
    int y_speed = mapJoystickToSpeed(y1_val, y1a, y1d, y1_center);
    int rotation = mapJoystickToSpeed(y2_val, y2a, y2d, y2_center);
    
    // 搖桿映射改正
    x_speed = -x_speed;
    rotation = (-rotation / 1.0) * 0.01 * maxTurnSpeed;// rotation 需已经降速过


    // 修改targetyaw變量
    /* 1.0
    if (millis() % 100 == 0){
        targetyaw += rotation / 10 + 360 * 10;
        targetyaw = targetyaw % 360;
    }
    rotation = ((targetyaw - yaw + 360 * 10) % 360 - 180);
    Serial.println(rotation);
    */
    /* 2.0
    targetyaw = (targetyaw + pos_angle(rotation)) % 360;
    rotation = pos_angle(targetyaw - yaw) % 360 - 180;
    */
    /* 不行，必須保留targetyaw
    // 3.0 rotation = pos_angle((targetyaw + pos_angle(rotation)) - yaw) % 360 - 180;
    //限制旋轉速度
    if (millis() % 100 = 0)
    rotation = pos_angle((targetyaw + pos_angle(rotation / 10)) - yaw) % 360 - 180;
    */
    /*
    //4.1 函数暂时先放这里
    int pos_angle(int angle) {
        return (angle % 360 + 360) % 360;
    }

    int angle_diff(int a, int b) {
        int d = pos_angle(a - b); 
        return d > 180 ? d - 360 : d;
    }

    // 更新目标角度和电机旋转值
    targetyaw = pos_angle(targetyaw + rotation / 10);
    int motar_rotation = angle_diff(targetyaw, yaw + 180);
    */
    
    // 简化 (v8)
	if (millis() % (1000 / maxTurnRate) == 0)  // 限制转向速度
    	targetyaw = ((targetyaw + rotation) % 360 + 360) % 360;
    int motar_rotation = (((targetyaw - yaw - 180)+ 360) % 360);
	motar_rotation=(abs(last_rotation < 160)) ? ((motar_rotation > 180) ? (motar_rotation - 360) : (motar_rotation)) : ((motar_rotation > 180) ? (motar_rotation) : (360 - motar_rotation));
    last_rotation = motar_rotation;
    

    // 計算麥克納姆輪速度
    int wheels[4];
    calculateMecanumWheels(x_speed, y_speed, motar_rotation * notLineCurvature, wheels);

    // 控制電機
    for (int i = 0; i < 4; i++)
    {
        int direction = (wheels[i] >= 0) ? 1 : 0;
        int speed = abs(wheels[i]);
        mo(i, direction, speed);
    }
}
