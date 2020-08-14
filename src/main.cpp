 /*
IMU-сенсор определяет ориентацию объекта в пространстве. Метод расчёта заключается в обработке показаний датчиков акселерометра,
гироскопа и компаса фильтром Маджвика, который рассчитывает положение объекта в пространстве с помощью кватернионов.
Кватернионы более мощный чем углы Эйлера инструмент для выполнения вычислений по ориентации объекта в пространстве,
но им не хватает наглядности. Поэтому полученные кватернионы из фильтра библиотека преобразует в углы Эйлера.

В фильтре Маджвика реализовано два варианта определение ориентации объекта в пространстве.

Первый вариант применим к инерционным навигационным системам (ИНС), включающим акселерометр и гироскоп. При этом направлением на север пренебрегаем.
Второй вариант применим к ИНС, включающих дополнительно трёхосевой магнитометр.

Пример фильтра без магнитометра

В качестве примера снимем показания с датчиков акселерометра и гироскопа. С помощью фильтра Маджвика получаем ориентацию объекта
и пересчитываем в углы Эйлера. При этом направление на север нас не интересует.
*/

#include <DFRobot_BMI160.h>
#include <TroykaIMU.h>

// создаём объект для работы с акселерометром и гироскопом
DFRobot_BMI160 bmi160;
// адрес i2c IMU
const int8_t i2c_addr = 0x69;

// создаём объект для фильтра Madgwick
Madgwick filter;

// множитель фильтра
#define BETA 0.22f

// ускорение в G
#define RANGE_2G        2
#define RANGE_4G        4
#define RANGE_8G        8
#define RANGE_G        64

// чувствительность датчика
#define SENS_FS_250     0.00875
#define SENS_FS_500     0.0175
#define SENS_FS_2000    0.07

// переменные для данных с гироскопа и акселерометра
float gx, gy, gz, ax, ay, az;
 
// получаемые углы ориентации
float yaw, pitch, roll;
 
// переменная для хранения частоты выборок фильтра
float fps = 100;

// структура данных Hatire для передачи значений в OpenTrack
typedef struct  {
  int16_t  Begin  ;   // 2  Начало
  uint16_t Cpt ;      // 2  Фрейм или код сообщения/ошибки
  float    gyro[3];   // 12 [Y, P, R]    gyro
  float    acc[3];    // 12 [x, y, z]    Acc
  int16_t  End ;      // 2  Конец
} _hatire;

// структура данных Hatire для передачи сообщений в OpenTrack
typedef struct  {
  int16_t  Begin  ;   // 2  Начало
  uint16_t Code ;     // 2  Код
  char     Msg[24];   // 24 Сообщение
  int16_t  End ;      // 2  Конец
} _msginfo;

_hatire hatire;
_msginfo msginfo;

// версия Hatire
char Version[] = "HAT V 1.10";

// Передача сообщения Hatire на ПК
void PrintCodeSerial(uint16_t code, const char Msg[24], bool EOL) {
  msginfo.Code = code;
  memset(msginfo.Msg, 0x00, 24);
  strcpy(msginfo.Msg,Msg);
  if (EOL) msginfo.Msg[23] = 0x0A;
  
  Serial.write((byte*)&msginfo, 30);
}

void setup(){
  Serial.begin(115200);
  delay(100);

  PrintCodeSerial(2000, Version, true);

  hatire.Begin=0xAAAA;
  hatire.Cpt=0;
  hatire.End=0x5555;

  msginfo.Begin=0xAAAA;
  msginfo.Code=0;
  msginfo.End=0x5555;
  
  // инициализация BMI160  
  if (bmi160.softReset() != BMI160_OK){
    PrintCodeSerial(9007, "BMI160 reset ERROR", true);
    while(1);
  }
  
  // выбираем i2c адрес
  PrintCodeSerial(3001, "Initializing I2C", true);
  if (bmi160.I2cInit(i2c_addr) != BMI160_OK){
    PrintCodeSerial(9007, "BMI160 init ERROR", true);
    while(1);
  }

  PrintCodeSerial(3003, "BMI160 connection OK", true);

  PrintCodeSerial(5000, "HAT BEGIN", true);
}

void loop(){ 
  // запоминаем текущее время
  unsigned long startMillis = millis();

  int rslt;
  int16_t dataGyroAccel[6] = {0};

  // считываем данные с BMI160
  rslt = bmi160.getAccelGyroData(dataGyroAccel);

  if(rslt == 0){
    // считываем данные с гироскопа в радианах в секунду
    gx = dataGyroAccel[0] * SENS_FS_500 * DEG_TO_RAD;
    gy = dataGyroAccel[1] * SENS_FS_500 * DEG_TO_RAD;
    gz = dataGyroAccel[2] * SENS_FS_500 * DEG_TO_RAD;
    // считываем данные с акселерометра в единицах G
    ax = dataGyroAccel[3] * RANGE_8G / 32767.0;
    ay = dataGyroAccel[4] * RANGE_8G / 32767.0;
    az = dataGyroAccel[5] * RANGE_8G / 32767.0;

    // устанавливаем коэффициенты фильтра
    filter.setKoeff(fps, BETA);
    // обновляем входные данные в фильтр
    filter.update(gx, gy, gz, ax, ay, az);

    // получение углов yaw, pitch и roll из фильтра
    yaw = filter.getYawDeg();
    pitch = filter.getPitchDeg();
    roll = filter.getRollDeg();

    hatire.gyro[0] = yaw;
    hatire.gyro[1] = pitch;
    hatire.gyro[2] = roll;

    hatire.acc[0] = ax;
    hatire.acc[1] = ay;
    hatire.acc[2] = az;
  
    Serial.write((byte*)&hatire, 30);
  }else{
    PrintCodeSerial(9007, "BMI160 read ERROR", true);
  }

  hatire.Cpt++;
  if (hatire.Cpt > 999) {
    hatire.Cpt = 0;
  }
  
  // вычисляем затраченное время на обработку данных
  unsigned long deltaMillis = millis() - startMillis;
  // вычисляем частоту обработки фильтра
  fps = 1000 / deltaMillis;
}










