#include <Wire.h>
#include <WiFiEsp.h>
#include <WiFiEspClient.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h> 
#include "fis_header.h"


IPAddress server(34,243,178,169);
char ssid[] = "HUAWEI-A0E4";
char pass[] = "05690998";
int status = WL_IDLE_STATUS;

const int fis_gcR = 1;

FIS_TYPE g_fisInput[9];
FIS_TYPE g_fisOutput[1];

WiFiEspClient espClient;
PubSubClient client(espClient);
SoftwareSerial wifi(5, 6);

TinyGPS gps;
float lat = 28.5458,lon = 77.1703;
SoftwareSerial gpsSerial(3, 4);

const int MPU6050_addr=0x68;
int16_t AccX,AccY,AccZ,Temp,GyroX,GyroY,GyroZ;
char nilai;
char hasil[8];
String fuzzy;
char lokasi[14];

void setup() {
  Wire.begin();
  Wire.beginTransmission(MPU6050_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(9600);
  gpsSerial.begin(9600);  
  wifi.begin(9600);
  WiFi.init(&wifi);
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    while (true);
  }

  while ( status != WL_CONNECTED) {
  Serial.print("Connecting to: ");
  Serial.println(ssid);
  status = WiFi.begin(ssid, pass);
  }
  Serial.println("Connected!");
  
  client.setServer(server, 17014);
  client.setCallback(callback);
}

void callback(char* topic, byte* payload, unsigned int length) {
  for (int i=0;i<length;i++) {
  char receivedChar = (char)payload;
  Serial.print(receivedChar);  
  }
}

void loop() {
  if (!client.connected()) {
  reconnect();
  }
  bacaMPU();
  bacaGPS();
  fuzzyfikasi();
  Serial.println();
  client.loop();
  delay(1000);
}

void fuzzyfikasi()  {
  int i = 0;
  while (i <= 8) {
  g_fisInput[i] = 0.25;
  i += 1;
  }
  g_fisOutput[0] = 0;
  fis_evaluate();
  fuzzy = String(g_fisOutput[0]);
  int str_len = fuzzy.length()+1;
  char kirim_fuzzy[str_len];
  fuzzy.toCharArray(kirim_fuzzy, str_len); 
  client.publish("fuzzy", kirim_fuzzy);
  
  Serial.print(" fuzzy = "); Serial.println(g_fisOutput[0]); 
}

void bacaGPS()  {
    while(gpsSerial.available()){  
      if(gps.encode(gpsSerial.read()))  {  
      gps.f_get_position(&lat,&lon);
      } 
  } 

  dtostrf(lat, 7, 7, lokasi);
  client.publish("lat", lokasi);
  Serial.print(" lat = "); Serial.print(lokasi);

  dtostrf(lon, 7, 7, lokasi);
  client.publish("lon", lokasi);
  Serial.print(" lon = "); Serial.print(lokasi);
  delay(100);
}

void bacaMPU() {
  Wire.beginTransmission(MPU6050_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_addr,14,true);
  AccX=Wire.read()<<8|Wire.read();
  AccY=Wire.read()<<8|Wire.read();
  AccZ=Wire.read()<<8|Wire.read();
  Temp=Wire.read()<<8|Wire.read();
  GyroX=Wire.read()<<8|Wire.read();
  GyroY=Wire.read()<<8|Wire.read();
  GyroZ=Wire.read()<<8|Wire.read();
  
   Serial.print("accX = "); Serial.print(AccX);
  dtostrf(AccX, 6, 2, hasil);
  client.publish("accX", hasil);

  Serial.print(" accY = "); Serial.print(AccY);
  dtostrf(AccY, 6, 2, hasil);
  client.publish("accY", hasil);

  Serial.print(" accZ = "); Serial.print(AccZ);
  dtostrf(AccZ, 6, 2, hasil);
  client.publish("accZ", hasil);

  Serial.print(" gyroX = "); Serial.print(GyroX);
  dtostrf(GyroX, 6, 2, hasil);
  client.publish("gyroX", hasil);

  Serial.print(" gyroY = "); Serial.print(GyroY);
  dtostrf(GyroY, 6, 2, hasil);
  client.publish("gyroY", hasil);

  Serial.print(" gyroZ = "); Serial.print(GyroZ);
  dtostrf(GyroZ, 6, 2, hasil);
  client.publish("gyroZ", hasil);

  delay(100);
}
  
void reconnect() {
  while (!client.connected()) {
  Serial.print("Connecting to MQTT");
  if (client.connect("esp8","mggxilrp","jw7BEm0GZrDT")) {
  Serial.println("connected"); 
  } else {
  Serial.print("error: ");
  Serial.print(client.state());
  delay(5000);
  }
  }
}

FIS_TYPE fis_trimf(FIS_TYPE x, FIS_TYPE* p)
{
    FIS_TYPE a = p[0], b = p[1], c = p[2];
    FIS_TYPE t1 = (x - a) / (b - a);
    FIS_TYPE t2 = (c - x) / (c - b);
    if ((a == b) && (b == c)) return (FIS_TYPE) (x == a);
    if (a == b) return (FIS_TYPE) (t2*(b <= x)*(x <= c));
    if (b == c) return (FIS_TYPE) (t1*(a <= x)*(x <= b));
    t1 = min(t1, t2);
    return (FIS_TYPE) max(t1, 0);
}

FIS_TYPE fis_gaussmf(FIS_TYPE x, FIS_TYPE* p)
{
    FIS_TYPE s = p[0], c = p[1];
    FIS_TYPE t = (x - c) / s;
    return exp(-(t * t) / 2);
}

FIS_TYPE fis_psigmf(FIS_TYPE x, FIS_TYPE* p)
{
    return (fis_sigmf(x, p) * fis_sigmf(x, p + 2));
}

FIS_TYPE fis_sigmf(FIS_TYPE x, FIS_TYPE* p)
{
    FIS_TYPE a = p[0], c = p[1];
    return (FIS_TYPE) (1.0 / (1.0 + exp(-a *(x - c))));
}

FIS_TYPE fis_gbellmf(FIS_TYPE x, FIS_TYPE* p)
{
    FIS_TYPE a = p[0], b = p[1], c = p[2];
    FIS_TYPE t = (x - c) / a;
    if ((t == 0) && (b == 0)) return (FIS_TYPE) 0.5;
    if ((t == 0) && (b < 0)) return (FIS_TYPE) 0;
    return (FIS_TYPE) (1.0 / (1.0 + pow(t, b)));
}

FIS_TYPE fis_min(FIS_TYPE a, FIS_TYPE b)
{
    return min(a, b);
}

FIS_TYPE fis_max(FIS_TYPE a, FIS_TYPE b)
{
    return max(a, b);
}

FIS_TYPE fis_array_operation(FIS_TYPE *array, int size, _FIS_ARR_OP pfnOp)
{
    int i;
    FIS_TYPE ret = 0;

    if (size == 0) return ret;
    if (size == 1) return array[0];

    ret = array[0];
    for (i = 1; i < size; i++)
    {
        ret = (*pfnOp)(ret, array[i]);
    }

    return ret;
}


_FIS_MF fis_gMF[] =
{
    fis_trimf, fis_gaussmf, fis_psigmf, fis_sigmf, fis_gbellmf
};

int fis_gIMFCount[] = { 3, 3, 3, 3, 3, 3, 3, 3, 3 };

int fis_gOMFCount[] = { 3 };

FIS_TYPE fis_gMFI0Coeff1[] = { -0.4, 0, 0.4 };
FIS_TYPE fis_gMFI0Coeff2[] = { 0.1, 0.5, 0.9 };
FIS_TYPE fis_gMFI0Coeff3[] = { 0.6, 1, 1.4 };
FIS_TYPE* fis_gMFI0Coeff[] = { fis_gMFI0Coeff1, fis_gMFI0Coeff2, fis_gMFI0Coeff3 };
FIS_TYPE fis_gMFI1Coeff1[] = { -0.4, 0, 0.4 };
FIS_TYPE fis_gMFI1Coeff2[] = { 0.1, 0.5, 0.9 };
FIS_TYPE fis_gMFI1Coeff3[] = { 0.6, 1, 1.4 };
FIS_TYPE* fis_gMFI1Coeff[] = { fis_gMFI1Coeff1, fis_gMFI1Coeff2, fis_gMFI1Coeff3 };
FIS_TYPE fis_gMFI2Coeff1[] = { -0.4, 0, 0.4 };
FIS_TYPE fis_gMFI2Coeff2[] = { 0.1, 0.5, 0.9 };
FIS_TYPE fis_gMFI2Coeff3[] = { 0.6, 1, 1.4 };
FIS_TYPE* fis_gMFI2Coeff[] = { fis_gMFI2Coeff1, fis_gMFI2Coeff2, fis_gMFI2Coeff3 };
FIS_TYPE fis_gMFI3Coeff1[] = { -0.4, 0, 0.4 };
FIS_TYPE fis_gMFI3Coeff2[] = { 0.1, 0.5, 0.9 };
FIS_TYPE fis_gMFI3Coeff3[] = { 0.6, 1, 1.4 };
FIS_TYPE* fis_gMFI3Coeff[] = { fis_gMFI3Coeff1, fis_gMFI3Coeff2, fis_gMFI3Coeff3 };
FIS_TYPE fis_gMFI4Coeff1[] = { -0.4, 0, 0.4 };
FIS_TYPE fis_gMFI4Coeff2[] = { 0.1, 0.5, 0.9 };
FIS_TYPE fis_gMFI4Coeff3[] = { 0.6, 1, 1.4 };
FIS_TYPE* fis_gMFI4Coeff[] = { fis_gMFI4Coeff1, fis_gMFI4Coeff2, fis_gMFI4Coeff3 };
FIS_TYPE fis_gMFI5Coeff1[] = { -0.4, 0, 0.4 };
FIS_TYPE fis_gMFI5Coeff2[] = { 0.1, 0.5, 0.9 };
FIS_TYPE fis_gMFI5Coeff3[] = { 0.6, 1, 1.4 };
FIS_TYPE* fis_gMFI5Coeff[] = { fis_gMFI5Coeff1, fis_gMFI5Coeff2, fis_gMFI5Coeff3 };
FIS_TYPE fis_gMFI6Coeff1[] = { -0.4, 0, 0.4 };
FIS_TYPE fis_gMFI6Coeff2[] = { 0.1, 0.5, 0.9 };
FIS_TYPE fis_gMFI6Coeff3[] = { 0.6, 1, 1.4 };
FIS_TYPE* fis_gMFI6Coeff[] = { fis_gMFI6Coeff1, fis_gMFI6Coeff2, fis_gMFI6Coeff3 };
FIS_TYPE fis_gMFI7Coeff1[] = { -0.4, 0, 0.4 };
FIS_TYPE fis_gMFI7Coeff2[] = { 0.1, 0.5, 0.9 };
FIS_TYPE fis_gMFI7Coeff3[] = { 0.6, 1, 1.4 };
FIS_TYPE* fis_gMFI7Coeff[] = { fis_gMFI7Coeff1, fis_gMFI7Coeff2, fis_gMFI7Coeff3 };
FIS_TYPE fis_gMFI8Coeff1[] = { -0.4, 0, 0.4 };
FIS_TYPE fis_gMFI8Coeff2[] = { 0.1, 0.5, 0.9 };
FIS_TYPE fis_gMFI8Coeff3[] = { 0.6, 1, 1.4 };
FIS_TYPE* fis_gMFI8Coeff[] = { fis_gMFI8Coeff1, fis_gMFI8Coeff2, fis_gMFI8Coeff3 };
FIS_TYPE** fis_gMFICoeff[] = { fis_gMFI0Coeff, fis_gMFI1Coeff, fis_gMFI2Coeff, fis_gMFI3Coeff, fis_gMFI4Coeff, fis_gMFI5Coeff, fis_gMFI6Coeff, fis_gMFI7Coeff, fis_gMFI8Coeff };

FIS_TYPE fis_gMFO0Coeff1[] = { 0.1699, 6.939e-18 };
FIS_TYPE fis_gMFO0Coeff2[] = { 13.73, 0.3, -6.865, 0.9401 };
FIS_TYPE fis_gMFO0Coeff3[] = { 0.2, 2.5, 1 };
FIS_TYPE* fis_gMFO0Coeff[] = { fis_gMFO0Coeff1, fis_gMFO0Coeff2, fis_gMFO0Coeff3 };
FIS_TYPE** fis_gMFOCoeff[] = { fis_gMFO0Coeff };

int fis_gMFI0[] = { 0, 0, 0 };
int fis_gMFI1[] = { 0, 0, 0 };
int fis_gMFI2[] = { 0, 0, 0 };
int fis_gMFI3[] = { 0, 0, 0 };
int fis_gMFI4[] = { 0, 0, 0 };
int fis_gMFI5[] = { 0, 0, 0 };
int fis_gMFI6[] = { 0, 0, 0 };
int fis_gMFI7[] = { 0, 0, 0 };
int fis_gMFI8[] = { 0, 0, 0 };
int* fis_gMFI[] = { fis_gMFI0, fis_gMFI1, fis_gMFI2, fis_gMFI3, fis_gMFI4, fis_gMFI5, fis_gMFI6, fis_gMFI7, fis_gMFI8};

int fis_gMFO0[] = { 1, 2, 4 };
int* fis_gMFO[] = { fis_gMFO0};

FIS_TYPE fis_gRWeight[] = { 1 };

int fis_gRType[] = { 1 };

int fis_gRI0[] = { 2, 2, 2, 1, 1, 3, 2, 2, 2 };
int* fis_gRI[] = { fis_gRI0 };

int fis_gRO0[] = { 2 };
int* fis_gRO[] = { fis_gRO0 };

FIS_TYPE fis_gIMin[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };

// Input range Max
FIS_TYPE fis_gIMax[] = { 1, 1, 1, 1, 1, 1, 1, 1, 1 };

FIS_TYPE fis_gOMin[] = { 0 };

FIS_TYPE fis_gOMax[] = { 1 };

FIS_TYPE fis_MF_out(FIS_TYPE** fuzzyRuleSet, FIS_TYPE x, int o)
{
    FIS_TYPE mfOut;
    int r;

    for (r = 0; r < fis_gcR; ++r)
    {
        int index = fis_gRO[r][o];
        if (index > 0)
        {
            index = index - 1;
            mfOut = (fis_gMF[fis_gMFO[o][index]])(x, fis_gMFOCoeff[o][index]);
        }
        else if (index < 0)
        {
            index = -index - 1;
            mfOut = 1 - (fis_gMF[fis_gMFO[o][index]])(x, fis_gMFOCoeff[o][index]);
        }
        else
        {
            mfOut = 0;
        }

        fuzzyRuleSet[0][r] = fis_min(mfOut, fuzzyRuleSet[1][r]);
    }
    return fis_array_operation(fuzzyRuleSet[0], fis_gcR, fis_max);
}

FIS_TYPE fis_defuzz_centroid(FIS_TYPE** fuzzyRuleSet, int o)
{
    FIS_TYPE step = (fis_gOMax[o] - fis_gOMin[o]) / (FIS_RESOLUSION - 1);
    FIS_TYPE area = 0;
    FIS_TYPE momentum = 0;
    FIS_TYPE dist, slice;
    int i;

    for (i = 0; i < FIS_RESOLUSION; ++i){
        dist = fis_gOMin[o] + (step * i);
        slice = step * fis_MF_out(fuzzyRuleSet, dist, o);
        area += slice;
        momentum += slice*dist;
    }

    return ((area == 0) ? ((fis_gOMax[o] + fis_gOMin[o]) / 2) : (momentum / area));
}

void fis_evaluate()
{
    FIS_TYPE fuzzyInput0[] = { 0, 0, 0 };
    FIS_TYPE fuzzyInput1[] = { 0, 0, 0 };
    FIS_TYPE fuzzyInput2[] = { 0, 0, 0 };
    FIS_TYPE fuzzyInput3[] = { 0, 0, 0 };
    FIS_TYPE fuzzyInput4[] = { 0, 0, 0 };
    FIS_TYPE fuzzyInput5[] = { 0, 0, 0 };
    FIS_TYPE fuzzyInput6[] = { 0, 0, 0 };
    FIS_TYPE fuzzyInput7[] = { 0, 0, 0 };
    FIS_TYPE fuzzyInput8[] = { 0, 0, 0 };
    FIS_TYPE* fuzzyInput[9] = { fuzzyInput0, fuzzyInput1, fuzzyInput2, fuzzyInput3, fuzzyInput4, fuzzyInput5, fuzzyInput6, fuzzyInput7, fuzzyInput8, };
    FIS_TYPE fuzzyOutput0[] = { 0, 0, 0 };
    FIS_TYPE* fuzzyOutput[1] = { fuzzyOutput0, };
    FIS_TYPE fuzzyRules[fis_gcR] = { 0 };
    FIS_TYPE fuzzyFires[fis_gcR] = { 0 };
    FIS_TYPE* fuzzyRuleSet[] = { fuzzyRules, fuzzyFires };
    FIS_TYPE sW = 0;

    // Transforming input to fuzzy Input
    int i, j, r, o;
    for (i = 0; i < 9; ++i)
    {
        for (j = 0; j < fis_gIMFCount[i]; ++j)
        {
            fuzzyInput[i][j] =
                (fis_gMF[fis_gMFI[i][j]])(g_fisInput[i], fis_gMFICoeff[i][j]);
        }
    }

    int index = 0;
    for (r = 0; r < fis_gcR; ++r)
    {
        if (fis_gRType[r] == 1)
        {
            fuzzyFires[r] = FIS_MAX;
            for (i = 0; i < 9; ++i)
            {
                index = fis_gRI[r][i];
                if (index > 0)
                    fuzzyFires[r] = fis_min(fuzzyFires[r], fuzzyInput[i][index - 1]);
                else if (index < 0)
                    fuzzyFires[r] = fis_min(fuzzyFires[r], 1 - fuzzyInput[i][-index - 1]);
                else
                    fuzzyFires[r] = fis_min(fuzzyFires[r], 1);
            }
        }
        else
        {
            fuzzyFires[r] = FIS_MIN;
            for (i = 0; i < 9; ++i)
            {
                index = fis_gRI[r][i];
                if (index > 0)
                    fuzzyFires[r] = fis_max(fuzzyFires[r], fuzzyInput[i][index - 1]);
                else if (index < 0)
                    fuzzyFires[r] = fis_max(fuzzyFires[r], 1 - fuzzyInput[i][-index - 1]);
                else
                    fuzzyFires[r] = fis_max(fuzzyFires[r], 0);
            }
        }

        fuzzyFires[r] = fis_gRWeight[r] * fuzzyFires[r];
        sW += fuzzyFires[r];
    }

    if (sW == 0)
    {
        for (o = 0; o < 1; ++o)
        {
            g_fisOutput[o] = ((fis_gOMax[o] + fis_gOMin[o]) / 2);
        }
    }
    else
    {
        for (o = 0; o < 1; ++o)
        {
            g_fisOutput[o] = fis_defuzz_centroid(fuzzyRuleSet, o);
        }
    }
}
