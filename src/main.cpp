#include <Arduino.h>
#include <String.h>
#include <sstream>
#include <iostream>
#include <stdlib.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <nvs.h>
#include <nvs_flash.h>
#include "esp_timer.h"
#include "driver/uart.h"
#include "sdkconfig.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

String NVS_Read_String(const char* name, const char* key);
int NVS_Write_String(const char* name, const char* key, const char* stringVal);
float decodePayload(String Payload, char operation);
float axisToDegress(float Axis_x, float Axis_y,float Axis_z, char operatation);
void uart_init(int baudRate, int tx_io_num, int rx_io_num, uart_port_t uart_num);
char* uartData();
void mountPackage();
void processingData();
void NVS_Erase();
void Task_stateGPIO(void * params);
void Task_registerBeacon(void * params);
void registerBeacon();


//ac:23:3f:ad:dd:d6@ac:23:3f:ad:dd:d7

#define SEMAPHORE_WAIT 2000
#define QUEUE_WAIT 3000
#define QUEUE_LENGHT 20
#define SERIAL_BAUDRATE 115200
#define TIME_SEARCH_BLE 1
#define UART0 UART_NUM_0
#define TIME_REGISTER 3000000  // esp_timer_get_time() 1s <-> 1000000 us

#define PIN_ED2 GPIO_NUM_13
#define PIN_ED3 GPIO_NUM_14
#define LED_BLUE GPIO_NUM_2


xQueueHandle QueuePackages;
xSemaphoreHandle state; // Semaphore para as Tasks

char estado;
char flag = 'I';
int lastTime;

struct acc_frame 
{
  String index;
  float ED2_Axis_x;
  float ED2_Axis_y;
  float ED2_Axis_z;
  float ED2_Angle_x;
  float ED2_Angle_y;
  float ED2_Angle_z;
  float ED3_Axis_x;
  float ED3_Axis_y;
  float ED3_Axis_z;
  float ED3_Angle_x;
  float ED3_Angle_y;
  float ED3_Angle_z;
  int gapTime;

}frame = {"BACC",0,0,0,0,0,0,0,0,0,0,0,0};


//String addressRef = "ac:23:3f:aa:82:29";
String ED2;
String ED3;
String addrMac;

BLEScan* pBLEScan;

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks
{
    void onResult(BLEAdvertisedDevice advertisedDevice)
    {

      size_t payloadLen = advertisedDevice.getPayloadLength();
      uint8_t* payloadData = advertisedDevice.getPayload();
      String payloadStr;
      String Address01;
      
      for (int i = 0; i < payloadLen; i++) 
      {
        char payloadByte[3];
        sprintf(payloadByte, "%02X", payloadData[i]);
        payloadStr = payloadStr + payloadByte;

      }
      //Serial.print(payloadStr);

      Address01 = advertisedDevice.getAddress().toString().c_str();
      
      if(strcmp(Address01.c_str(), ED2.c_str()) == 0)
      {
       //uart_write_bytes(UART0, payloadStr.c_str(),strlen(payloadStr.c_str()));
        frame.ED2_Axis_x = decodePayload(payloadStr,'x');
        frame.ED2_Axis_y = decodePayload(payloadStr,'y');
        frame.ED2_Axis_z = decodePayload(payloadStr,'z');
        frame.ED2_Angle_x = axisToDegress(frame.ED2_Axis_x,frame.ED2_Axis_y,frame.ED2_Axis_z,'x');
        frame.ED2_Angle_y = axisToDegress(frame.ED2_Axis_x,frame.ED2_Axis_y,frame.ED2_Axis_z,'y');
        frame.ED2_Angle_z = axisToDegress(frame.ED2_Axis_x,frame.ED2_Axis_y,frame.ED2_Axis_z,'z');
        frame.gapTime = (esp_timer_get_time())/1000000;
        mountPackage();
        processingData();
      }

      if(strcmp(Address01.c_str(), ED3.c_str()) == 0)
      {
        frame.ED3_Axis_x = decodePayload(payloadStr,'x');
        frame.ED3_Axis_y = decodePayload(payloadStr,'y');
        frame.ED3_Axis_z = decodePayload(payloadStr,'z');
        frame.ED3_Angle_x = axisToDegress(frame.ED3_Axis_x,frame.ED3_Axis_y,frame.ED3_Axis_z,'x');
        frame.ED3_Angle_y = axisToDegress(frame.ED3_Axis_x,frame.ED3_Axis_y,frame.ED3_Axis_z,'y');
        frame.ED3_Angle_z = axisToDegress(frame.ED3_Axis_x,frame.ED3_Axis_y,frame.ED3_Axis_z,'z');
        mountPackage();
        processingData();
      }

    }
};


void uart_init(int baudRate, int tx_io_num, int rx_io_num, uart_port_t uart_num)
{
    #define BUF_SIZE 1024

    #if CONFIG_UART_ISR_IN_IRAM
      intr_alloc_flags = ESP_INTR_FLAG_IRAM;
    #endif

    uart_config_t uart_config = 
    {
      .baud_rate = baudRate,
      .data_bits = UART_DATA_8_BITS,
      .parity    = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    
   /* Configure parameters of an UART driver,
   * communication pins and install the driver */  
   int intr_alloc_flags = 0;
   uart_driver_install(uart_num, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags);
   uart_param_config(uart_num, &uart_config);
   uart_set_pin(uart_num, tx_io_num, rx_io_num, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

}

char* uartData()
{
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    int len = uart_read_bytes(UART_NUM_0, data, BUF_SIZE, 20 / portTICK_RATE_MS);
    if (len > 0) {
      data[len] = 0;
      //uart_write_bytes(UART0, (const char *) data, len);
      char* output = (char *) data;
      free(data);
      return output;
    }

    return nullptr;
}


void Task_stateGPIO(void * params)
{
  while(1)
  {    
    switch (estado)
    {

      case 'A':
        gpio_set_level(PIN_ED2, 1);
        gpio_set_level(LED_BLUE,0);
        //uart_write_bytes(UART0, (const char *) "pos1 \n", strlen("pos1 \n"));
        break;

      case 'B':
        gpio_set_level(PIN_ED3, 1);
        gpio_set_level(LED_BLUE,0);
        //uart_write_bytes(UART0, (const char *) "Basculando \n", strlen("Basculando \n"));
      break;

      case 'D':
        gpio_set_level(PIN_ED2,0);
        gpio_set_level(LED_BLUE,1);
       // uart_write_bytes(UART0, (const char *) "pos2 \n", strlen("pos2 \n"));
        break;

      case 'N':
        gpio_set_level(PIN_ED2,1);
        gpio_set_level(PIN_ED3,0);
        gpio_set_level(LED_BLUE,0);
        break;
        
    }

    vTaskDelay(2000 / portTICK_RATE_MS);
  }
}


void Task_registerBeacon(void * params)
{
    while(1)
    {

      if(esp_timer_get_time() < TIME_REGISTER)
      {
        uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
        int len = uart_read_bytes(UART0, data, BUF_SIZE, 1000 / portTICK_RATE_MS);
        if (len > 0) 
        {
          data[len] = 0;
          //uart_write_bytes(UART_NUM_0, (const char *) data, len);
          //char* output = (char *) data;          
          addrMac = (char *) data;

          free(data);
          flag = 'G';
        }

      }

      if (flag == 'I' && (esp_timer_get_time() > TIME_REGISTER))
      {
        ED2 = NVS_Read_String("memoria", "ED2");
        ED3 = NVS_Read_String("memoria", "ED3");
        uart_write_bytes(UART0, (const char *) "\n", strlen("\n"));
        uart_write_bytes(UART0, (const char *) "iBeacon ED2 -> ", strlen("iBeacon ED2 -> "));
        uart_write_bytes(UART0, (const char *) ED2.c_str(), strlen(ED2.c_str()));
        uart_write_bytes(UART0, (const char *) "\n", strlen("\n"));
        uart_write_bytes(UART0, (const char *) "iBeacon ED3 -> ", strlen("iBeacon ED3 -> "));
        uart_write_bytes(UART0, (const char *) ED3.c_str(), strlen(ED3.c_str()));
        uart_write_bytes(UART0, (const char *) "\n", strlen("\n"));

        flag = 'N';

      }
      
      if (flag == 'G')
      {
        String str1 = addrMac.substring(0,17);
        String str2 = addrMac.substring(18,36);
        const char* addrMacED2 = str1.c_str();
        const char* addrMacED3 = str2.c_str();
        NVS_Write_String("memoria", "ED2", addrMacED2);
        NVS_Write_String("memoria", "ED3", addrMacED3);

        flag = 'I' ;
      }

      vTaskDelay(1000 / portTICK_RATE_MS);
    }
}

void mountPackage()
{
  String package = "BACC";

  package = (package + "," + frame.ED2_Angle_x + "," + frame.ED2_Angle_y + "," + frame.ED2_Angle_z + "," + frame.ED3_Angle_x + "," + frame.ED3_Angle_y + "," + frame.ED3_Angle_z  + "," + frame.gapTime);
  uart_write_bytes(UART0, (const char *) "Package: ", strlen("Package: "));
  uart_write_bytes(UART0, (const char *) package.c_str(), strlen(package.c_str()));
  uart_write_bytes(UART0, (const char *) "\n", strlen("\n"));

  long resposta = xQueueSend(QueuePackages, &frame, QUEUE_WAIT / portTICK_PERIOD_MS);
  
  if(resposta == true)
  {
    //uart_write_bytes(UART0, (const char *) "adicionado a fila", strlen("adicionado a fila"));
  }
  else
  {
    uart_write_bytes(UART0, (const char *) "Não adicionado a fila:  ", strlen("Não adicionado a fila:"));
    //Serial.printf("Não adicionado a fila: %s \r\n",package.c_str());
  } 
}


//X
//.........70-----------------------90................
//..15--60............................................

//Y
//---3.......................
//.....40-----------70.......


void processingData() 
{
  if(xQueueReceive(QueuePackages, &frame, QUEUE_WAIT / portTICK_PERIOD_MS))
  {


// X      Y      Z
// 85.52, 0.00, -85.52,
// 1.15, 83.50, -83.60,

    //  1s <--> 1000000 us
    if ( (frame.ED2_Angle_x >= 15 && frame.ED2_Angle_x <= 60) && (frame.ED2_Angle_y >= 40 && frame.ED2_Angle_y <= 70) )
    {
      //Serial.println("Bag desarmado");
      if (xSemaphoreTake(state, SEMAPHORE_WAIT / portTICK_PERIOD_MS)) // Pega o Semaphore se ele estiver disponivel
      {
        estado = 'D'; 
        lastTime = esp_timer_get_time();
        //uart_write_bytes(UART0, (const char *) "Posição A\n", strlen("posição A\n"));
        //uart_write_bytes(UART0, (const char*) frame.gapTime, 2147483647);

        xSemaphoreGive(state); // Devolve o Semaphore após terminar a função
      }
    }
    
    else if ( (frame.ED2_Angle_x >= 70 && frame.ED2_Angle_x <= 90) && (frame.ED2_Angle_y < 3 ) )
    {
      //Serial.println("Bag armado");
      if (xSemaphoreTake(state, SEMAPHORE_WAIT / portTICK_PERIOD_MS)) // Pega o Semaphore se ele estiver disponivel
      {
        estado = 'A';
        lastTime = frame.gapTime;
        //uart_write_bytes(UART0, (const char *) "posição B\n", strlen("Posição B\n"));

        xSemaphoreGive(state); // Devolve o Semaphore após terminar a função
      }
    }
    
    else if ((frame.ED3_Angle_y >= 70))
    {
      if (xSemaphoreTake(state, SEMAPHORE_WAIT / portTICK_PERIOD_MS)) // Pega o Semaphore se ele estiver disponivel
      {
        estado = 'B';
        xSemaphoreGive(state); // Devolve o Semaphore após terminar a função
      }
    }

    else
    {
      if (xSemaphoreTake(state, SEMAPHORE_WAIT / portTICK_PERIOD_MS)) // Pega o Semaphore se ele estiver disponivel
      {
        estado = 'N';
        xSemaphoreGive(state); // Devolve o Semaphore após terminar a função
      }
    }
  }
}


void buscar()
{
    BLEScanResults foundDevices;
    foundDevices = BLEDevice::getScan()->start(TIME_SEARCH_BLE);
    pBLEScan->clearResults();   // delete results fromBLEScan buffer to release memory
}



void setup() 
{


  gpio_pad_select_gpio(PIN_ED2);
  gpio_pad_select_gpio(PIN_ED3);
  gpio_pad_select_gpio(LED_BLUE);

  gpio_set_direction(PIN_ED2, GPIO_MODE_OUTPUT);
  gpio_set_direction(PIN_ED3, GPIO_MODE_OUTPUT);
  gpio_set_direction(LED_BLUE, GPIO_MODE_OUTPUT);

  gpio_pullup_dis(LED_BLUE);
  gpio_pulldown_en(LED_BLUE);

  gpio_pullup_dis(PIN_ED2);
  gpio_pulldown_en(PIN_ED2);

  gpio_pullup_dis(PIN_ED2);
  gpio_pulldown_en(PIN_ED3);

  uart_init(SERIAL_BAUDRATE, GPIO_NUM_1, GPIO_NUM_3, UART0);

  uart_write_bytes(UART0, (const char *) "\n", strlen("\n"));
  uart_write_bytes(UART0, (const char *) "Digite o endereço MAC dos iBeacons referentes a ED2 e ED3 sepados por '@'. \n", strlen("Digite o endereço MAC dos iBeacons referentes a ED2 e ED3 sepados por '@'. \n"));

  state = xSemaphoreCreateMutex();
  QueuePackages = xQueueCreate(QUEUE_LENGHT,sizeof(float));
  xTaskCreate(&Task_stateGPIO, "estado da porta", 2048, NULL, 1, NULL);
  xTaskCreate(&Task_registerBeacon, "Registrar Beacon", 2048, NULL, 2, NULL);

  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan(); //create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449); // less or equal setInterval value

}

float decodePayload(String Payload, char operation)
{

  String Payload1;
  String Payload2;
  float axis;
  float x;

  switch (operation)
  {
    case 'x':
      // Separando duas partes do payloaX //
      Payload1 = Payload.substring(28,30);
      Payload2 = Payload.substring(30,32);
      break;

    case 'y':
      // Separando duas partes do payloaY //
      Payload1 = Payload.substring(32,34);
      Payload2 = Payload.substring(34,36);
      break;

    case 'z':
      // Separando duas partes do payloaZ //
      Payload1 = Payload.substring(36,38);
      Payload2 = Payload.substring(38,40);
      break;

    default:
      break;
  }

  if (Payload1 == "FF")
  {
    x = -1;
  }
  else if (Payload1 == "00")
  {
    x = 0;
  }
  else
  {
    x = 1;
  }

  int n = Payload2.length();
  char char_array [n+1];
  strcpy(char_array,Payload2.c_str());
  axis = strtol(char_array,NULL,16);  //Conversão de base hexa to dec
  axis = (axis/256) + x;
  return axis;

}

float axisToDegress(float Axis_x, float Axis_y,float Axis_z, char operatation)
{
  float angle = 0;

  switch (operatation)
  {
    case 'x':
      angle = atan((Axis_x)/(sqrt(pow(Axis_y,2) + pow(Axis_z,2))));
      break;

    case 'y':
      angle = atan((Axis_y)/(sqrt(pow(Axis_x,2) + pow(Axis_z,2))));
      break;
    
    case 'z':
      angle = atan((sqrt(pow(Axis_x,2) + pow(Axis_y,2)))/(Axis_z));
      break;

    default:
      break;
  }
  
  angle = (angle*180)/3.14;
  return angle;

}


void NVS_Erase()
{
    ESP_ERROR_CHECK(nvs_flash_init());
    esp_err_t retorno = nvs_flash_erase();
    if ((retorno =! ESP_OK))
    {
      uart_write_bytes(UART0, (const char *) "Não Apagou: ", strlen("Não Apagou: "));
      uart_write_bytes(UART0, (const char *) esp_err_to_name(retorno), strlen(esp_err_to_name(retorno)));
      uart_write_bytes(UART0, (const char *) "\n", strlen("\n"));
    }
}

int NVS_Write_String(const char* name, const char* key, const char* stringVal)
{
    nvs_handle particao_handle;
    esp_err_t retVal;
    int aux;
    
    ESP_ERROR_CHECK(nvs_flash_init());
    retVal = nvs_open(name, NVS_READWRITE, &particao_handle);
    if(retVal != ESP_OK)
    {
        uart_write_bytes(UART0, (const char *) "Não foi possivel acessar a partição \n", strlen("Não foi possivel acessar a partição \n"));
        aux = 0;
    }
    else
    {
        //Serial.println("opening NVS Write handle Done");
        retVal = nvs_set_str(particao_handle, key, stringVal);

        if(retVal != ESP_OK)
        {
            uart_write_bytes(UART0, (const char *) "Não foi possivel  gravar o dado enviado\n", strlen("Não foi possivel  gravar o dado enviado\n"));
            aux = 0;
        }
 
        retVal = nvs_commit(particao_handle);
        if(retVal != ESP_OK)
        {
            uart_write_bytes(UART0, (const char *) "Não foi possivel  gravar o dado enviado na partição\n", strlen("Não foi possivel  gravar o dado enviado na partição\n"));
            aux = 0;
        }
        else
        {
            //Serial.println(" Dado Gravado na memoria com sucesso");
            aux = 1;
        }
       
    }
 
    nvs_close(particao_handle);
    return aux;

}


String NVS_Read_String(const char* name, const char* key)
{
    nvs_handle particao_handle;
    size_t required_size;
    ESP_ERROR_CHECK(nvs_flash_init());
    esp_err_t res_nvs = nvs_open(name, NVS_READONLY, &particao_handle);

    if (res_nvs == ESP_ERR_NVS_NOT_FOUND)
    {
      uart_write_bytes(UART0, (const char *) "NVS, Namespace: armazenamento, não encontrado", strlen("NVS, Namespace: armazenamento, não encontrado"));
    }
    else
    {
      nvs_get_str(particao_handle, key, NULL , &required_size);
      char* data = (char*)malloc(required_size);

      esp_err_t res = nvs_get_str(particao_handle, key, data, &required_size);
      switch (res)
      {
      case ESP_OK:
        //Serial.printf("Valor Armazenado: %s ",data);
        break;

      case ESP_ERR_NOT_FOUND:
        uart_write_bytes(UART0, (const char *) "NVS: Valor não encontrado", strlen("NVS: Valor não encontrado"));

        break;

      default:
        uart_write_bytes(UART0, (const char *) "NVS: Erro ao acessar o NVS: ", strlen("NVS: Erro ao acessar o NVS: "));
        uart_write_bytes(UART0, (const char *) esp_err_to_name(res), strlen(esp_err_to_name(res)));
        uart_write_bytes(UART0, (const char *) "\n", strlen("\n"));


        break;
      }
      nvs_close(particao_handle);
      return data;
    }
    return "";
}


//ac:23:3f:aa:82:29
void loop() 
{
  buscar();
}

