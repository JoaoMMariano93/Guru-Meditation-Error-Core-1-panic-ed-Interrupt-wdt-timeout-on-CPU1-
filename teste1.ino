#include <Wire.h>              //Utilizado para comunicação I2C
#include <LiquidCrystal_I2C.h> //biblioteca responsável pelo controle do display
#include <Arduino.h>           //Bibliotecas arduino
#include <analogWrite.h>       //Utilizado para fazer o esp32 poder ter pino analogico
#include "PID_v1.h"            //Biblioteca utilizada para fazer controle PID
#include "Adafruit_VL53L0X.h"  //Biblioteca com funções do sensor VL53L0X
#include <Servo.h>             //Biblioteca responsavel pelo funcionamento do servo motor


Servo myservo;                              //Definição do nome do servo motor
Adafruit_VL53L0X lox = Adafruit_VL53L0X();  //Definição do sensor
VL53L0X_RangingMeasurementData_t measure;   //Modo de utilização do senor VL53L0X
LiquidCrystal_I2C lcd(0x27, 16, 2);         //set the LCD address to 0x27 for a 16 chars and 2 line display
//=================================================================
//
// --- Mapeamento de Hardware ---
//
//=================================================================
#define ZERA_SSD 34
#define PLAY_PAUSE 35
#define EN 25
#define servo_pin 13
#define encoder_C1 32
#define encoder_C2 33
#define servo_pin 13
#define INCLUDE_vTaskDelayUntil 1
#define INCLUDE_vTaskDelay 1
// ===================================================================
//
// ---Definição de constantes ---
//
// ===================================================================
#define tempo_de_delay 50         //Definição de delay  
#define um_segundo 1000           //Definição de um segundo
#define t_captacao 10             //tempo de captação de dados
#define degrau 104                //Entrada r(t)
#define pulsos_por_volta 516      //define pulsos_por_volta 516
#define voltas_maxima 10          //Número de voltas máxima
//=================================================================
//
// --- Structs parametros ---
//
typedef struct {                  //Struct com parametros para task MOTOR_ON
  uint8_t d_T_CAPTACAO, d_ENABLE, d_DEGRAU, d_VOLTAS_MAXIMA;
  uint16_t d_PULSO_POR_VOLTA;
  int voltas;
  double Setpoint, Input, Output;
  long d_pulse_number, aux_pulse_number;
  int d_PLAY_PAUSE;
  int d_ZERA_SSD;
} d_MOTOR_ON;
typedef struct {                  //Struct com parametros para task LEITURA
  uint8_t v;
  int flag_PLAY_PAUSE;
  int voltas; 
  int pos; 
} d_LEITURA;
//=================================================================
//
// --- Variáveis Globais ---
//
//=================================================================
//variaveis que indicam o núcleo
static uint8_t taskCoreZero = 0;
static uint8_t taskCoreOne  = 1;
//Posicao inicial sensor
int pos = 0;
//Flags dos botões
int flag_ZERA_SSD = HIGH;
int flag_PLAY_PAUSE = LOW;
//Variáveis de controle do motor
long pulse_number = 0;
int voltas = 0;
//Variáveis do Controlador
double Setpoint, Input, Output;
long aux_pulse_number = 0;
bool flag_pulse_number = false;

//================================================================
//
// --- Inicialização myPID ---
//
//================================================================
PID myPID(&Input, &Output, &Setpoint, 11.55, 0.1, 0, DIRECT);
//================================================================
//
// --- Declaração de funções ---
//
//================================================================
void debounce_PLAY_PAUSE( void * pvParameters );
void debounce_ZERA_SSD( void * pvParameters );
void IRAM_ATTR CONTA_PULSO();
void VERIFICA_PULSO();
void motor_ON( void * pvParameters );
void LEITURA(void * pvParameters);
void setup() {
  Serial.begin(9600);
    while (! Serial) {
    delay(1);
  }
    Serial.println("Adafruit VL53L0X test");
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
  
  lcd.init();
  lcd.backlight();
  pinMode(ZERA_SSD, INPUT);   //Resistencia interna
  pinMode(PLAY_PAUSE, INPUT); //Resistencia interna
  pinMode(EN, OUTPUT);        //Pinos do motor
  pinMode(encoder_C1, INPUT); //Configura entrada C1 para leitura do encoder
  pinMode(encoder_C2, INPUT); //Configura entrada C2 para leitura do encoder
  
  //Interrupção externa por borda de subida
  attachInterrupt(encoder_C1,CONTA_PULSO, RISING);
  
  lcd.print("VOLTAS:");
  lcd.setCursor(8, 0);
  lcd.print(voltas);


  Setpoint = 10.45; // r(t) ajustado
  myPID.SetMode(AUTOMATIC);//Configuração para modo automatico
  myPID.SetSampleTime(10); // Computa controlador a cada 10 ms
  Input = aux_pulse_number; //Inicializa entrada
  myservo.attach(servo_pin); //Pino PWM do servo motor;


  d_MOTOR_ON MOTOR = {
    .d_T_CAPTACAO = t_captacao,
    .d_ENABLE = EN,
    .d_DEGRAU = degrau,
    .d_VOLTAS_MAXIMA = voltas_maxima,
    .d_PULSO_POR_VOLTA = pulsos_por_volta,
    .voltas = voltas,
    .Setpoint = Setpoint,
    .Input = Input,
    .Output = Output,
    .d_pulse_number = pulse_number,
    .aux_pulse_number = aux_pulse_number,
    .d_PLAY_PAUSE = flag_PLAY_PAUSE,
    .d_ZERA_SSD = flag_ZERA_SSD
  };
d_LEITURA LEITURA_SENSOR = {
    .v = voltas_maxima,
    .flag_PLAY_PAUSE = flag_PLAY_PAUSE,
    .voltas = voltas,
    .pos = pos
  };

 
  xTaskCreatePinnedToCore(debounce_PLAY_PAUSE, "debounce_PLAY_PAUSE",10000, &flag_PLAY_PAUSE, 1, NULL, taskCoreZero);
  delay(500); //tempo para a tarefa iniciar 
  xTaskCreatePinnedToCore(debounce_ZERA_SSD, "debounce_ZERA_SSD", 10000,&flag_ZERA_SSD, 1, NULL, taskCoreZero);
  delay(500); //tempo para a tarefa iniciar  
  xTaskCreatePinnedToCore(LEITURA, "LEITURA", 20000, &LEITURA_SENSOR, 2, NULL, taskCoreZero);
  delay(500); //tempo para a tarefa iniciar    
  xTaskCreatePinnedToCore(motor_ON, "motor_ON",20000, &MOTOR,2, NULL, taskCoreOne);
  delay(500); //tempo para a tarefa iniciar
  Serial.println("**0**");

}
void loop(){
  vTaskDelete(NULL);
  }
void debounce_PLAY_PAUSE( void * pvParameters ){//OK
  static int last_state = HIGH;
  static int state;
   while(1){

 TickType_t xLastWakeTime;
 const TickType_t xFrequency = 50;
 xLastWakeTime = xTaskGetTickCount();
 //Variáveis para debounce
vTaskDelayUntil( &xLastWakeTime, xFrequency );

  int reading = digitalRead(PLAY_PAUSE); //guarda leitura do botão A3
  static unsigned long guarda_millis = millis();//guarda tempo inicial
  if (reading != last_state) { //testa se botao mudou de estado
    //inicia contador para debounce
    guarda_millis = millis();
  }
  //Primeiro degrau
  if ((millis() - guarda_millis) > tempo_de_delay) { //testa se deu o
    //tempo de delay de 50 milisegundos
    //Caso verdade
    if (reading != state) { //Testa se botão mudou
      //Caso verdade
      state = reading; //Estado recebe leitura
      if (state == HIGH) { //Testa se foi borda de subida
        //Condição para botão pressionado
        flag_PLAY_PAUSE = !flag_PLAY_PAUSE;
      }
    }
  }
  last_state = reading; //Último estado recebe leitura mais recente 
  }
}
void debounce_ZERA_SSD( void * pvParameters ) {//OK
 TickType_t xLastWakeTime;
 const TickType_t xFrequency = 50;
 xLastWakeTime = xTaskGetTickCount();
  static int last_state = HIGH;
  static int state; 
 while(1){
vTaskDelayUntil( &xLastWakeTime, xFrequency );
  int reading = digitalRead(ZERA_SSD); //Guarda leitura do botão
  static unsigned long guarda_millis = millis();//Guarda tempo
  //inicial
  if (reading != last_state) { //Testa se botão mudou de estado
    //Inicia contador para debounce
    guarda_millis = millis();
  }
  //Primeiro degrau
  if ((millis() - guarda_millis) > tempo_de_delay) { //Testa se deu o
    //tempo de delay de 50 milisegundos
    //Caso verdade
    if (reading != state) //Testa se botao mudou
    { //Caso verdade
      state = reading; //Estado recebe leitura
      if (state == HIGH) { //Testa se foi borda de subida
        //Condição para botão pressionado
        flag_ZERA_SSD = HIGH;
      }
    }
  }
  last_state = reading;
  if (flag_ZERA_SSD == LOW) {
    //Abaixa flag que será tratada
    flag_ZERA_SSD = HIGH;
    //Reinicia contagem caso motor desativado
    Serial.println("**5.5**");
    if (flag_PLAY_PAUSE == HIGH) {
      //Zera variáveis
      voltas = 0;
      pulse_number = 0;
      //Zera display
      lcd.clear();
      lcd.print("VOLTAS:");
      lcd.print(voltas);
      lcd.setCursor(8, 0);
      Serial.println("**5.6**");
     }
   }
 }
 Serial.println("**6**");
}
void IRAM_ATTR CONTA_PULSO(){//OK
 Serial.println("**7.1**");
if(voltas<voltas_maxima){
  pulse_number++;//Incrementa número de pulsos
  flag_pulse_number = true;
}
if((voltas<voltas_maxima) && (flag_pulse_number == true)){
    flag_pulse_number = false;
  }
}
void VERIFICA_PULSO(){//OK
  //variaveis PWM
  float PER;
  static float PORCENTAGEM = PER; //constante de entrada dada
  //em porcentagem, referente
  //a saida do Arduino (5V)
  int TENSAO = map(PORCENTAGEM, 0, 100, 0, 4096); //Funções que mapeia a variavel
  //e converte para resolução
  //da saida PWM
  //de 2^12 (12 bits)
  //

  Serial.println("**8**");
}
void motor_ON( void * pvParameters ) {
   TickType_t xLastWakeTime;
  const TickType_t xFrequency = 100;
  xLastWakeTime = xTaskGetTickCount();
  static int aux_voltas;   
 while(1){
  Serial.println("**9**");
  vTaskDelayUntil( &xLastWakeTime, xFrequency );
  
  //Captação de informações para o controlador
  static unsigned long guarda_millis_Input = millis();  
  if(!flag_PLAY_PAUSE){
    Serial.println("**10**");
    if ((millis() - guarda_millis_Input) >= t_captacao){
    //reinicia contador
    guarda_millis_Input = millis();
    //captação de amostra
    Input = pulse_number - aux_pulse_number;
    aux_pulse_number = pulse_number;
    }
    
    myPID.Compute();//Computa controlador
    int TENSAO = map (degrau, 0,2048, 0,255); //Mapeia a variável e converte para resolução da saída PWM
   // int TENSAO = degrau/8;
    analogWrite(EN,(TENSAO+Output));
    
    Serial.println("**11**");
    
    if ((pulse_number % pulsos_por_volta) == 0
    && aux_voltas != pulse_number
    && pulse_number > 0 && voltas < voltas_maxima) {
    //Incrementa e mostra no SSD variavel voltas
    voltas = voltas + 1;
    lcd.setCursor(8, 0);
    lcd.print(voltas);
    //Armazena pulse_number
    aux_voltas = pulse_number;
   } 
  }
    if (voltas >= voltas_maxima) {
    lcd.setCursor(8,0);
    lcd.print(voltas);
    pulse_number = 0; //Zera pulse_number
    flag_PLAY_PAUSE = HIGH;
    flag_ZERA_SSD = LOW;
    dacWrite(EN,0);//Desliga motor
    //Abaixa flag do botão
  }
 }
}

void LEITURA( void * pvParameters ) {//OK
 VL53L0X_RangingMeasurementData_t measure;
 while(1){
  TickType_t xLastWakeTime;
 const TickType_t xFrequency = 20;
 xLastWakeTime = xTaskGetTickCount();
  Serial.println("**13**");
    vTaskDelayUntil( &xLastWakeTime, xFrequency );  
   if(!flag_PLAY_PAUSE && voltas<=voltas_maxima){
        for (pos = 120; pos <= 140; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
lox.rangingTest(&measure, false);
  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
  } else {
    Serial.println(" out of range ");
  }    
  }
  
  for (pos = 140; pos >= 120; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
                          // waits 15ms for the servo to reach the position
  }
  
  Serial.println("**14**");
   
   }
  }
}
