// Pines
const int motorPWM = 9;        // Pin PWM para controlar el motor
const int encoderA = 2;        // Pin del canal A del encoder
const int encoderB = 3;        // Pin del canal B del encoder

// Variables del encoder
volatile long encoderCount = 0;  // Cuenta de pulsos del encoder
int lastEncoderCount = 0;        // Para calcular la velocidad
unsigned long lastTime = 0;      // Tiempo anterior
float velocity = 0;              // Velocidad angular (rad/s)

// Parámetros del motor
const int PPR = 200;             // Pulsos por revolución del encoder
const float gearRatio = 1.0;     // Relación de engranaje si aplica
const float timeInterval = 100;  // Intervalo de tiempo para cálculo (ms)

// PID Variables
float Kp = 2.0, Ki = 0.5, Kd = 0.1;  // Ganancias del controlador
float setpoint = 50.0;               // Velocidad deseada (rad/s)
float error, lastError = 0, integral = 0, derivative, output;

// Setup
void setup() {
  pinMode(motorPWM, OUTPUT);
  pinMode(encoderA, INPUT_PULLUP);
  pinMode(encoderB, INPUT_PULLUP);
  
  // Interrupciones del encoder
  attachInterrupt(digitalPinToInterrupt(encoderA), readEncoder, RISING);
  
  Serial.begin(9600);
}

// Loop principal
void loop() {
  // Calcula la velocidad cada 'timeInterval' milisegundos
  if (millis() - lastTime >= timeInterval) {
    lastTime = millis();
    calculateVelocity();
    
    // Controlador PID
    error = setpoint - velocity;
    integral += error * (timeInterval / 1000.0);
    derivative = (error - lastError) / (timeInterval / 1000.0);
    output = Kp * error + Ki * integral + Kd * derivative;
    lastError = error;

    // Constrain output to PWM range
    output = constrain(output, 0, 255);
    analogWrite(motorPWM, (int)output);
    
    // Debug
    Serial.print("Setpoint: "); Serial.print(setpoint);
    Serial.print(" | Velocity: "); Serial.print(velocity);
    Serial.print(" | Output: "); Serial.println(output);
  }
}

// Función de interrupción del encoder
void readEncoder() {
  int stateB = digitalRead(encoderB);
  if (stateB == HIGH) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

// Calcula la velocidad angular (rad/s)
void calculateVelocity() {
  int pulses = encoderCount - lastEncoderCount;
  lastEncoderCount = encoderCount;
  velocity = (pulses * 2.0 * PI) / (PPR * gearRatio * (timeInterval / 1000.0));
}