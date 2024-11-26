// Pines
const int motorPWM = 9;        // Pin PWM para controlar la velocidad del motor
const int motorIN1 = 7;        // Pin para IN1 del L298N
const int motorIN2 = 8;        // Pin para IN2 del L298N
const int sensorPin = 2;       // Pin para el sensor de velocidad (interrupción)

// Variables del sensor
volatile unsigned long lastPulseTime = 0;  // Marca de tiempo del último pulso
volatile float currentRPM = 0;             // Velocidad medida en RPM
volatile unsigned int pulseCount = 0;      // Contador de pulsos para depuración

// Parámetros del motor
const float pulsesPerRevolution = 1.0;     // Pulsos por revolución (ajustar según el sensor)
const float gearRatio = 1.0;               // Relación de engranaje (si aplica)

// PID Variables
float Kp = 1.0, Ki = 0.005, Kd = 0.05;       // Ganancias del controlador PID
float setpoint = 80.0;                     // Velocidad deseada (RPM)
float error, lastError = 0, integral = 0, derivative, output;

// Control de tiempo
unsigned long lastTime = 0;                // Tiempo del último cálculo del PID
unsigned long lastSerialTime = 0;          // Tiempo del último debug

// Setup
void setup() {
  pinMode(motorPWM, OUTPUT);
  pinMode(motorIN1, OUTPUT);
  pinMode(motorIN2, OUTPUT);
  pinMode(sensorPin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(sensorPin), readSensorPulse, FALLING);

  Serial.begin(9600);

  // Configuración inicial del motor (giro en sentido horario)
  digitalWrite(motorIN1, HIGH);
  digitalWrite(motorIN2, LOW);

  Serial.println("Sistema iniciado.");
}

// Loop principal
void loop() {
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0; // Tiempo en segundos

  // **Cambia dinámicamente el setpoint**
  if (currentTime % 5000 < 2500) {
    setpoint = 60.0;  // Velocidad deseada durante los primeros 2.5 segundos
  } else {
    setpoint = 100.0;  // Cambia a un setpoint más alto
  }

  // Asegura que RPM no sea negativa
  currentRPM = max(currentRPM, 0);

  if (deltaTime >= 0.1) {  // Realiza el cálculo cada 100 ms
    // Cálculo del error
    error = setpoint - currentRPM;

    // Término Integral (con limitación para evitar "windup")
    integral += error * deltaTime;
    integral = constrain(integral, -50, 50);

    // Término Derivativo
    derivative = (error - lastError) / deltaTime;

    // Cálculo del PID
    output = Kp * error + Ki * integral + Kd * derivative;
    output = constrain(output, 0, 255);  // Limita la salida al rango PWM

    // Actualiza el PWM del motor
    analogWrite(motorPWM, (int)output);

    // Actualiza el último error y tiempo
    lastError = error;
    lastTime = currentTime;
  }

  // Control de impresión serial (cada 500 ms)
  if (currentTime - lastSerialTime >= 500) {
    Serial.print("Setpoint: "); Serial.print(setpoint);
    Serial.print(" RPM | Current RPM: "); Serial.print(currentRPM);
    Serial.print(" | Pulses: "); Serial.print(pulseCount); // Para depuración
    Serial.print(" | Output: "); Serial.println(output);
    lastSerialTime = currentTime;
  }
}

// Interrupción del sensor
void readSensorPulse() {
  unsigned long currentTime = micros(); // Tiempo actual en microsegundos
  unsigned long pulseDuration = currentTime - lastPulseTime;

  // Evita pulsos muy rápidos (debounce)
  if (pulseDuration > 1000) { // Ignora interrupciones menores a 1 ms
    if (pulseDuration > 0) {
      // Calcula la velocidad en RPM
      currentRPM = (60.0 * 1000000.0) / (pulseDuration * pulsesPerRevolution * gearRatio);
    }
    lastPulseTime = currentTime;
    pulseCount++; // Incrementa el contador de pulsos
  }
}
