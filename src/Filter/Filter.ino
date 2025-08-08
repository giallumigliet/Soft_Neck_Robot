// Configuración
const int pinResistencia = A0;    // Pin conectado a la resistencia
const int numeroMuestras = 60;   // Número de muestras para el filtro de media móvil
const float toleranciaPorcentual = 1; // Tolerancia del 100% para variaciones repentinas
const double umbralFijo = 50.0000;       // Umbral fijo adicional (en Ohm) 50

const double offset = 250.0000;

// Parámetros del circuito
const double Vcc = 3.3;           // Tensión de alimentación en voltios cambiar 
const double R_fija = 22000.0000;      // Valor de la resistencia fija en Ohm

// Variables para el filtro de media móvil
double lecturas[numeroMuestras];    // Array para almacenar las muestras
int indice = 0;                  // Índice del array
double suma = 0;                   // Suma de los valores en el buffer
double mediaFiltrada = 0.0000;         // Valor filtrado (media)

// Variables para el filtro de umbral
double valorAnterior = 0.0000;       // Último valor válido leído

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  // Inicializa el buffer de lecturas con 0
  for (int i = 0; i < numeroMuestras; i++) {
    lecturas[i] = 0;
  }

  // Inicializa el valor anterior con una lectura inicial calculada
  valorAnterior = calcularResistencia(analogRead(pinResistencia));
}

void loop() {
  // Lee el valor del ADC
  int valorADC = analogRead(pinResistencia);

  // Calcula la resistencia variable a partir del valor ADC
  double resistenciaCalculada = calcularResistencia(valorADC);

  // Aplica el filtro de umbral (porcentaje + fijo)
  if (!valorAceptable(resistenciaCalculada, valorAnterior)) {
    // Si el valor no es aceptable, usa el último valor válido
    resistenciaCalculada = valorAnterior;
  }

  // Actualiza el filtro de media móvil
  suma = suma - lecturas[indice];           // Elimina el valor más antiguo de la suma
  lecturas[indice] = resistenciaCalculada; // Reemplaza con el nuevo valor
  suma = suma + resistenciaCalculada;      // Agrega el nuevo valor a la suma
  indice = (indice + 1) % numeroMuestras;  // Incrementa el índice circular

  if (indice == numeroMuestras) indice = 0;


  // Calcula la media filtrada
  mediaFiltrada = suma / (long)numeroMuestras;

  mediaFiltrada = mediaFiltrada - offset;

  // Actualiza el valor anterior
  valorAnterior = resistenciaCalculada;
  
  // Imprime los resultados
  
  //Serial.print("Resistencia calculada (original): ");
  //Serial.print(resistenciaCalculada);
  //Serial.print(" Ohm\tResistencia filtrada: ");
  //Serial.println(mediaFiltrada);
  
  //Serial.print(resistenciaCalculada,4);
  //Serial.print(" ");
  Serial.println(mediaFiltrada,4);
  //Serial.println((valorADC*Vcc)/4095.0,4);

  // Espera antes de la próxima lectura (en ms), equivale al tiempo de muestreo = 1 / frecuencia de muestreo
  delay(1); // Frecuencia de muestreo = 125 Hz
}



// Función para calcular la resistencia variable a partir del valor ADC
double calcularResistencia(int valorADC) {
  // Calcula la tensión de salida del divisor de tensión
  double Vout = (valorADC * Vcc) / 4095.0000;

  // Calcula la resistencia variable usando la fórmula del divisor de tensión
  return R_fija * (Vout / (Vcc - Vout));
  //return Vout;
}

// Función para verificar si un valor es aceptable
bool valorAceptable(double valorActual, double valorAnterior) {
  // Calcula la variación porcentual y agrega el umbral fijo
  double variacion = abs(valorActual - valorAnterior);
  double umbral = (toleranciaPorcentual * valorAnterior) + umbralFijo;

  // Verifica si la variación está dentro del límite
  return variacion <= umbral;
}
