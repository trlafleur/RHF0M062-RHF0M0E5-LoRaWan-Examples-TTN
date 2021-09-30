# Librería Arduino Circular Buffer
La librería Circular Buffer implementa un Buffer Circular en Arduino. La clase Circular Buffer emplea templates para permitir funcionar con distintos tipos (int, long, float, …).

Más información https://www.luisllamas.es/libreria-arduino-circular-buffer/

## Instrucciones de uso
### Constructor
El Buffer Circular se instancia a través de su constructor que recibe el tamaño del Buffer como único parámetro.
```c++
CircularBuffer<T> circularBuffer(capacity);
```

### Obtener elementos
```c++
// Obtiene el elemento N
circularBuffer[N];

//Obtiene el primer elemento
circularBuffer.First();

//Obtiene los N primeros elementos
circularBuffer.First(numItems);

//Obtiene el último elemento
circularBuffer.Last();

//Obtiene los N últimos elementos
circularBuffer.Last(numItems);
```

### Añadir elementos
```c++
// Añade un elemento al final del buffer
circularBuffer.Add(T item);

// Inserta un elemento al principio del buffer
circularBuffer.Insert(T item);
```

### Eliminar elementos
```c++
// Elimina el primer elemento del buffer
circularBuffer.RemoveFirst();

// Elimina el último elemento del buffer
circularBuffer.RemoveLast();
```

### Extraer elementos
Obtienen y eliminan un elemento, útil para implementar colas y pilas de forma sencilla.
```c++
// Elimina el primer elemento, y devuelve el valor anterior
circularBuffer.ExtractFirst();

// Elimina el último elemento, y devuelve el valor anterior
circularBuffer.ExtractLast();
```

### Sustituir elementos
```c++
// Sustituye el primer elemento
circularBuffer.ReplaceFirst(item);

// Sustituye el último elemento
circularBuffer.ReplaceLast(item);

// Sustituye el elemento en el índice indicado
circularBuffer.ReplaceAt(index, item);
```

### Conversión
```c++
// Devuelve el buffer como un Array
circularBuffer.ToArray();

// Copia el contenido del buffer a un Array
circularBuffer.CopyTo(items);

// Rellena el buffer a partir del contenido de un Array
circularBuffer.FromArray(items, numItems);
```

## Ejemplos
La librería Circular Buffer incluye los siguientes ejemplos para ilustrar su uso.
* CircularBuffer: Ejemplo de uso para variables integer.
```c++
#include "CircularBufferLib.h"

void printArray(int* x, int length, char separator)
{
	for (int iCount = 0; iCount < length-1; iCount++)
	{
		Serial.print(x[iCount]);
		Serial.print(separator);
	}
	Serial.print(x[length - 1]);
	Serial.println();
}


CircularBuffer<int> circularBuffer(10);
void setup() {
	Serial.begin(115200);

	Serial.println("Store 0 - 14");
	for (int iCount = 1; iCount <= 25; iCount++)
		circularBuffer.Add(iCount);

	for (int iCount = 1; iCount <= 2; iCount++)
	{
		circularBuffer.ExtractFirst();
		circularBuffer.ExtractFirst();

		circularBuffer.Add(iCount + 100);
	}

	Serial.print("Count:");
	Serial.print(circularBuffer.Count());
	Serial.print("\t\tFirst:");
	Serial.print(circularBuffer.First());
	Serial.print("\t\tLast:");
	Serial.print(circularBuffer.Last());
	Serial.print("\t\tData:");
	printArray(circularBuffer.ToArray(), circularBuffer.Count(), ',');
}

void loop() {
}
```
