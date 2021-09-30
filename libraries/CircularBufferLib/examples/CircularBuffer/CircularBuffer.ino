/***************************************************
Copyright (c) 2017 Luis Llamas
(www.luisllamas.es)

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License
 ****************************************************/

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
