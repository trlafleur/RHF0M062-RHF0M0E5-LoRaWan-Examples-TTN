/***************************************************
Copyright (c) 2017 Luis Llamas
(www.luisllamas.es)

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License
 ****************************************************/

#ifndef _CircularBufferLib_h
#define _CircularBufferLib_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

template <typename T>
class CircularBuffer
{
public:
	CircularBuffer<T>(const size_t capacity);

	size_t Count() const;
	size_t Capacity() const;

	T& operator[](const size_t index);

	T First();
	T* First(const size_t numItems);
	T Last();
	T* Last(const size_t numItems);

	void Add(T item);
	void Insert(T item);
	
	void RemoveFirst();
	void RemoveLast();

	T ExtractFirst();
	T ExtractLast();

	void ReplaceFirst(T item);
	void ReplaceLast(T item);
	void ReplaceAt(const size_t index, T item);

	T* ToArray();
	void CopyTo(T* items);
	void FromArray(T* items, size_t numItems);

private:
	T* _items;
	T* _itemsEnd;

	T* _firstAccessor;
	T* _lastAccessor;
	size_t _capacity;
	size_t _count;
	
	void incCount();
	void decCount();
	void incAccessor(T** _accesor);
	void decAccessor(T** _accesor);
};

template <typename T>
CircularBuffer<T>::CircularBuffer(const size_t capacity)
{
	_items = new T[capacity];
	_itemsEnd = _items + capacity - 1;

	_firstAccessor = _items;
	_lastAccessor = _items;
	_capacity = capacity;
	_count = 0;
}

template<typename T>
size_t CircularBuffer<T>::Count() const
{
	return _count;
}

template<typename T>
size_t CircularBuffer<T>::Capacity() const
{
	return _capacity;
}


template<typename T>
T & CircularBuffer<T>::operator[](const size_t index)
{
	if (index > _count - 1) return;

	T* readAccessor = _items;
	for (size_t iCount = 0; iCount < index; iCount++)
		incAccessor(&readAccessor);

	return *readAccessor;
}

template<typename T>
T CircularBuffer<T>::First()
{
	return *_firstAccessor;
}

template<typename T>
T* CircularBuffer<T>::First(const size_t numItems)
{
	T* outItems = new T[numItems];
	T* readAccessor = _firstAccessor;

	for (size_t iCount = 0; iCount < numItems; iCount++)
	{
		outItems[iCount] = *readAccessor;
		incAccessor(&readAccessor);
	}

	return outItems;
}

template<typename T>
T CircularBuffer<T>::Last()
{
	return *_lastAccessor;
}

template<typename T>
T* CircularBuffer<T>::Last(const size_t numItems)
{
	T* outItems = new T[numItems];
	T* readAccessor = _lastAccessor;
	for (size_t iCount = 0; iCount < numItems - 1; iCount++)
		decAccessor(&readAccessor);

	for (size_t iCount = 0; iCount < numItems; iCount++)
	{
		outItems[iCount] = *readAccessor;
		incAccessor(&readAccessor);
	}

	return outItems;
}

template<typename T>
void CircularBuffer<T>::Add(T item)
{
	if (_count > 0)
		incAccessor(&_lastAccessor);

	*_lastAccessor = item;

	++_count;
	if (_count > _capacity)
	{
		_count = _capacity;
		incAccessor(&_firstAccessor);
	}

}

template<typename T>
void CircularBuffer<T>::Insert(T item)
{
	if (_count > 0)
		decAccessor(&_firstAccessor);

	*_firstAccessor = item;

	++_count;
	if (_count > _capacity)
	{
		_count = _capacity;
		decAccessor(&_lastAccessor);
	}
}

template<typename T>
void CircularBuffer<T>::RemoveFirst()
{
	incAccessor(&_firstAccessor);
	decCount();
}

template<typename T>
void CircularBuffer<T>::RemoveLast()
{
	decAccessor(&_lastAccessor);
	decCount();
}

template<typename T>
T CircularBuffer<T>::ExtractFirst()
{
	T outItem = First();
	RemoveFirst();
	return outItem;
}

template<typename T>
T CircularBuffer<T>::ExtractLast()
{
	T outItem = Last();
	RemoveLast();
	return outItem;
}

template<typename T>
void CircularBuffer<T>::ReplaceFirst(T item)
{
	if (_count == 0) return;
	*_firstAccessor = item;
}

template<typename T>
void CircularBuffer<T>::ReplaceLast(T item)
{
	if (_count == 0) return;
	*_lastAccessor = item;
}

template<typename T>
void CircularBuffer<T>::ReplaceAt(const size_t index, T item)
{
	if (_count == 0) return;

	T* writeAccessor = _firstAccessor;
	for (size_t iCount = 0; iCount < index; iCount++)
		incAccessor(&writeAccessor);

	*writeAccessor = item;
}

template<typename T>
T * CircularBuffer<T>::ToArray()
{
	T* outItems = new T[_count];
	T* readAccessor = _firstAccessor;

	for (size_t iCount = 0; iCount < _count; iCount++)
	{
		outItems[iCount] = *readAccessor;
		incAccessor(&readAccessor);
	}
	
	return outItems;
}

template <typename T>
void CircularBuffer<T>::FromArray(T* items, size_t numItems)
{
	_items = new T[numItems];
	_itemsEnd = _items + numItems - 1;

	_firstAccessor = _items;
	_lastAccessor = _items;
	_capacity = numItems;
	_count = 0;

	memmove(_items, items, numItems * sizeof(T));
}

template<typename T>
void CircularBuffer<T>::CopyTo(T* items)
{
	T* readAccessor = _firstAccessor;

	for (size_t iCount = 0; iCount < _count; iCount++)
	{
		items[iCount] = *readAccessor;
		incAccessor(&readAccessor);
	}
}


template <typename T>
void CircularBuffer<T>::incCount()
{
	if (_count < _capacity)
		++_count;
}

template <typename T>
void CircularBuffer<T>::decCount()
{
	if (_count > 0)
		--_count;
}

template <typename T>
void CircularBuffer<T>::incAccessor(T** _accesor)
{
	++(*_accesor);
	if ((*_accesor) > _itemsEnd)
		(*_accesor) = _items;
}

template <typename T>
void CircularBuffer<T>::decAccessor(T** _accesor)
{
	--(*_accesor);
	if ((*_accesor) < _items) 
		(*_accesor) = _itemsEnd;
}

#endif

