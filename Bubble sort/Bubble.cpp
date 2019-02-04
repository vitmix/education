#include "Bubble.h"

void BubbleSortOld(std::vector<int>& array)
{
	if (array.empty()) throw std::exception("Vector is empty");

	bool continueFlag = false;
	do
	{
		continueFlag = false;
		for (int i = 0; i < array.size() - 1; ++i)
		{
			if (array[i] > array[i + 1])
			{
				std::swap(array[i], array[i + 1]);
				continueFlag = true;
			}
		}
	} while (continueFlag);
}

void BubbleSortNew(std::vector<int>& arr)
{
	bool continueFlag = arr.empty();
	while (!continueFlag)
	{
		continueFlag = true;
		for (size_t i = 0; i < arr.size() - 1; ++i)
		{
			if (arr[i] > arr[i + 1])
			{
				std::swap(arr[i], arr[i + 1]);
				continueFlag = false;
			}
		}
	}
}
