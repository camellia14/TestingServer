﻿#pragma once

#include <iostream>
#include <vector>

#include "Vector3.h"

class Path
{
public:
	int start, goal;
	std::vector<int> indices;
	std::vector<Vector3<int>> positions;
	Path(int start, int goal):start(start), goal(goal) {}
	void Add(int index)
	{
		indices.emplace_back(index);
	}
	void AddFront(int index)
	{
		indices.emplace(indices.begin(), index);
	}
	void Print()
	{
		std::cout << "Start:" << start << ", Goal:" << goal << " ";
		for (int i = 0; i < indices.size(); i++) {
			std::cout << indices[i];
			if (i < indices.size() - 1) {
				std::cout << "->";
			}
		}
		std::cout << std::endl;
	}
};
