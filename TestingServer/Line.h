#pragma once

#include <vector>

#include "Vector3.h"

class Line_
{
public:
	std::vector<Vector3<int>> vertices;
	Line_() { vertices.resize(2); }
	Line_(const Vector3<int>& v1, const Vector3<int>& v2) :vertices({ v1, v2 }) {}
	Vector3<int>& operator[](int index) {
		return vertices[index];
	}
	Vector3<int> GetVector() {
		return vertices[1] - vertices[0];
	}
};
