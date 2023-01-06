#pragma once

#include <vector>

#include "Vector3.h"

class Line
{
public:
	std::vector<Vector3<int>> vertices;
	Line() { vertices.resize(2); }
	Line(const Vector3<int>& v1, const Vector3<int>& v2) :vertices({ v1, v2 }) {}
	inline Vector3<int>& operator[](int index) {
		return vertices[index];
	}
	inline const Vector3<int>& operator[](int index) const {
		return vertices[index];
	}
	inline Vector3<int> GetVector() const {
		return vertices[1] - vertices[0];
	}
	inline bool operator==(const Line& line) const
	{
		return vertices[0] == line.vertices[0] && vertices[1] == line.vertices[1];
	}
};
