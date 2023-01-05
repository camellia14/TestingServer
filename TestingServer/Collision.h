#pragma once

#include <vector>

#include "Vector3.h"
#include "Line.h"

class Collision
{
public:
	template <typename T>
	static int OuterProduct(const Vector3<T>& v1, const Vector3<T>& v2)
	{
		return v1.x * v2.y - v1.y * v2.x;
	}
	bool IsConvexSet(std::vector<Line>& lines)
	{
		if (lines.size() < 3) return false;
		for (int i = 0; i < lines.size(); i++)
		{
			auto&& line1 = lines[i];
			auto&& line2 = lines[(i + 1) % lines.size()];
			auto&& vector1 = line1.GetVector();
			auto&& vector2 = line2.GetVector();
			if (OuterProduct(vector1, vector2) < 0)
			{
				return false;
			}
		}
		return true;
	}
	static bool IsIntersectLineAndLineSegment(Line& line1, Line& line2)
	{
		auto&& vector1 = line1[0] - line1[1];
		bool op1 = OuterProduct(vector1, line2[0] - line1[0]);
		bool op2 = OuterProduct(vector1, line2[1] - line1[0]);
		if (op1 * op2 <= 0)
		{
			auto&& vector2 = line2[0] - line2[1];
			bool op3 = OuterProduct(vector2, line1[0] - line2[0]);
			bool op4 = OuterProduct(vector2, line1[1] - line2[0]);
			if (op3 * op4 <= 0)
			{
				return true;
			}
		}
		return false;
	}
	static bool IsIntersectLineSegment(Line& line1, Line& line2)
	{
		return IsIntersectLineAndLineSegment(line1, line2) &&
			IsIntersectLineAndLineSegment(line2, line1);
	}

};
