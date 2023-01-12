#pragma once

#include <vector>

#include "Vector3.h"
#include "Line.h"

class Collision
{
public:
	template <typename T>
	static T CalcSqDistance(const Vector3<T>& pos1, const Vector3<T>& pos2)
	{
		return
			(pos1.x - pos2.x) * (pos1.x - pos2.x) +
			(pos1.y - pos2.y) * (pos1.y - pos2.y);
	}

	template <typename T>
	static T OuterProduct(const Vector3<T>& v1, const Vector3<T>& v2)
	{
		return v1.x * v2.y - v1.y * v2.x;
	}
	
	// 凸集合か
	static bool IsConvexSet(std::vector<Line>& lines)
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
	
	static bool IsIntersectLineAndLineSegment(const Line& line1, const Line& line2, bool is_equal = true)
	{
		auto&& vector1 = line1.GetVector();
		auto op1 = OuterProduct(vector1, line2[0] - line1[0]);
		auto op2 = OuterProduct(vector1, line2[1] - line1[0]);
		if (is_equal)
		{
			if (op1 * op2 <= 0)
			{
				auto&& vector2 = line2.GetVector();
				auto op3 = OuterProduct(vector2, line1[0] - line2[0]);
				auto op4 = OuterProduct(vector2, line1[1] - line2[0]);
				if (op3 * op4 <= 0)
				{
					return true;
				}
			}
		}
		else
		{
			// equal無しの需要もあった
			if (op1 * op2 < 0)
			{
				auto&& vector2 = line2.GetVector();
				auto op3 = OuterProduct(vector2, line1[0] - line2[0]);
				auto op4 = OuterProduct(vector2, line1[1] - line2[0]);
				if (op3 * op4 < 0)
				{
					return true;
				}
			}
		}
		return false;
	}
	static bool IsIntersectLineSegment(const Line& line1, const Line& line2, bool is_equal = true)
	{
		return IsIntersectLineAndLineSegment(line1, line2, is_equal) &&
			IsIntersectLineAndLineSegment(line2, line1, is_equal);
	}
	static Vector3<int> IntersectLine(const Line& line1, const Line& line2)
	{
		double d = (line1[0].x - line1[1].x) * (line2[1].y - line2[0].y) - (line2[1].x - line2[0].x) * (line1[0].y - line1[1].y);
		double t = ((line2[1].y - line2[0].y) * (line2[1].x - line1[1].x) + (line2[0].y - line2[1].y) * (line2[1].y - line1[1].y)) / d;
		return Vector3<int>((int)(t * line1[0].x + (1.0 - t) * line1[1].x), (int)(t * line1[0].y + (1.0f - t) * line1[1].y), 0);
	}
};
