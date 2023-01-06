#include <iostream>
#include <vector>
#include <map>
#include <set>

#include "Vector3.h"
#include "Line.h"
#include "Collision.h"
#include "Path.h"

std::vector<Vector3<int>> vertices = {
	Vector3<int>(  0,  0, 0),	//0
	Vector3<int>( 20,  0, 0),
	Vector3<int>( 40,  0, 0),
	Vector3<int>( 60,  0, 0),
	Vector3<int>(  0, 20, 0),	//4
	Vector3<int>( 20, 20, 0),
	Vector3<int>( 40, 20, 0),
	Vector3<int>( 60, 20, 0),
	Vector3<int>(120, 10, 0),	//7
	Vector3<int>( 60, 30, 0),
	Vector3<int>( 85, 30, 0),
	Vector3<int>(140, 30, 0),	//11
	Vector3<int>(110, 40, 0),
	Vector3<int>( 80, 50, 0),	//13
	Vector3<int>(120, 50, 0),
	Vector3<int>( 40, 60, 0),
	Vector3<int>( 20, 80, 0),	//16
	Vector3<int>( 40, 80, 0),
	Vector3<int>( 60, 76, 0),	//18
	Vector3<int>( 80, 73, 0),	//19
	Vector3<int>(100, 70, 0),
	Vector3<int>(120, 70, 0),
	Vector3<int>(140, 60, 0),	//22
	Vector3<int>( 10,130, 0),
	Vector3<int>( 30,130, 0),
};
std::vector<int> indices = {
	0, 1, 4,	//0
	1, 5, 4,	//1
	1, 2, 5,	//2
	2, 6, 5,	//3
	2, 3, 6,	//4
	3, 7, 6,	//5
	3, 10, 7,	//6
	10, 9, 7,	//7
	5, 6, 15,	//8
	15, 16, 5,	//9
	15, 17, 16,	//10
	15, 13, 17,	//11
	13, 18, 17,	//12
	13, 19, 18,	//13
	13, 20, 19,	//14
	9, 10, 13,	//15
	10, 12, 13,	//16
	12, 14, 13,	//17
	14, 20, 13,	//18
	12, 8, 11,	//19
	11, 14, 12,	//20
	14, 11, 22,	//21
	22, 21, 14,	//22
	18, 19, 24,	//23
	24, 23, 18,	//24
};
typedef Line Wall;
typedef std::vector<Wall> Walls;

class ConvexPolygon;

// 三角形を順番につなげて、凹みが無い間はずっとひとまとめにし続ける
class ConvexPolygon
{
public:
	// 隣接情報
	class AdjacentData
	{
	public:
		Wall wall;
		ConvexPolygon* neighbour_polygon = nullptr;
	};
public:
	int current_index;
	int prev_path_index;
	std::vector<Vector3<int>> vertices;
	std::vector<Line> lines;
	std::vector<AdjacentData> adjacents;
	Vector3<int> center_pos;
	int cost = 0; // ここまで最短合計コスト
	int heuristic_cost = 0; // ゴールまでの予測コスト（距離）
	bool operator==(const ConvexPolygon& v) const
	{
		return vertices == v.vertices;
	}
	bool IsAdjacent(const ConvexPolygon& v)
	{
		for (auto&& line : lines)
		{
			for (auto&& vline : v.lines)
			{
				if ((line[0] == vline[0] && line[1] == vline[1]) ||
					(line[0] == vline[1] && line[1] == vline[0]))
				{
					return true;
				}
			}
		}
		return false;
	}
	Line GetAdjacentWall(const ConvexPolygon& v)
	{
		for (auto&& line : lines)
		{
			for (auto&& vline : v.lines)
			{
				if ((line[0] == vline[0] && line[1] == vline[1]) ||
					(line[0] == vline[1] && line[1] == vline[0]))
				{
					return line;
				}
			}
		}
		return Line();
	}
	bool IsPointInPolygon(const Vector3<int>& pos) const
	{
		for (int i = 0; i < vertices.size(); i++) {
			int next = (i + 1) % vertices.size();
			if (Collision::OuterProduct(vertices[i] - vertices[next], vertices[i] - pos) < 0) {
				return false;
			}
		}
		return true;
	}
};

class RoutingTable
{
public:
	//current_index, goal_index, next_index
	std::map<int, std::map<int, int>> routings;
	std::map<int, int>& operator[](int i) { return routings[i]; }
	Path PathFind(std::vector<ConvexPolygon>& polygons, int current_index, int goal_index)
	{
		Path path(current_index, goal_index);
		ConvexPolygon* target_polygon = nullptr;
		// コストをリセットする
		for (int i = 0; i < polygons.size(); i++)
		{
			polygons[i].cost = INT_MAX;
			polygons[i].heuristic_cost = -1;
			if (i == goal_index)
			{
				target_polygon = &polygons[i];
			}
		}
		if (target_polygon == nullptr) return path; // invalid argument
		// コスト計算する
		std::vector<int> open_list, close_list;
		open_list.emplace_back(current_index);
		polygons[current_index].cost = 0;
		while (open_list.size() != 0)
		{
			int index = *open_list.begin();
			open_list.erase(open_list.begin());

			for (auto&& adjacent : polygons[index].adjacents)
			{
				auto target = adjacent.neighbour_polygon;
				// 隣接ノードをチェックする
				auto&& it = std::find(close_list.begin(), close_list.end(), target->current_index);
				if (it != close_list.end()) continue;// すでにクローズ済み
				// ヒューリスティックコストを計算
				if (target->heuristic_cost == -1)
				{
					target->heuristic_cost = Collision::CalcSqDistance(target->center_pos, polygons[goal_index].center_pos);
				}
				int edge_cost = Collision::CalcSqDistance(target->center_pos, polygons[index].center_pos);
				int cost = polygons[index].cost + edge_cost + target->heuristic_cost;
				// 合計コストの比較
				if (target->cost < cost) continue; // コスト更新ならず
				target->cost = cost;
				open_list.emplace_back(target->current_index);
				target->prev_path_index = index;
				//std::cout << index << " -> " << target->current_index 
				//	<< " polygons.cost:" << polygons[index].cost
				//	<< " heuristic_cost:" << target->heuristic_cost
				//	<< " cost:" << cost << std::endl;
			}
			close_list.emplace_back(index);
		}
		//std::cout << "Start:" << current_index << ", Goal:" << goal_index << std::endl;
		// 逆順にチェックしてルートを確定する（経路テーブルに反映する）
		ConvexPolygon* p = target_polygon;
		while (p)
		{
			//std::cout << p->current_index << " -> ";
			path.AddFront(p->current_index);
			routings[p->current_index][goal_index] = polygons[p->prev_path_index].current_index;
			if (p->current_index == current_index) break;
			p = &polygons[p->prev_path_index];
		}
		//std::cout << std::endl;
		return path;
	}
	static RoutingTable Create(std::vector<ConvexPolygon>& polygons)
	{
		RoutingTable routing_table;
		for (int i = 0; i < polygons.size(); i++)
		{
			for (int j = 0; j < polygons.size(); j++)
			{
				routing_table[i][j] = -1;
			}
		}
		for (int i = 0; i < polygons.size(); i++)
		{
			for (int j = 0; j < polygons.size(); j++)
			{
				if (i == j) continue;//同じノードに対しては省略
				if (routing_table[i][j] != -1) continue;//探索済みなら省略
				// 経路探索
				routing_table.PathFind(polygons, i, j);
			}
		}
		return routing_table;
	}
	void Print()
	{
		for (int i = 0; i < routings.size(); i++)
		{
			for (int j = 0; j < routings[i].size(); j++)
			{
				if (i == j) continue;
				std::cout << "RoutingTable[" << i << "][" << j << "] = " << routings[i][j] << std::endl;
			}
		}
	}
};

// 隣接情報を設定する。
Walls MakeUpAdjacentData(std::vector<ConvexPolygon>& polygons)
{
	Walls walls;
	for (auto&& polygon : polygons)
	{
		polygon.adjacents.clear();
		for (auto&& adjacent_polygon : polygons)
		{
			if (polygon == adjacent_polygon) continue;
			if (polygon.IsAdjacent(adjacent_polygon))
			{
				ConvexPolygon::AdjacentData adjecent;
				adjecent.wall = std::move(polygon.GetAdjacentWall(adjacent_polygon));
				adjecent.neighbour_polygon = &adjacent_polygon;
				polygon.adjacents.emplace_back(adjecent);
				walls.emplace_back(polygon.GetAdjacentWall(adjacent_polygon));
			}
		}
	}
	return walls;
}

std::vector<Line> CreateLines(std::vector<Line> line1, std::vector<Line> line2)
{
	int insert_index = -1;
	bool is_adjacent = false;
	// insertするべき場所を求める
	for (int i = 0; i < line1.size(); i++)
	{
		// line1のものがline2の中にあるか？
		Line& line = line1[i];
		for (auto&& it = line2.begin(); it != line2.end(); it++)
		{
			if (line1[i][0] == (*it)[1] &&
				line1[i][1] == (*it)[0])
			{
				is_adjacent = true;
				line1.erase(line1.begin() + i);
				line2.erase(it);
				if (insert_index == -1) insert_index = i;
				break;
			}
		}
	}
	if (insert_index >= line1.size())
	{
		line1.insert(line1.end(), line2.begin(), line2.end());
	}
	else
	{
		line1.insert(line1.begin() + insert_index, line2.begin(), line2.end());
	}
	if (false == is_adjacent) return {};
	// 並び変え
	for (int i = 0; i < line1.size() - 1; i++)
	{
		for (int j = i + 1; j < line1.size(); j++)
		{
			if (line1[i][1] == line1[j][0])
			{
				if (i + 1 == j) break;//swap無し
				auto temp = line1[i + 1];
				line1[i + 1] = line1[j];
				line1[j] = temp;
				break;
			}
		}
	}
	return line1;
}
// 凸集合ポリゴンのリストをセットアップする（一度で完全な状態にはならない）
int SetupConvexSet(std::vector<ConvexPolygon>& polygons)
{
	int combine_count = 0;
	std::vector<ConvexPolygon> combine_polygons;
	for (int i = 0; i < polygons.size(); i++)
	{
		if (polygons[i].vertices.size() == 0) continue;
		ConvexPolygon convex_polygon = polygons[i];
		for (int j = i + 1; j < polygons.size(); j++)
		{
			if (polygons[j].vertices.size() == 0) continue;

			auto&& combine_lines = CreateLines(convex_polygon.lines, polygons[j].lines);
			if (Collision::IsConvexSet(combine_lines))
			{
				combine_count++;
				convex_polygon.vertices.clear();
				for (auto&& line : combine_lines) {
					convex_polygon.vertices.emplace_back(line[0]);
				}
				convex_polygon.lines = combine_lines;

				polygons[j].vertices.clear();
			}
		}
		combine_polygons.emplace_back(convex_polygon);
	}
	polygons = combine_polygons;
	return combine_count;
}

static int GetIndex(const std::vector<ConvexPolygon>& polygons, const Vector3<int>& pos)
{
	for (int i = 0; i < polygons.size(); i++) {
		if (polygons[i].IsPointInPolygon(pos)) {
			return i;
		}
	}
	return -1;
}

Vector3<int> GetNextPos(RoutingTable& routing_table, std::vector<ConvexPolygon>& polygons, const Vector3<int>& current_pos, const Vector3<int>& target_pos)
{
	int start = GetIndex(polygons, Vector3<int>(10, 10, 0));
	int goal = GetIndex(polygons, Vector3<int>(30, 120, 0));
	auto&& path = routing_table.PathFind(polygons, start, goal);
	Vector3<int> next;
	// ゴールまで直線的に行けるか？


	return next;
}

void OutputIndex(std::vector<ConvexPolygon>& polygons, const Vector3<int>& search_pos)
{
	int point_index = GetIndex(polygons, search_pos);
	std::cout << "Pos(" << search_pos.x << "," << search_pos.y << ") point_index:" << point_index << std::endl;
}

int main()
{
	std::cout << "Testing Server" << std::endl;
	// 三角形のポリゴンをリストアップする
	std::vector<ConvexPolygon> polygons;
	for (int i = 0; i < indices.size();)
	{
		ConvexPolygon polygon;
		polygon.vertices.emplace_back(vertices[indices[i++]]);
		polygon.vertices.emplace_back(vertices[indices[i++]]);
		polygon.vertices.emplace_back(vertices[indices[i++]]);
		polygon.lines.emplace_back(polygon.vertices[0], polygon.vertices[1]);
		polygon.lines.emplace_back(polygon.vertices[1], polygon.vertices[2]);
		polygon.lines.emplace_back(polygon.vertices[2], polygon.vertices[0]);
		polygons.emplace_back(polygon);
	}
	// 凸集合リストを作成(合成しなくなるまで繰り返す)
	while (SetupConvexSet(polygons) > 0);

	// 結合
	std::cout << "polygons.size:" << polygons.size() << std::endl;
	for (int i = 0; i < polygons.size(); i++)
	{
		polygons[i].center_pos = polygons[i].vertices[0];
		polygons[i].current_index = i;
	}
	
	// ③隣接するポリゴンとの接点インデックスを保持する
	MakeUpAdjacentData(polygons);
	
	// ④ポリゴン総当たり経路探索を行って経路テーブルを作る。
	auto&& routing_table = RoutingTable::Create(polygons);
	// 経路テーブル表示
	//routing_table.Print();

	// ⑤目的座標を元に次に行くべき座標を決定する。
	// 直線的に移動できるかチェック→壁の交差判定
	int start = GetIndex(polygons, Vector3<int>(10, 10, 0));
	int goal = GetIndex(polygons, Vector3<int>(30, 120, 0));
	auto&& path = routing_table.PathFind(polygons, start, goal);
	path.Print();
	
	return 0;
}
