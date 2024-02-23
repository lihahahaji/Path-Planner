# A*

## 算法特点

- 启发式搜索算法
- 适用于静态环境中的路径规划
- 求解源点到目标点之间的最短路径，需要指明目标点（即目标点位置已知，这样才能有效的计算$h(x)$）。

## 1 基本概念

- 启发式方法:

    > 一种根据经验规则进行发现的方法

- 启发函数 $h(x)$:

    > 1. 曼哈顿距离: 只允许上下左右移动的情况
    >     $$
    >     D= |x_1-x_2| + |y_1-y_2|
    >     $$
    >
    > 2. 欧几里得距离: 允许朝任意方向移动
    >     $$
    >     D=\sqrt{(x_1-x_2)^2+(y_1-y_2)^2}
    >     $$
    >
    > 3. 其他: 例如对角距离等

## 2 算法实现

### 评价函数

$$
f(n)=g(n)+h(n)
$$

- $g(n)$ 表示从起点 $S$ 到的最优路径的**实际成本**, 即从起点到当前节点已经走过的路程。（其实可以理解为 Dijkstra 算法中的 dis 数组）
- $h(n)$ 表示从当前节点到目标节点 $T$ 的预估成本, 使用启发函数来计算, 根据具体问题, 一般为曼哈顿距离或者欧氏距离

### 算法输入

要具体研究一个算法，首先定义输入结构：

- 起点 $S$ 坐标已知 $(x_1,y_1)$，终点 E 坐标已知 $(x_2,y_2)$。
- 输入地图为一个 $N*N$ 的方阵，其中每个元素代表一个实际地图的像素，假设每一个相邻像素之间的距离为 $1$，斜对角相邻元素的距离为 $\sqrt{2}$ ，其中有一些像素为障碍物。



这里首先定义两个集合:

- 开放列表（OpenList）：已经访问过，但是没有确定最短路径的节点。
- 关闭列表（CloseList）：已经确定了源点到其的最短路径的节点。

### 算法流程

1. 将起点 $S$ 放入开放列表 (openList) 中.

2. 遍历**开放列表**
   1. 计算列表中的每一个节点 $n$ 的评价函数$f(n)$ , 查找$f(n)$ 函数值最小的节点, 将这个节点作为当前要处理的节点$N$
   
   2. 对于当前正在处理的节点 $N$ , 遍历与之相邻的其他所有节点 $N_{near}$ 
      - 假设当前正在处理的相邻节点为 $N_{near}$ 
        1.  $N_{near}$ 已经在关闭列表中（已经确定最短路径），或者不可达（障碍物），则忽略该节点
        
        2. $N_{near}$ 不在开放列表中（未被访问过），则将其加入开放列表，将当前节点 $N$  设置为 $N_{near}$ 的父结点，计算节点 $N_{near}$ 的评价函数 $f(N_{near})$ ,$g(N_{near})$
        
        3. $N_{near}$ 已经在开放列表中（已经访问过但未确定最短路径），则需要检查这条路径（节点 $N$ 到节点 $N_{near}$ 的路径）是否更好，参考指标为  $g(N_{near})$ ,若$g(N_{near})$ 更小，则说明该路径更好，则将它的父亲节点（设为节点）设置为当前节点 $N$ ，并重新计算$f(N_{near})$ 。
        
           > $g(N_{near}) = min\left\{\ g(N)+distace(N,N_{near})\ ,\ g(N_{near}) \ \right\}$
   
   3. 当前节点 $N$ 处理完毕， 移动到关闭列表，表示该节点已经不需要被继续关注。
   
3. 重复步骤「2」直到满足以下条件之一，算法结束：

   1. 终点进入了开放（找到了起点到终点的路径）
   2. 开放列表为空，仍未找到终点（无法找到起点到终点的路径）

4. 路径回溯：如果在步骤「3」中，存在从终点到起点的路径，则回溯找到最短路径：

   从终点开始，每一个节点都沿着父亲节点移动，直到找到起点。



## 实例

### 问题描述

一个机器人 Rob 在一个 M*M 的地图中想目标点 D 前进,已知起点 S 和目标点 D 的坐标,求 S 到 D 的最短路径.

### 输入格式

m

x1 y1

x2 y2

m*m 的地图

### 输入样例

```
10
0 0
9 9
S*********
**#######*
#*****#**#
*****#****
#***#*****
**********
#*#**#****
**********
*****#****
*********D
```





回溯路径之后，将路径上点的坐标，在图上输出出来，显示路径。

```c++
#include <iostream>
#include <vector>
#include <list>
#include <queue>
#include <stack>
#include <map>
#include <unordered_map>
#include <set>
#include <unordered_set>
#include <algorithm>
#include <cstdio>
#include <cmath>
#include <string>
#include <cstring>
#include <complex>
#include <iterator>
using namespace std;

#define INF 0x3f3f3f3f
int m;
string mp[20];
int directions[4][2] = {
	{0, -1}, // 上
	{0, 1},	 // 下
	{-1, 0}, // 左
	{1, 0}	 // 右
};

struct Node
{
	// 定义构造函数
	Node() {}
	Node(int x, int y)
	{
		this->x = x;
		this->y = y;
		this->index = x * m + y;
		this->father = 0;
	}
	int x;
	int y;
	int index;
	int father;
};

Node node[200];

Node s, e;

// 定义评价函数
int f[200];
int g[200];
int h[200];

// 定义开放列表和关闭列表
set<int> OpenList;
set<int> CloseList;

// 下标转为坐标
pair<int, int> indexToCoordinates(int index)
{
	int x = index / m;
	int y = index % m;
	return pair<int, int>(x, y);
}

// 坐标转为下标
int CoordinatesToindex(int x, int y)
{
	return x * m + y;
}

// 计算两个点之间的曼哈顿距离
int manhattanDistance(int point1, int point2)
{
	pair<int, int> p1 = indexToCoordinates(point1);
	pair<int, int> p2 = indexToCoordinates(point2);
	return abs(p1.first - p2.first) + abs(p1.second - p2.second);
}

// 判断节点是否可达
bool isValidNode(int x, int y)
{
	if (x < 0 || x >= m || y < 0 || y >= m)
		return false;
	if (mp[x][y] == '#')
		return false;
	return true;
}

// 输出开放列表中的元素
void print_OpenList()
{
	cout << "OpenList: ";
	for (set<int>::iterator i = OpenList.begin(); i != OpenList.end(); i++)
	{
		cout << *i << ' ';
	}
	cout << endl;
}

void A_star()
{
	// 将起点 S 放入开放列表
	OpenList.insert(s.index);

	// 当开放列表不为空
	while (OpenList.size())
	{

		// print_OpenList();
		// 终点进入开放列表，找到路径，算法结束。
		if (OpenList.find(e.index) != OpenList.end())
			return;

		// 遍历开放列表，找到评价函数 f 的值最小的节点
		int min_f = INF;
		int n_ind = -1;

		for (set<int>::iterator i = OpenList.begin(); i != OpenList.end(); i++)
		{
			if (f[*i] < min_f)
			{
				min_f = f[*i];
				n_ind = *i;
			}
		}

		// 找到当前需要处理的节点 N 的下标 n_ind
		// 遍历节点 N 的邻接节点(上下左右)
		pair<int, int> n_coord = indexToCoordinates(n_ind);
		for (int i = 0; i < 4; i++)
		{
			int n_near_x = n_coord.first + directions[i][1];
			int n_near_y = n_coord.second + directions[i][0];
			int n_near_ind = CoordinatesToindex(n_near_x, n_near_y);

			if (!isValidNode(n_near_x, n_near_y) || CloseList.find(n_near_ind) != CloseList.end())
			{
				// 当前节点不可达 或者 已经在关闭列表中
				// 忽略
				// cout<<n_near_ind<<endl;
			}
			else if (OpenList.find(n_near_ind) == OpenList.end())
			{
				// 不在开放列表中（未被访问过），则将其加入开放列表
				OpenList.insert(n_near_ind);

				// 计算评估函数
				g[n_near_ind] = g[n_ind] + 1;
				f[n_near_ind] = g[n_ind] + 1 + manhattanDistance(n_near_ind, e.index);
				// 将 n 设为 n_near 的父结点
				node[n_near_ind].father = n_ind;
			}
			else
			{
				// 已经在开放列表中，更新 f
				if (g[n_near_ind] > g[n_ind] + 1)
				{
					// 如果有更优的路径
					g[n_near_ind] = g[n_ind] + 1;
					// 更新 f 的值
					f[n_near_ind] = g[n_near_ind] + manhattanDistance(n_near_ind, e.index);
					// 将 n 设为 n_near 的父结点
					node[n_near_ind].father = n_ind;
				}
			}
		}
		// 节点 n 已经处理完毕，移动到关闭列表。
		OpenList.erase(n_ind);
		CloseList.insert(n_ind);
	}
}

void init()
{
	cin >> m;
	int x, y;
	// 输入起点和终点的坐标
	cin >> x >> y;
	s = Node(x, y);
	cin >> x >> y;
	e = Node(x, y);
	// 输入地图数据
	for (int i = 0; i < m; i++)
		cin >> mp[i];

	memset(f, INF, sizeof(f));
	memset(g, INF, sizeof(g));
	memset(h, INF, sizeof(h));

	f[s.index] = manhattanDistance(s.index, e.index);
	g[s.index] = 0;
	h[s.index] = manhattanDistance(s.index, e.index);

	for (int i = 0; i < m; i++)
	{
		node[i].index = i;
		node[i].x = indexToCoordinates(i).first;
		node[i].y = indexToCoordinates(i).second;
		node[i].father = -1;
	}
}

void solve()
{
	init();
	A_star();

	// 输出访问过的节点
	for (int i = 0; i < m; i++)
	{
		for (int j = 0; j < m; j++)
		{
			if (CloseList.find(CoordinatesToindex(i, j)) == CloseList.end())
				cout << mp[i][j];
			else
				cout << 'A';
		}
		cout << endl;
	}

	// 输出最短路径
	set<int> path;
	int p = 99;
	while (p != 0)
	{
		path.insert(p);
		p = node[p].father;
	}

	for (int i = 0; i < m; i++)
	{
		for (int j = 0; j < m; j++)
		{
			if (path.find(CoordinatesToindex(i, j)) == path.end())
				cout << mp[i][j];
			else
				cout << '%';
		}
		cout << endl;
	}
}

int main()
{
	ios::sync_with_stdio(0);
	cin.tie(0);
#ifndef ONLINE_JUDGE
	freopen("/Applications/CppRunner.app/Contents/Resources/cpp/input.txt", "r", stdin);
#endif
	solve();
}
```

输出：

```
被访问过的节点:
A A A A A A A A A A 
A A # # # # # # # A 
# A A A A A # * * # 
* A A A A # * * * * 
# A A A # * * * * * 
* A A A A A A A A A 
# A # A A # A A A A 
* A A A A A A A A A 
* A A A A # A A A A 
* * * * * * * * * D 

最短路径:
S % * * * * * * * * 
* % # # # # # # # * 
# % % % * * # * * # 
* * * % * # * * * * 
# * * % # * * * * * 
* * * % % % % % % % 
# * # * * # * * * % 
* * * * * * * * * % 
* * * * * # * * * % 
* * * * * * * * * % 
```

