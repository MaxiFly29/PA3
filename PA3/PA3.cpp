#include <iostream>
#include <cmath>
#include <queue>
#include <vector>
#include <fstream>

using namespace std;


double AntWay(size_t vert, vector<bool>& is_visited,
	const vector<vector<double>>& distance, vector<vector<double>>& ant_track,
	queue<size_t>& vert_order);

double AntAlgorithm(const vector<vector<double>>& distance, size_t iter_cnt = 10'000, ostream& out = clog)
{
	const double ro = 0.7;
	const double Q = 0.1;
	queue<size_t> way;

	vector<vector<double>> ant_track{ distance.size(), vector<double>(distance.size(), 0) };
	double res = numeric_limits<double>::max();
	for (size_t i = 0; i < iter_cnt; i++) {
		for (size_t j = 0; j < 20; j++)
		{
			vector<bool> is_visited(distance.size(), 0);
			queue<size_t> vert_order;
			size_t cur_node = rand() % distance.size();
			double local_res = AntWay(rand() % distance.size(), is_visited, distance, ant_track, vert_order);
			if (local_res < res)
				way = vert_order;
			size_t b_node = vert_order.front(), e_node;
			vert_order.pop();
			while (!vert_order.empty())
			{
				e_node = vert_order.front();
				vert_order.pop();
				auto& track = ant_track[b_node][e_node];
				track = track + Q / local_res;
				b_node = e_node;
			}
			res = min(local_res, res);
		}
		for (auto& row : ant_track)
			for (auto& el : row)
				el *= (1 - ro);
		if (i % 1 == 0)
			out << i << '\t' << res << endl;
	}
	while (!way.empty())
	{
		cout << way.front() << ' ';
		way.pop();
	}
	cout << endl;
	return res;
}

double AntWay(size_t vert, vector<bool>& is_visited,
	const vector<vector<double>>& distance, vector<vector<double>>& ant_track,
	queue<size_t>& vert_order)
{
	vert_order.push(vert);
	is_visited[vert] = true;
	const double alpha = 2;
	const double beta = 4;
	vector<double> koefs(distance.size());
	double koef_sum = 0;
	//Get rand vertex----------
	for (size_t i = 0; i < distance.size(); i++)
	{
		if (i == vert || is_visited[i])
			continue;
		koefs[i] = 1 / pow(distance[vert][i], beta) + pow(ant_track[vert][i], alpha);
		koef_sum += koefs[i];
	}
	double rand_koef = 1.0 * static_cast<double>(rand() % 10'000) / 1e4;
	size_t next_vert = 0;
	while (next_vert == vert || is_visited[next_vert])
		next_vert++;
	rand_koef -= koefs[next_vert] / koef_sum;
	while (rand_koef >= 0. && next_vert < distance.size())
	{
		do
			next_vert++;
		while (next_vert == vert || is_visited[next_vert]);
		/*if (next_vert >= distance.size())
			break;*/
		rand_koef -= koefs[next_vert] / koef_sum;
	}
	//-------------------------
	size_t cnt_visited = 0;
	for (const bool f : is_visited)
		if (f)
			cnt_visited++;
	if (cnt_visited < is_visited.size() - 1)
	{
		return distance[vert][next_vert] + AntWay(next_vert, is_visited, distance, ant_track, vert_order);
	}
	vert_order.push(next_vert);
	return distance[vert][next_vert];
}

int main()
{
	ofstream out("logs.txt");
	vector<vector<double>> distances = {
		{0, 18, 40, 27, 15, 4, 13, 38, 15},
		{18, 0, 33, 9, 19, 26, 18, 8, 35},
		{38, 33, 0, 17, 22, 14, 26, 22, 11},
		{25, 10, 15, 0, 33, 22, 6, 20, 5},
		{15, 21, 21, 31, 0, 10, 26, 33, 27},
		{6, 27, 16, 24, 10, 0, 22, 25, 32},
		{12, 19, 26, 5, 25, 21, 0, 28, 20},
		{36, 7, 24, 21, 31, 27, 26, 0, 13},
		{15, 33, 10, 5, 27, 32, 19, 12, 0}
	};
	cout << AntAlgorithm(distances, 20, out);

}