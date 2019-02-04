#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility> // for std::pair
#include <deque>
#include <algorithm>
#include <functional>
#include <queue>

template <typename DataT>
class NodesContainer
{
public:
	void AddNode(DataT newNode);

	DataT GetNode(size_t id);

	size_t GetSize() const;

	size_t GetIndex(DataT & node) const;

	void PrintNodes() const;

private:
	std::deque<DataT> _container;
};

//////////////////////////////////////////////////////////////////////////////////////////////////

template <typename NodeT, typename WeightT>
class Graph
{
public:
	using adjacency_list = std::deque< std::pair<NodeT, WeightT> >;

	void AddEdge(NodeT nodeU, NodeT nodeV, WeightT weight);

	const adjacency_list GetAdjacencyList(NodeT node);

	WeightT GetWeight(NodeT nodeU, NodeT nodeV);

	void PrintGraph() const;

private:
	std::unordered_map<NodeT, adjacency_list> _container;
};

//////////////////////////////////////////////////////////////////////////////////////////////////

template <typename KeyT, typename ValT>
class Heuristic
{
public:
	void AddPair(KeyT key, ValT value);

	ValT & GetValue(const KeyT & key);

	void PrintHeuristic() const;

private:
	std::unordered_map<KeyT, ValT> _mapper;
};

//////////////////////////////////////////////////////////////////////////////////////////////////

// a function that reads data from a file into a string and passes 
// it to a parser, which can be a functor, lambda, or ordinary function
void ReadData(std::string fileName, std::function<void(std::string &)> parser);

// this function parses the data from the file (for example, input1.txt) 
// and fills the container of nodes and graph (adjacency lists)
template <typename DataT, typename NodeT, typename WeightT>
void ParseRoutes(std::string fileName, NodesContainer<DataT> & nodes, Graph<NodeT, WeightT> & graph);

// this function parses the data from the file (for example, h_kassel.txt) 
// and fills the Heuristic class container 
template <typename KeyT, typename ValT>
void ParseHeuristic(std::string fileName, Heuristic<KeyT, ValT> & heuristic);

//////////////////////////////////////////////////////////////////////////////////////////////////

template <typename NodeT, typename WeightT, typename DataT>
class Search
{
public:
	using elem = std::pair<WeightT, NodeT>;

	Search(NodeT from, NodeT to, size_t n_nodes);

	void UninformedSearch(Graph<NodeT, WeightT> & graph);

	void InformedSearch(Graph<NodeT, WeightT> & graph, 
						std::function<WeightT(NodeT)> heuristica);

	void ReconstructPath(std::deque<NodeT> & path);

	WeightT GetFinalCost() const;

	size_t GetExpandedNodes() const;

	bool IsRouteExist() const;

private:
	void FindRoute(Graph<NodeT, WeightT> & graph, std::function<WeightT(NodeT)> heuristica);

private:
	NodeT _from;
	NodeT _to;

	size_t _expanded_nodes;
	bool _is_route_exist;

	std::vector<NodeT> _routes;
	std::vector<WeightT> _cost;
	std::vector<bool> _visited;

	std::priority_queue<elem, std::vector<elem>, std::greater<elem>> _queue;
};

//////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char ** argv)
{
	// check if the number of parameters is correct
	if (argc < 4 || argc > 5) {
		std::cerr << "Invalid parameters\n";
		return -1;
	}

	NodesContainer<std::string> nodes;
	Graph<size_t, size_t> graph;

	std::string origin_city(argv[2]);
	std::string destin_city(argv[3]);

	ParseRoutes(argv[1], nodes, graph);

	size_t origin_idx = nodes.GetIndex(origin_city);
	size_t destin_idx = nodes.GetIndex(destin_city);

	
	if (argc == 4) { // in case of uninformed search
		Search<size_t, size_t, std::string> uninform(origin_idx, destin_idx, nodes.GetSize());

		uninform.UninformedSearch(graph);

		if (!uninform.IsRouteExist()) {
			std::cout	<< "nodes expanded: " << uninform.GetExpandedNodes()
						<< "\ndistance: infinity"
						<< "\nroute:\nnone\n";

			return 0;
		}

		std::deque<size_t> path;
		uninform.ReconstructPath(path);

		std::cout	<< std::endl;
		std::cout	<< "nodes expanded: " << uninform.GetExpandedNodes()
					<< "\ndistance: " << uninform.GetFinalCost()
					<< "\nroute:\n";

		for (size_t i = 0; i != path.size() - 1; ++i) {
			std::cout << nodes.GetNode(path[i]) << " to ";
			std::cout << nodes.GetNode(path[i + 1]) << ", ";
			std::cout << graph.GetWeight(path[i], path[i + 1]) << " km\n";
		}
		std::cout << std::endl;

	} else { // in case of informed search
		Heuristic<std::string, size_t> heuristic;
		ParseHeuristic(argv[4], heuristic);

		Search<size_t, size_t, std::string> inform(origin_idx, destin_idx, nodes.GetSize());

		inform.InformedSearch(graph, [&nodes, &heuristic](size_t node) -> size_t {
			// for example, Bremen
			std::string data = nodes.GetNode(node);
			// heuristic for Bremen is 200
			size_t heur = heuristic.GetValue(data);
			// return the heuristic value that will be used in algorithm A *
			return heur;
		});

		if (!inform.IsRouteExist()) {
			std::cout	<< "nodes expanded: " << inform.GetExpandedNodes()
						<< "\ndistance: infinity"
						<< "\nroute:\nnone\n";

			return 0;
		}

		std::deque<size_t> path;
		inform.ReconstructPath(path);

		std::cout << std::endl;
		std::cout	<< "nodes expanded: " << inform.GetExpandedNodes()
					<< "\ndistance: " << inform.GetFinalCost()
					<< "\nroute:\n";

		for (size_t i = 0; i != path.size() - 1; ++i) {
			std::cout << nodes.GetNode(path[i]) << " to ";
			std::cout << nodes.GetNode(path[i + 1]) << ", ";
			std::cout << graph.GetWeight(path[i], path[i + 1]) << " km\n";
		}
		std::cout << std::endl;
	}
	return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////

template<typename DataT>
void NodesContainer<DataT>::AddNode(DataT newNode)
{
	 auto existing = std::find(std::begin(_container), std::end(_container), newNode);
	 
	 if (existing != std::end(_container)) {
		 // node already exists;
		 return;
	 } else {
		 _container.push_back(newNode);
	 }
}

template<typename DataT>
DataT NodesContainer<DataT>::GetNode(size_t id)
{
	DataT elem{};
	try {
		elem = _container.at(id);
	} 
	catch(const std::out_of_range & ex) {
		std::cerr << ex.what() << std::endl;
	}
	return elem;
}

template<typename DataT>
size_t NodesContainer<DataT>::GetSize() const
{
	return _container.size();
}

template<typename DataT>
size_t NodesContainer<DataT>::GetIndex(DataT & node) const
{
	size_t index{};

	for (auto & elem : _container) {
		if (elem == node) break;
		++index;
	}
	return index;
}

template<typename DataT>
void NodesContainer<DataT>::PrintNodes() const
{
	size_t id{};
	for (auto & elem : _container) {
		std::cout << id++ << "." << elem << "\n";
	}
	std::cout << std::endl;
}

//////////////////////////////////////////////////////////////////////////////////////////////////

template<typename NodeT, typename WeightT>
void Graph<NodeT, WeightT>::AddEdge(NodeT nodeU, NodeT nodeV, WeightT weight)
{
	if (_container.find(nodeU) == _container.end()) {
		// if the nodeU is not yet in the container
		std::deque< std::pair<NodeT, WeightT> > adj;

		// store the adjacent node and edge weight
		adj.push_back({ nodeV, weight });

		// and finally put it in the container (adjacency list)
		_container.insert({ nodeU, adj });
	} else {
		// if the node was already in the container, 
		// then simply add a new adjacent node (nodeV)
		_container[nodeU].push_back({ nodeV, weight });
	}

	if (_container.find(nodeV) == _container.end()) {
		std::deque< std::pair<NodeT, WeightT> > adj;
		adj.push_back({ nodeU, weight });
		_container.insert({ nodeV, adj });
	} else {
		_container[nodeV].push_back({ nodeU, weight });
	}
}

template<typename NodeT, typename WeightT>
const std::deque<std::pair<NodeT, WeightT>> Graph<NodeT, WeightT>::GetAdjacencyList(NodeT node)
{
	// return a list of adjacent nodes for the specified node
	return _container[node];
}

template<typename NodeT, typename WeightT>
WeightT Graph<NodeT, WeightT>::GetWeight(NodeT nodeU, NodeT nodeV)
{
		auto adjList = _container.at(nodeU);

		// look for nodeV in nodeU adjacency list 
		// and return edge (NodeU -- NodeV) weight
		for (auto & elem : adjList) {
			if (nodeV == elem.first) return elem.second;
		}
		return 0;
}

template<typename NodeT, typename WeightT>
void Graph<NodeT, WeightT>::PrintGraph() const
{
	for (auto & elem : _container) {
		std::cout << "key : " << elem.first << ", adj : ";
		for (auto & adj : elem.second) {
			std::cout << adj.first << "(" << adj.second << ") ";
		}
		std::cout << std::endl;
	}
	std::cout << std::endl;
}

//////////////////////////////////////////////////////////////////////////////////////////////////

template<typename KeyT, typename ValT>
void Heuristic<KeyT, ValT>::AddPair(KeyT key, ValT value)
{
	// for example, { Bremen, 200 }
	_mapper.insert({key, value});
}

template<typename KeyT, typename ValT>
ValT & Heuristic<KeyT, ValT>::GetValue(const KeyT & key)
{
		return _mapper.at(key);
}

template<typename KeyT, typename ValT>
void Heuristic<KeyT, ValT>::PrintHeuristic() const
{
	for (auto & elem : _mapper) {
		std::cout << "key : " << elem.first << ", val : " << elem.second << std::endl;
	}
	std::cout << std::endl;
}

//////////////////////////////////////////////////////////////////////////////////////////////////

void ReadData(std::string fileName, std::function<void(std::string&)> parser)
{
	std::ifstream file(fileName);
	if (file.is_open())
	{
		std::string line;
		while (std::getline(file, line))
		{
			if (line == "END OF INPUT") break;
			// perform some action with line
			parser(line);
		}
		file.close();
	}
}

template<typename DataT, typename NodeT, typename WeightT>
void ParseRoutes(std::string fileName, NodesContainer<DataT> & nodes, Graph<NodeT, WeightT> & graph)
{
	ReadData(fileName, [&nodes, &graph](std::string & route) {
		std::string origin_city, dest_city;
		size_t distance{};

		std::stringstream stream(route);
		stream >> origin_city >> dest_city >> distance;

		size_t origin_num{}, dest_num{};

		nodes.AddNode(origin_city);
		nodes.AddNode(dest_city);

		origin_num = nodes.GetIndex(origin_city);
		dest_num = nodes.GetIndex(dest_city);

		graph.AddEdge(origin_num, dest_num, distance);
	});
}

template<typename KeyT, typename ValT>
void ParseHeuristic(std::string fileName, Heuristic<KeyT, ValT>& heuristic)
{
	ReadData(fileName, [&heuristic](std::string & data) {
		std::string city;
		size_t value{};

		std::stringstream stream(data);
		stream >> city >> value;

		heuristic.AddPair(city, value);
	});
}

//////////////////////////////////////////////////////////////////////////////////////////////////

template <typename NodeT, typename WeightT, typename DataT>
Search<NodeT, WeightT, DataT>::Search(NodeT from, NodeT to, size_t n_nodes)
	:	_from{ from }, _to{ to }, _expanded_nodes{}, _is_route_exist{ true },
		_routes( n_nodes, 0 ), _cost( n_nodes, 0 ), _visited( n_nodes, false ), _queue{}
{}

template<typename NodeT, typename WeightT, typename DataT>
void Search<NodeT, WeightT, DataT>::UninformedSearch(Graph<NodeT, WeightT> & graph)
{
	// for Dijkstra's algorithm
	FindRoute(graph, [](NodeT n) -> WeightT { return 0; });
}

template<typename NodeT, typename WeightT, typename DataT>
void Search<NodeT, WeightT, DataT>::InformedSearch(Graph<NodeT, WeightT> & graph, 
												   std::function<WeightT(NodeT)> heuristica)
{
	// for A*'s algorithm
	FindRoute(graph, heuristica);
}

template <typename NodeT, typename WeightT, typename DataT>
void Search<NodeT, WeightT, DataT>::FindRoute(Graph<NodeT, WeightT> & graph, 
											  std::function<WeightT(NodeT)> heuristica)
{
	_queue.push({ 0, _from });

	_routes[_from] = _from;
	_cost[_from] = 0;

	_visited[_from] = 1;

	while (!_queue.empty()) {
		auto node = _queue.top().second;
		_queue.pop();

		if (node == _to) break;

		for (auto & adj : graph.GetAdjacencyList(node)) {
			WeightT newCost = _cost[node] + adj.second;

			if (!_visited[adj.first] || newCost < _cost[adj.first]) {
				_cost[adj.first] = newCost;
				_visited[adj.first] = 1;
				_routes[adj.first] = node;
				// here is the main difference in the Dijkstra and A * algorithms
				// for the Dijkstra algorithm, the heuristic is always zero
				// for algorithm A *, the heuristic value is extracted by lambda 
				// from a Heuristic class container (lambda is described in main)
				_queue.push({ newCost + heuristica(adj.first), adj.first });
			}
		}
	}

	// initially the container of visited nodes is filled with false
	_expanded_nodes = std::count(std::begin(_visited), std::end(_visited), true);

	// here we determine whether the route exists
	if (_routes[_to] == 0) _is_route_exist = false;
}

template <typename NodeT, typename WeightT, typename DataT>
void Search<NodeT, WeightT, DataT>::ReconstructPath(std::deque<NodeT> & path)
{
	NodeT curr = _to;
	path.push_front(curr);

	while (curr != _from) {
		curr = _routes[curr];
		path.push_front(curr);
	}
}

template <typename NodeT, typename WeightT, typename DataT>
WeightT Search<NodeT, WeightT, DataT>::GetFinalCost() const
{
	return _cost[_to];
}

template <typename NodeT, typename WeightT, typename DataT>
size_t Search<NodeT, WeightT, DataT>::GetExpandedNodes() const
{
	return _expanded_nodes;
}

template <typename NodeT, typename WeightT, typename DataT>
bool Search<NodeT, WeightT, DataT>::IsRouteExist() const
{
	return _is_route_exist;
}

//////////////////////////////////////////////////////////////////////////////////////////////////