#include <vector>
#include <unordered_map>
#include <limits>
#include <iostream>
#include <functional>
#include <set>
#include <cmath>
#include <queue>

template<typename V, typename D = double>
class Graph {
public:
    struct Edge {
        V from;
        V to;
        D distance;
    };

private:
    std::unordered_map<V, std::vector<Edge>> _edges;
    std::set<V> _vertices;

public:
    bool has_vertex(const V& v) const {
        return _vertices.contains(v);
    }

    void add_vertex(const V& v) {
        _vertices.insert(v); 
        _edges.insert({ v, {} }); 
    }

    std::vector<Edge> get_incoming_edges(const V& vert) const {
        std::vector<Edge> incoming_vert; 
        for (const auto& v : _vertices) { 
            for (const auto& edge : _edges.at(v)) { 
                if (edge.to == vert) 
                    incoming_vert.push_back(edge);
            }
        }
        return incoming_vert;
    }

    bool remove_vertex(const V& vert) {
        if (_vertices.erase(vert)) {
            _edges.erase(vert);
            for (auto edge : get_incoming_edges(vert)) {
                std::erase_if(_edges[edge.from], [vert](const Edge& edge) {
                    return edge.to == vert;
                    });
            }
            return true;
        }
        return false;
    }

    std::vector<V> vertices() const {
        std::vector<V> vertices;
        for (const auto& vert : _vertices)
            vertices.push_back(vert);
        return vertices;
    }

    void add_edge(const V& from, const V& to, const D& distance) {
        if (!_vertices.contains(from))
            throw std::invalid_argument("from is not exist");
        if (!_vertices.contains(to))
            throw std::invalid_argument("to is not exist");
        if (distance < 0)
            throw std::invalid_argument("only unsigned distance"); 
        _edges[from].push_back({ from, to, distance });
    }

    bool remove_edge(const V& from, const V& to) {
        if (!_edges.contains(from) || !_vertices.contains(to))
            return false;
        auto cnt_erased = 0;
        cnt_erased += std::erase_if(_edges[from],
            [to](const Edge& edge) {
                return edge.to == to;
            });
        if (cnt_erased) return true;
        return false;
    }

    bool remove_edge(const Edge& ed) {
        if (!_edges.contains(ed.from) || !_vertices.contains(ed.to))
            return false;
        auto cnt_erased = 0;
        cnt_erased += std::erase_if(_edges[ed.from],
            [ed](const Edge& edge) {
                return edge.to == ed.to &&
                    std::fabs(edge.distance - ed.distance) < std::numeric_limits<double>::epsilon();
            });
        if (cnt_erased) return true;
        return false; 
    }

    bool has_edge(const V& from, const V& to) const {
        if (!_vertices.contains(from) || !_vertices.contains(to))
            return false; 
        return std::any_of(_edges.at(from).begin(),
            _edges.at(from).end(),
            [to](const Edge& ed) {
                return ed.to == to;
            });
    }

    bool has_edge(const Edge& ed) const {
        if (!_vertices.contains(ed.to) || !_vertices.contains(ed.from))
            return false;
        return std::any_of(_edges.at(ed.from).begin(),
            _edges.at(ed.from).end(),
            [ed](const Edge& edge) {
                return ed.to == edge.to &&
                    std::fabs(edge.distance - ed.distance) < std::numeric_limits<double>::epsilon();
            });
    }

    std::vector<Edge> edges(const V& vertex) const {
        if (!has_vertex(vertex)) return {}; 
        return _edges.at(vertex);
    }

    size_t order() const {
        return _edges.size();
    }

    size_t degree(const V& v) const {
        if (!has_vertex(v)) return 0;
        return _edges.at(v).size(); 
    }

    std::vector<Edge> shortest_path(const V& start, const V& end) const {
        if (!_vertices.contains(start) || !_vertices.contains(end)) throw std::invalid_argument("not found");

        std::unordered_map<V, D> distances;
        for (const auto& vert : _vertices) distances[vert] = std::numeric_limits<D>::infinity();
        distances[start] = 0; 

        std::vector<Edge> path;
        std::priority_queue<std::pair<D, V>> priority_queue;
        priority_queue.push({ 0, start });

        std::unordered_map<V, V> prev;
        while (!priority_queue.empty()) {
            std::pair<D, V> current = priority_queue.top();
            priority_queue.pop();

            for (const auto& edge : _edges.at(current.second)) {
                D new_dist = current.first + edge.distance;
                if (new_dist < distances[edge.to]) {
                    prev[edge.to] = current.second;
                    distances[edge.to] = new_dist;
                    priority_queue.push({ new_dist, edge.to });
                }
            }
        }
        if (distances[end] == std::numeric_limits<D>::infinity()) return {};

        V current = end;
        while (current != start) {
            V prev_v = prev[current];
            D dist = distances[current] - distances[prev_v];
            path.push_back(Edge{ prev_v, current, dist });
            current = prev_v;
        }
        reverse(path.begin(), path.end());
        return path;
    }

    void walk(const V& start, const std::function<void(const V&)>& action) const {
        std::unordered_map<V, bool> visited;

        for (const auto& vert : _vertices) visited[vert] = false;

        std::queue<V> queue;
        queue.push(start);

        while (!queue.empty()) {
            auto current = queue.front();
            queue.pop();

            if (!visited[current]) {
                visited[current] = true;
                action(current);

                for (const auto& edge : _edges.at(current))
                    if (!visited[edge.to]) queue.push(edge.to);
            }
        }
    }

};
template<typename V, typename D = double>
std::pair<V, D> find_the_farthest_vertex(Graph<V, D> graph) {
    std::vector<std::pair<V, D>> distances;
    std::vector<V> vertices = graph.vertices();
    std::vector<typename Graph<V, D>::Edge> edges; 
    std::vector<typename Graph<V, D>::Edge> incoming_edges;
    auto sum_dist = 0.0;

    for (auto& vert : vertices) {
        sum_dist = 0.0;
        edges.clear();
        incoming_edges.clear();
        edges = graph.edges(vert);
        incoming_edges = graph.get_incoming_edges(vert); 
        edges.insert(edges.end(), incoming_edges.begin(), incoming_edges.end()); 
        for (const auto& edge : edges) {
            sum_dist += edge.distance; 
        }
        distances.push_back({ vert, sum_dist / (static_cast<double>(graph.degree(vert) + incoming_edges.size())) });
    }

    std::pair<V, D> farthest_vert = distances[0]; 

    for (const auto& vert : distances) {
        if (farthest_vert.second < vert.second) farthest_vert = vert;
    }

    return farthest_vert;
}


int main() {
    Graph<int> g;
    for (size_t i = 1; i < 9; ++i) g.add_vertex(i);

    g.add_edge(1, 2, 1.0);
    g.add_edge(1, 3, 2.0);
    g.add_edge(2, 6, 4.0);
    g.add_edge(6, 1, 1.0);
    g.add_edge(2, 5, 1.0);
    g.add_edge(5, 6, 1.0);
    g.add_edge(2, 4, 7.0);
    g.add_edge(3, 7, 1.0);
    g.add_edge(8, 4, 2.0);
    g.add_edge(3, 8, 1.0);

    g.get_incoming_edges(6);

    g.remove_vertex(7);
    g.remove_edge(8, 4);
    g.remove_edge({ 4,3,6 });

    g.walk(1, [](int vert) {std::cout << vert << " "; });

    auto path = g.shortest_path(1, 6);
    std::cout << std::endl << "Shortest path from 1 to 6:" << std::endl;
    for (const auto& edge : path) {
        std::cout << edge.from << " -> " << edge.to << " (Distance: " << edge.distance << ")" << std::endl;
    }

    auto max_vertex = find_the_farthest_vertex(g);
    std::cout << std::endl << "Vertex with maximum average distance to neighbors and this distance: " << max_vertex.first << " (Distance: " << max_vertex.second << ")" << std::endl;
    std::cout << std::endl << "degree of 1: " << g.degree(1);
    std::cout << std::endl << "order of graph: " << g.order();
    return 0;
}

