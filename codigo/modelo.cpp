#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iterator>
#include <optional>
#include <random>
#include <ranges>
#include <span>
#include <sstream>
#include <stdexcept>
#include <unordered_set>
#include <vector>

#include <gurobi_c++.h>


template<size_t count, std::ranges::input_range Range>
static inline auto sample(Range&& input) {
    std::array<std::ranges::range_value_t<Range>, count> output;

    static auto rng = std::ranlux48(0x0123456789ABCDEFULL);
    std::ranges::sample(input, output.begin(), count, rng);
    return output;
}


struct vertex final {
private:
    static inline size_t next_id(void) noexcept {
        static size_t count = 0;
        return count++;
    }

    vertex(size_t id, double x1, double y1, double x2, double y2):
        id(id), x1(x1), y1(y1), x2(x2), y2(y2)
    { }

public:
    size_t id;
    double x1, y1;
    double x2, y2;

    vertex(): vertex(SIZE_MAX, 0, 0, 0, 0) {}

    vertex(double x1, double y1, double x2, double y2):
        vertex(vertex::next_id(), x1, y1, x2, y2)
    { }

    vertex(vertex const& v):
        vertex(v.id, v.x1, v.y1, v.x2, v.y2)
    { }

    constexpr inline double cost1(vertex const& other) const noexcept {
        return ceil(hypot(this->x1 - other.x1, this->y1 - other.y1));
    }

    constexpr inline double cost2(vertex const& other) const noexcept {
        return ceil(hypot(this->x2 - other.x2, this->y2 - other.y2));
    }

    constexpr inline bool operator==(vertex const& other) const noexcept {
        return this->id == other.id;
    }

    constexpr inline vertex& operator=(vertex const& other) noexcept {
        this->id = other.id;
        this->x1 = other.x1;
        this->y1 = other.y1;
        this->x2 = other.x2;
        this->y2 = other.y2;
        return *this;
    }

    static auto read(const char *filename) {
        auto vertices = std::vector<vertex>();
        read_into(filename, vertices);

        if (vertices.empty()) {
            auto message = std::string("File \"");
            message += filename;
            message += "\" empty or missing.";

            throw std::out_of_range(message);
        }
        return vertices;
    }

private:
    inline static void read_into(const char *filename, std::vector<vertex>& vertices) {
        auto file = std::ifstream(filename, std::ios::in);

        auto line = std::string();
        while (std::getline(file, line)) {
            auto str = std::stringstream(line);

            double x1, x2, y1, y2;
            str >> x1 >> y1 >> x2 >> y2;
            vertices.emplace_back(x1, y1, x2, y2);
        }
        file.close();
    }
};


inline std::ostream& operator<<(std::ostream& os, vertex const& vertex) {
    return os << "v:" << vertex.id;
}

template<size_t count, typename T>
using matrix = std::array<std::array<T, count>, count>;

template<size_t count>
struct graph final {
private:
    GRBModel model;
public:
    const std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
    const std::array<vertex, count> vertices;
    const matrix<count, GRBVar> vars = this->add_vars();

    graph(std::array<vertex, count> vertices, GRBEnv &env):
        model(env), vertices(vertices)
    {
        this->add_constraint_deg_2();
    }

    /** Number of vertices. */
    constexpr inline size_t order(void) const noexcept {
        return count;
    }

    /** Number of edges. */
    constexpr inline size_t size(void) const noexcept {
        return (count * (count + 1)) / 2;
    }

    inline double elapsed(void) const noexcept {
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> secs = end - this->start;
        return secs.count();
    }

    inline int64_t solution_count(void) const {
        return (int64_t) this->model.get(GRB_IntAttr_SolCount);
    }

    inline int64_t iterations(void) const {
        return (int64_t) this->model.get(GRB_DoubleAttr_IterCount);
    }

    static graph read(const char *filename, GRBEnv &env) {
        auto vertices = vertex::read(filename);

        if (count > vertices.size()) {
            auto message = std::string("Not enough vertices, requesting ");
            message += std::to_string(count);
            message += " out of ";
            message += std::to_string(vertices.size());
            message += " available";
            throw std::out_of_range(message);
        }
        return graph(sample<count>(vertices), env);
    }

    double solve(GRBCallback &cb) {
        this->model.setCallback(&cb);
        this->model.optimize();
        return this->elapsed();
    }

private:
    inline auto add_var(size_t u, size_t v, double obj_coef) {
        auto name = std::string("x_");
        name += std::to_string(u);
        name += '_';
        name += std::to_string(v);
        return this->model.addVar(0., 1., obj_coef, GRB_BINARY, name);
    }

    matrix<count, GRBVar> add_vars(void) {
        matrix<count, GRBVar> vars;

        for (size_t u = 0; u < count; u++) {
            for (size_t v = u + 1; v < count; v++) {
                auto vu = this->vertices[u];
                auto vv = this->vertices[v];

                auto x_uv = this->add_var(vu.id, vv.id, vu.cost1(vv));
                vars[u][v] = x_uv;
                vars[v][u] = x_uv;
            }
        }
        return vars;
    }

    void add_constraint_deg_2(void) {
        for (size_t u = 0; u < count; u++) {
            auto expr = GRBLinExpr();
            for (size_t v = 0; v < count; v++) {
                if (u != v) {
                    expr += this->vars[u][v];
                }
            }
            this->model.addConstr(expr, GRB_EQUAL, 2.);
        }
    }
};


template<size_t count>
class sub_tours final {
private:
    static auto empty_array(void) {
        std::array<bool, count> arr;
        for (size_t i = 0; i < count; i++) {
            arr[i] = false;
        }
        return arr;
    }

    std::array<bool, count> seen = sub_tours::empty_array();
public:
    std::array<vertex, count> const& vertices;
    matrix<count, double> const& solution;

    sub_tours(std::array<vertex, count> const& vertices, matrix<count, double> const& solution):
        vertices(vertices), solution(solution)
    { }

    inline std::optional<size_t> new_node(void) const noexcept {
        for (size_t node = 0; node < count; node++) {
            if (!this->seen[node]) {
                return node;
            }
        }
        return std::nullopt;
    }

    inline std::optional<size_t> best_next(size_t u) const noexcept {
        auto solution = this->solution[u];

        for (size_t v = 0; v < count; v++) {
            if (solution[v] > 0.5 && !this->seen[v]) {
                return v;
            }
        }
        return std::nullopt;
    }

    inline std::vector<size_t> next_tour(size_t node) noexcept {
        auto vertices = std::vector<size_t>();
        vertices.reserve(count);

        for (size_t len = count; len > 0; len--) {
            this->seen[node] = true;
            vertices.push_back(node);

            auto next = this->best_next(node);
            if (next.has_value()) {
                node = next.value();
            } else {
                return vertices;
            }
        }
        return vertices;
    }

    std::optional<std::vector<size_t>> next_tour(void) noexcept {
        auto node = this->new_node();
        if (node.has_value()) {
            return this->next_tour(node.value());
        }
        return std::nullopt;
    }
};

template<size_t count>
class subtour_elim final: public GRBCallback {
private:
    graph<count> const& g;

    static auto all_vertices(void) noexcept {
        auto vec = std::vector<size_t>();
        vec.reserve(count);

        for (size_t i = 0; i < count; i++) {
            vec[i] = i;
        }
        return vec;
    }

public:
    subtour_elim(graph<count> const& g): g(g) { }

    auto find_sub_tour(matrix<count, double> const& solution) const noexcept {
        auto tours = sub_tours<count>(this->g.vertices, solution);
        auto min_tour = subtour_elim<count>::all_vertices();

        do {
            auto tour = tours.next_tour();
            if (!tour.has_value()) {
                return min_tour;
            }

            if (tour.value().size() <= min_tour.size()) {
                min_tour = tour.value();
            }
        } while (true);

        return min_tour;
    }

    matrix<count, double> get_solutions(void) {
        matrix<count, double> sols;
        for (size_t u = 0; u < count; u++) {
            sols[u][u] = 0.0;
            for (size_t v = u + 1; v < count; v++) {
                auto val = this->getSolution(this->g.vars[u][v]);
                sols[u][v] = val;
                sols[v][u] = val;
            }
        }
        return sols;
    }

    void lazy_constraint_subtour_elimination(void) {
        auto tour = this->find_sub_tour(this->get_solutions());
        auto len = tour.size();

        if (len >= count) {
            return;
        }

        auto expr = GRBLinExpr();
        for (size_t u = 0; u < len; u++) {
            for (size_t v = u + 1; v < len; v++) {
                expr += this->g.vars[tour[u]][tour[v]];
            }
        }
        this->addLazy(expr, GRB_EQUAL, count-1);
    }

protected:
    void callback(void) {
        if (this->where == GRB_CB_MIPSOL) {
            this->lazy_constraint_subtour_elimination();
        }
    }
};

template<size_t count>
static void run(graph<count> g) {
    auto elimination = subtour_elim(g);
    auto elapsed = g.solve(elimination);

    std::cout << elapsed << '\n';
}



static inline GRBEnv quiet_env(void) {
    auto env = GRBEnv(true);
    env.set(GRB_IntParam_OutputFlag, 0);
    env.set(GRB_IntParam_LazyConstraints, 1);
    env.start();
    return env;
}

#ifndef GRAPH_SIZE
#define GRAPH_SIZE 100
#endif

int main(int argc, const char * const argv[]) {
    const auto filename = (argc >= 2) ? argv[1] : "coordenadas.txt";

    auto env = quiet_env();
#ifdef NDEBUG
    run(graph<GRAPH_SIZE>::read(filename, env));
#else
    try {
        run(graph<GRAPH_SIZE>::read(filename, env));

    } catch (std::exception &e) {
        std::cerr << "std::exception: " << e.what() << '\n';
        return EXIT_FAILURE;

    } catch (GRBException &e) {
        std::cerr << "GRBException: " << e.getMessage() << '\n';
        std::cerr << "GRBEnv::getErrorMsg: " << env.getErrorMsg() << '\n';
        return EXIT_FAILURE;

    } catch (...) {
        std::cerr << "unknown exception!" << '\n';
        return EXIT_FAILURE;
    }
#endif

    return EXIT_SUCCESS;
}
