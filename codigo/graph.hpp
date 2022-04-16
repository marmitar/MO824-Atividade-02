#pragma once

#include <array>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <vector>

#include <gurobi_c++.h>
#include "vertex.hpp"
#include "elimination.hpp"


namespace utils {
    static inline GRBEnv quiet_env(void) {
        auto env = GRBEnv(true);
        env.set(GRB_IntParam_OutputFlag, 0);
        env.set(GRB_IntParam_LazyConstraints, 1);
        env.start();
        return env;
    }
}


template<size_t count>
struct graph final {
private:
    inline GRBVar add_edge(const vertex& u, const vertex& v) {
        std::ostringstream name;
        name << "x_" << u.id << '_' << v.id;

        double objective = u.cost1(v);
        return this->model.addVar(0., 1., objective, GRB_BINARY, name.str());
    }

    inline auto add_vars(void) {
        utils::matrix<count, GRBVar> vars;

        for (size_t u = 0; u < count; u++) {
            for (size_t v = u + 1; v < count; v++) {
                auto x_uv = this->add_edge(this->vertices[u], this->vertices[v]);
                vars[u][v] = x_uv;
                vars[v][u] = x_uv;
            }
        }
        return vars;
    }

    inline void add_constraint_deg_2(void) {
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

    GRBModel model;
public:
    const std::array<vertex, count> vertices;
    const utils::matrix<count, GRBVar> vars;

    using clock = std::chrono::high_resolution_clock;
    const clock::time_point start = clock::now();

    graph(std::array<vertex, count> vertices, const GRBEnv& env):
        model(env), vertices(vertices), vars(this->add_vars())
    {
        this->add_constraint_deg_2();
    }

    /** Number of vertices. */
    constexpr inline size_t order(void) const noexcept {
        constexpr size_t size = this->vertices.size();
        return size;
    }

    /** Number of edges. */
    constexpr inline size_t size(void) const noexcept {
        constexpr size_t order = this->order();
        return (order * (order + 1)) / 2;
    }

    inline double elapsed(void) const noexcept {
        auto end = clock::now();
        std::chrono::duration<double> secs = end - this->start;
        return secs.count();
    }

    inline int64_t solution_count(void) const {
        return (int64_t) this->model.get(GRB_IntAttr_SolCount);
    }

    inline int64_t iterations(void) const {
        return (int64_t) this->model.get(GRB_DoubleAttr_IterCount);
    }

    double solve(void) {
        auto callback = subtour_elim(this->vertices, this->vars);

        this->model.setCallback(&callback);
        this->model.optimize();

        return this->elapsed();
    }
};
