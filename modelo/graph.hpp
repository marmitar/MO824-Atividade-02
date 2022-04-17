#pragma once

#include <chrono>
#include <vector>

#include <gurobi_c++.h>
#include "vertex.hpp"
#include "elimination.hpp"


namespace utils {
    [[gnu::cold]]
    static inline GRBEnv quiet_env(void) {
        auto env = GRBEnv(true);
        env.set(GRB_IntParam_OutputFlag, 0);
        env.set(GRB_IntParam_LazyConstraints, 1);
        env.start();
        return env;
    }
}


struct graph final {
private:
    GRBModel model;

    [[gnu::cold]]
    inline GRBVar add_edge(const vertex& u, const vertex& v) {
        std::ostringstream name;
        name << "x_" << u.id() << '_' << v.id();

        double objective = u.cost1(v);
        return this->model.addVar(0., 1., objective, GRB_BINARY, name.str());
    }

    [[gnu::cold]]
    inline auto add_vars(void) {
        utils::matrix<GRBVar> vars(this->order());

        for (unsigned u = 0; u < this->order(); u++) {
            for (unsigned v = u + 1; v < this->order(); v++) {
                auto x_uv = this->add_edge(this->vertices[u], this->vertices[v]);
                vars[u][v] = x_uv;
                vars[v][u] = x_uv;
            }
        }
        return vars;
    }

    [[gnu::cold]]
    inline void add_constraint_deg_2(void) {
        for (unsigned u = 0; u < this->order(); u++) {
            auto expr = GRBLinExpr();
            for (unsigned v = 0; v < this->order(); v++) {
                if (u != v) [[likely]] {
                    expr += this->vars[u][v];
                }
            }
            this->model.addConstr(expr, GRB_EQUAL, 2.);
        }
    }

public:
    [[gnu::cold]]
    graph(std::vector<vertex> vertices, const GRBEnv& env):
        model(env), vertices(vertices), vars(this->add_vars())
    {
        this->add_constraint_deg_2();
    }

    const std::vector<vertex> vertices;
    const  utils::matrix<GRBVar> vars;

    /** Number of vertices. */
    [[gnu::pure]] [[gnu::hot]] [[gnu::nothrow]]
    inline size_t order(void) const noexcept {
        return this->vertices.size();
    }

    /** Number of edges. */
    [[gnu::pure]] [[gnu::cold]] [[gnu::nothrow]]
    inline size_t size(void) const noexcept {
        const size_t order = this->order();
        return (order * (order + 1)) / 2;
    }

    [[gnu::pure]] [[gnu::cold]]
    inline int64_t solution_count(void) const {
        return (int64_t) this->model.get(GRB_IntAttr_SolCount);
    }

    [[gnu::pure]] [[gnu::cold]]
    inline int64_t iterations(void) const {
        return (int64_t) this->model.get(GRB_DoubleAttr_IterCount);
    }

    using clock = std::chrono::high_resolution_clock;
    const clock::time_point start = clock::now();

    [[gnu::cold]] [[gnu::nothrow]]
    inline double elapsed(void) const noexcept {
        auto end = clock::now();
        std::chrono::duration<double> secs = end - this->start;
        return secs.count();
    }

    [[gnu::hot]]
    double solve(void) {
        auto callback = subtour_elim(this->vertices, this->vars);

        this->model.setCallback(&callback);
        this->model.update();

        this->model.optimize();
        return this->elapsed();
    }
};
