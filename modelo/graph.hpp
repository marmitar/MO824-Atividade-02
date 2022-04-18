#pragma once

#include <chrono>
#include <iostream>
#include <optional>
#include <span>
#include <stdexcept>
#include <vector>

#include <gurobi_c++.h>
#include "vertex.hpp"
#include "elimination.hpp"


namespace utils {
    struct invalid_solution final : public std::domain_error {
    public:
        const std::span<const vertex> vertices;
        const std::optional<tour> subtour;

    private:
        [[gnu::cold]]
        explicit inline invalid_solution(std::span<const vertex> vertices, std::optional<tour> subtour, const char *message):
            std::domain_error(message), vertices(vertices), subtour(subtour)
        { }

    public:
        [[gnu::cold]]
        static invalid_solution zero_solutions(std::span<const vertex> vertices) {
            return invalid_solution(vertices, std::nullopt, "No integral solution could be found.");
        }

        [[gnu::cold]]
        static invalid_solution incomplete_tour(std::span<const vertex> vertices, tour& subtour) {
            return invalid_solution(vertices, subtour, "Solution found, but leads to incomplete tour.");
        }
    };

    [[gnu::cold]]
    static std::string join(std::ranges::forward_range auto range, const std::string_view& sep) {
        std::ostringstream buf;
        bool first = true;

        for (const auto& item : range) {
            if (!first) {
                buf << sep;
            }
            buf << item;
            first = false;
        }
        return buf.str();
    }
}


struct graph final {
private:
    GRBModel model;

    [[gnu::cold]]
    inline GRBVar add_edge(uint8_t i, const vertex& u, const vertex& v) {
        std::ostringstream name;
        name << 'x' << i << '_' << u.id() << '_' << v.id();

        double objective = u[i].cost(v[i]);
        return this->model.addVar(0., 1., objective, GRB_BINARY, name.str());
    }

    [[gnu::cold]]
    inline utils::matrix<GRBVar> add_vars(uint8_t i) {
        auto vars = utils::matrix<GRBVar>(this->order());

        for (unsigned u = 0; u < this->order(); u++) {
            for (unsigned v = u + 1; v < this->order(); v++) {
                auto xi_uv = this->add_edge(i, this->vertices[u], this->vertices[v]);
                vars[u][v] = xi_uv;
                vars[v][u] = xi_uv;
            }
        }
        return vars;
    }

    [[gnu::cold]]
    inline void add_constraint_deg_2(uint8_t i) {
        for (unsigned u = 0; u < this->order(); u++) {
            auto expr = GRBLinExpr();
            for (unsigned v = 0; v < this->order(); v++) {
                if (u != v) [[likely]] {
                    expr += this->vars[i][u][v];
                }
            }
            this->model.addConstr(expr, GRB_EQUAL, 2.);
        }
    }

    [[gnu::cold]]
    inline void add_constraint_similarity(double k) {
        auto expr = GRBQuadExpr();
        for (unsigned u = 0; u < this->order(); u++) {
            for (unsigned v = u + 1; v < this->order(); v++) {
                expr += this->vars[0][u][v] * this->vars[1][u][v];
            }
        }
        this->model.addQConstr(expr, GRB_GREATER_EQUAL, k);
    }

public:
    [[gnu::cold]]
    graph(std::span<const vertex> vertices, const GRBEnv& env, unsigned k = 0):
        model(env), vertices(vertices), vars({ this->add_vars(0), this->add_vars(1) })
    {
        this->add_constraint_deg_2(0);
        this->add_constraint_deg_2(1);
        if (k > 0) {
            this->add_constraint_similarity(k);
        }
        this->model.update();
    }

    const std::span<const vertex> vertices;
    const  utils::pair<utils::matrix<GRBVar>> vars;

    /** Number of vertices. */
    [[gnu::pure]] [[gnu::hot]] [[gnu::nothrow]]
    inline size_t order() const noexcept {
        return this->vertices.size();
    }

    /** Number of edges. */
    [[gnu::pure]] [[gnu::cold]] [[gnu::nothrow]]
    inline size_t size() const noexcept {
        const size_t order = this->order();
        return (order * (order - 1)) / 2;
    }

    using clock = std::chrono::high_resolution_clock;
    const clock::time_point start = clock::now();

    [[gnu::cold]] [[gnu::nothrow]]
    inline double elapsed() const noexcept {
        auto end = clock::now();
        std::chrono::duration<double> secs = end - this->start;
        return secs.count();
    }

    [[gnu::pure]] [[gnu::cold]]
    int64_t solution_count() const {
        return this->model.get(GRB_IntAttr_SolCount);
    }

    [[gnu::hot]]
    double solve() {
        auto callback = subtour_elim(this->vertices, this->vars);
        this->model.setCallback(&callback);

        this->model.optimize();
        auto total_time = this->elapsed();

        if (this->solution_count() <= 0) [[unlikely]] {
            throw utils::invalid_solution::zero_solutions(this->vertices);
        }
        return total_time;
    }

    [[gnu::pure]] [[gnu::cold]]
    int64_t iterations() const {
        return this->model.get(GRB_DoubleAttr_IterCount);
    }

    [[gnu::pure]] [[gnu::cold]]
    int64_t var_count() const {
        return this->model.get(GRB_IntAttr_NumVars);
    }

    [[gnu::pure]] [[gnu::cold]]
    int64_t lin_constr_count() const {
        return this->model.get(GRB_IntAttr_NumConstrs);
    }

    [[gnu::pure]] [[gnu::cold]]
    int64_t quad_constr_count() const {
        return this->model.get(GRB_IntAttr_NumQConstrs);
    }

    [[gnu::pure]] [[gnu::cold]]
    int64_t constr_count() const {
        return this->lin_constr_count() + this->quad_constr_count();
    }

    [[gnu::pure]] [[gnu::cold]]
    double solution_cost() const {
        return this->model.get(GRB_DoubleAttr_ObjVal);
    }

    [[gnu::pure]] [[gnu::hot]]
    inline bool edge(uint8_t i, unsigned u, unsigned v) const {
        if (u != v) [[likely]] {
            return this->vars[i][u][v].get(GRB_DoubleAttr_X) > 0.5;
        } else {
            return false;
        }
    }

    [[gnu::pure]] [[gnu::cold]]
    auto edges(uint8_t i) const {
        return utils::get_solutions(this->order(), [this, i](unsigned u, unsigned v) {
            return this->edge(i, u, v);
        });
    }

    [[gnu::pure]] [[gnu::cold]]
    auto tour(uint8_t i) const {
        auto min = utils::min_sub_tour(this->vertices, [this, i](unsigned u, unsigned v) {
            return this->edge(i, u, v);
        });

        if (min.size() != this->order()) [[unlikely]] {
            throw utils::invalid_solution::incomplete_tour(this->vertices, min);
        }
        return min;
    }

    [[gnu::pure]] [[gnu::cold]]
    unsigned similarity() const {
        unsigned total = 0;
        for (unsigned u = 0; u < this->order(); u++) {
            for (unsigned v = u + 1; v < this->order(); v++) {

                if (this->edge(0, u, v) && this->edge(1, u, v)) [[unlikely]] {
                    total += 1;
                }
            }
        }
        return total;
    }

    [[gnu::pure]] [[gnu::cold]]
    auto solution(uint8_t i) const {
        const auto tour = this->tour(i);

        auto vertices = std::vector<vertex>();
        vertices.reserve(tour.size());

        for (unsigned v : tour) {
            vertices.push_back(this->vertices[v]);
        }
        return vertices;
    }
};
