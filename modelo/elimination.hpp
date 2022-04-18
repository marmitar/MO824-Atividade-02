#pragma once

#include <array>
#include <chrono>
#include <concepts>
#include <functional>
#include <span>
#include <vector>

#include <gurobi_c++.h>
#include "vertex.hpp"
#include "tour.hpp"


namespace utils {
    template <typename Model>
    concept model = std::regular_invocable<Model, unsigned, unsigned>
        && std::same_as<std::invoke_result_t<Model, unsigned, unsigned>, bool>;

    [[gnu::hot]]
    static inline matrix<bool> get_solutions(size_t size, model auto&& get_solution) noexcept {
        matrix<bool> sols(size);

        for (unsigned u = 0; u < size; u++) {
            sols[u][u] = false;
            for (unsigned v = u + 1; v < size; v++) {
                bool has_edge = get_solution(u, v);
                sols[u][v] = has_edge;
                sols[v][u] = has_edge;
            }
        }
        return sols;
    }

    [[gnu::hot]]
    static tour min_sub_tour(std::span<const vertex> vertices, model auto&& get_solution) noexcept {
        const auto solutions = get_solutions(vertices.size(), get_solution);
        return tour::min_sub_tour(vertices, solutions);
    }
}

struct subtour_elim final : public GRBCallback {
public:
    const std::span<const vertex> vertices;
    const  utils::pair<utils::matrix<GRBVar>>& vars;

    [[gnu::cold]] [[gnu::nothrow]]
    inline subtour_elim(std::span<const vertex> vertices, const utils::pair<utils::matrix<GRBVar>>& vars) noexcept:
        GRBCallback(), vertices(vertices), vars(vars)
    { }

private:
    [[gnu::pure]] [[gnu::hot]] [[gnu::nothrow]]
    inline size_t count() const noexcept {
        return this->vertices.size();
    }

    [[gnu::hot]]
    inline void lazy_constraint_subtour_elimination(uint8_t i) {
        auto tour = utils::min_sub_tour(this->vertices, [this, i](unsigned u, unsigned v) {
            return this->getSolution(this->vars[i][u][v]) > 0.5;
        });

        if (tour.size() >= this->count()) [[unlikely]] {
            return;
        }

        auto expr = GRBLinExpr();
        for (unsigned u = 0; u < tour.size(); u++) {
            for (unsigned v = u + 1; v < tour.size(); v++) {
                expr += this->vars[i][tour[u]][tour[v]];
            }
        }
        this->addLazy(expr, GRB_LESS_EQUAL, tour.size()-1);
    }

protected:
    [[gnu::hot]]
    void callback() {
        if (this->where == GRB_CB_MIPSOL) [[likely]] {
            this->lazy_constraint_subtour_elimination(0);
            this->lazy_constraint_subtour_elimination(1);
        }
    }
};
