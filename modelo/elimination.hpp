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
    static inline tour min_sub_tour(const std::vector<vertex>& vertices, model auto&& get_solution) noexcept {
        const auto solutions = get_solutions(vertices.size(), get_solution);
        return tour::min_sub_tour(vertices, solutions);
    }
}

class subtour_elim final : public GRBCallback {
public:
    const std::vector<vertex>& vertices;
    const  utils::matrix<GRBVar>& vars;

    [[gnu::cold]] [[gnu::nothrow]]
    inline subtour_elim(const std::vector<vertex>& vertices, const  utils::matrix<GRBVar>& vars) noexcept:
        GRBCallback(), vertices(vertices), vars(vars)
    { }

private:
    [[gnu::pure]] [[gnu::hot]] [[gnu::nothrow]]
    inline size_t count(void) const noexcept {
        return this->vertices.size();
    }

    [[gnu::hot]]
    inline void lazy_constraint_subtour_elimination(void) {
        auto tour = utils::min_sub_tour(this->vertices, [this](unsigned i, unsigned j) {
            return this->getSolution(this->vars[i][j]) > 0.5;
        });

        if (tour.size() >= this->count()) [[unlikely]] {
            return;
        }

        auto expr = GRBLinExpr();
        for (unsigned u = 0; u < tour.size(); u++) {
            for (unsigned v = u + 1; v < tour.size(); v++) {
                expr += this->vars[tour[u]][tour[v]];
            }
        }
        this->addLazy(expr, GRB_EQUAL, this->count()-1);
    }

protected:
    [[gnu::hot]]
    void callback(void) {
        if (this->where == GRB_CB_MIPSOL) [[likely]] {
            this->lazy_constraint_subtour_elimination();
        }
    }
};
