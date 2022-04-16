#pragma once

#include <array>
#include <chrono>
#include <optional>
#include <cstdlib>
#include <stdexcept>
#include <vector>

#include <gurobi_c++.h>
#include "vertex.hpp"


namespace utils {
    template<size_t count, typename T>
    using matrix = std::array<std::array<T, count>, count>;
}


template<size_t count>
class subtour_elim final: public GRBCallback {
private:
    class sub_tours final {
    private:
        static auto empty_array(void) noexcept {
            std::array<bool, count> arr;
            for (size_t i = 0; i < count; i++) {
                arr[i] = false;
            }
            return arr;
        }

        std::array<bool, count> seen = sub_tours::empty_array();
    public:
        const std::array<vertex, count>& vertices;
        const utils::matrix<count, double>& solution;

        sub_tours(const std::array<vertex, count>& vertices, const utils::matrix<count, double>& solution) noexcept:
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

                if (auto next = this->best_next(node)) {
                    node = *next;
                } else {
                    return vertices;
                }
            }
            return vertices;
        }

        std::optional<std::vector<size_t>> next_tour(void) noexcept {
            if (auto node = this->new_node()) {
                return this->next_tour(*node);
            }
            return std::nullopt;
        }
    };

    const std::array<vertex, count>& vertices;
    const utils::matrix<count, GRBVar>& vars;

    static auto all_vertices(void) noexcept {
        auto vec = std::vector<size_t>();
        vec.reserve(count);

        for (size_t i = 0; i < count; i++) {
            vec[i] = i;
        }
        return vec;
    }

public:
    inline subtour_elim(const std::array<vertex, count>& vertices, const utils::matrix<count, GRBVar>& vars) noexcept:
        vertices(vertices), vars(vars)
    { }

    auto find_sub_tour(const utils::matrix<count, double>& solution) const noexcept {
        sub_tours tours(this->vertices, solution);

        auto min_tour = subtour_elim<count>::all_vertices();
        while (auto tour = tours.next_tour()) {
            if (tour->size() <= min_tour.size()) {
                min_tour = *tour;
            }
        }
        return min_tour;
    }

    utils::matrix<count, double> get_solutions(void) {
        utils::matrix<count, double> sols;
        for (size_t u = 0; u < count; u++) {
            sols[u][u] = 0.0;
            for (size_t v = u + 1; v < count; v++) {
                auto val = this->getSolution(this->vars[u][v]);
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
                expr += this->vars[tour[u]][tour[v]];
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
