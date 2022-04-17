#pragma once

#include <array>
#include <chrono>
#include <optional>
#include <cstdlib>
#include <span>
#include <stdexcept>
#include <vector>

#include <gurobi_c++.h>
#include "vertex.hpp"


namespace utils {
    template<typename T>
    class matrix final {
    private:
        size_t len;
        T *buf;
    public:
        inline matrix(size_t n): len(n) {
            this->buf = new T[n * n];
        }

        inline ~matrix() noexcept {
            delete[] this->buf;
            this->len = 0;
        }

        [[gnu::pure]] [[gnu::hot]] [[gnu::nothrow]]
        constexpr size_t size(void) const noexcept {
            return this->len;
        }

        [[gnu::pure]] [[gnu::hot]] [[gnu::nothrow]]
        constexpr size_t total(void) const noexcept {
            return this->len * this->len;
        }

        [[gnu::pure]] [[gnu::hot]] [[gnu::nothrow]]
        constexpr std::span<T> operator[](std::size_t idx) noexcept {
            return std::span<T>(this->buf + idx * this->len, this->len);
        }

        [[gnu::pure]] [[gnu::hot]] [[gnu::nothrow]]
        constexpr std::span<const T> operator[](std::size_t idx) const noexcept {
            return std::span<T>(this->buf + idx * this->len, this->len);
        }
    };
}


class subtour_elim final: public GRBCallback {
public:
    const std::vector<vertex>& vertices;
    const  utils::matrix<GRBVar>& vars;

    [[gnu::cold]] [[gnu::nothrow]]
    inline subtour_elim(const std::vector<vertex>& vertices, const  utils::matrix<GRBVar>& vars) noexcept:
        GRBCallback(), vertices(vertices), vars(vars)
    { }

private:
    class sub_tours final {
    public:
        inline sub_tours(const std::vector<vertex>& vertices, const  utils::matrix<double>& solution) noexcept:
            seen(vertices.size(), false), vertices(vertices), solution(solution)
        { }

    private:
        std::vector<bool> seen;
        const std::vector<vertex>& vertices;
        const  utils::matrix<double>& solution;

        [[gnu::pure]] [[gnu::hot]] [[gnu::nothrow]]
        inline size_t count(void) const noexcept {
            return this->vertices.size();
        }

        [[gnu::pure]] [[gnu::hot]] [[gnu::nothrow]]
        inline std::optional<size_t> new_node(void) const noexcept {
            for (size_t node = 0; node < this->count(); node++) {
                if (!this->seen[node]) [[likely]] {
                    return node;
                }
            }
            return std::nullopt;
        }

        [[gnu::pure]] [[gnu::hot]] [[gnu::nothrow]]
        inline std::optional<size_t> best_next(size_t u) const noexcept {
            auto solution = this->solution[u];
            for (size_t v = 0; v < this->count(); v++) {
                if (solution[v] > 0.5 && !this->seen[v]) [[likely]] {
                    return v;
                }
            }
            return std::nullopt;
        }

        [[gnu::hot]]
        inline std::vector<size_t> next_tour(size_t node) noexcept {
            auto vertices = std::vector<size_t>();
            vertices.reserve(this->count());

            for (size_t len = this->count(); len > 0; len--) {
                this->seen[node] = true;
                vertices.push_back(node);

                if (auto next = this->best_next(node)) [[likely]] {
                    node = *next;
                } else {
                    return vertices;
                }
            }
            return vertices;
        }

    public:
        [[gnu::hot]]
        std::optional<std::vector<size_t>> next_tour(void) noexcept {
            if (auto node = this->new_node()) [[likely]] {
                return this->next_tour(*node);
            }
            return std::nullopt;
        }
    };

    [[gnu::hot]]
    inline auto all_vertices(void) const noexcept {
        auto vec = std::vector<size_t>(this->count(), 0);

        for (size_t i = 0; i < vec.size(); i++) {
            vec[i] = i;
        }
        return vec;
    }

    [[gnu::pure]] [[gnu::hot]] [[gnu::nothrow]]
    inline size_t count(void) const noexcept {
        return this->vertices.size();
    }

    [[gnu::hot]] [[gnu::nothrow]]
    inline auto find_sub_tour(const  utils::matrix<double>& solution) const noexcept {
        sub_tours tours(this->vertices, solution);

        auto min_tour = this->all_vertices();
        while (auto tour = tours.next_tour()) [[likely]] {
            if (tour->size() <= min_tour.size()) [[unlikely]] {
                min_tour = *tour;
            }
        }
        return min_tour;
    }

    [[gnu::hot]]
    inline utils::matrix<double> get_solutions(void) {
        utils::matrix<double> sols(this->count());
        for (size_t u = 0; u < this->count(); u++) {
            sols[u][u] = 0.0;
            for (size_t v = u + 1; v < this->count(); v++) {
                auto val = this->getSolution(this->vars[u][v]);
                sols[u][v] = val;
                sols[v][u] = val;
            }
        }
        return sols;
    }

    [[gnu::hot]]
    inline void lazy_constraint_subtour_elimination(void) {
        auto tour = this->find_sub_tour(this->get_solutions());
        auto len = tour.size();

        if (len >= this->count()) [[unlikely]] {
            return;
        }

        auto expr = GRBLinExpr();
        for (size_t u = 0; u < len; u++) {
            for (size_t v = u + 1; v < len; v++) {
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
