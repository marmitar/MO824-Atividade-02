#pragma once

#include <optional>
#include <span>
#include <vector>

#include <gurobi_c++.h>
#include "vertex.hpp"


namespace utils {
    template <typename Item>
    struct matrix final {
    private:
        Item *buffer;
        size_t len;

    public:
        inline matrix(size_t n): len(n) {
            this->buffer = new Item[n * n];
        }

        inline ~matrix() {
            delete[] this->buffer;
        }

        [[gnu::pure]] [[gnu::hot]] [[gnu::nothrow]]
        constexpr size_t size() const noexcept {
            return this->len;
        }

        [[gnu::pure]] [[gnu::hot]] [[gnu::nothrow]]
        constexpr size_t total() const noexcept {
            return this->size() * this->size();
        }

        [[gnu::pure]] [[gnu::hot]] [[gnu::nothrow]]
        constexpr std::span<Item> operator[](std::size_t idx) noexcept {
            return std::span<Item>(this->buffer + idx * this->size(), this->size());
        }

        [[gnu::pure]] [[gnu::hot]] [[gnu::nothrow]]
        constexpr std::span<const Item> operator[](std::size_t idx) const noexcept {
            return std::span<const Item>(this->buffer + idx * this->size(), this->size());
        }
    };
}


struct tour final : public std::vector<unsigned> {
private:
    struct iter_tours final {
    public:
        inline iter_tours(
            std::span<const vertex> vertices,
            const  utils::matrix<bool>& solution
        ) noexcept:
            seen(vertices.size(), false), vertices(vertices), solution(solution)
        { }

    private:
        std::vector<bool> seen;
        std::span<const vertex> vertices;
        const  utils::matrix<bool>& solution;

        [[gnu::pure]] [[gnu::hot]] [[gnu::nothrow]]
        inline size_t count() const noexcept {
            return this->vertices.size();
        }

        [[gnu::pure]] [[gnu::hot]] [[gnu::nothrow]]
        inline std::optional<unsigned> new_node() const noexcept {
            for (unsigned node = 0; node < this->count(); node++) {
                if (!this->seen[node]) [[likely]] {
                    return node;
                }
            }
            return std::nullopt;
        }

        [[gnu::pure]] [[gnu::hot]] [[gnu::nothrow]]
        inline std::optional<unsigned> best_next(unsigned u) const noexcept {
            auto solution = this->solution[u];
            for (unsigned v = 0; v < this->count(); v++) {
                if (solution[v] && !this->seen[v]) [[likely]] {
                    return v;
                }
            }
            return std::nullopt;
        }

        [[gnu::hot]]
        inline tour next_tour(unsigned node) noexcept {
            auto vertices = tour();
            vertices.reserve(this->count());

            for (unsigned len = this->count(); len > 0; len--) {
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
        inline std::optional<tour> next_tour() noexcept {
            if (auto node = this->new_node()) [[likely]] {
                return this->next_tour(*node);
            }
            return std::nullopt;
        }
    };

public:
    [[gnu::hot]] [[gnu::nothrow]]
    static tour min_sub_tour(
        std::span<const vertex> vertices,
        const  utils::matrix<bool>& solution
    ) noexcept
    {
        iter_tours tours(vertices, solution);

        auto min_tour = tour();
        if (auto first = tours.next_tour()) {
            min_tour = *first;
        } else [[unlikely]] {
            return min_tour;
        }

        while (auto tour = tours.next_tour()) [[likely]] {
            if (tour->size() < min_tour.size()) [[unlikely]] {
                min_tour = *tour;
            }
        }
        return min_tour;
    }

    [[gnu::pure]] [[gnu::nothrow]]
    static double cost(uint8_t i, const std::vector<vertex>& tour) noexcept {
        double total_cost = 0.0;
        for (unsigned v = 0; v < tour.size(); v++) {
            const unsigned next = (v + 1) % tour.size();
            total_cost += tour[v][i].cost(tour[next][i]);
        }
        return total_cost;
    }
};
