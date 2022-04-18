#pragma once

#include <algorithm>
#include <array>
#include <fstream>
#include <random>
#include <ranges>
#include <sstream>
#include <stdexcept>
#include <vector>


namespace utils {
    struct invalid_file final : public std::invalid_argument {
    private:
        [[gnu::cold]]
        static inline std::string message(const std::string& filename, const char *reason) noexcept {
            std::ostringstream buf;
            buf << "File \"" << filename << "\" " << reason << ".";
            return buf.str();
        }

        [[gnu::cold]]
        explicit inline invalid_file(const std::string& filename, const char *reason):
            std::invalid_argument(message(filename, reason))
        { }

    public:
        [[gnu::cold]]
        static invalid_file is_empty_or_missing(const std::string& filename) {
            return invalid_file(filename, "is empty or missing");
        }

        [[gnu::cold]]
        static invalid_file contains_invalid_data(const std::string& filename) {
            return invalid_file(filename, "contains invalid data");
        }
    };


    struct not_enough_items final : public std::out_of_range {
    private:
        [[gnu::cold]]
        static inline std::string message(const char *type, size_t current, size_t expected) noexcept {
            std::ostringstream buf;
            buf << "Not enough '" << type << "', requesting " << expected << " out of " << current << " available." << '.';
            return buf.str();
        }

        [[gnu::cold]]
        explicit inline not_enough_items(const char *type, size_t current, size_t expected):
            std::out_of_range(message(type, current, expected))
        { }

    public:
        template <typename Item> [[gnu::cold]]
        static not_enough_items of(size_t current, size_t expected) {
            return not_enough_items(typeid(Item).name(), current, expected);
        }
    };


    using seed_type = std::ranlux48::result_type;

    [[gnu::cold]]
    static auto sample(std::ranges::forward_range auto input, size_t count, seed_type seed) {
        using item = std::ranges::range_value_t<decltype(input)>;

        if (count > input.size()) [[unlikely]] {
            throw not_enough_items::of<item>(input.size(), count);
        }
        std::vector<item> output;
        output.reserve(count);

        auto rng = std::ranlux48(seed);
        std::ranges::sample(input, std::back_inserter(output), count, rng);
        return output;
    }

    template <typename Item>
    using pair = std::array<Item, 2>;
}


struct vertex final {
public:
    struct point final {
    private:
        double x;
        double y;

    public:
        [[gnu::hot]] [[gnu::nothrow]]
        constexpr inline point(double x, double y) noexcept: x(x), y(y) { }

        [[gnu::pure]] [[gnu::hot]] [[gnu::nothrow]]
        constexpr inline double cost(const point& other) const noexcept {
            return ceil(hypot(this->x - other.x, this->y - other.y));
        }

        [[gnu::cold]]
        friend inline std::ostream& operator<<(std::ostream& os, const point& p) {
            return os << p.x << ',' << p.y;
        }

        [[gnu::cold]]
        friend inline std::istream& operator>>(std::istream& is, point& p) {
            return is >> p.x >> p.y;
        }
    };

private:
    [[gnu::cold]]
    static unsigned next_id() noexcept {
        static unsigned count = 1;
        return count++;
    }

    [[gnu::cold]]
    inline constexpr vertex(unsigned id, double x1, double y1, double x2, double y2) noexcept:
        ident(id), p({ point(x1, y1), point(x2, y2) })
    { }

    unsigned ident;
    utils::pair<point> p;

public:
    constexpr vertex() noexcept: vertex(0U, 0, 0, 0, 0) {}

    [[gnu::cold]]
    vertex(double x1, double y1, double x2, double y2) noexcept:
        vertex(vertex::next_id(), x1, y1, x2, y2)
    { }

    [[gnu::pure]] [[gnu::hot]] [[gnu::nothrow]]
    constexpr inline unsigned id() const noexcept {
        return this->ident;
    }

    [[gnu::pure]] [[gnu::hot]] [[gnu::nothrow]]
    constexpr inline const point& operator[](std::uint8_t idx) const noexcept {
        return this->p[idx];
    }

    template <unsigned id> [[gnu::cold]]
    constexpr static vertex with_id(double x1, double y1, double x2, double y2) noexcept {
        static_assert(id > 0, "'id' must be positive.");
        return vertex(id, x1, y1, x2, y2);
    }

    [[gnu::cold]]
    static std::vector<vertex> read(const std::string& filename) {
        auto vertices = std::vector<vertex>();
        auto file = std::ifstream(filename, std::ios::in);

        auto line = std::string();
        while (std::getline(file, line)) [[unlikely]] {
            vertex new_vertex;
            if (std::istringstream(line) >> new_vertex) {
                vertices.push_back(new_vertex);
            } else [[unlikely]] {
                throw utils::invalid_file::contains_invalid_data(filename);
            }
        }
        file.close();

        if (vertices.size() <= 0) [[unlikely]] {
            throw utils::invalid_file::is_empty_or_missing(filename);
        }
        return vertices;
    }

    [[gnu::cold]]
    friend inline std::ostream& operator<<(std::ostream& os, const vertex& vertex) {
        return os << "v<" << vertex.id() << ">(" << vertex.p[0] << "," << vertex.p[1] << ")";
    }

    [[gnu::cold]]
    friend inline std::istream& operator>>(std::istream& is, vertex& vertex) {
        return is >> vertex.p[0] >> vertex.p[1];
    }
};
