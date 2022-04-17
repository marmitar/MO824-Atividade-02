#include <array>
#include <chrono>
#include <cstdlib>
#include <optional>
#include <span>
#include <stdexcept>
#include <variant>
#include <vector>

#include "graph.hpp"
#include "coordinates.hpp"
#include "argparse.hpp"


class program final {
private:
    argparse::ArgumentParser args;
public:
    GRBEnv env = utils::quiet_env();

    explicit inline program(std::string name): args(name) {
        this->args.add_argument("filename")
            .help("file with coordinates: <x1> <y1> <x2> <y2>")
            .default_value(std::string {});

        this->args.add_argument("-s", "--seed")
            .help("seed for the sampling method (if empty, a random seed is generated)")
            .default_value<utils::seed_type>(std::random_device {}())
            .scan<'x', utils::seed_type>();

        this->args.add_argument("-n", "--nodes")
            .help("sample size for the subgraph")
            .default_value<size_t>(100)
            .scan<'u', size_t>();
    }

    explicit program(std::vector<std::string> arguments): program(arguments[0]) {
        try {
            this->args.parse_args(arguments);

        } catch (const std::runtime_error& err) {
            std::cerr << err.what() << std::endl;
            std::cerr << this->args << std::endl;
            std::exit(EXIT_FAILURE);
        }
    }

    inline std::optional<std::string> filename(void) const {
        auto filename = this->args.get("filename");
        if (filename.empty()) {
            return std::nullopt;
        } else {
            return filename;
        }
    }

    inline utils::seed_type seed(void) const {
        return this->args.get<utils::seed_type>("seed");
    }

    inline utils::seed_type nodes(void) const {
        return this->args.get<size_t>("nodes");
    }

    inline std::variant<std::vector<vertex>, decltype(DEFAULT_VERTICES)> vertices(void) const {
        if (auto filename = this->filename()) {
            return vertex::read(*filename);
        } else {
            return DEFAULT_VERTICES;
        }
    }

    std::vector<vertex> sample(void) const {
        auto sampler = [this](auto&& vertices) {
            return utils::sample(vertices, this->nodes(), this->seed());
        };
        return std::visit(sampler, this->vertices());
    }

    inline graph map(void) const {
        return graph(this->sample(), this->env);
    }

    void run(void) const {
        auto g = this->map();

        auto elapsed = g.solve();
        std::cout << elapsed << '\n';
    }
};


int main(int argc, const char * const argv[]) {
    const program program(std::vector<std::string>(argv, argv + argc));

#ifdef NDEBUG
    program.run();
#else
    try {
        program.run();

    } catch (const std::exception& err) {
        std::cerr << "std::exception: " << err.what() << std::endl;
        return EXIT_FAILURE;

    } catch (const GRBException& err) {
        std::cerr << "GRBException: " << err.getMessage() << std::endl;
        std::cerr << "GRBEnv::getErrorMsg: " << program.env.getErrorMsg() << std::endl;
        return EXIT_FAILURE;

    } catch (...) {
        std::cerr << "unknown exception!" << std::endl;
        return EXIT_FAILURE;
    }
#endif
    return EXIT_SUCCESS;
}
