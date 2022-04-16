#include <array>
#include <chrono>
#include <optional>
#include <cstdlib>
#include <stdexcept>
#include <vector>

#include "graph.hpp"
#include "coordinates.hpp"
#include "argparse.hpp"


#ifndef GRAPH_SIZE
#define GRAPH_SIZE 100
#endif

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

    template<size_t count>
    inline std::array<vertex, count> vertices(void) const {
        if (auto filename = this->filename()) {
            return utils::sample<count>(vertex::read(*filename), this->seed());
        } else {
            return utils::sample<count>(DEFAULT_VERTICES, this->seed());
        }
    }

    template<size_t count>
    inline graph<count> map(void) const {
        return graph<count>(this->vertices<count>(), this->env);
    }

    void run(void) {
        auto g = this->map<GRAPH_SIZE>();

        auto elapsed = g.solve();
        std::cout << elapsed << '\n';
    }
};


int main(int argc, const char * const argv[]) {
    program program(std::vector<std::string>(argv, argv + argc));

#ifdef NDEBUG
    program.run();
    return EXIT_SUCCESS;
#else
    try {
        program.run();
        return EXIT_SUCCESS;

    } catch (const std::exception& err) {
        std::cerr << "std::exception: " << err.what() << '\n';
        return EXIT_FAILURE;

    } catch (const GRBException& err) {
        std::cerr << "GRBException: " << err.getMessage() << '\n';
        std::cerr << "GRBEnv::getErrorMsg: " << program.env.getErrorMsg() << '\n';
        return EXIT_FAILURE;

    } catch (...) {
        std::cerr << "unknown exception!" << '\n';
        return EXIT_FAILURE;
    }
#endif
}
