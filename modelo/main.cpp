#include <chrono>
#include <csignal>
#include <optional>
#include <stdexcept>
#include <unistd.h>
#include <variant>
#include <vector>

#include "graph.hpp"
#include "coordinates.hpp"
#include "argparse.hpp"


class program final {
private:
    argparse::ArgumentParser args;

    [[gnu::cold]]
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

        this->args.add_argument("-t", "--timeout")
            .help("execution timeout (in minutes), disabled if zero or negative")
            .default_value<double>(30.0)
            .scan<'g', double>();
    }

public:
    [[gnu::cold]]
    explicit program(const std::vector<std::string>& arguments): program(arguments[0]) {
        try {
            this->args.parse_args(arguments);

        } catch (const std::runtime_error& err) {
            std::cerr << err.what() << std::endl;
            std::cerr << this->args << std::endl;
            std::exit(EXIT_FAILURE);
        }
    }

    const GRBEnv env = utils::quiet_env();

    [[gnu::cold]]
    inline std::optional<std::string> filename(void) const {
        auto filename = this->args.get("filename");
        if (filename.empty()) {
            return std::nullopt;
        } else [[likely]] {
            return filename;
        }
    }

    [[gnu::pure]] [[gnu::cold]]
    inline auto seed(void) const {
        return this->args.get<utils::seed_type>("seed");
    }

    [[gnu::pure]] [[gnu::cold]]
    inline auto nodes(void) const {
        return this->args.get<size_t>("nodes");
    }

    [[gnu::pure]] [[gnu::cold]]
    inline auto timeout(void) const {
        return this->args.get<double>("timeout");
    }

private:
    [[gnu::cold]]
    inline std::variant<std::vector<vertex>, decltype(DEFAULT_VERTICES)> vertices(void) const {
        if (auto filename = this->filename()) {
            return vertex::read(*filename);
        } else {
            return DEFAULT_VERTICES;
        }
    }

    [[gnu::cold]]
    std::vector<vertex> sample(void) const {
        auto sampler = [this](auto&& vertices) {
            return utils::sample(vertices, this->nodes(), this->seed());
        };
        return std::visit(sampler, this->vertices());
    }

    [[gnu::cold]]
    inline graph map(void) const {
        return graph(this->sample(), this->env);
    }

public:
    [[gnu::hot]]
    void run(void) const {
        auto g = this->map();
        std::cout << "Graph(n=" << g.order() << ",m=" << g.size() << "),"
            << " chosen with seed 0x" << std::hex << this->seed() << std::dec << std::endl;

        std::cout << "Model set up, with " << g.var_count() << " variables" <<
            " and " << g.constr_count() << " constraints." << std::endl;

        auto elapsed = g.solve();
        std::cout << "Found " << g.solution_count() << " solution(s)."  << std::endl;
        std::cout << "Iterations: " << g.iterations() << std::endl;
        std::cout << "Execution time: " << elapsed << " secs" << std::endl;
        std::cout << "Variables: " << g.var_count() << std::endl;
        std::cout << "Constraints: " << g.constr_count() << std::endl;

        std::cout << "Tour:" << std::endl;
        std::cout << utils::join(g.solution(), "\n") << std::endl;
    }
};

namespace timeout {
    static auto start = std::chrono::steady_clock::now();

    [[gnu::cold]] [[gnu::nothrow]]
    static void on_timeout(int signal) noexcept {
        if (signal == SIGALRM) [[likely]] {
            const auto end = std::chrono::steady_clock::now();
            std::chrono::duration<double, std::ratio<60>> elapsed = end - start;

            std::cerr << "Timeout: stopping execution for taking too long." << std::endl;
            std::cerr << "Instance has been running for " << elapsed.count() << " minutes." << std::endl;
            std::exit(EXIT_FAILURE);
        }
    }

    [[gnu::cold]] [[gnu::nothrow]]
    static void setup(double minutes) {

        if (std::signal(SIGALRM, on_timeout) == SIG_ERR) [[unlikely]] {
            std::cerr << "Warning: could not setup timeout for " << minutes << " minutes." << std::endl;
            return;
        }

        alarm((unsigned) std::ceil(minutes * 60));
    }
}


int main(int argc, const char * const argv[]) {
    const program program(std::vector<std::string>(argv, argv + argc));

    if (std::isfinite(program.timeout()) && program.timeout() > 0) [[likely]] {
        timeout::setup(program.timeout());
    }

    try {
        program.run();

    } catch (const utils::invalid_solution& err) {
        std::cerr << "utils::invalid_solution: " << err.what() << std::endl;
        std::cerr << "seed used: 0x" << std::hex << program.seed() << std::dec << std::endl;
        if (err.subtour) {
            std::cerr << "subtour(" << err.subtour->size() << "): "
                << utils::join(*err.subtour, " ") << std::endl;
        }
        std::cerr << "vertices:" << std::endl;
        std::cerr << utils::join(err.vertices, "\n") << std::endl;

    } catch (const std::exception& err) {
        std::cerr << "std::exception: " << err.what() << std::endl;
        return EXIT_FAILURE;

    } catch (const GRBException& err) {
        std::cerr << "GRBException: code " << err.getErrorCode() << ", " << err.getMessage() << std::endl;
        std::cerr << "GRBEnv::getErrorMsg: " << program.env.getErrorMsg() << std::endl;
        return EXIT_FAILURE;

    } catch (...) {
        std::cerr << "unknown exception!" << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
