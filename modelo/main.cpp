#include <chrono>
#include <csignal>
#include <optional>
#include <span>
#include <stdexcept>
#include <unistd.h>
#include <variant>
#include <vector>

#include "graph.hpp"
#include "coordinates.hpp"
#include "argparse.hpp"


namespace utils {
    [[gnu::cold]]
    static GRBEnv quiet_env() {
        auto env = GRBEnv(true);
        env.set(GRB_IntParam_OutputFlag, 0);
        env.set(GRB_IntParam_LazyConstraints, 1);
        env.start();
        return env;
    }
}

struct program final {
private:
    argparse::ArgumentParser args;

    [[gnu::cold]]
    explicit inline program(std::string name): args(name) {
        this->args.add_argument("-n", "--nodes")
            .help("sample size for the subgraph")
            .default_value<unsigned>(100)
            .scan<'u', unsigned>();

        this->args.add_argument("-k", "--similarity")
            .help("minimun number of shared edges between tours")
            .default_value<unsigned>(0)
            .scan<'u', unsigned>();

        this->args.add_argument("--timeout")
            .help("execution timeout (in minutes), disabled if zero or negative")
            .default_value<double>(30.0)
            .scan<'g', double>();

        this->args.add_argument("-t", "--tour")
            .help("show vertices present on each solution")
            .default_value(false)
            .implicit_value(true);
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

    [[gnu::pure]] [[gnu::cold]]
    inline unsigned nodes() const {
        return this->args.get<unsigned>("nodes");
    }

    [[gnu::pure]] [[gnu::cold]]
    inline unsigned similarity() const {
        return this->args.get<unsigned>("similarity");
    }

    [[gnu::pure]] [[gnu::cold]]
    inline std::optional<double> timeout() const {
        auto value = this->args.get<double>("timeout");
        if (std::isfinite(value) && value > 0) [[likely]] {
            return value;
        } else {
            return std::nullopt;
        }
    }

    [[gnu::pure]] [[gnu::cold]]
    inline bool tour() const {
        return this->args.get<bool>("tour");
    }

private:
    [[gnu::cold]]
    inline std::span<const vertex> vertices() const {
        if (this->nodes() > DEFAULT_VERTICES.size()) [[unlikely]] {
            throw utils::not_enough_items::in(DEFAULT_VERTICES, this->nodes());
        }
        return std::span(DEFAULT_VERTICES).first(this->nodes());
    }

    [[gnu::cold]]
    graph map() const {
        return graph(this->vertices(), this->env, this->similarity());
    }

public:
    [[gnu::hot]]
    void run() const {
        auto g = this->map();
        std::cout << "Graph(n=" << g.order() << ",m=" << g.size() << ")" << std::endl;

        const auto elapsed = g.solve();
        std::cout << "Found " << g.solution_count() << " solution(s)."  << std::endl;
        std::cout << "Iterations: " << g.iterations() << std::endl;
        std::cout << "Execution time: " << elapsed << " secs" << std::endl;
        std::cout << "Variables: " << g.var_count() << std::endl;
        std::cout << "Constraints: " << g.constr_count() << std::endl;
        std::cout << "    Linear: " << g.lin_constr_count() << std::endl;
        std::cout << "    Quadratic: " << g.quad_constr_count() << std::endl;
        std::cout << "Similarity: " << g.similarity() << std::endl;
        std::cout << "Objective cost: " << g.solution_cost() << std::endl;

        for (uint8_t i = 0; i <= 1; i++) {
            const auto solution = g.solution(i);
            std::cout << "Tour " << i+1 << ": total cost " << tour::cost(i, solution) << std::endl;
            if (this->tour()) [[unlikely]] {
                std::cout << utils::join(solution, "\n") << std::endl;
            }
        }
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

    if (auto minutes = program.timeout()) [[likely]] {
        timeout::setup(*minutes);
    }

    try {
        program.run();

    } catch (const utils::invalid_solution& err) {
        std::cerr << "utils::invalid_solution: " << err.what() << std::endl;
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
