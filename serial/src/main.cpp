#include "helper.h"

#if defined(USE_AOS)
    #include "layout_aos.h"
#elif defined(USE_SOA)
    #include "layout_soa.h"
#else
    #error "Must define either USE_AOS or USE_SOA"
#endif

bool run_astar(const GridMap &map, AStarState &state);

int main(int argc, char **argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <map_file>\n";
        return 1;
    }

    try {
        GridMap map = load_grid_map_from_file(argv[1]);
        int N = map.width * map.height;

        const int NUM_RUNS = 10;
        double total_ms = 0.0;
        bool found_any = false;

        for (int i = 0; i < NUM_RUNS; ++i) {
            AStarState state(N);  // fresh state each run

            Timer t;
            bool found = run_astar(map, state);
            double ms = t.elapsed_ms();

            if (i == 0) {
                found_any = found; // record result from first run
            }
            total_ms += ms;
        }

        double avg_ms = total_ms / NUM_RUNS;

        std::cout << "Found: " << found_any << "\n";
        std::cout << "Average time over " << NUM_RUNS
                  << " runs: " << avg_ms << " ms\n";

    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
