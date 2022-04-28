import pstats
from pstats import SortKey

def main():
    with open("output_time.txt", "w") as f:
        p = pstats.Stats("log.dat", stream=f)
        p.sort_stats("time").print_stats()

    with open("output_calls.txt", "w") as f:
        p = pstats.Stats("log.dat", stream=f)
        p.sort_stats("calls").print_stats()

    with open("output_cumtime.txt", "w") as f:
        p = pstats.Stats("log.dat", stream=f)
        p.sort_stats("cumtime").print_stats()

if __name__ == "__main__":
    main()
