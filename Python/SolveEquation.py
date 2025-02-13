import math
import random


def analytical_bound(k, N):
    q_ = (N / (N + 1)) ** (N+1)
    return (1 - (1.0 / N) * (q_ / (1 - q_))) * (1 - q_ ** (k + 1)) + (q_ ** k)


if __name__ == '__main__':
    N = 6.0
    r = 1.0
    q = ((N) / (N + 1)) ** N
    q_2 = ((N - 2) / (N  - 1)) ** (N - 2)
    r_0 = r * q
    beta = 0.25
    # K: number of divisions
    lowest_bound_a = 1
    iterations = 1000
    for i in range(iterations):
        for K in range(0, math.ceil(2*N) - 1):
            # randomize the radii
            radii_unsorted = [random.uniform(r_0, r) for _ in range(K)]
            radii_unsorted.append(r)
            radii_sorted = sorted(radii_unsorted)
            # now that you have the layers, get the upper-bound
            bound = r_0 ** N
            prev_radius = r_0
            for i in range(len(radii_sorted)):
                bound += radii_sorted[i] * (radii_sorted[i] ** N - prev_radius ** N)
                prev_radius = radii_sorted[i]
            if bound < lowest_bound_a:
                lowest_bound_a = bound
    lowest_bound_b = 1
    for i in range(iterations):
        for K in range(1, 100):
            # randomize the radii
            radii_unsorted = [random.uniform(r_0, r) for _ in range(K)]
            radii_unsorted.append(r)
            radii_sorted = sorted(radii_unsorted)
            # now that you have the layers, get the upper-bound
            bound = r_0 ** (N - 2)
            prev_radius = r_0
            for i in range(len(radii_sorted)):
                bound += radii_sorted[i] * (radii_sorted[i] ** (N - 2) - prev_radius ** (N - 2))
                prev_radius = radii_sorted[i]
            if bound < lowest_bound_b:
                lowest_bound_b = bound
    # basic_bound = ((1 / N) * (1.19 / beta) ** N * lowest_bound_a
    #                + ((1 / beta) * math.sqrt(N / 12)) ** (N-2) * (q_2 + q_2**(N-2) - q_2**(N - 3))) * lowest_bound_b
    # print(basic_bound)
    print(lowest_bound_a, lowest_bound_b)
    analytical_bound_a = 0
    analytical_bound_b = 0
    curr_r = 1
    N = 6
    low_r = 1
    for k in range(1, math.ceil(2*N)):
        low_r = curr_r * (N / (N + 1.0))
        analytical_bound_a += curr_r * (curr_r ** N - low_r ** N)
        analytical_bound_b += curr_r * (curr_r ** (N - 2) - low_r ** (N - 2))
        # print((1 - q) * (q ** (k - 1)) * curr_r)
        # print(curr_r * (curr_r ** N - low_r ** N))
        curr_r = low_r
    analytical_bound_a += curr_r ** (N + 1)
    analytical_bound_b += curr_r ** (N - 1)
    print(analytical_bound_a, analytical_bound_b)
    print("========")
    ## basic bound
    lowest_analytical_bound_a = lowest_analytical_bound_b = 1
    best_k = 1
    for k in range(0, math.ceil(N)+1):
        analytical_bound_a = analytical_bound(k, N)
        analytical_bound_b = analytical_bound(k, N - 2)
        # basic_bound = ((1 / N) * (1.19 / beta) ** N * analytical_bound_a
        #                + ((1 / beta) * math.sqrt(N / 12)) ** (N-2) * analytical_bound_b)
        if (analytical_bound_a < lowest_analytical_bound_a) and (analytical_bound_b < lowest_analytical_bound_b):
            lowest_analytical_bound_a = analytical_bound_a
            lowest_analytical_bound_b = analytical_bound_b
            best_k = k
    print(lowest_analytical_bound_a, lowest_analytical_bound_b, best_k, 2*N)

    ## normal bound
    basic_bound = ((1 / N) * (1.19 / beta) ** N
                   + ((1 / beta) * math.sqrt(N / 12)) ** (N-2))
    print(basic_bound)

