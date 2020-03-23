import math
from scipy import signal
import numpy as np
import matplotlib.pyplot as plt

def to_E_series(value, series=12):
    values = {6:  [100, 150, 220, 330, 470, 680],
              12: [100, 120, 150, 180, 220, 270, 330, 390, 470, 560, 680, 820],
              24: [100, 110, 120, 130, 150, 160, 180, 200, 220, 240, 270, 300, 330, 360, 390, 430, 470, 510, 560, 620,
                   680, 750, 820, 910],
              48: [100, 121, 147, 178, 215, 261, 316, 383, 464, 562, 681, 825, 105, 127, 154, 187, 226, 274, 332, 402, 487, 590,
          715, 866, 110, 133, 162, 196, 237, 287, 348, 422, 511, 619, 750, 909, 115, 140, 169, 205, 249, 301, 365, 442,
          536, 649, 787, 953],
              96: [100, 121, 147, 178, 215, 261, 316, 383, 464, 562, 681, 825, 102, 124, 150, 182, 221, 267, 324, 392, 475, 576,
          698, 845, 105, 127, 154, 187, 226, 274, 332, 402, 487, 590, 715, 866, 107, 130, 158, 191, 232, 280, 340, 412,
          499, 604, 732, 887, 110, 133, 162, 196, 237, 287, 348, 422, 511, 619, 750, 909, 113, 137, 165, 200, 243, 294,
          357, 432, 523, 634, 768, 931, 115, 140, 169, 205, 249, 301, 365, 442, 536, 649, 787, 953, 118, 143, 174, 210,
          255, 309, 374, 453, 549, 665, 806, 976],
              192: [100, 121, 147, 178, 215, 261, 316, 383, 464, 562, 681, 825, 101, 123, 149, 180, 218, 264, 320, 388, 470, 569,
           690, 835, 102, 124, 150, 182, 221, 267, 324, 392, 475, 576, 698, 845, 104, 126, 152, 184, 223, 271, 328, 397,
           481, 583, 706, 856, 105, 127, 154, 187, 226, 274, 332, 402, 487, 590, 715, 866, 106, 129, 156, 189, 229, 277,
           336, 407, 493, 597, 723, 876, 107, 130, 158, 191, 232, 280, 340, 412, 499, 604, 732, 887, 109, 132, 160, 193,
           234, 284, 344, 417, 505, 612, 741, 898, 110, 133, 162, 196, 237, 287, 348, 422, 511, 619, 750, 909, 111, 135,
           164, 198, 240, 291, 352, 427, 517, 626, 759, 920, 113, 137, 165, 200, 243, 294, 357, 432, 523, 634, 768, 931,
           114, 138, 167, 203, 246, 298, 361, 437, 530, 642, 777, 942, 115, 140, 169, 205, 249, 301, 365, 442, 536, 649,
           787, 953, 117, 142, 172, 208, 252, 305, 370, 448, 542, 657, 796, 965, 118, 143, 174, 210, 255, 309, 374, 453,
           549, 665, 806, 976, 120, 145, 176, 213, 258, 312, 379, 459, 556, 673, 816, 988]}

    power = 0
    while value>max(values[series]):
        value /= 10
        power += 1
    while value<min(values[series]):
        value *= 10
        power -= 1

    values[series].append(min(values[series])*10)
    values[series].append(max(values[series])/10)

    min_diff = 1000
    nearest_value = -1
    for i in range(len(values[series])):
        diff = abs(value - values[series][i])
        if diff < min_diff:
            min_diff = diff
            nearest_value = values[series][i]

    return nearest_value * (10**power)


def low_pass_filter():
    fc = 400e3
    R_range = [100, 110, 120, 130, 150, 160, 180, 200, 220, 240, 270, 300, 330, 360, 390, 430, 470, 510, 560, 620,
                   680, 750, 820, 910]
    R_range = [i*100 for i in R_range]

    min_err = math.inf
    for R1 in R_range:
        for R2 in R_range:
            C2 = math.sqrt(2) / (2 * math.pi * fc * (R1 + R2))
            C1 = 1 / ((2 * math.pi * fc) ** 2 * R1 * R2 * C2)
            if abs(C1-to_E_series(C1,series=24))/C1 + abs(C2-to_E_series(C2,series=24))/C2 < min_err:
                min_err = abs(C1-to_E_series(C1,series=24))/C1 + abs(C2-to_E_series(C2,series=24))/C2
                best_solution = [R1, R2, C1, C2]
    return best_solution


def high_pass_filter():
    fc = 1e3

    C_range = [100, 110, 120, 130, 150, 160, 180, 200, 220, 240, 270, 300, 330, 360, 390, 430, 470, 510, 560, 620,
                   680, 750, 820, 910]
    C_range = [i*1e-10 for i in C_range]

    min_err = math.inf
    for C1 in C_range:
        for C2 in C_range:
            R1 = math.sqrt(2) / (2 * math.pi * fc * (C1 + C2))
            R2 = 1 / ((2 * math.pi * fc) ** 2 * R1 * C1 * C2)
            if abs(R1-to_E_series(R1,series=24))/R1 + abs(R2-to_E_series(R2,series=24))/R2 < min_err:
                min_err = abs(R1-to_E_series(R1,series=24))/R1 + abs(R2-to_E_series(R2,series=24))/R2
                best_solution = [R1,R2,C1,C2]
    return best_solution


def plot_high_pass_input_impedance(values):
    R1, R2, C1, C2 = values

    tf = signal.TransferFunction([C1*C2*R1*R2,  R2*(C1 + C2), 1], [C1*C2*R2, C1, 0.0])
    print(tf)
    w, H = signal.freqresp(tf,np.logspace(1,5,1000))

    plt.figure()
    plt.semilogx(w/(2*np.pi), 20*np.log10(np.abs(H)))
    print("Minimum input impedance:", np.min(np.abs(H)))
    plt.figure()
    plt.semilogx(w / (2 * np.pi), np.angle(H, deg=True))
    plt.show()

def simple_low_pass_filter():
    fc = 10e3
    R_range = [100, 110, 120, 130, 150, 160, 180, 200, 220, 240, 270, 300, 330, 360, 390, 430, 470, 510, 560, 620,
                   680, 750, 820, 910]
    R_range = [i*10 for i in R_range]

    min_err = math.inf
    for R in R_range:
        C = 1 / (2 * math.pi * fc * R)
        err = abs(C-to_E_series(C,series=24))/C
        if err < min_err:
            min_err = err
            best_solution = [R, C]
    return best_solution

if __name__ == "__main__":
    print("Low pass filter:")
    values = simple_low_pass_filter()
    for i in values:
        print("  %.4e -> %.4e" % (i, to_E_series(i,series=24)))
    print(1/(2 * math.pi * to_E_series(values[0],series=24) * to_E_series(values[1],series=24)))