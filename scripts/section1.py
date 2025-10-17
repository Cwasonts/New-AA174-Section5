#!/usr/bin/env python3
import numpy as np


def main():
    np.random.seed(42)
    A = np.random.normal(size=(4,4))
    B = np.random.normal(size=(4,2))

    print(A@B)




if __name__ == "__main__":
    main()