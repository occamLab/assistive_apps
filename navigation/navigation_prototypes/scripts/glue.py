#!/usr/bin/env python
from em import Analysis
from optimization import Optimization


def main():
    analysis = Analysis('data_optimized.pkl')
    analysis.getvariance()
    analysis.updateEdges()
    analysis.writePosegraph()

    optimization = Optimization(
        "data_analyzed.pkl", "data_analyzed.pkl")
    optimization.run()


if __name__ == '__main__':
    main()
