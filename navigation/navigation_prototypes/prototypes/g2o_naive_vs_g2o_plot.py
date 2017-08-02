from g2o_error_viz import G2O_Error_Viz

if __name__ == "__main__":
    g2o_viz = G2O_Error_Viz('NavData/savedwaypoints/PunchlinePlot/result.g2o', 'NavData/savedwaypoints/PunchlinePlot/data_cp.g2o', "NavData/savedwaypoints/PunchlinePlot/naive.txt", (0,.085,.012))
    g2o_viz.run()
