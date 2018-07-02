from g2o_error_viz import G2O_Error_Viz

if __name__ == "__main__":
    #g2o_viz = G2O_Error_Viz('NavData/savedwaypoints/PunchlinePlot/result.g2o', 'NavData/savedwaypoints/PunchlinePlot/data_cp.g2o', "NavData/savedwaypoints/PunchlinePlot/naive.txt", (0,.085,.012))
    # g2o_viz = G2O_Error_Viz('prototypes/data_g2o_archived_data/6.19.2018.17.40/data_withnewline_optimized.g2o',
    #                         'prototypes/data_g2o_archived_data/6.19.2018.17.40/data_withnewline.g2o',
    #                         "prototypes/data_g2o_archived_data/6.19.2018.17.40/naive.txt", (0, .085, .012))

    g2o_viz = G2O_Error_Viz('prototypes/data_g2o_archived_data/6.19.2018.11.50/result.g2o',
                            'prototypes/data_g2o_archived_data/6.19.2018.11.50/data_cp.g2o',
                            "prototypes/data_g2o_archived_data/6.19.2018.11.50/naive.txt", (0, .085, .012))
    g2o_viz.run()
