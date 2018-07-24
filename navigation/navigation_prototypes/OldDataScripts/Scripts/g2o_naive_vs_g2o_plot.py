from g2o_error_viz import G2O_Error_Viz
from rospkg import RosPack
from os import path

if __name__ == "__main__":
    #g2o_viz = G2O_Error_Viz('NavData/savedwaypoints/PunchlinePlot/result.g2o', 'NavData/savedwaypoints/PunchlinePlot/data_cp.g2o', "NavData/savedwaypoints/PunchlinePlot/naive.txt", (0,.085,.012))
    package = RosPack().get_path('navigation_prototypes')
    g2o_result_path = path.join(package, 'data/data_g2o/result.g2o')
    g2o_data_path = path.join(package, 'data/data_g2o/data_cp.g2o')
    g2o_raw_result = path.join(package, 'data/data_g2o/naive.txt')
    g2o_viz = G2O_Error_Viz(g2o_result_path,g2o_data_path,g2o_raw_result, (0, .085, .012))

    # g2o_viz = G2O_Error_Viz('prototypes/data_g2o_archived_data/6.19.2018.11.50/result.g2o',
    #                         'prototypes/data_g2o_archived_data/6.19.2018.11.50/data_cp.g2o',
    #                         "prototypes/data_g2o_archived_data/6.19.2018.11.50/naive.txt", (0, .085, .012))
    g2o_viz.run()
