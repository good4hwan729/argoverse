import hybrid_astar_wrapper
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patch
from dtaidistance import dtw_ndim


def coordinate_path(x, y):
    result = np.zeros((len(x), 2))
    for i in range(len(x)):
    	result[i][0] = x[i]
    	result[i][1] = y[i]
    return result

def real_dtw_distance(path1, path2):
    # path1: base path (no obstacles)
    # path2: new path (with obstacles
    n = len(path1)
    idx = np.linspace(0, len(path2)-1, len(path1))
    dtw_dist = []
    for i in range(n):
        if i < 5:
            dtw_dist.append(dtw_ndim.distance(path1[:i+1], path2[:int(idx[i])+1]))
        else:
            dtw_dist.append(dtw_ndim.distance(path1[i-5:i+1], path2[int(idx[i])-5:int(idx[i])+1]))
    post_risk = dtw_ndim.distance(path1, path2)
    return dtw_dist, post_risk
    	        
    	    

def main():
    """A debug script for the hybrid astar planner.

    This script will solve the hybrid astar problem in a
    standalone simulation and visualize the results or raise an error if a
    path is not found.
    """
    print(__file__ + " start!!")
    sim_loop = 100
    area = 20.0  # animation area length [m]
    show_animation = True

    initial_conditions = {
        'start': np.array([10, 15, 0]),
        'end': np.array([50, 15, 0]),
        'obs': np.array([
            [26.0, 14.0, 28.0, 16.0],
            #[26.0, 24.0, 32.0, 30.0],
        ]),
    }  # paste output from debug log
    
    initial_conditions_2 = {
        'start': np.array([10, 15, 0]),
        'end': np.array([50, 15, 0]),
        'obs': np.array([
            [15.0, 11.0, 17.0, 13.0],
            #[26.0, 24.0, 32.0, 30.0],
        ]),
    }  # paste output from debug log
    
    baseline_conditions = {
        'start': np.array([10, 15, 0]),
        'end': np.array([50, 15, 0]),
        'obs': np.array([
            [0, 0, 0, 0],
        ]),
    }

    hyperparameters = {
        "step_size": 3.0,
        "max_iterations": 10000,
        "completion_threshold": 1.0,
        "angle_completion_threshold": 3.0,
        "rad_step": 0.5,
        "rad_upper_range": 4.0,
        "rad_lower_range": 4.0,
        "obstacle_clearance": 0.5,
        "lane_width": 20.0,
        "radius": 3.0,
        "car_length": 2.0,
        "car_width": 2.0,
    }
    start_time = time.time()
    
    base_x, base_y, base_yaw, base_obs, base_success = \
        hybrid_astar_wrapper.apply_hybrid_astar(baseline_conditions,
                                                hyperparameters)
    result_x, result_y, result_yaw, result_obs, success = \
        hybrid_astar_wrapper.apply_hybrid_astar(initial_conditions,
                                                hyperparameters)
    end_time = time.time() - start_time
    #print("x: ", result_x)
    #print("y: ", result_y)
    path1 = coordinate_path(base_x, base_y)
    path2 = coordinate_path(result_x, result_y)
    print("Post Risk", real_dtw_distance(path1, path2)[1])
    plt.plot(real_dtw_distance(path1, path2)[0], label = 'With obstacle')
    plt.plot(real_dtw_distance(path1, path1)[0], label = 'Without obstacle')
    plt.axvline(x=10, linestyle='dashed', label = 'Side vehicle sudden lane change', color = 'red')
    plt.axvline(x=40, linestyle='dashed', label = 'Vehicle stabilization', color = 'orange')
    plt.xlabel("Timestamp")
    plt.ylabel("Risk")
    plt.title('Dynamic Time Warp Risk for Scenario 1')
    plt.legend(loc="upper left")
    plt.show()
    print("Time taken: {}s".format(end_time))
    print(success)
    if not success:
        print("FAILED")
        return
    start = initial_conditions['start']
    end = initial_conditions['end']
    #obs = initial_conditions['obs']

    for i in range(sim_loop):
        obs = result_obs[i]
        #print(obs)
        print("Iteration: {}".format(i))
        x = result_x[0]
        y = result_y[0]
        result_x = result_x[1:]
        result_y = result_y[1:]

        if np.hypot(x - end[0], y - end[1]) <= 2:
            print("Goal")
            break

        if show_animation:  # pragma: no cover
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None]
            )
            ax = plt.gca()
            for o in obs:
                rect = patch.Rectangle((o[0], o[1]),
                                       o[2] - o[0],
                                       o[3] - o[1])
                ax.add_patch(rect)
            plt.plot(start[0], start[1], "og")
            plt.plot(end[0], end[1], "or")
            if success:
                agent = patch.Rectangle((result_x[1] - 2, result_y[1]-1),
                                       2,
                                       2)
                ax.add_patch(agent)
                plt.plot(result_x[1:], result_y[1:], ".r")
                plt.plot(result_x[1], result_y[1], "vc")
                plt.xlim(result_x[1] - area, result_x[1] + area)
                plt.ylim(result_y[1] - area, result_y[1] + area)
            plt.xlabel("X axis")
            plt.ylabel("Y axis")
            plt.grid(True)
            plt.pause(0.1)

    print("Finish")
    if show_animation:  # pragma: no cover
        plt.grid(True)
        plt.pause(1)
        plt.show()


if __name__ == '__main__':
    main()
