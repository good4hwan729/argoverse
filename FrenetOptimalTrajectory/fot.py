import fot_wrapper
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
    conds = {
        's0': 0,
        'target_speed': 20,
        'wp': [[0, 0], [50, 0], [150, 0]],
        'obs': [[-50.0, -2.0, -46.0, 2.0]],
        'pos': [0, 0],
        'vel': [0, 0],
    }  # paste output from debug log
    
    base_conds = {
        's0': 0,
        'target_speed': 20,
        'wp': [[0, 0], [50, 0], [150, 0]],
        'obs': [[30, 30, 30, 30]],
        'pos': [0, 0],
        'vel': [0, 0],
    } # paste output from debug log

    initial_conditions = {
        'ps': conds['s0'],
        'target_speed': conds['target_speed'],
        'pos': np.array(conds['pos']),
        'vel': np.array(conds['vel']),
        'wp': np.array(conds['wp']),
        'obs': np.array(conds['obs'])
    }
    
    base_conditions = {
        'ps': base_conds['s0'],
        'target_speed': base_conds['target_speed'],
        'pos': np.array(base_conds['pos']),
        'vel': np.array(base_conds['vel']),
        'wp': np.array(base_conds['wp']),
        'obs': np.array(base_conds['obs'])
    }

    hyperparameters = {
        "max_speed": 25.0,
        "max_accel": 15.0,
        "max_curvature": 15.0,
        "max_road_width_l": 10.0,
        "max_road_width_r": 10.0,
        "d_road_w": 5.0,
        "dt": 0.2,
        "maxt": 5.0,
        "mint": 2.0,
        "d_t_s": 0.5,
        "n_s_sample": 2.0,
        "obstacle_clearance": 2.0,
        "kd": 1.0,
        "kv": 0.1,
        "ka": 0.1,
        "kj": 0.1,
        "kt": 0.1,
        "ko": 0.1,
        "klat": 1.0,
        "klon": 1.0
    }
    # static elements of planner
    wx = initial_conditions['wp'][:, 0]
    wy = initial_conditions['wp'][:, 1]
    obs = np.array(conds['obs'])

    # simulation config
    show_animation = True
    sim_loop = 200
    area = 40
    total_time = 0
    result = np.zeros((40, 2))
    base = np.zeros((40, 2))
    for i in range(sim_loop):
        if i < 80:
            initial_conditions['obs'][0,0] += 1.4
            initial_conditions['obs'][0,2] += 1.4
        #if 10 <= i < 20:
        #    initial_conditions['obs'][0,0] += 0.3
        #    initial_conditions['obs'][0,2] += 0.3
        #    initial_conditions['obs'][0,1] += 0.3
        #    initial_conditions['obs'][0,3] += 0.3
        #else:
        #    initial_conditions['obs'][0,0] += 1.0
        #    initial_conditions['obs'][0,2] += 1.0
        # run FOT and keep time
        print("Iteration: {}".format(i))
        start_time = time.time()
        result_x, result_y, speeds, ix, iy, iyaw, d, s, speeds_x, \
            speeds_y, misc, costs, success = \
            fot_wrapper.run_fot(initial_conditions, hyperparameters)
        end_time = time.time() - start_time
        print("Time taken: {}".format(end_time))
        #obs = result_obs
        total_time += end_time
        
        base_x, base_y, base_speeds, base_ix, base_iy, base_iyaw, base_d, base_s, base_speeds_x, \
            base_speeds_y, base_misc, base_costs, base_success = \
            fot_wrapper.run_fot(base_conditions, hyperparameters)
        result[i][0] = result_x[1]
        result[i][1] = result_y[1]
        base[i][0] = result_x[1]
        base[i][1] = 0
        # reconstruct initial_conditions
        if success:
            initial_conditions['pos'] = np.array([result_x[1], result_y[1]])
            initial_conditions['vel'] = np.array([speeds_x[1], speeds_y[1]])
            initial_conditions['ps'] = misc['s']
            print(costs)
        else:
            print("Failed unexpectedly")
            break

        # break if near goal
        if np.hypot(result_x[1] - wx[-1], result_y[1] - wy[-1]) <= 3.0:
            print("Goal")
            break

        if show_animation:  # pragma: no cover
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None]
            )
            plt.plot(wx, wy)
            if obs.shape[0] == 0:
                obs = np.empty((0, 4))
            ax = plt.gca()
            for o in initial_conditions['obs']:
                rect = patch.Rectangle((o[0], o[1]),
                                       o[2] - o[0],
                                       o[3] - o[1])
                ax.add_patch(rect)
            agent = patch.Rectangle((result_x[1] - 4, result_y[1]-2), 4, 4)
            ax.add_patch(agent)
            plt.plot(result_x[1:], result_y[1:], "-or")
            plt.plot(result_x[1], result_y[1], "vc")
            plt.xlim(result_x[1] - area, result_x[1] + area)
            plt.ylim(result_y[1] - area, result_y[1] + area)
            plt.xlabel("X axis")
            plt.ylabel("Y axis")
            plt.title("v[m/s]:" + str(
                      np.linalg.norm(initial_conditions['vel']))[0:4]
            )
            plt.grid(True)
            plt.savefig("{}.jpg".format(i))
            plt.pause(0.1)
    plt.show()

    plt.plot(real_dtw_distance(result, base)[0], label ='with obstacle')
    plt.axhline(y=0, label ='without obstacle', color = 'green')
    #plt.axvline(x=10, linestyle='dashed', label = 'Side Vehicle sudden line change', color = 'red')
    #plt.axvline(x=45, linestyle='dashed', label = 'Vehicle Crash', color = 'orange')
    plt.xlabel("Timestamp")
    plt.ylabel("Risk")
    plt.title('Dynamic Time Warp Risk for Scenario 2')
    plt.legend(loc="upper left")
    plt.show()
    print("Finish")
    print(real_dtw_distance(result, base)[1])
    print("Average time per iteration: {}".format(total_time / i))

if __name__ == '__main__':
    main()
