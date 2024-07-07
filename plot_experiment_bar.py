import pickle
from pydrake.all import Hyperrectangle, HPolyhedron, RandomGenerator
import numpy as np
import os
from iris_environments.environments import env_names
import yaml
import matplotlib.gridspec as gridspec
import matplotlib.ticker as ticker
import matplotlib.pyplot as plt

i_seed = 4

def get_env_name(path):
    for e in env_names:
        if e in path:
            return e    

keys_stats = ['times', 'volumes', 'fraction_in_collision', 'num_faces']
axis_labels = ['time [s]', 'volume [rad^dof]', 'fraction_in_collision', 'num_faces']
stat_titles = ['Computation Time', 'Region Volume', 'Frac Region in Collision', 'Number Faces']

default_configs_to_plot = [#'config_1', 
                           #'config_2',
                        #    'config_3',
                        # 'config_precise',
                        # 'config_vfast',
                        # 'config_fast',
                        # 'config_medium',
                           ]
data = {}
for e in env_names:
    data[e] = {}
    for c in default_configs_to_plot:
        data[e][f"default/{c}"] = {}

# root = os.path.dirname(os.path.abspath(__file__))
root = os.path.abspath('')
for conf in default_configs_to_plot:
    default_exp_path = [e for e in os.listdir(root + f"/benchmarks/default_experiments/{conf}") if e.endswith('pkl')]
    for exp in default_exp_path:
        env_name = get_env_name(exp)
        with open(root + f"/benchmarks/default_experiments/{conf}/"+exp, 'rb') as f:
            result = pickle.load(f)
            
            data[env_name][f"default/{conf}"]['mean_stats'] = [ np.mean(result[k]) for k in keys_stats]
            data[env_name][f"default/{conf}"]['min_stats'] = [ np.min(result[k]) for k in keys_stats]
            data[env_name][f"default/{conf}"]['max_stats'] = [ np.max(result[k]) for k in keys_stats]


experiments_to_add = [
    #'fast_iris/setting_1', 
    #'fast_iris/setting_2',
    #'fast_iris/config_1',
    #'sampled_iris/config_4',
    # 'fast_iris/config_3',
    #'fast_iris/config_2',
    #'fast_iris/unadaptive_test_cfg_0',
    #'fast_iris/unadaptive_newtest_cfg_1',
    # 'ray_iris/setting_1',
    # 'ray_iris/max_iter_sep_planes_999',
    # 'ray_iris/face_ray_steps_20',
    # 'ray_iris/batch_size_500',
    # 'ray_iris/fast',
    # 'ray_iris/fast_1',
    # 'ray_iris/fast_2',
    # 'ray_iris/fast_3',
    # 'ray_iris/fast_4',
    # 'ray_iris/fast_5',
    # 'ray_iris/fast_6',
    # 'ray_iris/fast_final_original',
    # 'ray_iris/fast_final',
    # 'ray_iris/balanced_2',
    # 'ray_iris/balanced_3',
    # 'ray_iris/balanced_4',
    # 'ray_iris/balanced_final_original',
    # 'ray_iris/balanced_final',
    # 'greedy_iris/balanced'
    # 'ray_iris/fast_final_insufficient_mixing',
    'ray_iris/fast_final',
    # 'ray_iris/fast_final_only_walk_toward_collisions'
    # 'greedy_iris/fast',
    'greedy_iris/fast_after_sort',
    # 'greedy_iris/balanced_after_sort',
    # 'ray_iris/fast_final_smaller_batches',
    # 'ray_iris/batch_size_1500'
    # 'ray_iris/face_ray_steps_10_batch_size_500',
    # 'ray_iris/only_walk_toward_collisions'
    # 'ray_iris/balanced_final',
    # 'ray_iris/balanced_4',
    # 'ray_iris/precise_1',
    # 'ray_iris/precise_2',
    # 'ray_iris/precise_3',
    # 'ray_iris/precise_4',
    # 'ray_iris/precise_5',
    # 'ray_iris/precise_6',
    # 'ray_iris/precise_final',
    # 'greedy_iris/precise',
    # 'ray_iris/precise_3_mix_more',
    # 'ray_iris/precise_3_mix_medium',
    # 'ray_iris/precise_3_mix_medium_minus'
    # 'fast_iris/unadaptive_balanced_final',
    # 'fast_iris/unadaptive_fast_final',
    # 'greedy_iris/precise_after_sort',
    ]
# names = ['vf', 'IICS_f', 'medium','FastIris_doubletest']
# names = [
#          'default medium', 
#         #  'ray iris 1', 
#         #  'max_iter_sep_planes = 40', 
#         #  'face_ray_steps = 20',
#          'batch_size = 500',
#         #  'batch_size = 1500',
#         #  'face_ray_steps_10_batch_size_500',
#         #  'only_walk_toward_collisions'
#         ]
names = default_configs_to_plot + experiments_to_add
#"['2DOFFLIPPER_641ed63424.pkl', '3DOFFLIPPER_a33a92c6d1.pkl']

for exp_name in experiments_to_add:
    # get num_trials by loading params yaml
    num_trials = {}
    for e in os.listdir(root+f"/logs/{exp_name}/parameters"):
        with open(root+f"/logs/{exp_name}/parameters/" + e, 'r') as f:
            settings = yaml.safe_load(f)
            num_trials[e.split('_')[0]] = settings["num_trials"]
    print(num_trials)

    env_experiments = []
    for e in os.listdir(root+f"/logs/{exp_name}"):
        if e[-4:] == '.pkl':
            env_experiments.append(e)
    for e in env_names:
        data[e][exp_name] = {}
    for exp in env_experiments:
        env_name = get_env_name(exp)
        with open(root + f"/logs/{exp_name}/"+exp, 'rb') as f:
            num_trials_env = num_trials[env_name]
            result = pickle.load(f)
            data[env_name][exp_name]['mean_stats'] = [ np.mean(result[k][i_seed * num_trials_env:(i_seed + 1) * num_trials_env]) for k in keys_stats]
            data[env_name][exp_name]['min_stats'] = [ np.min(result[k][i_seed * num_trials_env:(i_seed + 1) * num_trials_env]) for k in keys_stats]
            data[env_name][exp_name]['max_stats'] = [ np.max(result[k][i_seed * num_trials_env:(i_seed + 1) * num_trials_env]) for k in keys_stats]

bar_width = 10
colors = ['red', 'blue', 'green', 'orange']

with open('iris_environments/env_statistics.txt', 'r') as f:
    lines = f.readlines()
env_stats = {}
for l in lines[1:]:
    chunks = l.split(',')
    stats = [int(chunks[0]), float(chunks[1]), float(chunks[2])]
    env_stats[chunks[-1].strip('\n').strip(' ')] = stats

fig = plt.figure(figsize=(25, 10))
outer_grid = gridspec.GridSpec(2, 2, wspace=0.05, hspace=0.17)
for statid, k in enumerate(keys_stats):
    experiments = list(data[env_names[0]].keys())
    inner_grid = gridspec.GridSpecFromSubplotSpec(1, len(env_names), subplot_spec=outer_grid[statid], wspace=0.5)
    for i_env, e in enumerate(env_names):
        ax = plt.Subplot(fig, inner_grid[i_env])
        
        for i_exp, exp in enumerate(experiments):
            xloc = []
            min_stats = []
            max_stats = []
            mean_stats = []
            vols = []
            if 'mean_stats' in data[e][exp].keys():
                min_stats = data[e][exp]['min_stats'][statid]
                max_stats = data[e][exp]['max_stats'][statid]
                mean_stats = data[e][exp]['mean_stats'][statid]
                vols = 1 # replace with appropriate value or logic if needed
                ax.set_yscale('log')
                if 'volume' in axis_labels[statid]:
                    mean_stats /= vols
                    min_stats /= vols
                    max_stats /= vols

                err = np.array([[mean_stats - min_stats], [max_stats - mean_stats]])
        
                ax.bar(i_exp * 1.25 * bar_width, mean_stats, width=bar_width, color=colors[i_exp], yerr=err, capsize=5, label=exp, alpha=0.4, edgecolor='none')
                
                # Apply scientific notation
                # ax.yaxis.set_major_formatter(ticker.ScalarFormatter(useMathText=True))
                # ax.yaxis.get_major_formatter().set_scientific(True)
                # ax.yaxis.get_major_formatter().set_powerlimits((0, 0))

                # Reduce the number of y-ticks
                ax.yaxis.set_major_locator(ticker.MaxNLocator(nbins=3, prune=None))
                ax.yaxis.set_minor_locator(ticker.MaxNLocator(nbins=3, prune=None))
                # ax.yaxis.set_major_locator(ticker.LogLocator(base=10, numticks=2))
                # ax.yaxis.set_minor_locator(ticker.LogLocator(base=10, subs='auto', numticks=1))

                ax.set_title(e, fontsize=9)
                ax.tick_params(axis='y', which='both', labelrotation=90, labelsize=7.5)

                fig.add_subplot(ax)
        
        ax.set_xticklabels([])
        
        # # Rotate y-axis tick labels
        # for label in ax.get_yticklabels():
        #     label.set_rotation(90)

    ax_outer = fig.add_subplot(outer_grid[statid])
    ax_outer.set_title(axis_labels[statid], pad=20, fontweight='bold')
    ax_outer.axis('off')  # Hide the axis to make it clean

# Adjust layout to make space for the main title
plt.tight_layout()
plt.subplots_adjust(top=0.85)

# Add a single legend for the entire figure
handles = [plt.Rectangle((0,0),1,1, color=colors[i], alpha=0.6, edgecolor='none') for i in range(len(experiments))]
labels = experiments
fig.legend(handles, labels, loc='upper center', bbox_to_anchor=(0.5, 1.05), fontsize='large', title='Experiments')

plt.show()