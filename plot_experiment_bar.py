import pickle
from pydrake.all import Hyperrectangle, HPolyhedron, RandomGenerator
import numpy as np
import os
from iris_environments.environments import env_names
import yaml
import matplotlib.gridspec as gridspec
import matplotlib.ticker as ticker
import matplotlib.pyplot as plt

# i_seed = 1
seed_nums = {}
# for env_name in env_names:
#     seed_nums[env_name] = [i for i in range(10)]

# seeds for paper:
seed_nums["5DOFUR3"] = [1]
seed_nums["3DOFFLIPPER"] = [7]
seed_nums["6DOFUR3"] = [1]
seed_nums["7DOFIIWA"] = [1]
seed_nums["7DOF4SHELVES"] = [1]
seed_nums["7DOFBINS"] = [1]
seed_nums["14DOFIIWAS"] = [7]
seed_nums["15DOFALLEGRO"] = [7]

paper_names = {}
paper_names["5DOFUR3"] = "UR3"
paper_names["3DOFFLIPPER"] = "Flipper"
paper_names["6DOFUR3"] = "UR3Wrist"
paper_names["7DOFIIWA"] = "IIWAShelf"
paper_names["7DOF4SHELVES"] = "IIWA4Shelves"
paper_names["7DOFBINS"] = "IIWABins"
paper_names["14DOFIIWAS"] = "2IIWAs"
paper_names["15DOFALLEGRO"] = "Allegro"


def get_env_name(path):
    for e in env_names:
        if e in path:
            return e    
        
do_legend = False

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
    # for c in default_configs_to_plot:
    #     data[e][f"default/{c}"] = {}

# root = os.path.dirname(os.path.abspath(__file__))
root = os.path.abspath('')
# for conf in default_configs_to_plot:
#     default_exp_path = [e for e in os.listdir(root + f"/benchmarks/default_experiments/{conf}") if e.endswith('pkl')]
#     for exp in default_exp_path:
#         env_name = get_env_name(exp)
#         with open(root + f"/benchmarks/default_experiments/{conf}/"+exp, 'rb') as f:
#             result = pickle.load(f)
            
#             data[env_name][f"default/{conf}"]['mean_stats'] = [ np.mean(result[k]) for k in keys_stats]
#             data[env_name][f"default/{conf}"]['min_stats'] = [ np.min(result[k]) for k in keys_stats]
#             data[env_name][f"default/{conf}"]['max_stats'] = [ np.max(result[k]) for k in keys_stats]


# experiments_to_add = [
    # "greedy_iris/precise_after_sort",
    # "ray_iris/precise_final",
    # "ray_iris/precise_final_sample_dist_step_size",
    # "ray_iris/precise_final_sample_dist_step_size_half_batch",
    # "ray_iris/precise_final_sample_dist_step_size_quarter_batch",
    # "ray_iris/precise_final_2",
    # "ray_iris/precise_final_2",
    # "ray_iris/precise_final_more_steps",
    # "ray_iris/precise_all_samples",
    # "ray_iris/precise_only_collisions",
    # "fast_iris/unadaptive_balanced_final",
    # "greedy_iris/fast_after_sort",
    # "ray_iris/fast_final",
    # "ray_iris/fast_final_sample_dist_step_size",
    # "ray_iris/fast_final_sample_dist_step_size_half_batch",
    # "ray_iris/fast_final_sample_dist_step_size_quarter_batch",
    # "ray_iris/fast_final_2"
    # "ray_iris/fast_all_samples",
    # "ray_iris/fast_only_collisions",
    # "fast_iris/unadaptive_fast_final",
    # ]
# experiments_to_add = ["paper_plots/fast/final_fast_paper",
#                       "paper_plots/greedy/fast_after_sort",
#                       "paper_plots/ray/fast_final_2_pete"]

experiments_to_add = ["paper_plots/fast/final_precise",
                      "paper_plots/greedy/precise_after_sort",
                      "paper_plots/ray/precise_final_2_pete"]

# experiments_to_add = ["paper_plots/ray/precise_final_2_pete",
#                       "ray_iris/precise_final_2"]

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
            # data[env_name][exp_name]['mean_stats'] = [ np.mean(result[k][i_seed * num_trials_env:(i_seed + 1) * num_trials_env]) for k in keys_stats]
            # data[env_name][exp_name]['min_stats'] = [ np.min(result[k][i_seed * num_trials_env:(i_seed + 1) * num_trials_env]) for k in keys_stats]
            # data[env_name][exp_name]['max_stats'] = [ np.max(result[k][i_seed * num_trials_env:(i_seed + 1) * num_trials_env]) for k in keys_stats]
            data[env_name][exp_name]['mean_stats'] = []
            data[env_name][exp_name]['min_stats'] = []
            data[env_name][exp_name]['max_stats']= []
            for k in keys_stats:
                stats = np.array([])
                for i_seed in seed_nums[env_name]:
                    stats = np.hstack((stats, result[k][i_seed * num_trials_env:(i_seed + 1) * num_trials_env]))
                    print(env_name)
                    print(exp_name)
                    print(k)
                    print(i_seed)
                    print(stats)
                
                data[env_name][exp_name]['mean_stats'].append(np.mean(stats)) 
                data[env_name][exp_name]['min_stats'].append(np.min(stats)) 
                data[env_name][exp_name]['max_stats'].append(np.max(stats)) 


bar_width = 10
colors = ['red', 'blue', 'green', 'orange']
# colors = ['blue', 'orange']

with open('iris_environments/env_statistics.txt', 'r') as f:
    lines = f.readlines()
env_stats = {}
for l in lines[1:]:
    chunks = l.split(',')
    stats = [int(chunks[0]), float(chunks[1]), float(chunks[2])]
    env_stats[chunks[-1].strip('\n').strip(' ')] = stats

fig = plt.figure(figsize=(20, 6))
outer_grid = gridspec.GridSpec(2, 2, wspace=-0.04, hspace=0.3)
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
                vols.append(env_stats[e][2])
                ax.set_yscale('log')
                if 'volume' in axis_labels[statid]:
                    assert len(vols) == 1
                    mean_stats /= vols[0]
                    min_stats /= vols[0]
                    max_stats /= vols[0]

                err = np.array([[mean_stats - min_stats], [max_stats - mean_stats]])
        
                ax.bar(i_exp * 1.25 * bar_width, mean_stats, width=bar_width, color=colors[i_exp], yerr=err, capsize=5, label=exp, alpha=0.4, edgecolor='none')
                
                # Apply scientific notation
                # ax.yaxis.set_major_formatter(ticker.ScalarFormatter(useMathText=True))
                # ax.yaxis.get_major_formatter().set_scientific(True)
                # ax.yaxis.get_major_formatter().set_powerlimits((0, 0))

                # Reduce the number of y-ticks
                # ax.yaxis.set_major_locator(ticker.MaxNLocator(nbins=3, prune=None))
                # ax.yaxis.set_minor_locator(ticker.MaxNLocator(nbins=3, prune=None))
                # ax.yaxis.set_major_locator(ticker.LogLocator(base=10, numticks=2))
                # ax.yaxis.set_minor_locator(ticker.LogLocator(base=10, subs='auto', numticks=1))

                ax.set_title(paper_names[e], fontsize=10.5)
                ax.tick_params(axis='y', which='both', labelrotation=90, labelsize=7.5)

                fig.add_subplot(ax)
        
        ax.set_xticklabels([])
        
        # # Rotate y-axis tick labels
        # for label in ax.get_yticklabels():
        #     label.set_rotation(90)

    ax_outer = fig.add_subplot(outer_grid[statid])
    ax_outer.set_title(axis_labels[statid], pad=20, fontweight='bold', fontsize=14)
    ax_outer.axis('off')  # Hide the axis to make it clean

# Adjust layout to make space for the main title
plt.tight_layout()
plt.subplots_adjust(top=0.85)

if do_legend:
    # Add a single legend for the entire figure
    handles = [plt.Rectangle((0,0),1,1, color=colors[i], alpha=0.6, edgecolor='none') for i in range(len(experiments))]
    labels = experiments
    fig.legend(handles, labels, loc='upper center', bbox_to_anchor=(0.5, 1.03), fontsize='large', title='Experiments')

plt.show()