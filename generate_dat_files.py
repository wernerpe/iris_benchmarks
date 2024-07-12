import matplotlib.pyplot as plt
import os
import pickle
from iris_environments.environments import env_names
import numpy as np

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
                        #'config_vfast',
                        'config_precise',
                           ]
data = {}
for e in env_names:
    data[e] = {}
    for c in default_configs_to_plot:
        data[e][f"default/{c}"] = {}

root = os.path.dirname(os.path.abspath(__file__))
for conf in default_configs_to_plot:
    default_exp_path = [e for e in os.listdir(root + f"/benchmarks/default_experiments/{conf}") if e.endswith('pkl')]
    for exp in default_exp_path:
        env_name = get_env_name(exp)
        with open(root + f"/benchmarks/default_experiments/{conf}/"+exp, 'rb') as f:
            result = pickle.load(f)

            data[env_name][f"default/{conf}"]['mean_stats'] = [ np.mean(result[k]) for k in keys_stats]
            data[env_name][f"default/{conf}"]['min_stats'] = [ np.min(result[k]) for k in keys_stats]
            data[env_name][f"default/{conf}"]['max_stats'] = [ np.max(result[k]) for k in keys_stats]

            regs = result['regions']
            vols_ellipsoids= [r.MaximumVolumeInscribedEllipsoid().Volume() for r in regs]
            mean_vol = np.mean(vols_ellipsoids)
            min_vol = np.min(vols_ellipsoids)
            max_vol = np.max(vols_ellipsoids)

            data[env_name][f"default/{conf}"]['mean_stats'][1] = mean_vol
            data[env_name][f"default/{conf}"]['min_stats'][1] = min_vol
            data[env_name][f"default/{conf}"]['max_stats'][1] = max_vol

            print(f'env {env_name}')
            print(result['fraction_in_collision'])
            print('OVERWRITNGIN VOLUMES WITH ELLIPSOID VOLUMES')
#build irisNP table


experiments_to_add = [
    #'fast_iris/setting_1', 
    #'fast_iris/setting_2',
    #'fast_iris/config_1',
    #'sampled_iris/config_4',
    #'fast_iris/config_3',
    #'fast_iris/config_2',
    #'fast_iris/unadaptive_test_cfg_0',
    'fast_iris/unadaptive_balanced_3'
    ]
names = ['IICS_vf', 'IICS_f', 'FastIris_doubletest']
#"['2DOFFLIPPER_641ed63424.pkl', '3DOFFLIPPER_a33a92c6d1.pkl']

for exp_name in experiments_to_add:
    env_experiments = []
    for e in os.listdir(root+f"/logs/{exp_name}"):
        if e[-4:] == '.pkl':
            env_experiments.append(e)
    for e in env_names:
        data[e][exp_name] = {}
    for exp in env_experiments:
        env_name = get_env_name(exp)
        with open(root + f"/logs/{exp_name}/"+exp, 'rb') as f:
            result = pickle.load(f)
            data[env_name][exp_name]['mean_stats'] = [ np.mean(result[k]) for k in keys_stats]
            data[env_name][exp_name]['min_stats'] = [ np.min(result[k]) for k in keys_stats]
            data[env_name][exp_name]['max_stats'] = [ np.max(result[k]) for k in keys_stats]

column_names_iris_np = [] 
column_names_iris_np += [s+'_mean' for s in keys_stats]
column_names_iris_np += [s+'_err_minus' for s in keys_stats]
column_names_iris_np += [s+'_err_plus' for s in keys_stats]
# column_names_iris_np += [s+'_balanced_mean' for s in keys_stats]
# column_names_iris_np += [s+'_balanced_min' for s in keys_stats]
# column_names_iris_np += [s+'_balanced_max' for s in keys_stats]
# column_names_iris_np += [s+'_precise_mean' for s in keys_stats]
# column_names_iris_np += [s+'_precise_min' for s in keys_stats]
# column_names_iris_np += [s+'_precise_max' for s in keys_stats]

with open('iris_environments/env_statistics.txt', 'r') as f:
    lines = f.readlines()
env_stats = {}
for l in lines[1:]:
    chunks = l.split(',')
    stats = [int(chunks[0]), float(chunks[1]), float(chunks[2])]
    env_stats[chunks[-1].strip('\n').strip(' ')] = stats


def get_statid(colname):
    substr = colname.split('_')[0]
    for i, k in enumerate(keys_stats):  
        if substr in k:
            return i
        
cfg_names = ['unadaptive_balanced_3']
dirname = 'fast_iris'
data_iris_np = {}
i = 0
for col_idx, colum_name in enumerate(column_names_iris_np):
    corresp_cfg = cfg_names[i]
    column_data = []
    for env_name in env_names:
        if len(data[env_name][f"{dirname}/{corresp_cfg}"])==0:
            column_data.append(0)
        else:
            statid = get_statid(colum_name)
            if 'mean' in colum_name:
                dat = data[env_name][f"{dirname}/{corresp_cfg}"]['mean_stats'][statid]
            if '_err_minus' in colum_name:
                
                min = data[env_name][f"{dirname}/{corresp_cfg}"]['min_stats'][statid]
                mean = data[env_name][f"{dirname}/{corresp_cfg}"]['mean_stats'][statid]
                dat = mean-min
            if '_err_plus' in colum_name:
                max = data[env_name][f"{dirname}/{corresp_cfg}"]['max_stats'][statid]
                mean = data[env_name][f"{dirname}/{corresp_cfg}"]['mean_stats'][statid]
                dat = max-mean
            if 'volume' in colum_name:
                dat/=env_stats[env_name][2]
            column_data.append(dat)
    data_iris_np[colum_name] = column_data
    i = int(col_idx/12)

data_iris_np['env'] = np.arange(len(env_names))
def save_dict_to_dat(data_dict, file_path):
    with open(file_path, 'w') as f:
        # Write column names
        f.write("\t".join(data_dict.keys()) + "\n")
        
        # Write data rows
        for row in zip(*data_dict.values()):
            f.write("\t".join(map(str, row)) + "\n")

save_dict_to_dat(data_iris_np, f'{dirname}_{cfg_names[0].split("_")[1:]}_3.tex')
# fig, axs = plt.subplots(nrows=2, ncols=2, figsize= (15,10))
# axs_squeezed = [axs[0][0], axs[0][1], axs[1][0], axs[1][1]]
# for statid, (k, ax) in enumerate(zip(keys_stats, axs_squeezed)):
#     experiments = list(data[env_names[0]].keys())
#     for i_exp, exp in enumerate(experiments):
#         xloc = []
#         min_stats = []
#         max_stats = []
#         mean_stats = []
#         for xl, e in enumerate(env_names):
#             if 'mean_stats' in data[e][exp].keys():
#                 xloc.append(xl)
#                 min_stats.append(data[e][exp]['min_stats'][statid])
#                 max_stats.append(data[e][exp]['max_stats'][statid])
#                 mean_stats.append(data[e][exp]['max_stats'][statid])
#         ax.scatter(xloc, mean_stats, label = names[i_exp], s= 80)
#         ax.set_yscale('log')
#         err = [np.array(min_stats),
#                np.array(max_stats)]
#         artist = ax.errorbar(xloc, mean_stats, yerr = err, fmt='o', capsize=5, capthick=2)
#         ax.plot(xloc, mean_stats, linewidth = 0.5, color= artist.lines[0].get_color())
#         ax.set_xlabel('Environment')
#         ax.set_ylabel(axis_labels[statid])
#         ax.set_xticks(range(len(env_names)))
#         ax.set_xticklabels(env_names, fontsize = 8)
#         ax.legend()
#         ax.set_title(stat_titles[statid])
        
# plt.show()
# print(data)

