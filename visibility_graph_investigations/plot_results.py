import numpy as np
import pickle
import os

def load_data(directory):
    """
    Load all pickle files from the given directory.
    
    Args:
    directory (str): Path to the directory containing pickle files.
    
    Returns:
    dict: A dictionary where keys are filenames and values are loaded data.
    """
    data = {}
    for filename in os.listdir(directory):
        if filename.endswith('.pkl'):
            with open(os.path.join(directory, filename), 'rb') as f:
                data_file = pickle.load(f)
                split_name = filename.split('_')
                approach = split_name[0]
                num_nodes = int(split_name[-4])
                sampler_key = split_name[5]
                sampler_name = ''
                if sampler_key == 'uniform':
                    sampler_name = 'uniform'
                elif sampler_key == 'ts':
                    sampler_name = 'fullts'
                elif sampler_key == 'shelves':
                    sampler_name = 'shelves'
                else:
                    raise ValueError("unkonwn sampler type")
                if sampler_name not in data.keys():
                    data[sampler_name] = {}
                if num_nodes not in data[sampler_name].keys():
                    data[sampler_name][num_nodes] = {}                   
                
                data[sampler_name][num_nodes][approach] = data_file
    return data

cwd = os.path.dirname(__file__)
dir = cwd + '/graphs/iiwa_bins/clique_covers'

data = load_data(dir)

print(data.keys())
print(data['uniform'].keys())

# make 3 rows (one per sampler type) of 4 for plots showing the following
# ratio of cliques containing collisions
# ratio of non-geometric cliques
# size of clique cover
# num connected components in graph

# make the x axis the number of nodes. Make bar plots for the plots that are comparing stats for the different approaches
# you can find the 'stats' dictionary under data[sampler_name][num_nodes][approach]['stats']. It contains the following 
# (some of the values are not filled in in this example)


    # stats = {
    #     'num_connected_components': count_connected_components(vg_adj),
    #     'is_vg_uniformly_sampled': is_vg_sampled_uniformly,  
    #     'num_cliques': len(cliques),
    #     'max_clique_size': np.max(clique_sizes),
    #     'min_clique_size': np.min(clique_sizes),
    #     'num_non_cliques': 0,
    #     'num_non_geometric_cliques': 0,
    #     'num_samples_in_cvxh_per_clique': num_samples_in_cvxh,
    #     'num_cliques_enclosing_found_collisions': 0,
    #     'num_collisions_in_cvxh_of_clique': [],
    #     'clique_sizes': clique_sizes,
    #     'non_cliques': [],
    #     'non_geometric_cliques': []
    # }

import matplotlib.pyplot as plt


def plot_metric(ax, data, sampler, metric, title, ylabel, is_ratio=False):
    approaches = list(data[sampler][list(data[sampler].keys())[0]].keys())
    x = sorted(data[sampler].keys())
    
    width = 0.35
    multiplier = 0
    
    for approach in approaches:
        offset = width * multiplier
        if is_ratio:
            values = []
            for nodes in x:
                if approach in data[sampler][nodes]:
                    stats = data[sampler][nodes][approach]['stats']
                    ratio = stats[metric] / stats['num_cliques'] if stats['num_cliques'] > 0 else 0
                    values.append(ratio)
                else:
                    values.append(0)
        else:
            values = [data[sampler][nodes][approach]['stats'][metric] if approach in data[sampler][nodes] else 0 for nodes in x]
        
        rects = ax.bar([xi + offset for xi in range(len(x))], values, width, label=approach)
        ax.bar_label(rects, padding=3, fmt='%.2f' if is_ratio else '%d')
        multiplier += 1

    ax.set_xlabel('Number of Nodes')
    ax.set_ylabel(ylabel)
    ax.set_title(f'{title} - {sampler.capitalize()} Sampler')
    ax.set_xticks([xi + width/2 for xi in range(len(x))])
    ax.set_xticklabels(x)
    ax.legend(loc='best')

cwd = os.path.dirname(__file__)
dir = cwd + '/graphs/iiwa_bins/clique_covers'

data = load_data(dir)

print(data.keys())
print(data['uniform'].keys())

# Create the plot
fig, axs = plt.subplots(3, 4, figsize=(20, 15))
fig.suptitle('Clique Cover Analysis', fontsize=16)

samplers = ['uniform', 'fullts', 'shelves']
metrics = [
    ('num_cliques_enclosing_found_collisions', 'Ratio of Cliques Containing Collisions', 'Ratio', True),
    ('num_non_geometric_cliques', 'Ratio of Non-Geometric Cliques', 'Ratio', True),
    ('num_cliques', 'Size of Clique Cover', 'Number of Cliques', False),
    ('num_connected_components', 'Number of Connected Components', 'Number of Components', False)
]

for i, sampler in enumerate(samplers):
    for j, (metric, title, ylabel, is_ratio) in enumerate(metrics):
        plot_metric(axs[i, j], data, sampler, metric, title, ylabel, is_ratio)

# plt.tight_layout()
plt.show()
