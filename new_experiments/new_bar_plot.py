import pickle
import numpy as np
from pathlib import Path
seed_nums = {}
seed_nums["5DOFUR3"] = [1]
seed_nums["3DOFFLIPPER"] = [7]
seed_nums["6DOFUR3"] = [1]
seed_nums["7DOFIIWA"] = [1]
seed_nums["7DOF4SHELVES"] = [1]
seed_nums["7DOFBINS"] = [1]
seed_nums["14DOFIIWAS"] = [7]
seed_nums["15DOFALLEGRO"] = [7]

num_trials =10

def load_results_for_seed(experiment_name, settings, algs, env_names):
    """Load all experimental results from pickle files"""
    data = {}
    
    for env_name in env_names:
        data[env_name] = {}
        for alg in algs:
            pickle_path = f"new_experiments/{experiment_name}/{settings}/{alg}/{env_name}.pkl"
            try:
                with open(pickle_path, 'rb') as f:
                    results = pickle.load(f)
                slice_start = num_trials*seed_nums[env_name][0]
                slice_end = num_trials*(seed_nums[env_name][0]+1)
                # Calculate statistics
                times = np.array(results['times'][slice_start:slice_end])
                volumes = np.array(results['volumes'][slice_start:slice_end])
                num_faces = np.array([len(region.A()) for region in results['regions'][slice_start:slice_end]])
                frac_collision = np.array(results['fraction_in_collision'][slice_start:slice_end])
                
                data[env_name][alg] = {
                    'mean_stats': {
                        'time': np.mean(times),
                        'volume': np.mean(volumes),
                        'num_faces': np.mean(num_faces),
                        'frac_collision': np.mean(frac_collision)
                    },
                    'std_stats': {
                        'time': np.std(times),
                        'volume': np.std(volumes),
                        'num_faces': np.std(num_faces),
                        'frac_collision': np.std(frac_collision)
                    },
                    'all_times': times,
                    'all_volumes': volumes,
                    'all_num_faces': num_faces,
                    'all_frac_collision': frac_collision
                }
            except FileNotFoundError:
                print(f"Warning: Could not find {pickle_path}")
                data[env_name][alg] = {'mean_stats': {'time': 0, 'volume': 0, 'num_faces': 0, 'frac_collision': 0}}
    
    return data

def calculate_relative_volumes(data, env_names, algs):
    """Calculate relative volumes compared to iris_np baseline"""
    for env_name in env_names:
        if 'iris_np' in data[env_name] and data[env_name]['iris_np']['mean_stats']['volume'] > 0:
            baseline_volume = data[env_name]['iris_np']['mean_stats']['volume']
            for alg in algs:
                if alg in data[env_name]:
                    current_volumes = data[env_name][alg]['all_volumes']
                    rel_volumes = current_volumes / baseline_volume
                    data[env_name][alg]['mean_stats']['rel_volume'] = np.mean(rel_volumes)
                    data[env_name][alg]['std_stats']['rel_volume'] = np.std(rel_volumes)
                    data[env_name][alg]['all_rel_volumes'] = rel_volumes
        else:
            # If no baseline, set all relative volumes to 1
            for alg in algs:
                if alg in data[env_name]:
                    data[env_name][alg]['mean_stats']['rel_volume'] = 1.0
    
    return data

def find_best_values(data, env_names, algs, stat):
    """Find the best (lowest for time/frac_collision, highest for volume/rel_volume) value for each environment"""
    best_values = {}
    
    for env_name in env_names:
        values = []
        for alg in algs:
            if alg in data[env_name] and 'mean_stats' in data[env_name][alg]:
                values.append(data[env_name][alg]['mean_stats'][stat])
        
        if values:
            if stat in ['time', 'frac_collision', 'num_faces']:
                best_values[env_name] = min(values)
            else:  # volume, rel_volume
                best_values[env_name] = max(values)
    
    return best_values

def format_value_with_bold(value, is_best, stat, do_3f=False):
    """Format a value with proper precision and bold if it's the best"""
    if stat == 'rel_volume' and value >= 1000:
        formatted = f"{value:.2e}"
    else:
        if do_3f:
            formatted = f"{value:.3f}"
        else:
            formatted = f"{value:.3g}"
    
    if is_best:
        return f"\\textbf{{{formatted}}}"
    else:
        return formatted

import fitz  # PyMuPDF
import matplotlib.gridspec as gridspec
import matplotlib.ticker as ticker
import matplotlib.pyplot as plt
from matplotlib import rcParams
from pathlib import Path

rcParams['pdf.fonttype'] = 42  # Use TrueType fonts
rcParams['ps.fonttype'] = 42   # For saving as EPS (if needed)
env_name_mapping = {
    "3DOFFLIPPER": "3DOFFLIPPER",
    "5DOFUR3": "5DOFUR3",
    "6DOFUR3": "6DOFUR3", 
    "7DOFIIWA": "7DOFIIWA",
    "7DOF4SHELVES": "7DOF4SHELVES",
    "7DOFBINS": "7DOFBINS",
    "14DOFIIWAS": "14DOFIIWAS",
    "15DOFALLEGRO": "15DOFALLEGRO"
}
paper_names = {}
paper_names["3DOFFLIPPER"] = "Flipper"
paper_names["5DOFUR3"] = "UR3"
paper_names["6DOFUR3"] = "UR3Wrist"
paper_names["7DOFIIWA"] = "IIWAShelf"
paper_names["7DOF4SHELVES"] = "4Shelves"
paper_names["7DOFBINS"] = "IIWABins"
paper_names["14DOFIIWAS"] = "2IIWAs"
paper_names["15DOFALLEGRO"] = "Allegro"

def create_bar_plots(data, env_names, algs, settings_name):
    """Create the bar plots using the processed data"""
    
    # Map new data format to old format expected by plotting code
    mapped_data = {}
    
    # Create mapping for statistics
    stat_mapping = {
        'times': 'time',
        'volumes': 'rel_volume',  # Use relative volume instead of absolute
        'fraction_in_collision': 'frac_collision',
        'num_faces': 'num_faces'
    }
    
    for env_name in env_names:
        if env_name in env_name_mapping:
            old_env_name = env_name_mapping[env_name]
            if old_env_name in seed_nums:  # Only include environments we have seeds for
                mapped_data[env_name] = {}
                
                for alg in algs:
                    if alg in data[env_name] and 'mean_stats' in data[env_name][alg]:
                        mapped_data[env_name][alg] = {
                            'mean_stats': {},
                            'err': {}
                        }
                        
                        for old_key, new_key in stat_mapping.items():
                            if new_key in data[env_name][alg]['mean_stats']:
                                mapped_data[env_name][alg]['mean_stats'][old_key] = data[env_name][alg]['mean_stats'][new_key]
                                # Use std as error bars
                                if new_key in data[env_name][alg]['std_stats']:
                                    std_dev = data[env_name][alg]['std_stats'][new_key]
                                    mapped_data[env_name][alg]['err'][old_key] = [np.array([std_dev]), np.array([std_dev])]
                                else:
                                    mapped_data[env_name][alg]['err'][old_key] = [np.array([0]), np.array([0])]
    
    keys_stats = ['times', 'num_faces']
    axis_labels = {}
    axis_labels["times"] = 'Time [s]'
    axis_labels["volumes"] = 'Relative Volume'
    axis_labels["fraction_in_collision"] = 'Frac Region in Collision'
    axis_labels["num_faces"] = "Number of hyperplanes"
    
    stats_to_plot = keys_stats
    do_legend = False
    bar_width = 10
    colors = ['black', 'red', 'blue', 'green']
    
    # Filter environments that have data
    envs_with_data = [env for env in mapped_data.keys()]
    
    if not envs_with_data:
        print("No environments with data found!")
        return
    
    # Skip the first environment (like in original code with env_names[1:])
    if len(envs_with_data) > 1:
        envs_with_data = envs_with_data
    
    fig = plt.figure(figsize=(11, 7 * len(stats_to_plot)/4))
    outer_grid = gridspec.GridSpec(len(stats_to_plot), 1, wspace=-0.04, hspace=0.53)

    for statid, k in enumerate(stats_to_plot):
        if len(envs_with_data) == 0:
            continue
            
        inner_grid = gridspec.GridSpecFromSubplotSpec(1, len(envs_with_data), 
                                                    subplot_spec=outer_grid[statid], 
                                                    wspace=0.5, hspace=0.25)
        
        for i_env, env_name in enumerate(envs_with_data):
            ax = plt.Subplot(fig, inner_grid[i_env])
            
            for i_exp, alg in enumerate(algs):
                if alg in mapped_data[env_name] and 'mean_stats' in mapped_data[env_name][alg]:
                    mean_stats = mapped_data[env_name][alg]['mean_stats'][k]
                    err = mapped_data[env_name][alg]['err'][k]
                    
                    ax.bar(i_exp * 1.25 * bar_width, mean_stats, width=bar_width, 
                          yerr=err, capsize=5, label=alg, alpha=0.8, edgecolor='none',
                          )
            
            ax.set_yscale('log')
            ax.set_xlabel(paper_names.get(env_name, env_name), fontsize=11, labelpad=0.5)
            ax.tick_params(axis='y', which='both', labelrotation=50, labelsize=11, pad=0)
            ax.grid(True, color='gray', linestyle='-', linewidth=0.5, alpha=0.5, zorder=-10, axis="y")
            ax.grid(True, which='minor', color='gray', linestyle='-', linewidth=0.5, alpha=0.3, zorder=0, axis="y")
            
            # Handle tick formatting like in original code
            all_minor_ticks = ax.get_yticks(minor=True)
            all_major_ticks = ax.get_yticks(minor=False)
            ylim = ax.get_ylim()
            ax_tol = 0
            
            minor_ticks = [tick for tick in all_minor_ticks if ylim[0] + ax_tol <= tick <= ylim[1] - ax_tol]
            major_ticks = [tick for tick in all_major_ticks if ylim[0] + ax_tol <= tick <= ylim[1] - ax_tol]
            
            fig.add_subplot(ax)
        
        # Add outer axis for the statistic title
        ax_outer = fig.add_subplot(outer_grid[statid])
        ax_outer.set_title(f"{settings_name} settings: {axis_labels[k]}", 
                          pad=5, fontweight='bold', fontsize=12)
        ax_outer.axis('off')

    plt.tight_layout()
    
    if do_legend:
        # Create algorithm name mapping
        alg_display_names = {
            'iris_np': 'NP',
            'iris_zo': 'ZO',
            'iris_np2_greedy': 'Greedy', 
            'iris_np2_ray': 'Ray'
        }
        
        handles = [plt.Rectangle((0,0),1,1, color=colors[i], alpha=0.6, edgecolor='none') 
                  for i in range(len(algs))]
        labels = [alg_display_names.get(alg, alg) for alg in algs]
        fig.legend(handles, labels, loc='upper center', bbox_to_anchor=(0.5, 1.03), 
                  fontsize='large', title='Algorithms')
    
    filename = f"{settings_name}_new_benchmarks"
    plt.savefig("new_experiments/run1/"+filename + '.pdf', bbox_inches='tight', pad_inches=0)
    
    # Crop the PDF like in original code
    try:
        pdf_document = fitz.open(filename + '.pdf')
        page = pdf_document[0]
        left_crop = 60
        rect = page.rect
        crop_rect = fitz.Rect(left_crop, rect.y0, rect.x1, rect.y1)
        page.set_cropbox(crop_rect)
        cropped_pdf_path = filename + '_cropped.pdf'
        pdf_document.save(cropped_pdf_path)
        pdf_document.close()
        print(f"Cropped plots saved as {cropped_pdf_path}")
    except:
        print("Could not crop PDF (PyMuPDF not available)")
    
    plt.show()
    
    return filename

def main():
    # Configuration
    experiment_name = 'run1'
    settings = 'fast'
    algs = ['iris_np', 'iris_zo','iris_np2_greedy', 'iris_np2_ray']
    
    # You'll need to define your env_names list here
    # For example:
    from iris_environments.environments import env_names
    env_names = env_names[1:]
    
    print("Loading experimental results...")
    data = load_results_for_seed(experiment_name, settings, algs, env_names)
    
    print("Calculating relative volumes...")
    data = calculate_relative_volumes(data, env_names, algs)
    
    print("Creating bar plots...")
    filename = create_bar_plots(data, env_names, algs, settings.title())
    
    print(f"Plots saved as {filename}.pdf")
    

if __name__ == "__main__":
    main()