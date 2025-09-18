import pickle
import numpy as np
from pathlib import Path

def load_results(experiment_name, settings, algs, env_names):
    """Load all experimental results from pickle files"""
    data = {}
    
    for env_name in env_names:
        data[env_name] = {}
        for alg in algs:
            pickle_path = f"new_experiments/{experiment_name}/{settings}/{alg}/{env_name}.pkl"
            try:
                with open(pickle_path, 'rb') as f:
                    results = pickle.load(f)
                    
                # Calculate statistics
                times = np.array(results['times'])
                volumes = np.array(results['volumes'])
                num_faces = np.array([len(region.A()) for region in results['regions']])
                frac_collision = np.array(results['fraction_in_collision'])
                
                data[env_name][alg] = {
                    'mean_stats': {
                        'time': np.mean(times),
                        'volume': np.mean(volumes),
                        'num_faces': np.mean(num_faces),
                        'frac_collision': np.mean(frac_collision)
                    },
                    'all_times': times
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
                    current_volume = data[env_name][alg]['mean_stats']['volume']
                    data[env_name][alg]['mean_stats']['rel_volume'] = current_volume / baseline_volume
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

def generate_latex_table(data, env_names, algs, settings_name):
    """Generate the complete LaTeX table"""
    
    # Environment name mapping (adjust as needed)
    paper_names = {
        '3DOFFLIPPER': 'Flipper',
        '5DOFUR3': 'UR3', 
        '6DOFUR3': 'UR3Wrist',
        '7DOFIIWA': 'IIWAShelf',
        '7DOF4SHELVES': '4Shelves',
        '7DOFBINS': 'IIWABins',
        '14DOFIIWAS': '2IIWAs',
        '15DOFALLEGRO': 'Allegro'
    }
    
    # Algorithm mapping for column headers
    alg_names = {
        'iris_np': 'NP',
        'iris_zo': 'ZO', 
        'iris_np2_greedy': 'Greedy',
        'iris_np2_ray': 'Ray'
    }
    
    stat_names = ['time', 'num_faces', 'rel_volume', 'frac_collision']
    stat_headers = ['Time [s]', 'Num Hyperplanes', 'Rel Volume', 'Frac in collision']
    
    # Find best values for each statistic
    best_values = {}
    for stat in stat_names:
        best_values[stat] = find_best_values(data, env_names, algs, stat)
    
    # Generate table header
    latex_output = []
    latex_output.append("\\begin{table*}")
    latex_output.append("\\centering")
    latex_output.append("\\begin{minipage}{1\\linewidth}")
    latex_output.append("        \\resizebox{\\linewidth}{!}{")
    latex_output.append("\\begin{tabular}{c l cccc | cccc | cccc | cccc}")
    latex_output.append("\\hline")
    
    # Multi-column headers
    header1 = "\\multirow{2}{*}{}&\\multirow{2}{*}{Environment}"
    for header in stat_headers:
        header1 += f" & \\multicolumn{{4}}{{c}}{{{header}}}"
    header1 += " \\\\"
    latex_output.append(header1)
    
    # Algorithm headers
    header2 = " &&"
    for _ in stat_headers:
        for alg in algs:
            header2 += f"{alg_names.get(alg, alg)}"
            if alg != algs[-1] or _ != stat_headers[-1]:
                header2 += " & "
    header2 += " \\\\"
    latex_output.append(header2)
    
    latex_output.append("\\hline")
    
    # Settings section header
    latex_output.append(f"\\multirow{{{len(env_names)}}}{{*}}{{\\rotatebox[]{{90}}{{\\textbf{{{settings_name.title()}}}}}\\;}} ")
    
    # Data rows
    for i, env_name in enumerate(env_names):
        if env_name in paper_names:
            row = f"&\\texttt{{{paper_names[env_name]}}} &"
        else:
            row = f"&\\texttt{{{env_name}}} &"
        
        # Add data for each statistic and algorithm
        for stat_idx, stat in enumerate(stat_names):
            for alg_idx, alg in enumerate(algs):
                if alg in data[env_name] and 'mean_stats' in data[env_name][alg]:
                    value = data[env_name][alg]['mean_stats'][stat]
                    is_best = (value == best_values[stat][env_name]) if env_name in best_values[stat] else False
                    if stat == "frac_collision":
                        is_best = False
                    if stat in ['frac_collision', 'time','rel_volume']:
                        formatted_value = format_value_with_bold(value, is_best, stat, True)
                    else:
                        formatted_value = format_value_with_bold(value, is_best, stat, False)
                        
                else:
                    formatted_value = "N/A"
                
                row += formatted_value
                
                # Add separator or line ending
                if stat_idx == len(stat_names) - 1 and alg_idx == len(algs) - 1:
                    row += "\\\\"
                else:
                    row += " & "
        
        latex_output.append(row)
    
    # Table footer
    latex_output.append("\\hline")
    latex_output.append("\\end{tabular}")
    latex_output.append("}")
    latex_output.append("\\end{minipage}")
    latex_output.append("\\vspace{0.2cm}")
    latex_output.append("\\caption{New experiments}")
    latex_output.append("\\label{tab:results}")
    latex_output.append("\\end{table*}")
    
    return "\n".join(latex_output)

def main():
    # Configuration
    experiment_name = 'run1'
    settings = 'precise'
    algs = ['iris_np', 'iris_zo','iris_np2_greedy', 'iris_np2_ray']
    
    # You'll need to define your env_names list here
    # For example:
    from iris_environments.environments import env_names
    env_names = env_names[1:]
    
    print("Loading experimental results...")
    data = load_results(experiment_name, settings, algs, env_names)
    
    print("Calculating relative volumes...")
    data = calculate_relative_volumes(data, env_names, algs)
    
    print("Generating LaTeX table...")
    latex_table = generate_latex_table(data, env_names, algs, settings)
    
    print("\n" + "="*80)
    print("LATEX TABLE OUTPUT:")
    print("="*80)
    print(latex_table)
    print("="*80)

    time_zo = np.array([data[en]['iris_zo']['mean_stats']['time'] for en in env_names])
    time_np = np.array([data[en]['iris_np']['mean_stats']['time'] for en in env_names])
    time_greedy = np.array([data[en]['iris_np2_greedy']['mean_stats']['time'] for en in env_names])
    time_ray = np.array([data[en]['iris_np2_ray']['mean_stats']['time'] for en in env_names])
    
    hyp_zo = np.array([data[en]['iris_zo']['mean_stats']['num_faces'] for en in env_names])
    hyp_np = np.array([data[en]['iris_np']['mean_stats']['num_faces'] for en in env_names])
    hyp_greedy = np.array([data[en]['iris_np2_greedy']['mean_stats']['num_faces'] for en in env_names])
    hyp_ray = np.array([data[en]['iris_np2_ray']['mean_stats']['num_faces'] for en in env_names])
    
    print(f" zo speedup {np.mean(time_np/time_zo)} greedy speedup {np.mean(time_np/time_greedy)} ray speedup {np.mean(time_np/time_ray)}")
    print(f" zo face_red {np.mean(hyp_np/hyp_zo)} greedy face_red {np.mean(hyp_np/hyp_greedy)} ray face_red {np.mean(hyp_np/hyp_ray)}")

    # Optionally save to file
    output_file = f"new_experiments/{experiment_name}/{settings}_table.tex"
    with open(output_file, 'w') as f:
        f.write(latex_table)
    print(f"Table saved to {output_file}")

if __name__ == "__main__":
    main()