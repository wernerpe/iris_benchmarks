import datetime

def run_default_settings(env_name):
    pass

def run_custom_experiment(env_name, iris_handle):
    pass

def build_regions():
    pass

def evaluate_regions():
    pass

def get_experiment_name(env_name, settings= 'default'):
    if settings != 'default':
        current_time = datetime.datetime.now()
        timestamp = current_time.strftime("%Y%m%d%H%M%S")
        name = f"logs/{timestamp}_{env_name}_{settings}.pkl"
        return name
    else:
        return f"benchmarks/default_experiments/{env_name}.pkl"