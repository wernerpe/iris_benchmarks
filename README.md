# iris_benchmarks

Setup
1. Install iris_environments
`cd iris_benchmarks && pip install -e .`
2. Set up file structure for new experiment to compare against. Edit and run the script `misc/make_new_custom_experiment.py `

3. Populate the generated files in `logs/my_experiment`. (The `algorithm.py` and `config_1/parameter/parameters.yml` files.)

4. Run `python run_iris_benchmarks_custom.py`. Make sure to set the correct experiment and configuration.

An example is included for fast iris