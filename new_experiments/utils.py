import importlib

def import_function_with_spec(module_name, function_name, file_path):
    """
    Imports a specific function from a module using util.spec_from_file_location.

    Args:
        module_name: The name of the module containing the function.
        function_name: The name of the function to import.
        file_path: The absolute path to the module file.

    Returns:
        The imported function, or None if not found.
    """

    spec = importlib.util.spec_from_file_location(
        module_name, file_path, loader=None, submodule_search_locations=None
    )

    if spec is None:
        print(f"Module '{module_name}' not found at path '{file_path}'.")
        return None

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    try:
        return getattr(module, function_name)
    except AttributeError:
        print(f"Function '{function_name}' not found in module '{module_name}'.")
        return None