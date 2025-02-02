import json

def load_config(config_path="config.json"):
    """
    Load configuration from a JSON file and compute additional parameters.
    
    Parameters:
    - config_path: Path to the JSON configuration file (default: "config.json").
    
    Returns:
    - config: Dictionary containing configuration parameters.
    """
    with open(config_path, "r") as file:
        config = json.load(file)
    
    # Compute STEP_NUM based on VERTEX and DX, DY values
    config["STEP_NUM"] = [
        int(abs(config["VERTEX"]["0,0"][0] - config["VERTEX"]["1,1"][0]) // config["DX"]),
        int(abs(config["VERTEX"]["0,0"][1] - config["VERTEX"]["1,1"][1]) // config["DY"]),
    ]
    
    return config