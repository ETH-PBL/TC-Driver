import argparse

def init_parser():
    parser = argparse.ArgumentParser(description="Runs the RL train for TC-Driver")
    parser.add_argument(
        "file_name",
        type=str,
        help="Full path of the config yaml file",
    )
    
    return parser
