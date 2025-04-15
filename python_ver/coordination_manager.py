import system_compiler as sc

import os
import argparse as arg

def get_parser():
    parser = arg.ArgumentParser(description="Coordination Manager CLI")
    parser.add_argument(
        "--halton-file",
        type=str,
        help="Path to the Halton file",
        default="halton_points.csv"
    )
    return parser

def main(args, **kwargs):

    coordination_manager_dir = os.path.dirname(os.path.abspath(__file__))
    print(f"Repository directory: {coordination_manager_dir}")
    env_dir = os.path.join(coordination_manager_dir, "env")
    print(f"Environment directory: {env_dir}")
    
    print("Starting Coordination Manager...")

    halton_file = os.path.join(env_dir,args.halton_file)

    compiler = sc.SystemCompiler(halton_file)

if __name__ == "__main__":
    parser = get_parser()
    args = parser.parse_args()

    main(args=args)