# SwarmSwIM/utility/cli.py
import argparse
from pathlib import Path
import shutil
import importlib.resources as resources


def copy_templates(destination: str):
    destination = Path(destination)
    destination.mkdir(parents=True, exist_ok=True)

    files = ["simulation.xml", "default.xml"]

    # Use resources.files() to access package data
    package = "SwarmSwIM"
    for file in files:
        src_path = resources.files(package) / file
        shutil.copy(src_path, destination / file)

    print(f"Initiated SwarmSwIM env templates in {destination}")


def main():
    parser = argparse.ArgumentParser(prog="SwarmSwIM")
    subparsers = parser.add_subparsers(dest="command", required=True)

    # create_new command
    create_parser = subparsers.add_parser("create_new", help="Create a new project")
    create_parser.add_argument(
        "destination",
        nargs="?",
        default=".",
        help="Destination folder (default: current directory)"
    )

    args = parser.parse_args()

    if args.command == "create_new":
        copy_templates(args.destination)

if __name__ == "__main__":
    main()