#!/usr/bin/env python3
# -*- mode: python -*-

import os
import argparse
import sys
from io import StringIO
import generate_base


def parse_arguments(arguments):
    """
    Parse command line arguments.
    """
    parser = argparse.ArgumentParser(
        description="Generate description of  message definitions for C/Cpp programming languages."
    )

    parser.add_argument(
        "idf_names",
        nargs="*",
        help="IDF file(s) to generate message definitions from",
    )

    # Message definition search path (directories)
    parser.add_argument(
        "--idf-dir",
        "-I",
        dest="md_dir",
        metavar="DIR",
        action="append",
        default=[],
        help="Add additional message-definition base directory to search",
    )

    # All message definitions found below specified folder
    parser.add_argument(
        "--recursive",
        "-R",
        action="append",
        default=[],
        metavar="DIR",
        help="Add DIR to search path and assume that each recursively found file below that directory is a message definition to generate",
    )
    parser.add_argument(
        "--cpp_out",
        "-O",
        default=".",
        metavar="OUT_DIR",
        dest="output_dir",
        help="Generate C++ header",
    )
    parser.add_argument(
        "--force",
        "-F",
        action="store_true",
        help="Force writing to output file, even if no change is detected",
    )

    return parser.parse_args(arguments)


def add_recursive_files(directory, base, recursive_files):
    """
    Recursively add files from the specified directory to the list.
    """
    for entry in os.listdir(directory):
        if entry[0] == ".":
            continue
        path = os.path.join(directory, entry)
        if os.path.isdir(path):
            add_recursive_files(path, base, recursive_files)
        elif os.path.isfile(path):
            recursive_files.append(path[len(base) + 1 :])


def generate_message_definitions(args, message_definitions):
    """
    Generate message definitions based on the provided arguments and message names.
    """
    additional_md_dirs = args.md_dir
    recheck_md_dirs = bool(additional_md_dirs)

    recursive_files = []
    for directory in args.recursive:
        add_recursive_files(
            directory.rstrip(os.sep), directory.rstrip(os.sep), recursive_files
        )

    message_definitions += recursive_files
    generate_base.logger.info(f"Message definitions: {message_definitions}")
    if recheck_md_dirs:
        generate_base.additional_message_definition_dirs = additional_md_dirs
        generate_base.rescan_message_definition_dirs()

    service_base_map = {}
    message_definition_header = generate_base.MessageDefinitionHeader(
        service_base_map=service_base_map
    )

    for md in message_definitions:
        message_definition_header.add_message_definition(md)
        file_p = StringIO()
        output_file_name = os.path.join(args.output_dir, md + ".sl.h")
        message_definition_header.generate(file_p, output_file_name)
        new_content = file_p.getvalue()

        output_file_name = os.path.realpath(output_file_name)
        if not args.force and os.path.isfile(output_file_name):
            with generate_base.FileHandler(output_file_name, "rb") as file_p:
                data = file_p.read()
            if data == new_content:
                generate_base.logger.info(
                    f"{sys.argv[0]}: {output_file_name} is up to date."
                )
                continue  # Skip writing operation if file content is the same
        os.makedirs(os.path.dirname(output_file_name), exist_ok=True)
        with generate_base.FileHandler(output_file_name, "wb") as file_p:
            file_p.write(new_content)


def main(arguments):
    args = parse_arguments(arguments)
    generate_message_definitions(args, args.idf_names)


if __name__ == "__main__":
    main(sys.argv[1:])
