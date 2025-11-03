import sys
import os
import logging
from generate_base.message_definitions import (
    FileHandler,
    find_message_definition,
    MessageDefinitionHeader,
    MessageDefinition,
)


# init log
def init_logging():
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    )
    return logging.getLogger("sl")


def initialize_globals():
    global additional_message_definition_dirs
    additional_message_definition_dirs = []
    global message_definition_dirs
    message_definition_dirs = []
    global logger
    logger = init_logging()


def rescan_message_definition_dirs():
    global message_definition_dirs, additional_message_definition_dirs
    message_definition_dirs = list(
        filter(os.path.isdir, additional_message_definition_dirs)
    )


initialize_globals()
