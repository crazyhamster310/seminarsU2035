import logging
import sys


class SingleLevelFilter(logging.Filter):
    def __init__(self, level):
        self.level = level

    def filter(self, record):
        return record.levelno == self.level


def get_logger(name: str = "") -> logging.Logger:
    logger = logging.getLogger(name)
    logger.setLevel(logging.DEBUG)

    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(logging.INFO)
    console_formatter = logging.Formatter(
        "%(asctime)s :: %(levelname)s :: %(name)s :: %(message)s"
    )
    console_handler.setFormatter(console_formatter)
    logger.addHandler(console_handler)

    file_handler = logging.FileHandler("log.log", mode="w")
    file_handler.setLevel(logging.DEBUG)
    file_formatter = logging.Formatter("%(asctime)s :: %(message)s")
    file_handler.setFormatter(file_formatter)
    file_filter = SingleLevelFilter(logging.DEBUG)
    file_handler.addFilter(file_filter)
    logger.addHandler(file_handler)

    return logger