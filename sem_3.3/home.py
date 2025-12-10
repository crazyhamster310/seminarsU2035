import logging
import sys

logger = logging.Logger(...)  # YOUR CODE HERE
# ...


def spam_logs():
    for i in range(5):
        logger.warning(f"warning №{i + 1}")
        logger.error(f"error №{i * 2}")
        logger.critical(f"critical №{i % 2}")
        logger.info(f"info №{-i}")
        logger.debug(f"debug №{19872436 + i * 1236234}")


spam_logs()
