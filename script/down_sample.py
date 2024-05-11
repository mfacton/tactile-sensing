"""Downsampler tool"""

from tools.loader import Loader
from tools.logger import Logger

loader = Loader("data/input.csv", down_sample=2)
logger = Logger("data/output.csv")

while loader.available():
    logger.push(loader.read())

logger.close()
