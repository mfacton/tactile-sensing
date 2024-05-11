"""Downsampler tool"""

from tools.loader import Loader
from tools.logger import Logger

loader = Loader("dummy.csv", down_sample=2)
logger = Logger("dummy_sampled.csv")

while loader.available():
    logger.push(loader.read())

logger.close()
