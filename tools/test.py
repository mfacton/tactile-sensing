from loader import Loader
from logger import Logger

# loader = Loader("dummy.csv", down_sample=5)
logger = Logger("dummy.csv", down_sample=2)

for x in range(100):
    if logger.push([x]):
        print(x)

# while loader.available():
#     print(loader.read_line())