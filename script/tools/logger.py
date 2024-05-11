"""Logging utilities"""


class Logger:
    """Simple csv logger"""

    def __init__(self, filename, append=False, down_sample=1) -> None:
        assert down_sample > 0

        self.file = open(filename, "a" if append else "w", encoding="utf-8")
        self.skip_amount = down_sample
        self.skip_count = 0

    def push(self, data):
        """Push data to next line returns if the data was logged"""

        self.skip_count += 1
        self.skip_count %= self.skip_amount

        if self.skip_count == 0:
            self.file.write(",".join([str(item) for item in data]) + "\n")
            return True

        return False

    def close(self):
        """Stop logger and close file"""
        self.file.close()
