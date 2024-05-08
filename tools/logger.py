"""Logging utilities"""

class Logger:
    """Simple csv logger"""

    def __init__(self, filename, append=False) -> None:
        self.filename = filename
        self.append = append
        self.file = None

    def open(self):
        """Opens file ready to append"""
        self.file = open(self.filename, 'a' if self.append else 'w', encoding='utf-8')

    def push(self, data):
        """Push data to next line"""
        self.file.write(','.join([str(item) for item in data])+"\n")

    def close(self):
        """Stop logger and close file"""
        self.file.close()
