class Loader:
    def __init__(self, filename, down_sample=1) -> None:
        assert down_sample > 0
        self.down_sample = down_sample

        with open(filename, "r", encoding="utf-8") as file:
            self.lines = [
                line.strip()
                for line in file.readlines()[down_sample - 1 :: down_sample]
            ]
            self.length = len(self.lines)

        # line that is next to be read
        self.current_line = 0

    def available(self):
        """Returns if there is new line to be read"""
        return self.current_line < self.length

    def read(self):
        """Returns next line in file"""
        line = self.lines[self.current_line].split(",")
        self.current_line += 1
        return line
