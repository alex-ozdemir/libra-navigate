class WallBuffer(object):
    def __init__(self, size, start):
        self.i = 0;
        self.data = [start for i in xrange(size)]
        self.size = size
        self.state = start
        self.falseCount = size * (1 - start)
        self.thresh = 0.70

    def submit(self, data_in):
        data_out = self.data[self.i]
        self.data[self.i] = data_in
        if data_in != data_out:
            self.falseCount += 1 if not data_in else -1

        if self.falseCount > self.thresh * self.size:
            self.state = False
        elif self.falseCount < (1 - self.thresh) * self.size:
            self.state = True

        self.i = (self.i + 1) % self.size
        return self.state
