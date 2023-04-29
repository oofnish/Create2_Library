class ErrHistory(object):
    def __init__(self, size):
        self.cursor = 0
        self.size = size
        self._data = []

    def add(self, err_val):
        if len(self._data) == self.size:
            self._data[self.cursor] = err_val
        else:
            self._data.append(err_val)
        self.cursor = (self.cursor + 1) % self.size

    def __len__(self):
        return len(self._data)

    def __iter__(self):
        self._i = 0
        return self

    def __next__(self):
        if self._i < len(self._data):
            item = self[self._i]
            self._i += 1
            return item
        else:
            raise StopIteration

    def __getitem__(self, idx):
        if len(self._data) == 0:
            return 0
        if len(self._data) == self.size:
            return self._data[(idx + self.cursor) % self.size]
        else:
            if idx <= 0:
                return self._data[idx]
            else:
                return self._data[idx % len(self._data)]


class PID_Control:
    def __init__(self, kp, ki, kd, dt):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.past = ErrHistory(10)

    def PID(self, error, dt):
        p = self.kp * error

        if len(self.past) > 1:
            # bad index ########################################################################
            d = self.kd * (error - self.past[-1]) / dt
        else:
            d = self.kd * (error - self.past[len(self.past) - 1]) / dt

        # Add the latest entry into the history list before calculating the integral sum over the whole list
        self.past.add(error)
        i = self.ki * sum([e*dt for e in self.past])

        # p = self.kp * error
        # if len(self.past) > 0:
        #     i = self.ki * (sum(self.past) / len(self.past))
        # else:
        #     i = 0
        # if len(self.past) > 1:
        #     d = self.kd * (error - self.past[-1])
        # else:
        #     d = 0
        #self.past.add(error)

        return p + i + d



