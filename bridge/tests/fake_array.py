class FakeArray:
    """A small stand-in for a numpy array: it has `len()`, `shape` and rows, and, like numpy's,
    its truth value raises."""

    def __init__(self, rows, columns):
        self.shape = (rows, columns)

    def __len__(self):
        return self.shape[0]

    def __iter__(self):
        return ([float(i)] * self.shape[1] for i in range(self.shape[0]))

    def __bool__(self):
        raise ValueError("The truth value of an array with more than one element is ambiguous")
