class Constant(Block):
    '''
    Implementations of Block class
    Returns a constant value
    '''
    def __init__(self, id, outputs, value):
        Block.__init__(self, id, None, outputs)
        self.value = value

    def execute(self):
        return self.value
