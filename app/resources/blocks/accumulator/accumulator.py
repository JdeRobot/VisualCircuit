class AcumulatorBlock (Block):
    '''
    Implementations of Block class
    Accumulates the value of input 
    '''
    def __init__(self, id, inputs, outputs, initialValue=0):
        Block.__init__(self, id, inputs, outputs)
        self.accumulated = initialValue
    
    def execute(self, pIn1):
        self.accumulated += pIn1
        return self.accumulated