class SumBlock (Block):
    '''
    Implementations of Block class
    Performs sum of the 2 input parameters
    '''
    def __init__(self, id, inputs, outputs):
        Block.__init__(self, id, inputs, outputs)
    
    def execute(self, pIn1, pIn2):
        return pIn1 + pIn2