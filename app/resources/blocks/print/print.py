class Print(Block):
    '''
    Implementations of Block class
    Prints in console the value of input
    '''
    def __init__(self, id, inputs):
        Block.__init__(self, id, inputs, None)

    def execute(self, pIn1):
        print str(pIn1)
