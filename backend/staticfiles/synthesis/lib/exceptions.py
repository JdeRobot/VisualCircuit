class InvalidOutputNameException(Exception):
    """Raised when Output name has not been declared in ports"""


class InvalidInputNameException(Exception):
    """Raised when Input name has not been declared in ports"""

class InvalidParameterNameException(Exception):
    """Raised when Parameter name has not been declared in ports"""
