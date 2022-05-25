from lib.exceptions import InvalidParameterNameException


class Parameters:
    def __init__(self, parameter_data) -> None:
        self.parameters = parameter_data

    def read_number(self, name):
        if self.parameters.get(name) is None:
            raise InvalidParameterNameException(f"{name} is not declared in parameters")
        
        return float(self.parameters[name])

    def read_string(self, name):
        if self.parameters.get(name) is None:
            raise InvalidParameterNameException(f"{name} is not declared in parameters")
        
        return str(self.parameters[name])

    def read_bool(self, name):
        if self.parameters.get(name) is None:
            raise InvalidParameterNameException(f"{name} is not declared in parameters")

        return bool(self.parameters[name])