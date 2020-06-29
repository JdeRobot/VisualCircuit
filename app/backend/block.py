class Block:

    def __init__(self, id, id_type):

        self.id = id
        self.id_type = id_type
        self.name = None
        self.input_ports = []
        self.port_map = []
        self.output_ports = []
        self.parameters = None

    def set_name(self, name_dict):

        for name in name_dict:
            if self.id_type == name[0]:
                self.name = name[1]
                break

    def connect_input_wire(self, wire):
        if int(wire) not in self.input_ports:
            self.input_ports.append(int(wire))

    def connect_output_wire(self, wire):
        if int(wire) not in self.output_ports:
            self.output_ports.append(int(wire))

    def sort_ports(self):
        self.input_ports.sort()
        self.input_ports = [str(i) for i in self.input_ports]
        self.output_ports.sort()
        self.output_ports = [str(i) for i in self.output_ports]

    def set_parameters(self, parameters):
        self.parameters = parameters
