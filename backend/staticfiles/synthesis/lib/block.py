class Block:

    def __init__(self, id, id_type):

        self.id = id
        self.id_type = id_type
        self.name = None
        self.input_ports = []
        self.port_map = []
        self.output_ports = []
        self.parameters = []

    def set_name(self, name_dict):

        for name in name_dict:
            if self.id_type == name[0]:
                self.name = name[1]
                break

    def connect_input_wire(self, wire_id, port):
        self.input_ports.append({'port':port, 'wire':wire_id})

    def connect_output_wire(self, wire_id, port):
        self.output_ports.append({'port':port, 'wire':wire_id})

    def add_parameter(self, parameter, port):
        self.parameters.append({'port':port, 'parameter':parameter})

    def sort_ports(self):

        sorter = sorted(self.input_ports, key=lambda k: k['port'])
        self.input_ports = []

        for element in sorter:
            self.input_ports.append(element['wire'])

        sorter = sorted(self.output_ports, key=lambda k: k['port'])
        self.output_ports = []

        for element in sorter:
            self.output_ports.append(element['wire'])

        sorter = sorted(self.parameters, key=lambda k: k['port'])
        self.parameters = []

        for element in sorter:
            self.parameters.append(element['parameter'])

