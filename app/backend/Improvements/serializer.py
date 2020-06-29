
# Example Input:
# serialize('1234567', [''], ['wire_2', 'wire_3'], ['0.3', '0.5'])

# pid seperator = ~
# input wires seperator = <
# output wires seperator = >
# parameters seperator = $


def serialize(pid, input_wires, output_wires, parameters):


	str = '~' + pid + '~' + '<'

	for i in input_wires:
		str += i + '&'

	str = str[:-1]
	str += '<'

	str += '>'
	for i in output_wires:
		str += i + '&'
	str = str[:-1]
	str += '>'

	str += '$'
	for i in parameters:
		str += i + '&'
	str = str[:-1]
	str += '$'
	
	return str

