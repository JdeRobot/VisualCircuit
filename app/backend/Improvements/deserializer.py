
# Example String:
#str = "~1234567~<wire_1&wire_2<>wire_2&wire_3>$0.3&0.5$"

# pid seperator = ~
# input wires seperator = <
# output wires seperator = >
# parameters seperator = $

def find_between(s, start, end):
  return (s.split(start))[1].split(end)[0]

def deserialize(str):

	pid = find_between(str, "~", "~")

	input_wires = find_between(str, "<", "<")
	input_wires = [x.strip() for x in input_wires.split('&')]

	output_wires = find_between(str, ">", ">")
	output_wires = [x.strip() for x in output_wires.split('&')]

	parameters = find_between(str, "$", "$")
	parameters = [x.strip() for x in parameters.split('&')]

	return pid, input_wires, output_wires, parameters

