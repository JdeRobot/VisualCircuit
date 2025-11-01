#!/usr/bin/env python
import sys
def main():
	try:
		shelf = input("Enter which shelf to be moved (1-3): ")
	except EOFError:
		print("No interactive input received. Defaulting to shelf '1'.")
		shelf = "1"  # Provide a default value

	if(shelf=="1"):
		print("go to shelf 1")
	elif(shelf=="2"):
		print("go to shelf 2")
	elif(shelf=="3"):
		print("go to shelf 3")
	else:
		print("shelf doesn't exist")
	
if __name__ == '__main__':
	main()
