import json
import glob

def generateFile(data):
    components = data['design']['graph']['blocks']
    
    for block in components:
            if block['type'] == 'basic.code':
                # If code, generate python file.
                script = block['data']['code']
                script_name = data['package']['name'] 
                # The file name is stored in script_name here
                file = open("Blocks/"+script_name+".py", "w")
                file.write(script)
                file.close()

                # This part is to get all the svg image data in a text file
                # file = open("dictionary.txt", "a")
                # item = "\""+script_name +"\""+ ":" + "\""+ data['package']['image'] + "\"" + "," +'\n'
                # file.write(item)
                # file.close()


def main():
    for path in glob.glob('../frontend/src/components/blocks/collection/**/*.json'):
        f = open(path)
        data = json.load(f)
        generateFile(data)
    
    # Closing file
    f.close()

if __name__ == "__main__":
    main()