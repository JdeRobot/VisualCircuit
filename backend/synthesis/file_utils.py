import zipfile
from io import BytesIO

# Thanks to stack overflow :) 
# https://stackoverflow.com/questions/2463770/python-in-memory-zip-library
class InMemoryZip(object):
    def __init__(self):
        # Create the in-memory file-like object
        self.zip = BytesIO()

    def append(self, filename_in_zip : str, file_contents: str):
        '''Appends a file with name filename_in_zip and contents of 
        file_contents to the in-memory zip.'''
        # Get a handle to the in-memory zip in append mode
        zf = zipfile.ZipFile(self.zip, "a", zipfile.ZIP_DEFLATED, False)

        # Write the file to the in-memory zip
        zf.writestr(filename_in_zip, file_contents)

        # Mark the files as having been created on Windows so that
        # Unix permissions are not inferred as 0000
        for zfile in zf.filelist:
            zfile.create_system = 0

    def get_zip(self) -> BytesIO:
        self.zip.seek(0)
        return self.zip