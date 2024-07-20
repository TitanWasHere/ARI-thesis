#! /usr/bin/env python
import subprocess

def call_python3_script():
    try:
        # Call the Python 3.5 script using subprocess
        result = subprocess.check_output(['python3', 'sub_process.py'])
        # Decode the result from bytes to string
        return result.decode('utf-8').strip()
    except subprocess.CalledProcessError as e:
        return "An error occurred: {}".format(e)

if __name__ == "__main__":
    output = call_python3_script()
    print("Output from Python 3.5 script:", output)
