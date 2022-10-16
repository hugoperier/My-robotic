import json
import os

MYROBOTICS_ROOT = "MYROBOTICS_ROOT"

def load_configuration(path, with_prefix=True):
	prefix = ""
	if with_prefix:
		prefix = os.environ.get(MYROBOTICS_ROOT, "")
		if (prefix == ""):
			print("MYROBOTICS_ROOT environment variable not set")
	file_path = os.path.join(prefix, path)
	with open(file_path) as json_file:
		return json.load(json_file)