import json

def load_configuration(path):
	with open(path) as json_file:
		return json.load(json_file)