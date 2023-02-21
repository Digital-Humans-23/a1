import json

with open('my-info.json') as json_file:
    data = json.load(json_file)
    print("full name: " + data["full name"])
    print("student id: " + data["student id"])
    print("nethz username: " + data["nethz username"])
