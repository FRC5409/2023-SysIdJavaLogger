import json

with open('data.txt', 'r') as f:
    data = f.read()

data_list = list(map(float, data.split(', ')))

counter = 0
new_data = []
temp_list = []

for i in range(0, len(data_list)):
    if counter >= 9:
        counter = 0
        new_data.append(temp_list)
        temp_list = []

    if counter < 9:
        counter += 1
        temp_list.append(data_list[i])


with open('data.json', 'w') as f:
    json.dump(new_data, f)
