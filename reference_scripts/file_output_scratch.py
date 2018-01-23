import csv
data_dir = "experiment_data"
test_file = "test_file2"

data = [
    ['LLeg', 0.17999999225139618, [0.03999999910593033, 0.10000000149011612, 0.0010000000474974513]],
    ['RLeg', 0.41999998688697815, [0.03999999910593033, -0.10000000149011612, 0.0020000000949949026]],
    ['LLeg', 0.41999998688697815, [0.03999999910593033, 0.10000000149011612, 0.003000000026077032]]
]


with open(data_dir+"/"+test_file+".csv", 'w+') as csvFile:
    writer = csv.writer(csvFile, delimiter=',')
    writer.writerow(['Leg Name', 'Time Left', 'X', 'Y', 'Theta'])
    for thing in data:
        #writer.writerows([thing])
        writer.writerow([thing[0], thing[1], thing[2][0], thing[2][1], thing[2][2]])
        # print thing[0]
        # print thing[1]
        # print thing[2]
        #writer.writerow([thing[0], thing[1], thing[2]])
    #writer.writerows(data)

