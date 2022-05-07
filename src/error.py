# libraries to visualize the error
import datetime as dt
import matplotlib.pyplot as plt

file = open("/home/alirezakarimi/Desktop/catkin_ws/src/ros_tutorial/src/error-data.txt", "r")
file_lines = file.read()
list_of_lines = file_lines.split("\n")

sum_errors = 0
for i in range(0, 6000):
    sum_errors += float(list_of_lines[i])
sum_errors = round(sum_errors, 2)

graph = plt.figure().add_subplot(111)
graph.plot([float(i) for i in list_of_lines])
plt.title("Error Over Callbacks")
plt.ylabel("error")
plt.xlabel('average of errors after 6000 callbacks: {}'.format(round(sum_errors/6000, 5)))
plt.show()