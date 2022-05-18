# Minimal input validation since I'm the only one using this
import math
circumference = -1
mtoi = 39.3700787  # 39.37 inches in a meter


def get_circumference():
    # C = d / (ticks/360)
    global circumferene

    print("Make the robot go straight forward/sideways. Take the average reading if forward")

    ticks = int(input("Enter number of forward / sideways ticks: "))
    distance = float(input("Enter distance travelled (m): "))
    circumference = distance / (ticks/360)
    diameter = circumference / math.pi
    print(f"Diameter: {diameter * mtoi}in")


def get_track_width():
    # b = (l-r)/theta
    global circumference
    print("Turn the robot a given angle. Record the ticks on each side.")

    if circumference == -1:
        diameter = float(input("Enter diameter(in): ")) / mtoi
        circumference = diameter * math.pi

    left_ticks = int(input("Enter number of left ticks: "))
    right_ticks = int(input("Enter number of right ticks: "))
    left_d = circumference * (left_ticks/360)
    right_d = circumference * (right_ticks/360)

    theta_d = float(input("Enter number of degrees turned: "))
    theta = math.radians(theta_d)

    track_width = (left_d - right_d)/theta
    print(f"Track width: {track_width*mtoi}in")


def get_middle_distance():
    global circumference
    print("Turn the robot a given angle. Record the sideways ticks.")

    if circumference == -1:
        diameter = float(input("Enter diameter(in): ")) / mtoi
        circumference = diameter * math.pi

    ticks = int(input("Enter number of sideways ticks: "))
    theta_d = float(input("Enter number of degrees turned: "))
    theta = math.radians(theta_d)

    middle_distance = (circumference*ticks) / (theta*360)
    print(f"Middle Distance: {middle_distance*mtoi}in")


funcs = [get_circumference, get_track_width, get_middle_distance]
while True:
    print("1. Wheel Circumference")
    print("2. Track Width")
    print("3. Middle Distance")
    selection = int(input("Enter selection: "))

    funcs[selection-1]()  # select function from list
