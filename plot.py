import matplotlib.pyplot as plt

def plot_2d_data(filename):
    x = []
    y = []

    with open(filename) as f:
        content = f.readlines()

    for line in content:
        points = line.split(', ')
        points[1] = points[1][:-1]
        points = list(map(int, points))
        print(points)

        x.append(points[0])
        y.append(points[1])

    plt.figure()
    plt.plot(x, 'bo-', label='x')
    plt.plot(y, 'ro-', label='y')
    plt.title(filename.split('.')[0])
    plt.legend()
    plt.show()

def plot_3d_data(filename, integrate):
    x = []
    y = []
    z = []
    Sum = [0]

    with open(filename) as f:
        content = f.readlines()

    for line in content:
        points = line.split(', ')
        points[2] = points[2][:-1]
        points = list(map(int, points))
        print(points)

        x.append(points[0])
        Sum.append(Sum[-1] + points[0])
        y.append(points[1])
        z.append(points[2])

    plt.figure()
    plt.plot(x, label='x')
    plt.plot(y, label='y')
    plt.plot(z, label='z')
    plt.title(filename.split('.')[0])
    plt.legend()

    if integrate == True:
        plt.figure()
        plt.plot(Sum, label='Sum')
        plt.title('Roll')
    plt.show()

plot_3d_data('gyro.dat', True)
plot_3d_data('accel.dat', False)
plot_2d_data('pos.dat')
