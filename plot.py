import matplotlib.pyplot as plt

def plot_data(filename):
    x = []
    y = []
    z = []

    with open(filename) as f:
        content = f.readlines()

    for line in content:
        points = line.split(', ')
        points[2] = points[2][:-1]
        points = list(map(int, points))
        print(points)

        x.append(points[0])
        y.append(points[1])
        z.append(points[2])

    plt.figure()
    plt.plot(x, label='x')
    plt.plot(y, label='y')
    plt.plot(z, label='z')
    plt.title(filename.split('.')[0])
    plt.legend()
    plt.show()

plot_data('gyro.dat')
plot_data('accel.dat')
