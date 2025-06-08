import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
from mpl_toolkits.mplot3d import Axes3D

from utils import POINTS_CLOUD_FILENAME

TOLERANCE = 0.05

def approx(value, test):
    return test - TOLERANCE < value < test + TOLERANCE

def show_points_cloud(postfix_name = '', window_title = None):
    # data = []
    X = []
    Y = []
    Z = []
    onlyY = []
    onlyZ = []
    with open(POINTS_CLOUD_FILENAME, 'r', encoding="utf-8") as f:
        count = int(f.readline())
        for line in f:
            point_arr = list(map(float, line.split(' ')))
            # data.append(point_arr)
            X.append(point_arr[0])
            Y.append(point_arr[1])
            Z.append(point_arr[2])
            if approx(point_arr[0], 0):
                onlyY.append(point_arr[1])
                onlyZ.append(point_arr[2])
    X = np.array(X)
    Y = np.array(Y)
    Z = np.array(Z)
    onlyY = np.array(onlyY)
    onlyZ = np.array(onlyZ)

    # ----------------
    ## Matplotlib Sample Code using 2D arrays via meshgrid
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    img = ax.scatter(X, Y, Z, c=Z, cmap=plt.autumn())
    fig.colorbar(img)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    fig.savefig(f'plots_out/3D_zone{postfix_name}.png', bbox_inches='tight')

    #----------------

    fig = plt.figure()
    ax = fig.add_subplot()
    img = ax.scatter(onlyY, onlyZ, c=onlyZ, cmap=plt.autumn())
    fig.colorbar(img)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')

    fig.savefig(f'plots_out/2D_zone{postfix_name}.png', bbox_inches='tight')

    plt.title(window_title)
    plt.show()
    # surf = ax.plot_surface(X, Y, Z, rstride=1, cstride=1, cmap=cm.coolwarm,
    #                        linewidth=0, antialiased=False)

    # surf = ax.plot_trisurf(X, Y, Z, cmap=cm.jet, linewidth=0.2)
    # ax.set_zlim(-1.01, 1.01)
    #
    # ax.zaxis.set_major_locator(LinearLocator(10))
    # ax.zaxis.set_major_formatter(FormatStrFormatter('%.02f'))
    #
    # fig.colorbar(surf, shrink=0.5, aspect=5)
    # plt.title('Original Code')
    # plt.show()


if __name__ == '__main__':
    show_points_cloud()
