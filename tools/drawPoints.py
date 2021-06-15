import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.image as mpimg
import numpy as np
import sys

def main():
    while True:
        mode = input('Mode: ')
        if mode == '0':
            image = mpimg.imread('test0.png')
            fig, ax = plt.subplots()
            ax.imshow(image)

            rect = patches.Rectangle((600, 570), 250, 250, linewidth=1, edgecolor='r', facecolor='none')
            ax.add_patch(rect)

            while True:
                in_str = input('Points: ')
                in_list  = in_str.split(',')
                if in_str == 'x': break

                c = [float(i) for i in in_list]
                plt.scatter(c[0]+600, c[1]+570, color='red', s=4)
                

        if (mode == '1'):
            if (len(sys.argv) > 2):
                raise Exception("len(sys.argv) > 2")
            
            image = mpimg.imread(sys.argv[1] + ".png")

            fig, ax = plt.subplots()
            ax.imshow(image)
            plt.scatter(711, 455, color='cyan', s=6)
            rect = patches.Rectangle((320, 240), 640, 500, linewidth=1, edgecolor='r', facecolor='none')
            ax.add_patch(rect)

            while True:
                in_str = input('Points: ')
                in_list  = in_str.split(', ')

                if len(in_list) > 2:
                    cArr = np.array([int(i) for i in in_list])
                    cArr2D = np.reshape(cArr, (4, 2))
                    plt.plot(cArr2D[:, 0], cArr2D[:, 1], 'ro-', markersize=3)
                    plt.plot([cArr2D[0, 0], cArr2D[-1, 0]], [cArr2D[0, 1], cArr2D[-1, 1]] ,'ro-', markersize=3)

                elif len(in_list) == 2:
                    pArr = [float(i) for i in in_list]
                    plt.scatter(pArr[0], pArr[1], color='red', s=6)

                elif in_str == 'x': break

        if mode == 'x': break

    plt.show()


if __name__ == '__main__':
    main()
