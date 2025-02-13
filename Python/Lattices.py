import itertools
import math

import numpy as np
from matplotlib import pyplot as plt
import matplotlib.patches as patches
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


class Zn:
    n: int
    epsilon: float
    delta: float
    rescale: float

    def __init__(self, n, epsilon, delta):
        self.n = n
        self.epsilon = epsilon
        self.delta = delta
        self.rescale = (2 * delta * self.epsilon) / math.sqrt(self.n * (1 + self.epsilon ** 2))

    def get_samples_in_ball(self, R: float)->list:
        samples = []
        max_i = math.ceil(R / self.rescale)
        int_vectors = list(itertools.product(range(-max_i, max_i + 1), repeat=self.n))
        for vector in int_vectors:
            vector_rescaled = np.array([val * self.rescale for val in vector])
            if np.linalg.norm(vector_rescaled) < R:
                samples.append(vector_rescaled)
        return samples

    def show_samples_in_ball(self, R:float):
        samples = self.get_samples_in_ball(R)
        if self.n == 2:
            X = [sample[0] for sample in samples]
            Y = [sample[1] for sample in samples]
            plt.scatter(X, Y)
            # plot several lines
            for i in range(-1, 1):
                for j in range(-1, 1):
                    v0 = np.array([val * self.rescale for val in [i, j]])
                    v1 = np.array([val * self.rescale for val in [i + 1, j]])
                    v2 = np.array([val * self.rescale for val in [i, j + 1]])
                    v12 = np.array([val * self.rescale for val in [i + 1, j + 1]])
                    X = [v0[0], v1[0], v12[0], v2[0], v0[0]]
                    Y = [v0[1], v1[1], v12[1], v2[1], v0[1]]
                    plt.plot(X, Y)
            circle = patches.Circle((0, 0), 1, fill=False, edgecolor='red', linewidth=2)
            plt.gca().add_patch(circle)
            plt.gca().set_aspect('equal', adjustable='box')
            # plt.title(f"Dim={self.n}, R={R}, delta={self.delta}, epsilon={self.epsilon}")
            # plt.xlabel("X Axis")
            # plt.ylabel("Y Axis")
            plt.show()
        elif self.n == 3:
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            # get points
            X = [sample[0] for sample in samples]
            Y = [sample[1] for sample in samples]
            Z = [sample[2] for sample in samples]
            # Create a 3D scatter plot
            ax.scatter(X, Y, Z, color='green', edgecolor='black', linewidth=1,)
            # plot several lines
            # plot several lines
            for (i, j, w) in [[0,0,0], [-1, 0, 0]]:
                v0 = np.array([val * self.rescale for val in [i, j, w]])
                v1 = np.array([val * self.rescale for val in [i + 1, j, w]])
                v2 = np.array([val * self.rescale for val in [i, j + 1, w]])
                v3 = np.array([val * self.rescale for val in [i, j, w + 1]])
                v12 = np.array([val * self.rescale for val in [i + 1, j + 1, w]])
                v13 = np.array([val * self.rescale for val in [i + 1, j, w + 1]])
                v23 = np.array([val * self.rescale for val in [i, j + 1, w + 1]])
                v123 = np.array([val * self.rescale for val in [i + 1, j + 1, w + 1]])
                X = [v0[0], v1[0], v12[0], v2[0], v0[0]]
                Y = [v0[1], v1[1], v12[1], v2[1], v0[1]]
                Z = [v0[2], v1[2], v12[2], v2[2], v0[2]]
                ax.plot(X, Y, Z, linewidth=1, color='blue')
                X = [v0[0], v1[0], v13[0], v3[0], v0[0]]
                Y = [v0[1], v1[1], v13[1], v3[1], v0[1]]
                Z = [v0[2], v1[2], v13[2], v3[2], v0[2]]
                ax.plot(X, Y, Z, linewidth=1, color='blue')
                X = [v0[0], v2[0], v23[0], v3[0], v0[0]]
                Y = [v0[1], v2[1], v23[1], v3[1], v0[1]]
                Z = [v0[2], v2[2], v23[2], v3[2], v0[2]]
                ax.plot(X, Y, Z, linewidth=1, color='blue')
                X = [v123[0], v12[0], v2[0], v23[0], v123[0]]
                Y = [v123[1], v12[1], v2[1], v23[1], v123[1]]
                Z = [v123[2], v12[2], v2[2], v23[2], v123[2]]
                ax.plot(X, Y, Z, linewidth=1, color='blue')
                X = [v123[0], v12[0], v1[0], v13[0], v123[0]]
                Y = [v123[1], v12[1], v1[1], v13[1], v123[1]]
                Z = [v123[2], v12[2], v1[2], v13[2], v123[2]]
                ax.plot(X, Y, Z, linewidth=1, color='blue')
            # Generate data for a sphere
            u = np.linspace(0, 2 * np.pi, 100)
            v = np.linspace(0, np.pi, 100)
            x = R * np.outer(np.cos(u), np.sin(v))
            y = R * np.outer(np.sin(u), np.sin(v))
            z = R * np.outer(np.ones(np.size(u)), np.cos(v))
            # Plot the surface
            ax.plot_surface(x, y, z, color='r', alpha=0.15)  # 'alpha' controls the transparency

            # Plot a circle to help "see" the sphere
            # Parameters for the circle
            radius = 0.6  # Radius of the circle
            theta = np.linspace(0, 2 * np.pi, 100)  # Angle
            # Parametric equations for a circle in the XY-plane (z = 0)
            x = radius * np.cos(theta)
            y = radius * np.sin(theta)
            z = np.zeros_like(x)  # The circle lies in the XY-plane
            # Plot the circle
            ax.plot(x, y, z, color='black', alpha=0.5)
            # plot the poles
            ax.scatter([0, 0], [0, 0], [0.6, -0.6], s=35, color='black', edgecolor='white', linewidth=1)

            # matplotlib settings
            ax.set_box_aspect([1, 1, 1])
            ax.set_xticks([])
            ax.set_yticks([])
            ax.set_zticks([])
            ax.w_xaxis.line.set_color((0.5, 0.5, 0.5, 0.0))  # X-axis line invisible
            ax.w_yaxis.line.set_color((1.0, 1.0, 1.0, 0.0))  # Y-axis line invisible
            ax.w_zaxis.line.set_color((1.0, 1.0, 1.0, 0.0))  # Z-axis line invisible
            ax.w_xaxis.set_pane_color((0.5, 0.5, 0.5, 1))  # Light blue, semi-transparent
            ax.w_yaxis.set_pane_color((0.6, 0.6, 0.6, 1))  # Light red, semi-transparent
            ax.w_zaxis.set_pane_color((0.55, 0.55, 0.55, 1))  # Light green, semi-transparent
            ax.set_xlim([-0.5, 0.5])  # X-axis zoom
            ax.set_ylim([-0.5, 0.5])  # Y-axis zoom
            ax.set_zlim([-0.5, 0.5])  # Z-axis zoom
            ax.set_facecolor('white')
            fig.set_facecolor('black')
            ax.view_init(elev=25, azim=-75)

            plt.savefig('ZN_3D.png', format='png')
            plt.show()

class DnStar:
    n: int
    epsilon: float
    delta: float
    rescale: float

    def __init__(self, n, epsilon, delta):
        self.n = n
        self.epsilon = epsilon
        self.delta = delta
        if n % 2 == 0:
            self.rescale = (4 * delta * self.epsilon) / math.sqrt((2*self.n) * (1 + self.epsilon ** 2))
        else:
            self.rescale = (4 * delta * self.epsilon) / math.sqrt((2*self.n - 1) * (1 + self.epsilon ** 2))
        # create generator
        self.generator = np.eye(n)
        last_row = 0.5 * np.ones(n,)
        self.generator[-1] = last_row

    def get_samples_in_ball(self, R: float)->list:
        samples = []
        max_i = 100 * math.ceil(R / self.rescale)
        int_vectors = list(itertools.product(range(-max_i, max_i + 1), repeat=self.n))
        for vector in int_vectors:
            generated_v = np.matmul(np.array(vector).reshape(1,-1), self.generator)
            vector_rescaled = np.array([val * self.rescale for val in generated_v])
            if np.linalg.norm(vector_rescaled) < R:
                samples.append(vector_rescaled.reshape(-1,))
        return samples

    def show_samples_in_ball(self, R:float):
        samples = self.get_samples_in_ball(R)
        if self.n == 2:
            X = [sample[0] for sample in samples]
            Y = [sample[1] for sample in samples]
            plt.scatter(X, Y)
            # plot several lines
            for i in range(-1, 1):
                for j in range(-1, 1):
                    v0 = np.array([val * self.rescale for val in [i, j]])
                    v1 = np.array([val * self.rescale for val in [i + 1, j]])
                    v2 = np.array([val * self.rescale for val in [i, j + 1]])
                    v12 = np.array([val * self.rescale for val in [i + 1, j + 1]])
                    X = [v0[0], v1[0], v12[0], v2[0], v0[0]]
                    Y = [v0[1], v1[1], v12[1], v2[1], v0[1]]
                    plt.plot(X, Y)
            circle = patches.Circle((0, 0), 1, fill=False, edgecolor='red', linewidth=2)
            plt.gca().add_patch(circle)
            plt.gca().set_aspect('equal', adjustable='box')
            plt.title(f"Dim={self.n}, R={R}, delta={self.delta}, epsilon={self.epsilon}")
            plt.xlabel("X Axis")
            plt.ylabel("Y Axis")
            plt.show()
        elif self.n == 3:
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            # get points
            X = [sample[0] for sample in samples]
            Y = [sample[1] for sample in samples]
            Z = [sample[2] for sample in samples]
            # Create a 3D scatter plot
            ax.scatter(X, Y, Z)
            # plot several lines
            # plot several lines
            for (i, j, w) in [[0,0,0], [-1, 0, 0]]:
                v0 = np.array([val * self.rescale for val in [i, j, w]])
                v1 = np.array([val * self.rescale for val in [i + 1, j, w]])
                v2 = np.array([val * self.rescale for val in [i, j + 1, w]])
                v3 = np.array([val * self.rescale for val in [i, j, w + 1]])
                v12 = np.array([val * self.rescale for val in [i + 1, j + 1, w]])
                v13 = np.array([val * self.rescale for val in [i + 1, j, w + 1]])
                v23 = np.array([val * self.rescale for val in [i, j + 1, w + 1]])
                v123 = np.array([val * self.rescale for val in [i + 1, j + 1, w + 1]])
                X = [v0[0], v1[0], v12[0], v2[0], v0[0]]
                Y = [v0[1], v1[1], v12[1], v2[1], v0[1]]
                Z = [v0[2], v1[2], v12[2], v2[2], v0[2]]
                ax.plot(X, Y, Z)
                X = [v0[0], v1[0], v13[0], v3[0], v0[0]]
                Y = [v0[1], v1[1], v13[1], v3[1], v0[1]]
                Z = [v0[2], v1[2], v13[2], v3[2], v0[2]]
                ax.plot(X, Y, Z)
                X = [v0[0], v2[0], v23[0], v3[0], v0[0]]
                Y = [v0[1], v2[1], v23[1], v3[1], v0[1]]
                Z = [v0[2], v2[2], v23[2], v3[2], v0[2]]
                ax.plot(X, Y, Z)
                X = [v123[0], v12[0], v2[0], v23[0], v123[0]]
                Y = [v123[1], v12[1], v2[1], v23[1], v123[1]]
                Z = [v123[2], v12[2], v2[2], v23[2], v123[2]]
                ax.plot(X, Y, Z)
                X = [v123[0], v12[0], v1[0], v13[0], v123[0]]
                Y = [v123[1], v12[1], v1[1], v13[1], v123[1]]
                Z = [v123[2], v12[2], v1[2], v13[2], v123[2]]
                ax.plot(X, Y, Z)
            # Generate data for a sphere
            u = np.linspace(0, 2 * np.pi, 100)
            v = np.linspace(0, np.pi, 100)
            x = R * np.outer(np.cos(u), np.sin(v))
            y = R * np.outer(np.sin(u), np.sin(v))
            z = R * np.outer(np.ones(np.size(u)), np.cos(v))
            # Plot the surface
            ax.plot_surface(x, y, z, color='r', alpha=0.3)  # 'alpha' controls the transparency
            # Set axis labels
            plt.gca().set_aspect('equal', adjustable='box')
            ax.set_xlabel('X Axis')
            ax.set_ylabel('Y Axis')
            ax.set_zlabel('Z Axis')
            plt.title(f"Dim={self.n}, R={R}, delta={self.delta}, epsilon={self.epsilon}")
            plt.show()


class AnStar:
    n: int
    epsilon: float
    delta: float
    rescale: float

    def __init__(self, n, epsilon, delta):
        self.n = n
        self.epsilon = epsilon
        self.delta = delta
        self.rescale = math.sqrt((12.0 * (n + 1)) / (n * (n + 2))) * (delta * self.epsilon) / math.sqrt(1 + self.epsilon ** 2)
        # create generator
        self.generator = np.eye(n - 1)
        self.generator = np.vstack([np.ones(self.n - 1,), -self.generator])
        new_col = np.ones((self.n, 1)) * (1.0 / (self.n + 1-math.sqrt(self.n + 1)))
        new_col[0] -= 1
        self.generator = np.hstack([self.generator, new_col])

    def get_samples_in_ball(self, R: float)->list:
        samples = []
        max_i = 20 * math.ceil(R / self.rescale)
        int_vectors = list(itertools.product(range(-max_i, max_i + 1), repeat=self.n))
        for vector in int_vectors:
            generated_v = np.matmul(self.generator, np.array(vector).reshape(-1, 1))
            vector_rescaled = np.array([val * self.rescale for val in generated_v])
            if np.linalg.norm(vector_rescaled) < R:
                samples.append(vector_rescaled.reshape(-1,))
                print(f"int_v={vector}, real_v={vector_rescaled}")
        return samples

    def show_samples_in_ball(self, R:float):
        samples = self.get_samples_in_ball(R)
        if self.n == 2:
            X = [sample[0] for sample in samples]
            Y = [sample[1] for sample in samples]
            plt.scatter(X, Y)
            # plot several lines
            v0 = np.zeros(self.n, ).reshape((-1,1))
            v0 = np.array([val * self.rescale for val in v0])
            v1 = np.matmul(self.generator, np.array([0,1]).reshape(-1, 1))
            v1 = np.array([val * self.rescale for val in v1])
            v2 = np.matmul(self.generator, np.array([1,2]).reshape(-1, 1))
            v2 = np.array([val * self.rescale for val in v2])
            X = [v0[0], v1[0], v2[0], v0[0]]
            Y = [v0[1], v1[1], v2[1], v0[0]]
            plt.plot(X, Y)
            v0 = np.zeros(self.n, ).reshape((-1,1))
            v0 = np.array([val * self.rescale for val in v0])
            v1 = np.matmul(self.generator, np.array([1,2]).reshape(-1, 1))
            v1 = np.array([val * self.rescale for val in v1])
            v2 = np.matmul(self.generator, np.array([1,1]).reshape(-1, 1))
            v2 = np.array([val * self.rescale for val in v2])
            X = [v0[0], v1[0], v2[0], v0[0]]
            Y = [v0[1], v1[1], v2[1], v0[0]]
            plt.plot(X, Y)
            circle = patches.Circle((0, 0), 1, fill=False, edgecolor='red', linewidth=2)
            plt.gca().add_patch(circle)
            plt.gca().set_aspect('equal', adjustable='box')
            plt.title(f"Dim={self.n}, R={R}, delta={self.delta}, epsilon={self.epsilon}")
            plt.xlabel("X Axis")
            plt.ylabel("Y Axis")
            plt.show()
        elif self.n == 3:
            fig = plt.figure(figsize=(8, 6))
            ax = fig.add_subplot(111, projection='3d')
            # get points
            X = [sample[0] for sample in samples]
            Y = [sample[1] for sample in samples]
            Z = [sample[2] for sample in samples]
            # Create a 3D scatter plot
            ax.scatter(X, Y, Z, color='green', edgecolor='black', linewidth=1,)
            # ax.scatter(X[14], Y[14], Z[14], color='red', edgecolor='black', linewidth=1,)
            C1 = [X[14], Y[14], Z[14]]
            C2 = [X[23], Y[23], Z[23]]
            # ax.scatter(X[23], Y[23], Z[23], color='red', edgecolor='black', linewidth=1,)
            # plot several lines
            # plot several lines
            for (i, j, w) in [[0,0,0], [-1, 0, 0]]:
                v0 = np.array([val * self.rescale for val in [i, j, w]])
                v1 = np.array([val * self.rescale for val in [i + 1, j, w]])
                v2 = np.array([val * self.rescale for val in [i, j + 1, w]])
                v3 = np.array([val * self.rescale for val in [i, j, w + 1]])
                v12 = np.array([val * self.rescale for val in [i + 1, j + 1, w]])
                v13 = np.array([val * self.rescale for val in [i + 1, j, w + 1]])
                v23 = np.array([val * self.rescale for val in [i, j + 1, w + 1]])
                v123 = np.array([val * self.rescale for val in [i + 1, j + 1, w + 1]])
                X = [v0[0], v1[0], v12[0], v2[0], v0[0]]
                Y = [v0[1], v1[1], v12[1], v2[1], v0[1]]
                Z = [v0[2], v1[2], v12[2], v2[2], v0[2]]
                ax.plot(X, Y, Z, linewidth=1, color='blue')
                X = [v0[0], v1[0], v13[0], v3[0], v0[0]]
                Y = [v0[1], v1[1], v13[1], v3[1], v0[1]]
                Z = [v0[2], v1[2], v13[2], v3[2], v0[2]]
                ax.plot(X, Y, Z, linewidth=1, color='blue')
                X = [v0[0], v2[0], v23[0], v3[0], v0[0]]
                Y = [v0[1], v2[1], v23[1], v3[1], v0[1]]
                Z = [v0[2], v2[2], v23[2], v3[2], v0[2]]
                ax.plot(X, Y, Z, linewidth=1, color='blue')
                X = [v123[0], v12[0], v2[0], v23[0], v123[0]]
                Y = [v123[1], v12[1], v2[1], v23[1], v123[1]]
                Z = [v123[2], v12[2], v2[2], v23[2], v123[2]]
                ax.plot(X, Y, Z, linewidth=1, color='blue')
                X = [v123[0], v12[0], v1[0], v13[0], v123[0]]
                Y = [v123[1], v12[1], v1[1], v13[1], v123[1]]
                Z = [v123[2], v12[2], v1[2], v13[2], v123[2]]
                ax.plot(X, Y, Z, linewidth=1, color='blue')
                if i == 0:
                    ax.plot([v0[0], C2[0]], [v0[1], C2[1]], [v0[2], C2[2]], linewidth=1, color='blue')
                    ax.plot([v1[0], C2[0]], [v1[1], C2[1]], [v1[2], C2[2]], linewidth=1, color='blue')
                    ax.plot([v2[0], C2[0]], [v2[1], C2[1]], [v2[2], C2[2]], linewidth=1, color='blue')
                    ax.plot([v3[0], C2[0]], [v3[1], C2[1]], [v3[2], C2[2]], linewidth=1, color='blue')
                    ax.plot([v12[0], C2[0]], [v12[1], C2[1]], [v12[2], C2[2]], linewidth=1, color='blue')
                    ax.plot([v13[0], C2[0]], [v13[1], C2[1]], [v13[2], C2[2]], linewidth=1, color='blue')
                    ax.plot([v23[0], C2[0]], [v23[1], C2[1]], [v23[2], C2[2]], linewidth=1, color='blue')
                    ax.plot([v123[0], C2[0]], [v123[1], C2[1]], [v123[2], C2[2]], linewidth=1, color='blue')
            # Generate data for a sphere
            u = np.linspace(0, 2 * np.pi, 100)
            v = np.linspace(0, np.pi, 100)
            x = R * np.outer(np.cos(u), np.sin(v))
            y = R * np.outer(np.sin(u), np.sin(v))
            z = R * np.outer(np.ones(np.size(u)), np.cos(v))
            # Plot the surface
            ax.plot_surface(x, y, z, color='r', alpha=0.15)  # 'alpha' controls the transparency

            # Plot a circle to help "see" the sphere
            # Parameters for the circle
            radius = 0.6  # Radius of the circle
            theta = np.linspace(0, 2 * np.pi, 100)  # Angle
            # Parametric equations for a circle in the XY-plane (z = 0)
            x = radius * np.cos(theta)
            y = radius * np.sin(theta)
            z = np.zeros_like(x)  # The circle lies in the XY-plane
            # Plot the circle
            ax.plot(x, y, z, color='black', alpha=0.5)
            # plot the poles
            ax.scatter([0, 0], [0, 0], [0.6, -0.6], s=35, color='black', edgecolor='white', linewidth=1)

            # matplotlib settings
            ax.set_box_aspect([1, 1, 1])
            ax.set_xticks([])
            ax.set_yticks([])
            ax.set_zticks([])
            ax.w_xaxis.line.set_color((0.5, 0.5, 0.5, 0.0))  # X-axis line invisible
            ax.w_yaxis.line.set_color((1.0, 1.0, 1.0, 0.0))  # Y-axis line invisible
            ax.w_zaxis.line.set_color((1.0, 1.0, 1.0, 0.0))  # Z-axis line invisible
            ax.w_xaxis.set_pane_color((0.5, 0.5, 0.5, 1))  # Light blue, semi-transparent
            ax.w_yaxis.set_pane_color((0.6, 0.6, 0.6, 1))  # Light red, semi-transparent
            ax.w_zaxis.set_pane_color((0.55, 0.55, 0.55, 1))  # Light green, semi-transparent
            ax.set_xlim([-0.5, 0.5])  # X-axis zoom
            ax.set_ylim([-0.5, 0.5])  # Y-axis zoom
            ax.set_zlim([-0.5, 0.5])  # Z-axis zoom
            ax.set_facecolor('white')
            fig.set_facecolor('black')
            ax.view_init(elev=25, azim=-75)

            plt.savefig('AN_3D.png', format='png')
            plt.show()

    def show_lattice_construction(self):
        alpha = 0.5
        n = 2
        N = n + 1
        # step 1: G^t*v
        G_t = np.zeros((n-1, n+1))
        G_t_last = np.ones(n + 1) * (1.0 / N)
        G_t_last[0] *= -n
        G_t = np.vstack([G_t, G_t_last])
        for i in range(n):
           if i < n - 1:
               G_t[i, 0] = 1
               G_t[i, i + 1] = -1
        G_t = alpha * G_t.T

        # step 2: PG^t*v
        P = np.eye(n) - np.ones((n, n)) * (1.0 / (N - math.sqrt(N)))
        P = np.hstack([P, np.ones((n, 1)) * (1.0 / math.sqrt(N))])
        P = np.vstack([P, np.ones((1, n+1)) * (1.0 / math.sqrt(N))])

        # step 3: plot them both on the same 3D graph
        fig = plt.figure()

        ax = fig.add_subplot(111, projection='3d')

        pts = [
            [0, 0], [1, 2], [-1, -2],
            [0, 1], [0, -1], [1, 1],
            [-1, -1], [1, 3], [-1, -3],
            [2, 3], [-2, -3]
        ]
        X = []
        Y = []
        Z = []
        for p in pts:
            v = np.array(p)
            v_post = G_t @ v
            X.append(v_post[0])
            Y.append(v_post[1])
            Z.append(v_post[2])
        ax.scatter(X, Y, Z, alpha=0.5, color='red')
        X = []
        Y = []
        Z = []
        for p in pts:
            v = np.array(p)
            v_post = (P @ G_t) @ v
            X.append(v_post[0])
            Y.append(v_post[1])
            Z.append(v_post[2])
        ax.scatter(X, Y, Z, alpha=0.5, color='red')

        # pre rotation plane
        X = []
        Y = []
        Z = []
        p1 = G_t @ np.array([2, 3])
        p2 = G_t @ np.array([1, 3])
        p3 = G_t @ np.array([-2, -3])
        p4 = G_t @ np.array([-1, -3])
        edges_X = [p1[0], p2[0], p3[0], p4[0]]
        edges_Y = [p1[1], p2[1], p3[1], p4[1]]
        edges_Z = [p1[2], p2[2], p3[2], p4[2]]
        # ax.scatter(edges_X, edges_Y, edges_Z, color='blue')
        vertices = [list(zip(edges_X, edges_Y, edges_Z))]
        poly = Poly3DCollection(vertices, facecolors='blue', alpha=0.24, edgecolors='black')
        ax.add_collection3d(poly)

        # post rotation plane
        p1 = (P @ G_t) @ np.array([2, 3])
        p2 = (P @ G_t) @ np.array([1, 3])
        p3 = (P @ G_t) @ np.array([-2, -3])
        p4 = (P @ G_t) @ np.array([-1, -3])
        edges_X = [p1[0], p2[0], p3[0], p4[0]]
        edges_Y = [p1[1], p2[1], p3[1], p4[1]]
        edges_Z = [p1[2], p2[2], p3[2], p4[2]]
        # ax.scatter(edges_X, edges_Y, edges_Z, color='blue')
        vertices = [list(zip(edges_X, edges_Y, edges_Z))]
        poly = Poly3DCollection(vertices, facecolors='green', zorder=5, alpha=0.24, edgecolors='black')
        ax.add_collection3d(poly)

        p1 = (P @ G_t) @ np.array([1, 0])
        p2 = (P @ G_t) @ np.array([-1, 0])
        edges_X = [p1[0] / 2, p2[0] / 2]
        edges_Y = [p1[1] / 2, p2[1] / 2]
        edges_Z = [p1[2] / 2, p2[2] / 2]
        ax.plot(edges_X, edges_Y, edges_Z, linewidth=1, color='black')



        map_resize = 0.75
        vertices = [list(zip([-map_resize, -map_resize, map_resize, map_resize],
                             [-map_resize, map_resize, map_resize, -map_resize],
                             [0, 0, 0, 0]))]
        ax.view_init(elev=20, azim=-25)
        poly = Poly3DCollection(vertices, facecolors='lightblue', zorder=1, alpha=0.3, edgecolors='black')
        ax.add_collection3d(poly)
        ax.set_box_aspect([1, 1, 1])
        ax.set_xlim([-map_resize, map_resize])
        ax.set_ylim([-map_resize, map_resize])
        ax.set_zlim([-map_resize, map_resize])

        # matplotlib settings
        ax.set_xticks([])
        ax.set_yticks([])
        ax.set_zticks([])
        ax.w_xaxis.line.set_color((0.5, 0.5, 0.5, 0.0))  # X-axis line invisible
        ax.w_yaxis.line.set_color((1.0, 1.0, 1.0, 0.0))  # Y-axis line invisible
        ax.w_zaxis.line.set_color((1.0, 1.0, 1.0, 0.0))  # Z-axis line invisible
        ax.w_xaxis.set_pane_color((0.5, 0.5, 0.5, 1))  # Light blue, semi-transparent
        ax.w_yaxis.set_pane_color((0.6, 0.6, 0.6, 1))  # Light red, semi-transparent
        ax.w_zaxis.set_pane_color((0.55, 0.55, 0.55, 1))  # Light green, semi-transparent
        ax.set_facecolor('white')
        fig.set_facecolor('black')

        plt.savefig('EPGt_visualization.png', format='png')
        plt.show()



