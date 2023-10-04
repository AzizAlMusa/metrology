import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D
from matplotlib.patches import Circle


def force_function():
    # Define the coefficients
    A = 1.25
    B = -2.375
    C = 0.0
    D = 1.125

    # Create an array of w values (r_s is always 1, so w = r)
    w_values = np.linspace(0, 2, 100)  # Adjust the range and number of points as needed

    # Calculate f for each w value using the given coefficients
    f_values = A * w_values**3 + B * w_values**2 + C * w_values + D

    # Make values of f zero for w > 1.5
    f_values[w_values > 1.5] = 0.0

    # Create the piecewise plot in terms of w
    plt.figure(figsize=(8, 6))
    plt.plot(w_values, f_values, label='f = 1.25 * w^3 - 2.375 * w^2 + 1.125 (0 <= w <= 1.5)', color='b')
    plt.xlabel('w')
    plt.ylabel('f')
    plt.title('Piecewise Plot of f = 1.25 * w^3 - 2.375 * w^2 + 1.125 (0 <= w <= 1.5)')
    plt.grid(True)
    plt.legend()
    plt.xlim(0, 2)  # Set the x-axis limits
    plt.show()
import pdb

def rhombus_construction(u, v, l):

    # assume l was calculated for the rhombus
    
    u1, v1 = u - 3.0 * l / 4.0, v - np.sqrt(3.0) * l / 4.0
    u2, v2 = u + l / 4.0, v - np.sqrt(3.0) * l / 4.0
    u3, v3 = u + 3.0 * l / 4.0, v + np.sqrt(3.0) * l / 4.0
    u4, v4 = u - l / 4.0, v + np.sqrt(3.0) * l / 4.0
    # Create a Matplotlib figure and axis
    fig, ax = plt.subplots()

    # Plot the rhombus corners
    ax.plot([u1, u2, u3, u4, u1], [v1, v2, v3, v4, v1], marker='o', linestyle='-', color='b')

    # Set axis limits and labels (optional)
    ax.set_xlim(u - l, u + l)
    ax.set_ylim(v - l, v + l)
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')

    # Display the plot
    plt.grid()
    plt.gca().set_aspect('equal', adjustable='box')  # Equal aspect ratio
    plt.show()


def check_bubble_overlap(u, v, l):
   # Define the coordinates of the rhombus corners
    u, v = 0.0, 0.0
    l = 1.0  # Side length of the rhombus
    r = 0.499  # Radius of each bubble

    u1, v1 = u - 3.0 * l / 4.0, v - np.sqrt(3.0) * l / 4.0
    u2, v2 = u + l / 4.0, v - np.sqrt(3.0) * l / 4.0
    u3, v3 = u + 3.0 * l / 4.0, v + np.sqrt(3.0) * l / 4.0
    u4, v4 = u - l / 4.0, v + np.sqrt(3.0) * l / 4.0

    # Create a dictionary to store the bubble positions
    rhombus = {
        1: (u1, v1),
        2: (u2, v2),
        3: (u3, v3),
        4: (u4, v4)
    }

    # Define the combinations to check for overlap
    edges = [
        (1, 2),
        (2, 3),
        (3, 4),
        (2, 4)
    ]

    # Function to check if two bubbles overlap
    def bubbles_overlap(bubble1, bubble2, l):
        u1, v1 = bubble1
        u2, v2 = bubble2
        return np.linalg.norm(np.array([u1, v1]) - np.array([u2, v2])) <= 2 * r

    # Check if ANY two bubbles overlap
    overlap = any(bubbles_overlap(rhombus[i], rhombus[j], l) for i, j in edges)
    print(overlap)
    # Plot the rhombus and bubbles
    fig, ax = plt.subplots()
    ax.set_aspect('equal', 'box')
    ax.plot([u1, u2, u3, u4, u1], [v1, v2, v3, v4, v1], 'b-')  # Rhombus
    for i in rhombus:
        u, v = rhombus[i]
        circle = plt.Circle((u, v), r, fill=False, color='r')
        ax.add_artist(circle)

    if overlap:
        plt.title("At least two bubbles overlap.")
    else:
        plt.title("No bubbles overlap.")

    plt.xlabel('u')
    plt.ylabel('v')
    plt.grid()
    plt.show()


def draw_rhombi(l):
    # Given vertices
    # Given vertices for the quadrilateral
    l = 1.0
    A = np.array([-3 * l / 4, -np.sqrt(3) / 4 * l])
    B = np.array([l / 4, -np.sqrt(3) / 4 * l])
    C = np.array([3 * l / 4, np.sqrt(3) * l / 4])
    D = np.array([-l / 4, np.sqrt(3) / 4 * l])

    # Given vertices for the smaller rhombus
    P = np.array([3 / 8.0 * l, np.sqrt(3) / 8.0 * l])
    Q = np.array([-l / 8.0, np.sqrt(3) / 8.0 * l])
    R = np.array([- 3 / 8.0 * l, - np.sqrt(3) / 8.0 * l])
    S = np.array([l / 8.0, - np.sqrt(3) / 8.0 * l])

    # Create a list of points for the vertices of both shapes
    quadrilateral_vertices = [A, B, C, D, A]  # Repeat A at the end to close the quadrilateral
    rhombus_vertices = [P, Q, R, S, P]  # Repeat P at the end to close the rhombus
    print(rhombus_vertices)
    # Extract x and y coordinates for plotting
    quadrilateral_x, quadrilateral_y = zip(*quadrilateral_vertices)
    rhombus_x, rhombus_y = zip(*rhombus_vertices)

    # Create a plot
    plt.figure(figsize=(6, 6))  # Optional: set the figure size

    # Plot the quadrilateral
    plt.plot(quadrilateral_x, quadrilateral_y, marker='o', linestyle='-', label='Quadrilateral')

    # Plot the smaller rhombus
    plt.plot(rhombus_x, rhombus_y, marker='o', linestyle='-', label='Smaller Rhombus')

 

    # Set axis limits for better visualization
    plt.xlim(-1.2, 1.2)
    plt.ylim(-1.2, 1.2)

    # Add labels and title
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.title('Quadrilateral ABCD and Smaller Rhombus PQRS')

    # Show the legend
    plt.legend()

    # Show the plot
    plt.grid()
    plt.show()


# rhombus_construction(0, 0 , 1)
# check_bubble_overlap(0, 0, 1)

# Call the function with the desired side length 'l'
l = 1  # You can change this to any positive number
# draw_rhombi(l)

def plot_edges(edges1, edges2):
    fig, ax = plt.subplots()

    for edge in edges1:
        x, y = zip(*edge)
        ax.plot(x, y, marker='o', markersize=6, linestyle='-', color='orange', label='Edges 1')

    for edge in edges2:
        x, y = zip(*edge)
        ax.plot(x, y, marker='o', markersize=6, linestyle='-', color='blue', label='Edges 2')

    ax.set_aspect('equal')
    ax.autoscale()
    plt.legend()
    plt.show()



def test_bubble_force():
    # Define the coefficients
    A = 1.25
    B = -2.375
    C = 0.0
    D = 1.125

    def interbubble_force(r):
        w = r
        if w > 1.5:
            return 0.0
        else:
            return A * w**3 + B * w**2 + C * w + D

  
def spawn_bubble(center, radius):
    """
    Spawn a 2D bubble with a specified radius at a given center.

    Args:
    - center (tuple): The center coordinates of the bubble (x, y).
    - radius (float): The radius of the bubble.

    Returns:
    - plt.figure: The Matplotlib figure containing the bubble.
    """
    # Create a Matplotlib figure and axis
    fig, ax = plt.subplots()

    # Create a circle patch for the bubble
    bubble = Circle(center, radius, fill=False, color='blue', edgecolor='blue')

    # Add the bubble to the axis
    ax.add_patch(bubble)

    # Set axis limits to display the bubble properly
    ax.set_xlim(center[0] - radius - 1, center[0] + radius + 1)
    ax.set_ylim(center[1] - radius - 1, center[1] + radius + 1)

    # Set axis aspect ratio to be equal
    ax.set_aspect('equal', adjustable='box')

    # Show the plot
    plt.show()


def calculate_interbubble_force(radius1, radius2, distance):
    """
    Calculate the interbubble force between two bubbles.

    Args:
    - radius1 (float): Radius of the first bubble.
    - radius2 (float): Radius of the second bubble.
    - distance (float): Distance between the centers of the two bubbles.

    Returns:
    - force (float): The interbubble force.
    """
    # Calculate the stable distance (r0) as the sum of the radii
    r0 = radius1 + radius2

    # Calculate w (r/r0)
    w = distance / r0

    # Define the coefficients A, B, C, and D
    A = 1.25
    B = -2.375
    C = 0.0
    D = 1.125

    # Calculate the force using the provided formula
    if w > 1.5:
        force = 0.0
    else:
        force = A * w**3 + B * w**2 + C * w + D

    return force

# Example usage:
radius1 = 2.0
radius2 = 1.5
distance = 4.0


force = calculate_interbubble_force(radius1, radius2, distance)
# Example usage:
spawn_bubble((5, 5), 2.5)  # Spawn a bubble with radius 2.5 at (5, 5)