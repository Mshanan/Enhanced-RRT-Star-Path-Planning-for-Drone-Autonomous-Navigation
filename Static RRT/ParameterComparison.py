# import matplotlib.pyplot as plt
# import numpy as np

# # # Data for the obstacle-free room
# algorithms = ['RRT* Base', 'RRT* with Obstacle Avoidance', 'RRT* with Maximum Angle', 'Improved RRT*']
# # measurables = ['Total Generated\nNodes Count', 'Path Nodes\nCount', 'Node Utilisation (%)', 'Total Path\nLength (m)', 'Time (s)']
# measurables = ['Path Nodes\nCount', 'Node Utilisation (%)', 'Total Path\nLength (m)', 'Maximum Iterations N']

# # data_free = [
# #     # [217, 217, 150, 352],  # Total Generated Nodes Count
# #     [61, 61, 50, 60],      # Path Nodes Count
# #     [28.1, 28.1, 33.3, 17.1],  # Node Utilisation (%)
# #     [14.76, 14.76, 12.23, 14.59],   # Total Path Length (m)
# #     [400/100, 400/100, 1050/100, 1800/100]           # max iter 
# # ]

# data_cluttered = [
#     # [217, 284, 372, 909],   # Total Generated Nodes Count
#     [61, 70, 61, 79],      # Path Nodes Count
#     [28.1, 24.6, 16.4, 8.6],   # Node Utilisation (%)
#     [14.76, 17.01, 14.90, 19.80],   # Total Path Length (m)
#     [400/100, 1000/100, 4000/100, 20000/100]          # max iter 
# ]

# # Plotting
# fig, axs = plt.subplots(1, 2, figsize=(16, 6))

# x = np.arange(len(measurables))
# width = 0.15

# colors = ['red', 'green', 'blue', 'darkorange']

# # Obstacle-free room
# # for i, algorithm in enumerate(algorithms):
# #     axs[0].bar(x + i*width, [data_free[j][i] for j in range(len(measurables))], width, label=algorithm, color=colors[i])

# # axs[0].set_xlabel('Measurables')
# # axs[0].set_ylabel('Values')
# axs[0].set_title('RRT Comparison in an Obstacle-Free Environment')
# axs[0].set_xticks(x + width)
# axs[0].set_xticklabels(measurables)
# axs[0].legend()

# # Cluttered room
# for i, algorithm in enumerate(algorithms):
#     axs[1].bar(x + i*width, [data_cluttered[j][i] for j in range(len(measurables))], width, label=algorithm, color=colors[i])

# # axs[1].set_xlabel('Measurables')
# # axs[1].set_ylabel('Values')
# axs[1].set_title('RRT Comparison in a Cluttered Environment')
# axs[1].set_xticks(x + width)
# axs[1].set_xticklabels(measurables)
# axs[1].legend()

# plt.tight_layout()
# plt.show()



import plotly.graph_objects as go
import numpy as np

# Data and labels
algorithms = ['RRT* Base', 'RRT* with Obstacle Avoidance', 'RRT* with Maximum Angle', 'Improved RRT*']
measurables = ['Path Nodes\nCount', 'Node Utilisation (%)', 'Total Path\nLength (m)', 'Maximum Iterations N']

data_cluttered = [
    [61, 70, 61, 73],      # Path Nodes Count
    [28.1, 24.6, 16.4, 8.6],   # Node Utilisation (%)
    [14.76, 17.01, 14.90, 19.80],   # Total Path Length (m)
    [400/100, 1000/100, 4000/100, 20000/100]          # max iter 
]

# Setup
x = np.arange(len(measurables))
width = 0.2  # Width of the bars

# Create a bar chart using plotly
fig = go.Figure()

# Improved colour palette for academic presentation
colors = ['#636EFA', '#EF553B', '#00CC96', '#AB63FA']  # Plotly's professional colour scheme

# Add bars for each algorithm
for i, algorithm in enumerate(algorithms):
    fig.add_trace(go.Bar(
        x=[x_val + (i - (len(algorithms) - 1) / 2) * width for x_val in x],  # Adjust x position for each algorithm
        y=[data_cluttered[j][i] for j in range(len(measurables))],
        name=algorithm,
        marker_color=colors[i],
        width=width
    ))

# Update layout
fig.update_layout(
    # title='RRT Comparison in a Cluttered Environment',
    xaxis=dict(
        # title='Measurables',
        tickvals=x,  # Set the positions for ticks
        ticktext=measurables  # Set the tick labels
    ),
    yaxis=dict(
        # title='Values',
        range=[0, max(max(data_cluttered[j]) for j in range(len(measurables)))]  # Set the y-axis to start from 0
    ),
    barmode='group',  # Group bars to keep them closely packed
    bargap=0.1,  # Small gap between groups
    bargroupgap=0.05,  # Small gap within groups
    legend=dict(
        x=0.0,
        y=1,
        xanchor='left',
        yanchor='top',
        ),
    font=dict(size=12),  # Smaller font size for a more compact figure
    margin=dict(l=50, r=50, t=50, b=50),  # Adjust margins to make the figure smaller
    width=800,  # Smaller width
    height=400  # Smaller height
)

# Show the figure
fig.show()


# # Data for the obstacle-free and cluttered rooms
# algorithms = ['RRT* Base', 'RRT* with Obstacle Avoidance', 'RRT* with Maximum Angle', 'Improved RRT*']
# data_free = [400, 400, 1050, 1800]           # max iter in obstacle-free room
# data_cluttered = [400, 800, 4000, 20000]     # max iter in cluttered room

# # Plotting
# fig, ax = plt.subplots(figsize=(12, 6))

# x = np.arange(len(algorithms))
# width1 = 0.25  # Width of bars for obstacle-free environment
# width2 = 0.25  # Width of bars for cluttered environment, narrower to fit alongside

# # Positions for the bars
# positions1 = x - width1 / 2
# positions2 = x + width1 / 2  # Offset for the second set of bars

# # Plot bars for obstacle-free environment
# bars1 = ax.bar(positions1, data_free, width1, label='Obstacle-Free Environment', color='steelblue')

# # Plot bars for cluttered environment
# bars2 = ax.bar(positions2, data_cluttered, width2, label='Cluttered Environment', color='red')

# # Add some text for labels, title, and custom x-axis tick labels, etc.
# ax.set_xlabel('Algorithms')
# ax.set_ylabel('Maximum Iterations')
# ax.set_title('Comparison of Maximum Iterations for Different RRT Algorithms')
# ax.set_xticks(x)
# ax.set_xticklabels(algorithms)
# ax.legend()

# # Display the plot
# plt.tight_layout()
# plt.show()