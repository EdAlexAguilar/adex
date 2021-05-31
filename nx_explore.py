import networkx as nx
import matplotlib.pyplot as plt

G = nx.DiGraph()

topo = [(0,1),(1,2),(2,0),(2,2),(1,0)]

G.add_edges_from(topo)

# nx.draw(G, with_labels=True)
# plt.show()

print(G.edges())