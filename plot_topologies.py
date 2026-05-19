#!/usr/bin/env python3
# plot_topologies.py - Topologies des simulations QoS V14

import matplotlib.pyplot as plt
import networkx as nx

def plot_topology_sc1(qos=False):
    """Topologie Wi-Fi Statique SC1"""
    fig, ax = plt.subplots(figsize=(12, 8))
    
    # Positions manuelles
    pos = {
        'AP': (0, 0),
        'VoIP': (2, 2),
        'Video': (2, 0),
        'FTP': (2, -2),
        'BG': (-2, 0),
        'Serveur': (0, -4)
    }
    
    # Couleurs par type
    colors = {
        'AP': 'gold',
        'VoIP': 'lightcoral',
        'Video': 'lightblue',
        'FTP': 'lightgreen',
        'BG': 'lightgray',
        'Serveur': 'orange'
    }
    
    labels = {node: node for node in pos.keys()}
    
    # Création du graphe
    G = nx.Graph()
    edges = [('AP', 'VoIP'), ('AP', 'Video'), ('AP', 'FTP'), ('AP', 'BG'), ('AP', 'Serveur')]
    G.add_edges_from(edges)
    
    node_colors = [colors.get(node, 'lightgray') for node in G.nodes()]
    
    nx.draw(G, pos, with_labels=True, labels=labels,
            node_color=node_colors, node_size=3000,
            font_size=10, font_weight='bold',
            edge_color='gray', width=2,
            ax=ax)
    
    # Délais sur les liens
    delays = {
        'VoIP': '19ms' if qos else '140ms',
        'Video': '50ms' if qos else '116ms',
        'FTP': '96ms' if qos else '199ms',
        'BG': '8ms' if qos else '60ms'
    }
    
    for node, delay in delays.items():
        if node in pos:
            x1, y1 = pos['AP']
            x2, y2 = pos[node]
            mx, my = (x1 + x2)/2, (y1 + y2)/2 + 0.2
            ax.text(mx, my, f'Delay: {delay}', fontsize=8,
                   ha='center', bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
    
    title = f"SC1 - WiFi Static {'with QoS' if qos else 'without QoS'}"
    ax.set_title(title, fontsize=14, fontweight='bold')
    ax.axis('off')
    plt.tight_layout()
    plt.savefig(f'topology_sc1_{"qos" if qos else "noqos"}.png', dpi=150, bbox_inches='tight')
    plt.show()

def plot_topology_sc3(qos=False):
    """Topologie LTE SC3"""
    fig, ax = plt.subplots(figsize=(14, 10))
    
    # Positions
    pos = {
        'eNB': (0, 0),
        'UE0_VoIP': (-2, 2),
        'UE1_Video': (0, 2.5),
        'UE2_FTP': (2, 2),
        'UE3_BG': (-3, -1),
        'UE4_BG': (-1, -1.5),
        'UE5_BG': (1, -1.5),
        'UE6_BG': (3, -1),
        'PGW': (0, -3),
        'RH_VoIP': (-3, -4),
        'RH_Video': (-1, -4),
        'RH_FTP': (1, -4),
        'RH_BG': (3, -4)
    }
    
    colors = {
        'eNB': 'darkorange',
        'UE0_VoIP': 'lightcoral',
        'UE1_Video': 'lightblue',
        'UE2_FTP': 'lightgreen',
        'UE3_BG': 'lightgray',
        'UE4_BG': 'lightgray',
        'UE5_BG': 'lightgray',
        'UE6_BG': 'lightgray',
        'PGW': 'gold',
        'RH_VoIP': 'coral',
        'RH_Video': 'skyblue',
        'RH_FTP': 'yellowgreen',
        'RH_BG': 'silver'
    }
    
    G = nx.Graph()
    # Liens eNB -> UE
    for i in range(7):
        if i == 0:
            G.add_edge('eNB', 'UE0_VoIP')
        elif i == 1:
            G.add_edge('eNB', 'UE1_Video')
        elif i == 2:
            G.add_edge('eNB', 'UE2_FTP')
        else:
            G.add_edge('eNB', f'UE{i}_BG')
    
    # Liens PGW -> RH
    G.add_edge('PGW', 'RH_VoIP')
    G.add_edge('PGW', 'RH_Video')
    G.add_edge('PGW', 'RH_FTP')
    G.add_edge('PGW', 'RH_BG')
    G.add_edge('eNB', 'PGW')
    
    node_colors = [colors.get(node, 'lightgray') for node in G.nodes()]
    
    nx.draw(G, pos, with_labels=True, labels={n:n for n in G.nodes()},
            node_color=node_colors, node_size=2800,
            font_size=8, font_weight='bold',
            edge_color='gray', width=2,
            ax=ax)
    
    title = f"SC3 - LTE {'with QoS (QCI 1/2/8)' if qos else 'without QoS'}"
    ax.set_title(title, fontsize=14, fontweight='bold')
    ax.axis('off')
    plt.tight_layout()
    plt.savefig(f'topology_sc3_{"qos" if qos else "noqos"}.png', dpi=150, bbox_inches='tight')
    plt.show()

if __name__ == "__main__":
    print("Generating topologies...")
    plot_topology_sc1(qos=False)
    plot_topology_sc1(qos=True)
    plot_topology_sc3(qos=False)
    plot_topology_sc3(qos=True)
    print("Done!")