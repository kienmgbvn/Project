import streamlit as st
import time
from pyvis.network import Network
import networkx as nx
import pandas as pd
from ADFS import Tree, calculate_adfs

# Cấu hình giao diện
st.set_page_config(layout="wide", page_title="ADFS Visualizer")
st.markdown("<style>body { background-color: #111111; color: white; }</style>", unsafe_allow_html=True)

st.title("🌳 ADFS Algorithm Visualization")

# Sidebar: input
st.sidebar.header("Input Tree")
n = st.sidebar.number_input("Number of vertices", min_value=2, max_value=50, value=5)
B = st.sidebar.number_input("Energy budget (B)", min_value=1.0, value=16.0)
root = st.sidebar.number_input("Root vertex", min_value=0, max_value=n-1, value=0)

st.sidebar.markdown("### Enter edges list (each line: u v weight)")
edge_text = st.sidebar.text_area(
    label="Edges (e.g., 0 1 2.0)",
    value="0 1 2.0\n0 2 4.0\n2 3 2.0\n2 4 2.0",
    height=150
)

# Parse edges
edges = []
for line in edge_text.strip().splitlines():
    try:
        u, v, w = line.strip().split()
        edges.append((int(u), int(v), float(w)))
    except:
        st.sidebar.warning(f"Invalid edge format: {line}")

# Init session state
for key, default in {
    "tree_built": False,
    "routes": [],
    "current_route_idx": 0,
    "current_path_step": 0,
    "auto_play": False,
    "worst_B_prime": 0.0,
    "total_cost": 0.0
}.items():
    if key not in st.session_state:
        st.session_state[key] = default

# Run ADFS button
if st.sidebar.button("Run ADFS"):
    tree = Tree(n, root)
    for u, v, w in edges:
        tree.add_edge(u, v, w)

    cost, routes, worst_B = calculate_adfs(tree, B)

    st.session_state.routes = routes
    st.session_state.tree_built = True
    st.session_state.current_route_idx = 0
    st.session_state.current_path_step = 0
    st.session_state.auto_play = True
    st.session_state.worst_B_prime = worst_B
    st.session_state.total_cost = cost

# Hiển thị cây nếu đã xây
if st.session_state.tree_built:
    G = nx.Graph()
    for i in range(n):
        G.add_node(i, label=str(i))
    for u, v, w in edges:
        G.add_edge(u, v, weight=w, label=str(w))

    net = Network(height="550px", width="100%", directed=False, bgcolor="#111111", font_color="white")
    net.from_nx(G)

    net.set_options("""
    var options = {
      "layout": {
        "hierarchical": {
          "enabled": true,
          "levelSeparation": 100,
          "nodeSpacing": 150,
          "treeSpacing": 200,
          "direction": "UD",
          "sortMethod": "hubsize"
        }
      },
      "physics": {
        "enabled": false
      }
    }
    """)
    levels = nx.single_source_shortest_path_length(G, root)
    for node in net.nodes:
        node["level"] = levels.get(node["id"], 0)

    route_idx = st.session_state.current_route_idx
    step = st.session_state.current_path_step
    routes = st.session_state.routes
    visited_nodes = set()

    if route_idx < len(routes):
        route = routes[route_idx]
        vertices = route.vertices
        for i in range(1, step + 1):
            u = vertices[i - 1]
            v = vertices[i]
            visited_nodes.update([u, v])
            for e in net.edges:
                if (e["from"] == u and e["to"] == v) or (e["from"] == v and e["to"] == u):
                    e["color"] = "red"
                    e["width"] = 4

        for node in net.nodes:
            if node["id"] in visited_nodes:
                node["color"] = "yellow"
                node["borderWidth"] = 3

    net.write_html("tree_step.html")
    with open("tree_step.html", "r", encoding="utf-8") as f:
        html = f.read()
    st.components.v1.html(html, height=600, scrolling=False)

    st.subheader(f"📌 Current Route R{route_idx + 1}")
    if route_idx < len(routes):
        current_route = routes[route_idx]
        st.code(" → ".join(map(str, current_route.vertices)))

    if st.session_state.auto_play:
        time.sleep(1.0)
        route = st.session_state.routes[route_idx]
        if st.session_state.current_path_step < len(route.vertices) - 1:
            st.session_state.current_path_step += 1
            st.rerun()
        else:
            if st.session_state.current_route_idx < len(st.session_state.routes) - 1:
                st.session_state.current_route_idx += 1
                st.session_state.current_path_step = 0
                st.rerun()
            else:
                st.session_state.auto_play = False

    col1, col2 = st.columns([1, 4])
    with col1:
        if st.button("⏹ Stop"):
            st.session_state.auto_play = False
        if st.button("🔁 Replay"):
            st.session_state.current_route_idx = 0
            st.session_state.current_path_step = 0
            st.session_state.auto_play = True
            st.rerun()

    with col2:
        st.subheader("📋 ADFS Routes")
        data = [{"Route ID": f"R{i+1}", "Vertices": r.vertices, "Energy Used": r.length} for i, r in enumerate(st.session_state.routes)]
        st.dataframe(pd.DataFrame(data))

    st.subheader("📊 ADFS Summary")
    st.metric("Worst B′", round(st.session_state.worst_B_prime, 3))
    st.metric("Total Cost", round(st.session_state.total_cost, 3))
