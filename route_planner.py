"""A* route planner. Uses OpenStreetMap via OSMnx when available; otherwise a reproducible demo grid."""
import argparse, heapq, math, random, time, csv, json, os
from collections import deque

R = 6371000.0

def haversine(a, b):
    lat1, lon1 = math.radians(a[0]), math.radians(a[1])
    lat2, lon2 = math.radians(b[0]), math.radians(b[1])
    dlat, dlon = lat2-lat1, lon2-lon1
    x = math.sin(dlat/2)**2 + math.cos(lat1)*math.cos(lat2)*math.sin(dlon/2)**2
    return 2*R*math.asin(math.sqrt(x))

def build_demo_graph(n=35, seed=42):
    """Offline grid-shaped road network; explicitly not real OSM data."""
    rng=random.Random(seed); adj={}; coords={}; step=0.001
    for y in range(n):
        for x in range(n):
            u=y*n+x; coords[u]=(28.60+y*step,77.20+x*step); adj[u]=[]
    for y in range(n):
        for x in range(n):
            u=y*n+x
            for dx,dy in [(1,0),(0,1)]:
                xx,yy=x+dx,y+dy
                if xx<n and yy<n:
                    v=yy*n+xx
                    # Road length is slightly greater than straight-line distance.
                    d=haversine(coords[u],coords[v])*rng.uniform(1.03,1.25)
                    adj[u].append((v,d)); adj[v].append((u,d))
    return adj,coords,'Synthetic offline grid (not OpenStreetMap)'

def build_osm_graph(place, radius):
    import osmnx as ox
    center=ox.geocode(place)
    G=ox.graph_from_point(center, dist=radius, network_type='drive', simplify=True)
    G=ox.utils_graph.get_largest_component(G, strongly=False)
    coords={n:(d['y'],d['x']) for n,d in G.nodes(data=True)}
    adj={n:[] for n in G.nodes}
    for u,v,d in G.edges(data=True):
        w=float(d.get('length',1e9)); adj[u].append((v,w))
    return adj,coords,f'OpenStreetMap road network: {place}'

def search(adj, coords, start, goal, mode='astar'):
    t0=time.perf_counter(); g={start:0.0}; parent={}; closed=set(); expanded=0
    h=lambda n: haversine(coords[n],coords[goal])
    pq=[((h(start) if mode=='astar' else 0.0),start)]
    while pq:
        _,u=heapq.heappop(pq)
        if u in closed: continue
        closed.add(u); expanded+=1
        if u==goal:
            path=[u]
            while u in parent: u=parent[u]; path.append(u)
            path.reverse(); return path,g[goal],expanded,(time.perf_counter()-t0)*1000
        for v,w in adj[u]:
            if v in closed: continue
            ng=g[u]+w
            if ng<g.get(v,float('inf')):
                g[v]=ng; parent[v]=u
                score=ng+(h(v) if mode=='astar' else 0.0)
                heapq.heappush(pq,(score,v))
    return [],float('inf'),expanded,(time.perf_counter()-t0)*1000

def make_map(coords, path, out='route_map.html'):
    import folium
    pts=[coords[n] for n in path]
    center=pts[len(pts)//2] if pts else next(iter(coords.values()))
    m=folium.Map(location=center,zoom_start=14,tiles='OpenStreetMap')
    if pts:
        folium.Marker(pts[0],tooltip='Start',icon=folium.Icon(color='green')).add_to(m)
        folium.Marker(pts[-1],tooltip='Destination',icon=folium.Icon(color='red')).add_to(m)
        folium.PolyLine(pts,color='blue',weight=5,opacity=.85,tooltip='A* shortest route').add_to(m)
    m.save(out)

def main():
    p=argparse.ArgumentParser(); p.add_argument('--place',default='India Gate, New Delhi, India'); p.add_argument('--radius',type=int,default=2500); p.add_argument('--offline',action='store_true'); p.add_argument('--seed',type=int,default=42); args=p.parse_args()
    if not args.offline:
        try: adj,coords,label=build_osm_graph(args.place,args.radius)
        except Exception as e:
            print('OSM download failed:',e); print('Switching to offline demo graph.'); adj,coords,label=build_demo_graph(seed=args.seed)
    else: adj,coords,label=build_demo_graph(seed=args.seed)
    nodes=list(adj); rng=random.Random(args.seed); pairs=[]
    # 30 reproducible routes; use corners/far-apart nodes in demo, random pairs in OSM.
    for _ in range(30):
        s=rng.choice(nodes); t=rng.choice(nodes)
        while t==s: t=rng.choice(nodes)
        pairs.append((s,t))
    rows=[]; chosen=None
    for i,(s,t) in enumerate(pairs,1):
        pa,ca,ea,ta=search(adj,coords,s,t,'astar'); pd,cd,ed,td=search(adj,coords,s,t,'dijkstra')
        rows.append({'route':i,'astar_nodes':ea,'dijkstra_nodes':ed,'astar_ms':ta,'dijkstra_ms':td,'astar_distance_m':ca,'dijkstra_distance_m':cd,'optimal':abs(ca-cd)<1e-6})
        if chosen is None or ca>chosen[0]: chosen=(ca,pa,s,t)
    with open('results.csv','w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=rows[0].keys()); w.writeheader(); w.writerows(rows)
    avg=lambda k:sum(float(r[k]) for r in rows)/len(rows)
    summary={'dataset':label,'nodes':len(adj),'directed_edges':sum(map(len,adj.values())),'routes':len(rows),'astar_avg_nodes':avg('astar_nodes'),'dijkstra_avg_nodes':avg('dijkstra_nodes'),'astar_avg_ms':avg('astar_ms'),'dijkstra_avg_ms':avg('dijkstra_ms'),'astar_avg_distance_m':avg('astar_distance_m'),'dijkstra_avg_distance_m':avg('dijkstra_distance_m'),'optimal_routes':sum(r['optimal'] for r in rows),'node_reduction_pct':(1-avg('astar_nodes')/avg('dijkstra_nodes'))*100}
    with open('results_summary.json','w') as f: json.dump(summary,f,indent=2)
    make_map(coords,chosen[1])
    print(json.dumps(summary,indent=2)); print('Created results.csv, results_summary.json, route_map.html')
if __name__=='__main__': main()
