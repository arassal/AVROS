#!/usr/bin/env python3
"""Offline graph generation: OSMnx campus roads → nav2_route GeoJSON.

Downloads the CPP campus road network via OSMnx, densifies edges by
interpolating along road geometry at a configurable spacing, converts
GPS coordinates to map-frame (meters relative to a fixed datum) using
UTM projection, and writes a GeoJSON file compatible with nav2_route's
GeoJsonGraphFileLoader.

nav2_route only supports straight-line interpolation between graph nodes
(edge geometry in GeoJSON is ignored). This script works around that by
placing graph nodes along road curves at regular intervals, so the
straight-line segments between closely-spaced nodes naturally follow
the road shape.

Reference: AV2.1-API/planning/navigator.py (Shapely interpolation approach)

Usage:
    python3 generate_graph.py
    python3 generate_graph.py --output /path/to/output.geojson
    python3 generate_graph.py --spacing 5.0
    python3 generate_graph.py --datum-lat 34.059270 --datum-lon -117.820934

Requirements:
    pip install osmnx networkx pyproj shapely
"""

import argparse
import json
import datetime
import math

import osmnx as ox
import networkx as nx
from pyproj import Transformer
from shapely.geometry import LineString


# CPP campus datum — must match navsat.yaml datum
DEFAULT_DATUM_LAT = 34.059270
DEFAULT_DATUM_LON = -117.820934

DEFAULT_PLACE = (
    'California State Polytechnic University Pomona, California, USA'
)


def gps_to_map(lat, lon, datum_lat, datum_lon, transformer):
    """Convert GPS (lat, lon) to map-frame (x, y) meters relative to datum."""
    east, north = transformer.transform(lon, lat)
    datum_east, datum_north = transformer.transform(datum_lon, datum_lat)
    x = east - datum_east
    y = north - datum_north
    return x, y


def build_graph(place, network_type='drive'):
    """Download OSMnx graph for the given place."""
    print(f'Downloading {network_type} network for: {place}')
    G = ox.graph_from_place(place, network_type=network_type, simplify=True)
    print(f'  Raw graph: {len(G.nodes)} nodes, {len(G.edges)} edges')
    return G


def densify_graph(G, spacing_meters=5.0):
    """Densify graph by inserting intermediate nodes along edge geometry.

    For each edge that has road curve geometry (from OSMnx), interpolates
    points at the specified spacing along the curve and inserts them as
    new graph nodes. The original edge is replaced with a chain of short
    edges through the new nodes.

    Edges without geometry (straight node-to-node) are kept if shorter
    than 2x spacing, otherwise densified as straight lines.

    Uses the same Shapely interpolation approach as AV2.1-API navigator.py.
    """
    # Project graph to metric CRS for accurate distance calculations
    G_proj = ox.project_graph(G)
    crs = G_proj.graph['crs']

    # Build new graph
    G_dense = nx.MultiDiGraph(**G_proj.graph)

    # Copy all original nodes (projected coordinates)
    for node, data in G_proj.nodes(data=True):
        G_dense.add_node(node, **data)

    next_node_id = max(G_proj.nodes) + 1
    stats = {'edges_original': 0, 'edges_densified': 0,
             'nodes_added': 0, 'edges_output': 0}

    for u, v, key, data in G_proj.edges(keys=True, data=True):
        stats['edges_original'] += 1

        # Build the full edge polyline in projected coordinates
        if 'geometry' in data:
            coords = list(data['geometry'].coords)
        else:
            coords = [
                (G_proj.nodes[u]['x'], G_proj.nodes[u]['y']),
                (G_proj.nodes[v]['x'], G_proj.nodes[v]['y']),
            ]

        line = LineString(coords)
        edge_length = line.length

        if edge_length < spacing_meters * 1.5:
            # Edge is short enough — keep as-is (no intermediate nodes needed)
            G_dense.add_edge(u, v, key=key, length=edge_length, **{
                k: val for k, val in data.items()
                if k not in ('geometry', 'length')
            })
            stats['edges_output'] += 1
            continue

        # Interpolate points along the road geometry
        stats['edges_densified'] += 1
        num_segments = max(int(math.ceil(edge_length / spacing_meters)), 1)
        actual_spacing = edge_length / num_segments

        # Generate chain: u → intermediate_1 → ... → intermediate_n → v
        chain = [u]
        for i in range(1, num_segments):
            dist = i * actual_spacing
            pt = line.interpolate(dist)

            new_id = next_node_id
            next_node_id += 1
            G_dense.add_node(new_id, x=pt.x, y=pt.y,
                             street_count=2)  # intermediate node
            chain.append(new_id)
            stats['nodes_added'] += 1

        chain.append(v)

        # Create edges along the chain
        extra_data = {k: val for k, val in data.items()
                      if k not in ('geometry', 'length')}
        for i in range(len(chain) - 1):
            n1, n2 = chain[i], chain[i + 1]
            x1, y1 = G_dense.nodes[n1]['x'], G_dense.nodes[n1]['y']
            x2, y2 = G_dense.nodes[n2]['x'], G_dense.nodes[n2]['y']
            seg_len = math.hypot(x2 - x1, y2 - y1)
            G_dense.add_edge(n1, n2, length=seg_len, **extra_data)
            stats['edges_output'] += 1

    print(f'  Densified: {stats["edges_original"]} edges → '
          f'{stats["edges_output"]} edges ({stats["edges_densified"]} split)')
    print(f'  Added {stats["nodes_added"]} intermediate nodes '
          f'({len(G_proj.nodes)} original + {stats["nodes_added"]} = '
          f'{len(G_dense.nodes)} total)')

    return G_dense


def graph_to_geojson(G_dense, datum_lat, datum_lon):
    """Convert a densified projected graph to nav2_route GeoJSON format.

    All edges are simple 2-point straight lines between adjacent nodes.
    nav2_route's path_density parameter handles sub-meter interpolation.
    """
    # Set up UTM transformer for datum offset
    utm_zone = int((datum_lon + 180) / 6) + 1
    hemisphere = 'north' if datum_lat >= 0 else 'south'
    epsg = 32600 + utm_zone if hemisphere == 'north' else 32700 + utm_zone

    # The densified graph is already in projected CRS — get its EPSG
    graph_crs = G_dense.graph.get('crs', f'EPSG:{epsg}')

    # Transformer from projected CRS to WGS84 for datum offset calculation
    transformer_to_wgs = Transformer.from_crs(
        str(graph_crs), 'EPSG:4326', always_xy=True)
    transformer_from_wgs = Transformer.from_crs(
        'EPSG:4326', str(graph_crs), always_xy=True)

    # Compute datum in projected coordinates
    datum_x, datum_y = transformer_from_wgs.transform(datum_lon, datum_lat)

    features = []

    # Map node IDs to sequential integers
    node_list = list(G_dense.nodes)
    node_to_seq = {nid: i for i, nid in enumerate(node_list)}

    # Nodes (Point features) — coordinates in map frame (offset from datum)
    for nid in node_list:
        data = G_dense.nodes[nid]
        seq_id = node_to_seq[nid]
        # Project coordinates are already in meters — offset from datum
        mx = data['x'] - datum_x
        my = data['y'] - datum_y

        features.append({
            'type': 'Feature',
            'properties': {
                'id': seq_id,
                'frame': 'map',
            },
            'geometry': {
                'type': 'Point',
                'coordinates': [round(mx, 3), round(my, 3)],
            },
        })

    # Edges (MultiLineString features) — simple 2-point straight lines
    edge_id = len(node_list)
    seen_edges = set()

    for u, v, data in G_dense.edges(data=True):
        # Deduplicate (multigraph may have parallel edges)
        edge_key = (node_to_seq[u], node_to_seq[v])
        if edge_key in seen_edges:
            continue
        seen_edges.add(edge_key)

        start_id = node_to_seq[u]
        end_id = node_to_seq[v]

        ux, uy = G_dense.nodes[u]['x'] - datum_x, G_dense.nodes[u]['y'] - datum_y
        vx, vy = G_dense.nodes[v]['x'] - datum_x, G_dense.nodes[v]['y'] - datum_y

        length = data.get('length', math.hypot(vx - ux, vy - uy))

        feature = {
            'type': 'Feature',
            'properties': {
                'id': edge_id,
                'startid': start_id,
                'endid': end_id,
                'cost': round(length, 2),
            },
            'geometry': {
                'type': 'MultiLineString',
                'coordinates': [[
                    [round(ux, 3), round(uy, 3)],
                    [round(vx, 3), round(vy, 3)],
                ]],
            },
        }

        # Speed limit metadata
        maxspeed = data.get('maxspeed')
        if maxspeed:
            if isinstance(maxspeed, list):
                maxspeed = maxspeed[0]
            try:
                speed_mph = float(str(maxspeed).replace(' mph', ''))
                speed_ms = speed_mph * 0.44704
                feature['properties']['metadata'] = {
                    'speed_limit': round(speed_ms, 2),
                }
            except (ValueError, TypeError):
                pass

        features.append(feature)
        edge_id += 1

    geojson = {
        'type': 'FeatureCollection',
        'name': 'graph',
        'crs': {
            'type': 'name',
            'properties': {'name': f'urn:ogc:def:crs:EPSG::{epsg}'},
        },
        'date_generated': datetime.datetime.now().strftime('%c'),
        'features': features,
    }

    return geojson


def main():
    parser = argparse.ArgumentParser(
        description='Generate nav2_route GeoJSON graph from OSMnx'
    )
    parser.add_argument(
        '--output', '-o',
        default='cpp_campus_graph.geojson',
        help='Output GeoJSON file path',
    )
    parser.add_argument(
        '--place',
        default=DEFAULT_PLACE,
        help='OSMnx place query',
    )
    parser.add_argument(
        '--network-type',
        default='drive',
        help='OSMnx network type (drive, bike, walk, all)',
    )
    parser.add_argument(
        '--spacing',
        type=float,
        default=5.0,
        help='Node spacing in meters along road curves (default: 5.0)',
    )
    parser.add_argument(
        '--datum-lat',
        type=float,
        default=DEFAULT_DATUM_LAT,
        help='Datum latitude (must match navsat.yaml)',
    )
    parser.add_argument(
        '--datum-lon',
        type=float,
        default=DEFAULT_DATUM_LON,
        help='Datum longitude (must match navsat.yaml)',
    )
    args = parser.parse_args()

    G = build_graph(args.place, args.network_type)
    G_dense = densify_graph(G, spacing_meters=args.spacing)
    geojson = graph_to_geojson(G_dense, args.datum_lat, args.datum_lon)

    num_nodes = sum(
        1 for f in geojson['features']
        if f['geometry']['type'] == 'Point'
    )
    num_edges = sum(
        1 for f in geojson['features']
        if f['geometry']['type'] == 'MultiLineString'
    )

    with open(args.output, 'w') as f:
        json.dump(geojson, f, indent=2)

    print(f'Wrote {args.output}: {num_nodes} nodes, {num_edges} edges')


if __name__ == '__main__':
    main()
