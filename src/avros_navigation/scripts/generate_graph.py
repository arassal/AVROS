#!/usr/bin/env python3
"""Offline graph generation: OSMnx campus roads → nav2_route GeoJSON.

Downloads the CPP campus road network via OSMnx, converts GPS coordinates
to map-frame (meters relative to a fixed datum) using UTM projection,
and writes a GeoJSON file compatible with nav2_route's GeoJsonGraphFileLoader.

Usage:
    python3 generate_graph.py
    python3 generate_graph.py --output /path/to/output.geojson
    python3 generate_graph.py --datum-lat 34.059270 --datum-lon -117.820934

Requirements:
    pip install osmnx networkx pyproj
"""

import argparse
import json
import datetime

import osmnx as ox
import networkx as nx
from pyproj import Transformer


# CPP campus datum — must match navsat.yaml datum
DEFAULT_DATUM_LAT = 34.059270
DEFAULT_DATUM_LON = -117.820934

DEFAULT_PLACE = (
    'California State Polytechnic University Pomona, California, USA'
)


def gps_to_map(lat, lon, datum_lat, datum_lon, transformer):
    """Convert GPS (lat, lon) to map-frame (x, y) meters relative to datum."""
    # UTM coordinates of the point
    east, north = transformer.transform(lon, lat)
    # UTM coordinates of the datum
    datum_east, datum_north = transformer.transform(datum_lon, datum_lat)
    # Map frame = offset from datum
    x = east - datum_east
    y = north - datum_north
    return x, y


def build_graph(place, network_type='drive'):
    """Download OSMnx graph for the given place."""
    print(f'Downloading {network_type} network for: {place}')
    G = ox.graph_from_place(place, network_type=network_type, simplify=True)
    print(f'  Raw graph: {len(G.nodes)} nodes, {len(G.edges)} edges')
    return G


def graph_to_geojson(G, datum_lat, datum_lon):
    """Convert an OSMnx graph to nav2_route GeoJSON format.

    Nodes become Point features, edges become MultiLineString features.
    Coordinates are in map frame (meters from datum).
    """
    # Set up UTM transformer
    # Determine UTM zone from datum longitude
    utm_zone = int((datum_lon + 180) / 6) + 1
    hemisphere = 'north' if datum_lat >= 0 else 'south'
    epsg = 32600 + utm_zone if hemisphere == 'north' else 32700 + utm_zone
    transformer = Transformer.from_crs('EPSG:4326', f'EPSG:{epsg}', always_xy=True)

    features = []

    # Map OSMnx node IDs to sequential integers
    osm_to_seq = {}
    for i, osm_id in enumerate(G.nodes):
        osm_to_seq[osm_id] = i

    # Nodes (Point features)
    for osm_id, data in G.nodes(data=True):
        seq_id = osm_to_seq[osm_id]
        lat = data['y']
        lon = data['x']
        mx, my = gps_to_map(lat, lon, datum_lat, datum_lon, transformer)

        feature = {
            'type': 'Feature',
            'properties': {
                'id': seq_id,
                'frame': 'map',
            },
            'geometry': {
                'type': 'Point',
                'coordinates': [round(mx, 3), round(my, 3)],
            },
        }
        features.append(feature)

    # Edges (MultiLineString features)
    edge_id = len(G.nodes)  # Start edge IDs after node IDs
    for u, v, data in G.edges(data=True):
        start_id = osm_to_seq[u]
        end_id = osm_to_seq[v]

        # Get edge geometry (intermediate points)
        if 'geometry' in data:
            coords_gps = list(data['geometry'].coords)
        else:
            # Straight line between nodes
            coords_gps = [
                (G.nodes[u]['x'], G.nodes[u]['y']),
                (G.nodes[v]['x'], G.nodes[v]['y']),
            ]

        # Convert all coordinates to map frame
        coords_map = []
        for lon, lat in coords_gps:
            mx, my = gps_to_map(lat, lon, datum_lat, datum_lon, transformer)
            coords_map.append([round(mx, 3), round(my, 3)])

        # Edge length in meters
        length = data.get('length', 0.0)

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
                'coordinates': [coords_map],
            },
        }

        # Add speed limit metadata if available
        maxspeed = data.get('maxspeed')
        if maxspeed:
            if isinstance(maxspeed, list):
                maxspeed = maxspeed[0]
            try:
                # Convert mph string to m/s
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
    geojson = graph_to_geojson(G, args.datum_lat, args.datum_lon)

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
