// One entry point that builds everything physical about the cave from CaveConfiguration.json:
// network -> signed-distance field -> floor heightfield -> (optionally) the render mesh.
import { buildNetwork } from './CaveTunnelNetwork.js';
import { buildField, buildFloorMap } from './CaveField.js';
import { extractSurface } from './CaveMesh.js';
import { solidBounds } from './CaveTunnelNetwork.js';

export function buildCaveWorld({ withMesh = false } = {}) {
  const network = buildNetwork();
  const field = buildField(network);
  const floor = buildFloorMap(field, network);
  const mesh = withMesh ? extractSurface(field) : null;
  return { network, field, floor, mesh, solids: network.solids, bounds: network.cfg.meta.bounds, config: network.cfg };
}

export { solidBounds };

/** Structured, Gazebo-friendly description of the cave (tunnel graph, formations with bounds, landmarks, sensor frames). */
export function exportCaveJSON(world) {
  const net = world.network, cfg = world.config;
  return {
    meta: { ...cfg.meta, generator: 'signed-distance field + surface nets; rebuild the same field in Gazebo or export the mesh', voxel: world.field.voxel },
    robot: cfg.robot,
    nodes: cfg.nodes,
    tunnels: cfg.tunnels.map((t) => ({ ...t, lengthMetres: +net.tunnels.find((x) => x.id === t.id).length.toFixed(1) })),
    graph: { intersections: net.intersections, deadEnds: net.deadEnds, loops: net.loops },
    formations: net.solids.map((s) => ({ ...s, bounds: solidBounds(s) })),
    landmarks: net.landmarks,
    lamps: cfg.lamps,
    semanticCategories: ['floor', 'wall', 'ceiling', 'pillar', 'boulder', 'stalactite'],
  };
}
