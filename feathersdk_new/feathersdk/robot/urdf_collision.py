"""The URDF's collision geometry read without pinocchio's mesh loader, for the arms' collision check.

Pinocchio loads meshes in C++ holding the GIL: 11.6 s for the robot's collision meshes on the Orin, during which no
other thread of the robot's process runs, and the Teensy drops a client it hears nothing from for 5 s.
Here a binary STL is one `np.frombuffer` (0.2 s for the robot's meshes) and every later step is a numpy call
of at most a fraction of a second, so the comms thread keeps its turns. Pure functions of files and arrays: the
geometry models are built from them in `URDFLoader`, on pinocchio's frames, and tested against pinocchio's own."""
import struct
import xml.etree.ElementTree as ElementTree
from pathlib import Path

import numpy as np

from feathersdk.utils.feathertypes import List, NamedTuple, Optional, Sequence, Tuple

_STL_TRIANGLE = np.dtype([("normal", "<f4", (3,)), ("corners", "<f4", (3, 3)), ("attribute", "<u2")])
"""One triangle of a binary STL file, after its 80-byte header and the triangle count."""


class CollisionElement(NamedTuple):
    """One `<collision>` of the URDF, as pinocchio's URDF parser reads it."""
    name: str
    """Pinocchio's name for it: the link's name and the element's index among that link's collisions (`base_link_0`)."""
    link: str
    xyz: np.ndarray
    """The element's origin in its link's frame (m)."""
    rpy: np.ndarray
    """The origin's roll, pitch, yaw (rad)."""
    mesh_path: Optional[str]
    """The resolved file of a mesh; None for a primitive."""
    scale: np.ndarray
    """A mesh's scale per axis; ones for a primitive."""
    primitive: Optional[Tuple[str, Tuple[float, ...]]]
    """("box", (x, y, z)), ("cylinder", (radius, length)) or ("sphere", (radius,)); None for a mesh."""


def _floats(text: Optional[str], default: Sequence[float]) -> np.ndarray:
    return np.array([float(v) for v in text.split()]) if text else np.array(default, dtype=float)


def resolve_mesh_path(filename: str, package_dirs: Sequence[str]) -> str:
    """The file a URDF mesh `filename` names, as pinocchio resolves it: `package://<package>/<path>` and relative paths
    in `package_dirs`, in order; `file://` and absolute paths as they are. Raises FileNotFoundError naming every place
    looked."""
    if filename.startswith("file://"):
        filename = filename[len("file://"):]
    relative = filename[len("package://"):] if filename.startswith("package://") else filename
    if Path(relative).is_absolute():
        candidates = [Path(relative)]
    else:
        candidates = [Path(d) / relative for d in package_dirs]
    for candidate in candidates:
        if candidate.is_file():
            return str(candidate)
    raise FileNotFoundError(f"mesh {filename!r} not found; looked at {', '.join(map(str, candidates))}")


def collision_elements(urdf_path: str, package_dirs: Sequence[str]) -> List[CollisionElement]:
    """Every `<collision>` of the URDF at `urdf_path`, in file order, its mesh resolved in `package_dirs`."""
    elements = []
    for link in ElementTree.parse(urdf_path).getroot().iter("link"):
        for index, collision in enumerate(link.findall("collision")):
            origin = collision.find("origin")
            xyz = _floats(origin.get("xyz") if origin is not None else None, (0.0, 0.0, 0.0))
            rpy = _floats(origin.get("rpy") if origin is not None else None, (0.0, 0.0, 0.0))
            geometry = collision.find("geometry")
            shape = geometry[0] if geometry is not None and len(geometry) else None
            if shape is None:
                raise ValueError(f"link {link.get('name')!r}: a <collision> without a geometry")
            mesh_path, scale, primitive = None, np.ones(3), None
            if shape.tag == "mesh":
                mesh_path = resolve_mesh_path(shape.get("filename", ""), package_dirs)
                scale = _floats(shape.get("scale"), (1.0, 1.0, 1.0))
            elif shape.tag == "box":
                primitive = ("box", tuple(_floats(shape.get("size"), ())))
            elif shape.tag == "cylinder":
                primitive = ("cylinder", (float(shape.get("radius")), float(shape.get("length"))))
            elif shape.tag == "sphere":
                primitive = ("sphere", (float(shape.get("radius")),))
            else:
                raise ValueError(f"link {link.get('name')!r}: collision geometry <{shape.tag}> is not supported")
            elements.append(CollisionElement(f"{link.get('name')}_{index}", link.get("name"), xyz, rpy, mesh_path,
                                             scale, primitive))
    return elements


def _binary_stl_corners(data: bytes) -> Optional[np.ndarray]:
    """A binary STL file's corners as stored, (triangles * 3, 3) float32; None if `data` isn't one (its size must be
    the header, the count and 50 bytes a triangle)."""
    if len(data) < 84:
        return None
    count = struct.unpack_from("<I", data, 80)[0]
    if 84 + count * _STL_TRIANGLE.itemsize != len(data):
        return None
    return np.frombuffer(data, _STL_TRIANGLE, count, 84)["corners"].reshape(-1, 3)


def _ascii_stl_corners(data: bytes) -> np.ndarray:
    text = data.decode("latin1", errors="ignore")
    return np.array([[float(v) for v in line.split()[1:4]] for line in text.splitlines()
                     if line.strip().startswith("vertex")], dtype=np.float64).reshape(-1, 3)


def stl_corners(path: str) -> np.ndarray:
    """Every triangle's three corners in order, (triangles * 3, 3) float64 in the file's units, from a binary or ASCII
    STL file."""
    data = Path(path).read_bytes()
    corners = _binary_stl_corners(data)
    return corners.astype(np.float64) if corners is not None else _ascii_stl_corners(data)


def unique_rows(cells: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """(the distinct rows of an integer (n, k) array in lexicographic order, each row's index among them): what
    `np.unique(cells, axis=0, return_inverse=True)` gives, sorting one packed int64 key instead of rows (several
    times faster on millions of rows). Rows whose ranges don't pack into 63 bits go through `np.unique` itself."""
    if len(cells) == 0:  # an empty mesh: nothing to merge
        return cells.copy(), np.zeros(0, np.int64)
    low = cells.min(0)
    span = cells.max(0) - low + 1
    if np.sum(np.log2(span.astype(np.float64))) >= 63:  # tens of metres at 10 um: same answer, slower
        distinct, inverse = np.unique(cells, axis=0, return_inverse=True)
        return distinct, inverse.ravel()
    key = np.zeros(len(cells), np.int64)
    for column, size in zip((cells - low).T, span):
        key = key * size + column
    _, first, inverse = np.unique(key, return_index=True, return_inverse=True)
    return cells[first], inverse.ravel()


def mesh_vertices_triangles(path: str, scale: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """A mesh file as (vertices (n, 3) m, triangles (m, 3) indices), each position once, as pinocchio's loader
    (assimp) joins them. STL through numpy; any other format through coal's loader, one call per triangle."""
    if path.lower().endswith(".stl"):
        data = Path(path).read_bytes()
        corners = _binary_stl_corners(data)
        if corners is not None:
            # Identical positions are identical float32 bytes (after + 0: -0.0 is 0.0): one 12-byte value a corner,
            # sorted once, instead of rows compared column by column.
            bytes_per_corner = np.ascontiguousarray(corners + np.float32(0.0)).view(np.dtype((np.void, 12))).ravel()
            distinct, inverse = np.unique(bytes_per_corner, return_inverse=True)
            positions = distinct.view(np.float32).reshape(-1, 3).astype(np.float64)
        else:
            positions, inverse = np.unique(_ascii_stl_corners(data) + 0.0, axis=0, return_inverse=True)
        return positions * scale, inverse.ravel().reshape(-1, 3)
    import coal  # installed with pinocchio
    mesh = coal.MeshLoader().load(path, np.asarray(scale, dtype=float))
    triangles = np.array([[t[0], t[1], t[2]] for t in map(mesh.tri_indices, range(mesh.num_tris))], np.int64)
    return np.asarray(mesh.vertices()), triangles.reshape(-1, 3)


def cluster(vertices: np.ndarray, triangles: np.ndarray, cell: float) -> Tuple[np.ndarray, np.ndarray]:
    """The mesh simplified by vertex clustering: the vertices within each `cell` cube merge into one at their mean,
    so each moves at most the cube's diagonal and a concave shape stays concave; triangles whose corners merged are
    dropped, and each remaining one is kept once. Returns (points, triangles)."""
    cells, merged = unique_rows(np.round(vertices / cell).astype(np.int64))
    points = np.zeros((len(cells), 3))
    for axis in range(3):  # each cell's mean: the sum of its vertices over their count
        points[:, axis] = np.bincount(merged, vertices[:, axis], len(cells))
    points /= np.maximum(np.bincount(merged, minlength=len(cells)), 1)[:, None]  # every cell has a vertex
    corners = merged[triangles]
    corners = corners[(corners[:, 0] != corners[:, 1]) & (corners[:, 1] != corners[:, 2])
                      & (corners[:, 0] != corners[:, 2])]
    kept, _ = unique_rows(np.sort(corners, axis=1)) if len(corners) else (corners, None)
    return points, kept


def hull_points(vertices: np.ndarray) -> np.ndarray:
    """Each vertex once, to 10 um (the values `np.unique(vertices.round(5), axis=0)` gives): the points a convex hull
    is built from, since a mesh repeats its vertices across triangles."""
    rows, _ = unique_rows(np.rint(vertices * 1e5).astype(np.int64))
    return rows / 1e5
