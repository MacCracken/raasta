//! NavMesh pathfinding example — bake a mesh and find a path.

use raasta::{NavMesh, Vec2};

fn main() {
    // Bake a navmesh from an L-shaped boundary
    let boundary = vec![
        Vec2::new(0.0, 0.0),
        Vec2::new(20.0, 0.0),
        Vec2::new(20.0, 10.0),
        Vec2::new(10.0, 10.0),
        Vec2::new(10.0, 20.0),
        Vec2::new(0.0, 20.0),
    ];
    let mesh = NavMesh::bake(&boundary);
    println!("Baked navmesh: {} polygons", mesh.poly_count());

    // Find a path
    let start = Vec2::new(5.0, 5.0);
    let goal = Vec2::new(5.0, 15.0);

    match mesh.find_path(start, goal) {
        Some(path) => {
            println!("Path found through {} polygons:", path.len());
            for (i, id) in path.iter().enumerate() {
                if let Some(poly) = mesh.get_poly(*id) {
                    let c = poly.centroid();
                    println!("  [{i}] poly {} — centroid ({:.1}, {:.1})", id.0, c.x, c.y);
                }
            }
        }
        None => println!("No path found!"),
    }

    // Serialize and deserialize
    let bytes = mesh.to_bytes();
    println!("\nSerialized: {} bytes", bytes.len());
    let loaded = NavMesh::from_bytes(&bytes).unwrap();
    println!("Deserialized: {} polygons", loaded.poly_count());
}
