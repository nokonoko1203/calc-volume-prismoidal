use delaunator::{triangulate, Point};
use nalgebra::{Matrix3, RowVector3, Vector2, Vector3};

struct Polygon {
    vertices: Vec<Vector3<f32>>,
    indices: Vec<Vec<i32>>,
}

impl Polygon {
    fn new(vertices: Vec<Vector3<f32>>, indices: Vec<Vec<i32>>) -> Self {
        Polygon { vertices, indices }
    }

    fn create_triangles(&mut self) {
        // すべての頂点を利用して、ドロネー三角分割を行い、三角形のインデックスを生成する
        // 土量計算では、TINは比較的平坦になるため、今回の計算では利用可能
        let points: Vec<Point> = self
            .vertices
            .iter()
            .map(|v| Point {
                x: v.x as f64,
                y: v.y as f64,
            })
            .collect();

        let triangulation = triangulate(&points);

        let mut indices = Vec::new();
        for i in 0..triangulation.triangles.len() / 3 {
            let triangle = [
                triangulation.triangles[i * 3] as i32,
                triangulation.triangles[i * 3 + 1] as i32,
                triangulation.triangles[i * 3 + 2] as i32,
            ];
            indices.push(triangle.to_vec());
        }

        self.indices = indices;
    }

    fn project_vertices(&mut self, other: &Polygon) {
        // otherの頂点を、自身の頂点に投影する処理を行う
        let mut projected_vertices = Vec::new();

        // otherの頂点を取り出す
        for vertex in &other.vertices {
            // 自身の頂点に取り出された頂点と、同じXY平面上に点が存在している場合は投影の必要がない
            let mut found = false;
            for self_vertex in &self.vertices {
                if vertex.x == self_vertex.x && vertex.y == self_vertex.y {
                    found = true;
                    break;
                }
            }
            if !found {
                for triangle_indices in &self.indices {
                    // 自身の三角形を構築し、project_point_onto_triangle関数でotherの頂点を投影
                    let triangle = [
                        self.vertices[triangle_indices[0] as usize],
                        self.vertices[triangle_indices[1] as usize],
                        self.vertices[triangle_indices[2] as usize],
                    ];
                    if let Some(projected_vertex) = project_point_onto_triangle(vertex, &triangle) {
                        projected_vertices.push(projected_vertex);
                        break;
                    }
                }
            }
        }

        self.vertices.extend(projected_vertices);
    }

    fn find_edge_intersections(&mut self, other: &mut Polygon) {
        // 自身とotherの三角形の辺同士をz軸に平行移動した場合に、交点が見つかるかどうか確認し、交点があれば新たな頂点として格納
        let mut intersection_vertices_self = Vec::new();
        let mut intersection_vertices_other = Vec::new();

        // 自身の最初の三角形を取り出す
        for i in 0..self.indices.len() {
            // 頂点から線分を取り出す
            for j in 0..3 {
                let a = self.vertices[self.indices[i][j] as usize];
                let b = self.vertices[self.indices[i][(j + 1) % 3] as usize];

                // otherの線分も同様に取り出す
                for k in 0..other.indices.len() {
                    for l in 0..3 {
                        let c = other.vertices[other.indices[k][l] as usize];
                        let d = other.vertices[other.indices[k][(l + 1) % 3] as usize];

                        // 線分同士が交差するか確認する
                        // この時、互いにz軸に並行に移動することができる
                        if let Some((intersection_ab, intersection_cd)) =
                            line_segment_intersection_z(&a, &b, &c, &d)
                        {
                            if !self.vertices.contains(&intersection_ab) {
                                intersection_vertices_self.push(intersection_ab);
                            }

                            if !other.vertices.contains(&intersection_cd) {
                                intersection_vertices_other.push(intersection_cd);
                            }
                        }
                    }
                }
            }
        }

        self.vertices.extend(intersection_vertices_self);
        other.vertices.extend(intersection_vertices_other);
    }

    fn remove_duplicate_vertices(&mut self) {
        let mut unique_vertices = Vec::new();
        let mut vertex_indices = Vec::new();

        for vertex in &self.vertices {
            if let Some(index) = unique_vertices.iter().position(|v| v == vertex) {
                vertex_indices.push(index);
            } else {
                vertex_indices.push(unique_vertices.len());
                unique_vertices.push(*vertex);
            }
        }

        let mut new_indices = Vec::new();
        for triangle_indices in &self.indices {
            let mut new_triangle_indices = Vec::new();
            for &index in triangle_indices {
                let new_index = vertex_indices[index as usize] as i32;
                new_triangle_indices.push(new_index);
            }
            new_indices.push(new_triangle_indices);
        }

        self.vertices = unique_vertices;
        self.indices = new_indices;
    }
}

fn project_point_onto_triangle(
    point: &Vector3<f32>,
    triangle: &[Vector3<f32>; 3],
) -> Option<Vector3<f32>> {
    // 三角形から頂点3つを取り出す
    let v0 = triangle[0];
    let v1 = triangle[1];
    let v2 = triangle[2];

    // x座標が最も小さい座標から、そうではない2つの頂点へのベクトル
    let u = v1 - v0;
    let v = v2 - v0;

    // 2つのベクトルから外積を算出し、法線ベクトルとする
    let n = u.cross(&v);

    // 三角形の平面の方程式（ax + by + cz + d = 0）からdを求める
    let a = n.x;
    let b = n.y;
    let c = n.z;
    // 平面と投影させたい点の符号付き距離を表し、d = 0なら平面上にある
    let d = -a * v0.x - b * v0.y - c * v0.z;

    // z軸に平行な直線と平面の交点を求める
    // 三角形の法線ベクトルの座標が0の場合、三角形がz軸に平行
    if c.abs() < f32::EPSILON {
        // 平面がz軸に平行な場合は投影できない
        None
    } else {
        // Pointはz軸に対して垂直にのみ移動させたい
        // つまり、方向ベクトルは(0,0,1)もしくは(0,0,-1)
        // 例えばPointが(x,y,10)であった場合、投影後のPointが取りうる値はXY平面まで垂直に下ろした(x,y,0)との直線の方程式になる
        // z軸上に平行に移動するため、xとyの値は固定になり、zは任意のパラメータtを取りうる
        // 直線と平面の交点を求めるために、直線の方程式を平面の方程式に代入する
        let t = -(a * point.x + b * point.y + d) / c;
        // 平面に投影されたポイント
        let projected_point = Vector3::new(point.x, point.y, t);

        // 内部判定
        // 平面には投影されたが、点が三角形の内部にあるかは不明のため、全ての頂点と投影された点とのベクトルの外積を計算する
        let v0_to_point = projected_point - v0;
        let v1_to_point = projected_point - v1;
        let v2_to_point = projected_point - v2;

        // 3つの頂点から生成されるベクトルの外積を求める
        let cross1 = u.cross(&v0_to_point);
        let cross2 = v.cross(&v1_to_point);
        let cross3 = (v2 - v1).cross(&v2_to_point);

        // 全ての外積のz成分が同じなら、外積のベクトルは全て同じ方向を向く
        if cross1.z.signum() == cross2.z.signum() && cross2.z.signum() == cross3.z.signum() {
            // 投影された点が三角形のXY平面上の範囲内にあるかどうかを確認
            let min_x = v0.x.min(v1.x).min(v2.x);
            let max_x = v0.x.max(v1.x).max(v2.x);
            let min_y = v0.y.min(v1.y).min(v2.y);
            let max_y = v0.y.max(v1.y).max(v2.y);

            if projected_point.x >= min_x
                && projected_point.x <= max_x
                && projected_point.y >= min_y
                && projected_point.y <= max_y
            {
                Some(projected_point)
            } else {
                None
            }
        } else {
            None
        }
    }
}

fn calculate_volume(polygon1: &mut Polygon, polygon2: &Polygon) -> f32 {
    let mut volume = 0.0;

    polygon1.create_triangles();

    for triangle_indices in &polygon1.indices {
        let v1 = polygon1.vertices[triangle_indices[0] as usize];
        let v2 = polygon1.vertices[triangle_indices[1] as usize];
        let v3 = polygon1.vertices[triangle_indices[2] as usize];

        // 三角形の面積を計算
        let v1_rowvec = RowVector3::new(v1[0], v1[1], 1.0);
        let v2_rowvec = RowVector3::new(v2[0], v2[1], 1.0);
        let v3_rowvec = RowVector3::new(v3[0], v3[1], 1.0);
        let area = Matrix3::from_rows(&[v1_rowvec, v2_rowvec, v3_rowvec])
            .determinant()
            .abs()
            * 0.5;

        // 三角形の重心点を計算
        let centroid = (v1 + v2 + v3) / 3.0;

        // 重心点をpolygon2の面に投影
        let mut projected_centroid = None;
        for other_triangle_indices in &polygon2.indices {
            let other_triangle = [
                polygon2.vertices[other_triangle_indices[0] as usize],
                polygon2.vertices[other_triangle_indices[1] as usize],
                polygon2.vertices[other_triangle_indices[2] as usize],
            ];
            if let Some(projected_point) = project_point_onto_triangle(&centroid, &other_triangle) {
                projected_centroid = Some(projected_point);
                break;
            }
        }

        // 重心点が投影できた場合のみ体積を計算
        if let Some(projected_point) = projected_centroid {
            let height = (projected_point.z - centroid.z).abs();
            let prism_volume = area * height;
            volume += prism_volume;
        }
    }

    volume
}

fn line_segment_intersection_z(
    a: &Vector3<f32>,
    b: &Vector3<f32>,
    c: &Vector3<f32>,
    d: &Vector3<f32>,
) -> Option<(Vector3<f32>, Vector3<f32>)> {
    // 線分ABと線分CDをxy平面に投影（z座標を除去してベクトルを計算する）
    let ab = Vector2::new(b.x - a.x, b.y - a.y);
    let cd = Vector2::new(d.x - c.x, d.y - c.y);
    // 線分ABと線分CDのどちらか一方の端点を結ぶベクトル
    let ac = Vector2::new(c.x - a.x, c.y - a.y);

    // xy平面上の線分の交差判定
    // perpは外積を計算して、そのベクトルの大きさを算出
    let det = ab.perp(&cd);
    // f32::EPSILONは0に非常に近い値で、許容誤差を持たせて比較している
    // 外積の大きさが0（に非常に近い）場合、ベクトルが平行になっているため交点がない
    if det.abs() < f32::EPSILON {
        return None;
    }

    // tとuはベクトル上の位置を示すパラメータ
    // 交点Pが存在するとき、点PまでのベクトルAPやCPはABやCDと同じ方向を向く
    // tが1であればABの1倍なので、同じ大きさになり、APとABは一致する
    // 逆に0であれば、ABの0倍なので、長さが0になり頂点aとPの座標が一致する
    // なので、交点Pが存在している場合はtやuは「0 <= t or u <= 1」となる
    let t = ac.perp(&cd) / det;
    let u = ac.perp(&ab) / det;

    // パラメータtとuが0から1の範囲内にあるかを確認
    // 範囲外の場合は、交点が線分上にないため、Noneを返す
    if t < 0.0 || t > 1.0 || u < 0.0 || u > 1.0 {
        return None;
    }

    // 線分AB上の交点のx座標とy座標を計算
    let intersection_ab_x = a.x + t * ab.x;
    let intersection_ab_y = a.y + t * ab.y;

    // 線分CD上の交点のx座標とy座標を計算
    let intersection_cd_x = c.x + u * cd.x;
    let intersection_cd_y = c.y + u * cd.y;

    // 2次元から3次元に戻す際に、交点のz座標は線形補間で算出
    let intersection_ab_z = a.z + t * (b.z - a.z);
    let intersection_cd_z = c.z + u * (d.z - c.z);

    let intersection_ab = Vector3::new(intersection_ab_x, intersection_ab_y, intersection_ab_z);
    let intersection_cd = Vector3::new(intersection_cd_x, intersection_cd_y, intersection_cd_z);

    Some((intersection_ab, intersection_cd))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_simple_triangle() {
        let vertices1 = vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(1.0, 1.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
        ];
        let indices1 = vec![vec![0, 1, 2], vec![0, 2, 3]];
        let mut polygon1 = Polygon::new(vertices1, indices1);

        let vertices2 = vec![
            Vector3::new(0.0, 0.0, 1.0),
            Vector3::new(1.0, 0.0, 1.0),
            Vector3::new(1.0, 1.0, 1.0),
            Vector3::new(0.0, 1.0, 1.0),
        ];
        let indices2 = vec![vec![0, 1, 2], vec![0, 2, 3]];
        let mut polygon2 = Polygon::new(vertices2, indices2);

        polygon1.project_vertices(&polygon2);
        polygon2.project_vertices(&polygon1);

        polygon1.find_edge_intersections(&mut polygon2);

        polygon1.remove_duplicate_vertices();
        polygon2.remove_duplicate_vertices();

        let volume = calculate_volume(&mut polygon1, &polygon2);
        assert_eq!(volume, 1.0);
    }

    #[test]
    fn test_projected_triangle() {
        let vertices1 = vec![
            Vector3::new(0., 0., 0.),
            Vector3::new(2., 0., 0.),
            Vector3::new(2., 2., 0.),
            Vector3::new(0., 2., 0.),
        ];
        let indices1 = vec![vec![0, 1, 2], vec![0, 2, 3]];
        let mut polygon1 = Polygon::new(vertices1, indices1);

        let vertices2 = vec![
            Vector3::new(1., 1., 1.),
            Vector3::new(3., 1., 1.),
            Vector3::new(3., 3., 1.),
            Vector3::new(1., 3., 1.),
        ];
        let indices2 = vec![vec![0, 1, 2], vec![0, 2, 3]];
        let mut polygon2 = Polygon::new(vertices2, indices2);

        polygon1.project_vertices(&polygon2);
        polygon2.project_vertices(&polygon1);

        polygon1.find_edge_intersections(&mut polygon2);

        polygon1.remove_duplicate_vertices();
        polygon2.remove_duplicate_vertices();

        let volume = calculate_volume(&mut polygon1, &polygon2);
        assert_eq!(volume, 1.0);
    }

    #[test]
    fn test_project_point_onto_triangle() {
        // Case 1: 点が三角形の内部にある場合
        let point = Vector3::new(0.5, 0.5, 1.0);
        let triangle = [
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
        ];
        let expected_projection = Some(Vector3::new(0.5, 0.5, 0.0));
        assert_eq!(
            project_point_onto_triangle(&point, &triangle),
            expected_projection
        );

        // Case 2: 点が三角形の外部にある場合
        let point = Vector3::new(1.5, 1.5, 1.0);
        let triangle = [
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
        ];
        let expected_projection = None;
        assert_eq!(
            project_point_onto_triangle(&point, &triangle),
            expected_projection
        );

        // Case 3: 点が三角形の辺上にある場合
        let point = Vector3::new(0.5, 0.0, 1.0);
        let triangle = [
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
        ];
        let expected_projection = Some(Vector3::new(0.5, 0.0, 0.0));
        assert_eq!(
            project_point_onto_triangle(&point, &triangle),
            expected_projection
        );

        // Case 4: 点が三角形の頂点上にある場合
        let point = Vector3::new(0.0, 0.0, 1.0);
        let triangle = [
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
        ];
        let expected_projection = Some(Vector3::new(0.0, 0.0, 0.0));
        assert_eq!(
            project_point_onto_triangle(&point, &triangle),
            expected_projection
        );

        // Case 5: 三角形がXY平面に平行な場合
        let point = Vector3::new(0.5, 0.5, 1.0);
        let triangle = [
            Vector3::new(0.0, 0.0, 1.0),
            Vector3::new(1.0, 0.0, 1.0),
            Vector3::new(0.0, 1.0, 1.0),
        ];
        let expected_projection = Some(Vector3::new(0.5, 0.5, 1.0));
        assert_eq!(
            project_point_onto_triangle(&point, &triangle),
            expected_projection
        );

        // Case 6: 点が三角形の外部にある場合
        let point = Vector3::new(0.0, 0.0, 0.0);
        let triangle = [
            Vector3::new(1.0, 1.0, 1.0),
            Vector3::new(3.0, 3.0, 1.0),
            Vector3::new(1.0, 3.0, 1.0),
        ];
        let expected_projection = None;
        assert_eq!(
            project_point_onto_triangle(&point, &triangle),
            expected_projection
        );
    }

    #[test]
    fn test_find_intersection_with_z_translation() {
        // Case 1: 交点が存在する場合
        let a1 = Vector3::new(0.0, 0.0, 0.0);
        let b1 = Vector3::new(2.0, 2.0, 0.0);
        let c1 = Vector3::new(0.0, 2.0, 1.0);
        let d1 = Vector3::new(2.0, 0.0, 1.0);
        let expected_intersection1_ab = Vector3::new(1.0, 1.0, 0.0);
        let expected_intersection1_cd = Vector3::new(1.0, 1.0, 1.0);
        assert_eq!(
            line_segment_intersection_z(&a1, &b1, &c1, &d1),
            Some((expected_intersection1_ab, expected_intersection1_cd))
        );

        // Case 2: 交点が存在しない場合（平行な線分）
        let a2 = Vector3::new(0.0, 0.0, 0.0);
        let b2 = Vector3::new(2.0, 0.0, 0.0);
        let c2 = Vector3::new(0.0, 1.0, 1.0);
        let d2 = Vector3::new(2.0, 1.0, 1.0);
        assert_eq!(line_segment_intersection_z(&a2, &b2, &c2, &d2), None);

        // Case 3: 交点が存在しない場合（XY平面上で交わっていない）
        let a3 = Vector3::new(0.0, 0.0, 0.0);
        let b3 = Vector3::new(2.0, 0.0, 0.0);
        let c3 = Vector3::new(3.0, 1.0, 1.0);
        let d3 = Vector3::new(5.0, 1.0, 1.0);
        assert_eq!(line_segment_intersection_z(&a3, &b3, &c3, &d3), None);

        // Case 4: 交点が線分の端点である場合
        let a4 = Vector3::new(0.0, 0.0, 0.0);
        let b4 = Vector3::new(2.0, 0.0, 0.0);
        let c4 = Vector3::new(2.0, 0.0, 1.0);
        let d4 = Vector3::new(4.0, 4.0, 1.0);
        let expected_intersection4_ab = Vector3::new(2.0, 0.0, 0.0);
        let expected_intersection4_cd = Vector3::new(2.0, 0.0, 1.0);
        assert_eq!(
            line_segment_intersection_z(&a4, &b4, &c4, &d4),
            Some((expected_intersection4_ab, expected_intersection4_cd))
        );
    }
}
