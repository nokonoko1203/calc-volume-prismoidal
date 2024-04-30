//! ```cargo
//! [dependencies]
//! plotters = "0.3.5"
//! ```

extern crate plotters;

use plotters::prelude::*;

const OUT_FILE_NAME: &str = "images/triangle.png";

// `$ cargo script src/triangle.rs`のように利用します
fn main() -> Result<(), Box<dyn std::error::Error>> {
    // 描画先をBackendとして指定
    // 画像に出力するにはBitMapBackendを選択
    let area = BitMapBackend::new(OUT_FILE_NAME, (1024, 760)).into_drawing_area();

    // 背景を塗る
    area.fill(&WHITE)?;

    // 可視化される範囲と、表示されるグリッドを決める
    let x_axis = (-3.0..3.0).step(0.1);
    let z_axis = (-3.0..3.0).step(0.1);

    // グラフのキャプションなど設定
    let mut chart = ChartBuilder::on(&area)
        .caption("3D Polygon Plot", ("sans-serif", 40))
        .build_cartesian_3d(-5..5, -5..5, -1..4)?;

    // 表示される空間を傾けたり拡大縮小したり回転させたり
    chart.with_projection(|mut pb| {
        pb.yaw = 0.5;
        pb.scale = 0.9;
        pb.into_matrix()
    });

    chart
        .configure_axes()
        .light_grid_style(BLACK.mix(0.15))
        .max_light_lines(3)
        .draw()?;

    // 頂点を描画していく
    let vertices1 = vec![[0., 0., 0.], [2., 0., 0.], [2., 2., 0.], [0., 2., 0.]];
    let indices1 = vec![[0, 1, 2], [0, 2, 3]];

    chart.draw_series(indices1.iter().map(|idx| {
        let points: Vec<_> = idx
            .iter()
            .map(|&i| {
                let p = vertices1[i as usize];
                (p[0] as i32, p[1] as i32, p[2] as i32)
            })
            .collect();
        Polygon::new(points, &RED.mix(0.5))
    }))?;

    let vertices2 = vec![[1., 1., 1.], [3., 1., 1.], [3., 3., 1.], [1., 3., 1.]];
    let indices2 = vec![[0, 1, 2], [0, 2, 3]];

    chart.draw_series(indices2.iter().map(|idx| {
        let points: Vec<_> = idx
            .iter()
            .map(|&i| {
                let p = vertices2[i as usize];
                (p[0] as i32, p[1] as i32, p[2] as i32)
            })
            .collect();
        Polygon::new(points, &BLUE.mix(0.5))
    }))?;

    chart.configure_series_labels().border_style(BLACK).draw()?;

    Ok(())
}

#[test]
fn entry_point() {
    main().unwrap()
}
