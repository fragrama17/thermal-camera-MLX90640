use std::net::TcpListener;
use std::thread;
use serde::Serialize;
// use std::time::Duration;
// use rand::Rng;
use tungstenite::{accept, Message};
use linux_mlx90640::{ThermalCamera, TOT_COLUMNS, TOT_ROWS};

#[derive(Serialize)]
struct ThermalMessage {
    thermalFrame: Vec<Vec<f32>>,
}

fn main() {
    let server = TcpListener::bind("0.0.0.0:8080").unwrap();
    println!("WebSocket server listening on ws://0.0.0.0:8080");

    for stream in server.incoming() {
        thread::spawn(move || {
            let mut websocket = accept(stream.unwrap()).unwrap();
            let mut camera = ThermalCamera::default();
            loop {
                // MOCK
                // Generate a 32x24 matrix of f32 values
                // let mut matrix = vec![0f32; 32 * 24];
                // rand::thread_rng().fill(&mut matrix[..]);

                // Get thermal data from mock service
                let flat_matrix = camera.get_image().unwrap();

                // Convert flat to 2D matrix
                let mut matrix = vec![vec![0.0f32; TOT_COLUMNS]; TOT_ROWS];
                for y in 0..TOT_ROWS {
                    for x in 0..TOT_COLUMNS {
                        matrix[y][x] = flat_matrix[y * TOT_COLUMNS + x];
                    }
                }
                
                println!("{:?}", matrix);

                // Serialize to JSON
                let message = ThermalMessage { thermalFrame: matrix };
                let json = serde_json::to_string(&message).unwrap();

                if let Err(e) = websocket.send(Message::Text(json)) {
                    eprintln!("Client disconnected or error: {}", e);
                    break;
                }

                // MOCK 
                // thread::sleep(Duration::from_secs(1));
            }
        });
    }
}
