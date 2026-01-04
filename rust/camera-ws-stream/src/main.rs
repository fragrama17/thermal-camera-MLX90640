use std::net::TcpListener;
use std::thread;
use linux_embedded_hal::{Delay, I2cdev};
use serde::Serialize;
use serde_json::to_string;
use tungstenite::{accept, Message};
use mlx90640_hal::{Mlx90640, TOT_COLUMNS, TOT_ROWS};

#[derive(Serialize)]
struct ThermalMessage {
    thermalFrame: Vec<Vec<f32>>,
}

fn main() {
    let server = TcpListener::bind("0.0.0.0:8080").unwrap();
    println!("WebSocket server listening on ws://0.0.0.0:8080");

    for stream in server.incoming() {
        thread::spawn(|| {
            let mut websocket = accept(stream.unwrap()).unwrap();
            let i2c = I2cdev::new("/dev/i2c-1").unwrap();
            let mut camera = Mlx90640::new(0x33, i2c, Delay);
            loop {
                let flat_matrix = camera.get_image().unwrap();

                let mut matrix = vec![vec![0.0f32; TOT_COLUMNS]; TOT_ROWS];
                for y in 0..TOT_ROWS {
                    for x in 0..TOT_COLUMNS {
                        matrix[y][x] = flat_matrix[y * TOT_COLUMNS + x];
                    }
                }
                
                let message = ThermalMessage { thermalFrame: matrix };
                let json = to_string(&message).unwrap();

                if let Err(e) = websocket.send(Message::Text(json)) {
                    eprintln!("Client disconnected or error: {}", e);
                    break;
                }
            }
        });
    }
}
