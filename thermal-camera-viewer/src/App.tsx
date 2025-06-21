import './App.css'
import {useEffect, useState, ReactNode} from "react";
import {scaleSequential} from "d3-scale";
import {interpolateInferno} from "d3-scale-chromatic";

function App() {
    const [socketStatus, setSocketStatus] = useState<ReactNode | null>()

    useEffect(() => {

        const thermalCameraSocket = new WebSocket("ws://10.0.0.113:8080")

        setSocketStatus(<h3>Connecting to ws...</h3>)

        thermalCameraSocket.onopen = ev => {
            console.debug("connection successfully established", ev)
            setSocketStatus(null)
        }

        thermalCameraSocket.onclose = ev => {
            console.debug("received ws closed event", ev)
            setSocketStatus(<h3>Thermal Camera socket not available right now, try again later</h3>)
        }

        thermalCameraSocket.onmessage = ev => {
            drawThermalFrame(JSON.parse(ev.data) as { thermalFrame: number[][] })
        }

        return () => {
            thermalCameraSocket.close()
            console.debug("socket successfully disposed")
        }

    }, [])

    return (
        <>
            <h1>
                <img width={80} src="/thermal-camera.png" alt="thermal camera icon"/>
                Thermal Camera Viewer
            </h1>
            <div style={{
                border: "1px solid #AAAAAA",
            }}>
                <div>
                    {socketStatus}
                </div>
                <canvas style={{
                    width: 480,
                    height: 360,
                }} id="cameraCanvas">
                    Your browser does not support the HTML canvas tag.
                </canvas>
            </div>
        </>
    )
}

export default App

const colorScale = scaleSequential(interpolateInferno).domain([0, 50])

function drawThermalFrame(thermalFrame: {
    thermalFrame: number[][]
}) {
    const canvas = document.getElementById("cameraCanvas") as HTMLCanvasElement;
    if (!canvas) return

    const ctx = canvas.getContext("2d");
    if (!ctx) return

    const totRows = thermalFrame.thermalFrame.length;
    const totCols = thermalFrame.thermalFrame[0].length;

    const pixelHeight = canvas.height / totRows;
    const pixelWidth = canvas.width / totCols;

    for (let row = 0; row < totRows; row++) {
        for (let col = 0; col < totCols; col++) {
            const value = thermalFrame.thermalFrame[row][col]
            ctx.fillStyle = colorScale(value)
            ctx.fillRect(col * pixelWidth, row * pixelHeight, pixelWidth, pixelHeight)
        }
    }
}