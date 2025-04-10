// src/CameraViewer.tsx
import {
    Box,
    Card,
    CardContent,
    Typography,
    Select,
    MenuItem,
    Button,
    Dialog,
    DialogTitle,
    DialogContent,
    TextField,
    DialogActions,
} from "@mui/material";
import { useState, useEffect } from "react";
import axios from "axios";

export default function CameraViewer() {
    const [cameraTopics, setCameraTopics] = useState<string[]>([]);
    const [selectedCamera, setSelectedCamera] = useState<string>("");

    // Calibration state
    const [open, setOpen] = useState(false);
    const [pattern, setPattern] = useState<"chessboard" | "charuco">("chessboard");
    const [rows, setRows] = useState("6");
    const [cols, setCols] = useState("9");
    const [squareSize, setSquareSize] = useState("0.024");
    const [outputFile, setOutputFile] = useState("calibration.yaml");
    const [charucoDict, setCharucoDict] = useState("DICT_5X5_1000");
    const [markerLength, setMarkerLength] = useState("");
    const [calibrating, setCalibrating] = useState(false);
    const [calibrationResult, setCalibrationResult] = useState<string | null>(null);
    const [calibrationTopic, setCalibrationTopic] = useState<string | null>(null);
    const [calibrationActive, setCalibrationActive] = useState(false);
    const [webVideoStarted, setWebVideoStarted] = useState(false);
    const [feedback, setFeedback] = useState<any>(null);
    const host = window.location.hostname;
    const API_BASE = `http://${host}:8080`;
    const STREAM_BASE = `http://${host}:8081`;


    useEffect(() => {
        // Fetch list of camera topics
        axios
            .get(API_BASE + "/api/camera_topics")
            .then((res) => setCameraTopics(res.data.topics))
            .catch(console.error);

        // Only try to start web_video_server once
        if (!webVideoStarted) {
            axios
                .post(API_BASE + "/api/camera/start") // Starts the server, doesn't need a topic
                .then(() => setWebVideoStarted(true))
                .catch((err) => {
                    console.error("Failed to start web video server:", err);
                    setWebVideoStarted(false);
                });
        }
    }, [API_BASE, webVideoStarted]);

    const handleCancelCalibration = async () => {
        try {
            await axios.post(API_BASE + "/api/calibrate/cancel");
            setCalibrationActive(false); // Exit calibration mode
            setCalibrationTopic(null);
        } catch (err) {
            console.error("Failed to cancel calibration:", err);
        }
    };

    const handleCalibrate = async () => {
        setCalibrating(true);
        setCalibrationResult(null);
        setFeedback(null);

        const payload: any = {
            pattern,
            camera_topic: selectedCamera,
            square_size: parseFloat(squareSize),
            rows: parseInt(rows),
            cols: parseInt(cols),
            output_file: outputFile,
        };

        if (pattern === "charuco") {
            payload.charuco_dict = charucoDict;
            if (markerLength) {
                payload.charuco_marker_length = parseFloat(markerLength);
            }
        }

        try {
            await axios.post(API_BASE + "/api/calibrate/intrinsic", payload);
            setCalibrationResult("Calibration started.");
            setCalibrationTopic("/calibration");
            setOpen(false);
            setCalibrationActive(true);

            // Connect to calibration feedback
            const source = new EventSource(API_BASE + "/api/calibrate/feedback");

            source.onmessage = (event) => {
                try {
                    const data = JSON.parse(event.data);
                    setFeedback(data);
                } catch (err) {
                    console.error("Failed to parse feedback:", err);
                }
            };

            source.onerror = (err) => {
                console.error("SSE connection error", err);
                source.close();
            };
        } catch (err: any) {
            setCalibrationResult("Calibration failed: " + (err.response?.data?.detail || err.message));
        }

        setCalibrating(false);
    };

    return (
        <Card>
            <CardContent>
                <Typography variant="h5" gutterBottom>
                    Camera Viewer
                </Typography>
                <Select
                    fullWidth
                    displayEmpty
                    value={selectedCamera}
                    onChange={(e) => setSelectedCamera(e.target.value)}
                    disabled={calibrationActive}
                >
                    <MenuItem value="">-- Select a Camera Topic --</MenuItem>
                    {cameraTopics.map((topic) => (
                        <MenuItem key={topic} value={topic}>
                            {topic}
                        </MenuItem>
                    ))}
                </Select>
                {selectedCamera ? (
                    <Box mt={2}>
                        <Typography gutterBottom>Live feed from: {selectedCamera}</Typography>
                        <img
                            key={calibrating ? calibrationTopic : selectedCamera}
                            src={`${STREAM_BASE}/stream?topic=${(
                                calibrating ? calibrationTopic : selectedCamera
                            )}`}
                            alt="Camera Stream"
                            style={{
                                width: "100%",
                                maxHeight: "500px",
                                objectFit: "contain",
                                borderRadius: 4,
                                display: "block"
                            }}
                        />
                    </Box>
                ) : (
                    <Typography variant="body2" mt={2} color="textSecondary">
                        No camera selected.
                    </Typography>
                )}

                {calibrationActive && (
                    <Box mt={2} display="flex" gap={2}>
                        {feedback && (
                            <Box mt={2}>
                                <Typography>Frames Captured: {feedback.frames_captured}</Typography>
                                <Typography>Offset X: {feedback.x_offset}, Y: {feedback.y_offset}</Typography>
                                <Typography>Area: {feedback.area?.toFixed(2)}</Typography>
                                <Typography>Skew: {feedback.skew_score}</Typography>
                                <Typography>Status: {feedback.reason}</Typography>
                            </Box>
                        )}

                        <Button
                            variant="contained"
                            color="primary"
                            onClick={() => axios.post(API_BASE + "/api/calibrate/capture")}
                        >Capture</Button>
                        <Button
                            variant="contained"
                            color="success"
                            onClick={() => {
                                axios.post(API_BASE + "/api/calibrate/cancel");
                                setCalibrationActive(false);
                                setCalibrationTopic(null);
                            }}
                        >Commit Calibration</Button>
                        <Button
                            variant="outlined"
                            color="error"
                            onClick={() => { handleCancelCalibration(); }}
                        >
                            Cancel
                        </Button>
                    </Box>
                )}

                {selectedCamera && (
                    <>
                        <Box mt={4}>
                            <Button variant="outlined" onClick={() => setOpen(true)}>
                                Run Intrinsic Calibration
                            </Button>
                        </Box>

                        <Dialog open={open} onClose={() => setOpen(false)}>
                            <DialogTitle>Intrinsic Calibration</DialogTitle>
                            <DialogContent>
                                <Select
                                    fullWidth
                                    value={pattern}
                                    onChange={(e) => setPattern(e.target.value as any)}
                                    sx={{ mt: 1, mb: 2 }}
                                >
                                    <MenuItem value="chessboard">Chessboard</MenuItem>
                                    <MenuItem value="charuco">Charuco</MenuItem>
                                </Select>

                                <TextField
                                    fullWidth
                                    label="Rows"
                                    value={rows}
                                    onChange={(e) => setRows(e.target.value)}
                                    margin="dense"
                                />
                                <TextField
                                    fullWidth
                                    label="Cols"
                                    value={cols}
                                    onChange={(e) => setCols(e.target.value)}
                                    margin="dense"
                                />
                                <TextField
                                    fullWidth
                                    label="Square Size (meters)"
                                    value={squareSize}
                                    onChange={(e) => setSquareSize(e.target.value)}
                                    margin="dense"
                                />
                                {pattern === "charuco" && (
                                    <>
                                        <TextField
                                            fullWidth
                                            label="Marker Length (meters)"
                                            value={markerLength}
                                            onChange={(e) => setMarkerLength(e.target.value)}
                                            margin="dense"
                                        />
                                        <TextField
                                            fullWidth
                                            label="Charuco Dictionary"
                                            value={charucoDict}
                                            onChange={(e) => setCharucoDict(e.target.value)}
                                            margin="dense"
                                        />
                                    </>
                                )}
                                <TextField
                                    fullWidth
                                    label="Output File"
                                    value={outputFile}
                                    onChange={(e) => setOutputFile(e.target.value)}
                                    margin="dense"
                                />
                                {calibrating && <Typography mt={2}>Launching calibration...</Typography>}
                                {calibrationResult && <Typography mt={2}>{calibrationResult}</Typography>}
                            </DialogContent>
                            <DialogActions>
                                <Button onClick={() => setOpen(false)} disabled={calibrating}>
                                    Cancel
                                </Button>
                                <Button onClick={handleCalibrate} disabled={calibrating}>
                                    Run
                                </Button>
                            </DialogActions>
                        </Dialog>
                    </>
                )}

            </CardContent>
        </Card>
    );
}
