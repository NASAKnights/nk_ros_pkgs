import {
    Box,
    Card,
    CardContent,
    CircularProgress,
    Typography,
    LinearProgress,
    Grid,
} from "@mui/material";
import { useEffect, useState } from "react";
import axios from "axios";

interface TempSensor {
    label: string;
    sensor: string;
    temp: number;
}

interface SystemInfo {
    platform: string;
    platform_release: string;
    cpu_percent: number;
    cpu_cores: number;
    cpu_threads: number;
    ram_used: number;
    ram_total: number;
    disk_used: number;
    disk_total: number;
    uptime_seconds: number;
    temps: TempSensor[];
}

export default function SystemMonitorPanel() {
    const [info, setInfo] = useState<SystemInfo | null>(null);

    useEffect(() => {
        const fetchInfo = () => {
            axios
                .get("http://localhost:8080/api/system")
                .then((res) => setInfo(res.data))
                .catch((err) => {
                    console.error("Failed to fetch system info", err);
                    setInfo(null);
                });
        };

        fetchInfo();
        const interval = setInterval(fetchInfo, 5000);
        return () => clearInterval(interval);
    }, []);

    const formatUptime = (seconds: number) => {
        const hours = Math.floor(seconds / 3600);
        const minutes = Math.floor((seconds % 3600) / 60);
        return `${hours}h ${minutes}m`;
    };

    if (!info) {
        return (
            <Card>
                <CardContent>
                    <Typography variant="h6">System Monitor</Typography>
                    <Box display="flex" justifyContent="center" alignItems="center" minHeight="100px">
                        <CircularProgress />
                    </Box>
                </CardContent>
            </Card>
        );
    }

    const ramPercent = Math.round((info.ram_used / info.ram_total) * 100);
    const diskPercent = Math.round((info.disk_used / info.disk_total) * 100);

    return (
        <Card>
            <CardContent>
                <Typography variant="h6" gutterBottom>
                    System Monitor
                </Typography>

                <Typography color="text.secondary" variant="body2" gutterBottom>
                    {info.platform} {info.platform_release} — Uptime: {formatUptime(info.uptime_seconds)}
                </Typography>

                <Box mb={2}>
                    <Typography variant="subtitle2">CPU Usage ({info.cpu_percent}%)</Typography>
                    <LinearProgress variant="determinate" value={info.cpu_percent} />
                </Box>

                <Box mb={2}>
                    <Typography variant="subtitle2">
                        RAM Usage ({info.ram_used} / {info.ram_total} MB)
                    </Typography>
                    <LinearProgress variant="determinate" value={ramPercent} />
                </Box>

                <Box mb={2}>
                    <Typography variant="subtitle2">
                        Disk Usage ({info.disk_used} / {info.disk_total} GB)
                    </Typography>
                    <LinearProgress variant="determinate" value={diskPercent} />
                </Box>
                {info.temps.length > 0 && (
                    <Box mt={2}>
                        <Typography variant="subtitle2" gutterBottom>Temperatures</Typography>
                        {info.temps.map((t, i) => (
                            <Typography key={i} variant="body2">
                                {t.label} / {t.sensor}: {t.temp}°C
                            </Typography>
                        ))}
                    </Box>
                )}
            </CardContent>
        </Card>
    );
}
