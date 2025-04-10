import { useState, useEffect } from "react";
import {
  Box,
  Card,
  CardContent,
  Typography,
  Grid,
  CircularProgress
} from "@mui/material";
import axios from "axios";
import SystemMonitorPanel from "./components/SystemMonitorPanel";

export default function Dashboard() {
  const [nodes, setNodes] = useState<string[]>([]);
  const [topics, setTopics] = useState<string[]>([]);
  const [systemInfo, setSystemInfo] = useState<any | null>(null);

  useEffect(() => {
    axios.get("http://localhost:8080/api/nodes")
      .then((res) => setNodes(res.data.nodes))
      .catch(console.error);

    axios.get("http://localhost:8080/api/topics")
      .then((res) => setTopics(res.data.topics))
      .catch(console.error);

    const fetchSystemInfo = () => {
      axios.get("http://localhost:8080/api/system")
        .then(res => setSystemInfo(res.data))
        .catch(err => {
          console.error("Failed to fetch system info", err);
          setSystemInfo(null);
        });
    };

    fetchSystemInfo();
    const interval = setInterval(fetchSystemInfo, 5000);
    return () => clearInterval(interval);
  }, []);

  const formatUptime = (seconds: number) => {
    const hours = Math.floor(seconds / 3600);
    const minutes = Math.floor((seconds % 3600) / 60);
    return `${hours}h ${minutes}m`;
  };

  return (
    <Box>
      <Typography variant="h4" gutterBottom>ROS Dashboard</Typography>

      <Grid container spacing={3}>
        {/* ROS Nodes Panel */}
        <Grid size={6}>
          <Card>
            <CardContent>
              <Typography variant="h6" gutterBottom>ROS Nodes</Typography>
              {nodes.length === 0 ? (
                <Typography>No nodes found</Typography>
              ) : (
                nodes.map((node) => (
                  <Typography key={node} variant="body2">{node}</Typography>
                ))
              )}
            </CardContent>
          </Card>
        </Grid>

        {/* Topics Panel */}
        <Grid size={6}>
          <Card>
            <CardContent>
              <Typography variant="h6" gutterBottom>ROS Topics</Typography>
              {topics.length === 0 ? (
                <Typography>No topics found</Typography>
              ) : (
                topics.map((topic) => (
                  <Typography key={topic} variant="body2">{topic}</Typography>
                ))
              )}
            </CardContent>
          </Card>
        </Grid>

        {/* System Monitor Panel */}
        <Grid size={12}>
          <SystemMonitorPanel />
        </Grid>
      </Grid>
    </Box>
  );
}
