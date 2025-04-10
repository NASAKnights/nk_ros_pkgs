import { useState } from "react";
import {
  AppBar,
  Toolbar,
  Typography,
  Drawer,
  List,
  ListItem,
  ListItemButton,
  ListItemText,
  Box
} from "@mui/material";
import Dashboard from "./Dashboard";
import ConfigManager from "./ConfigManager";
import CameraManagement from "./CameraManagement";
import PowerMenu from "./components/PowerMenu";

const drawerWidth = 240;

function App() {
  const [page, setPage] = useState("dashboard");

  const renderPage = () => {
    switch (page) {
      case "dashboard":
        return <Dashboard />;
      case "camera":
        return <CameraManagement />;
      case "config":
        return <ConfigManager />;
      default:
        return <Dashboard />;
    }
  };


  return (
    <Box sx={{ display: "flex" }}>
      {/* Sidebar */}
      <Drawer
        variant="permanent"
        sx={{
          width: drawerWidth,
          flexShrink: 0,
          [`& .MuiDrawer-paper`]: {
            width: drawerWidth,
            boxSizing: "border-box"
          }
        }}
      >
        <Toolbar />
        <List>
          <ListItem disablePadding>
            <ListItemButton onClick={() => setPage("dashboard")}>
              <ListItemText primary="Dashboard" />
            </ListItemButton>
          </ListItem>
          <ListItem disablePadding>
            <ListItemButton onClick={() => setPage("camera")}>
              <ListItemText primary="Camera Viewer" />
            </ListItemButton>
          </ListItem>
          <ListItem disablePadding>
            <ListItemButton onClick={() => setPage("config")}>
              <ListItemText primary="Config Manager" />
            </ListItemButton>
          </ListItem>
        </List>

      </Drawer>

      {/* App Content */}
      <Box component="main" sx={{ flexGrow: 1, p: 3 }}>
        <AppBar position="fixed" sx={{ zIndex: 1201 }}>
          <Toolbar sx={{ display: "flex", justifyContent: "space-between" }}>
            <Typography variant="h6" noWrap component="div">
              ROS 2 Visual Dashboard
            </Typography>
            <PowerMenu />
          </Toolbar>
        </AppBar>
        <Toolbar /> {/* Spacer under AppBar */}
        {renderPage()}
      </Box>
    </Box>
  );
}

export default App;
