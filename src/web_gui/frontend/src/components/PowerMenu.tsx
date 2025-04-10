import {
    IconButton,
    Menu,
    MenuItem,
    ListItemIcon,
    ListItemText,
    Tooltip,
} from "@mui/material";
import PowerSettingsNewIcon from "@mui/icons-material/PowerSettingsNew";
import RestartAltIcon from "@mui/icons-material/RestartAlt";
import SyncIcon from "@mui/icons-material/Sync";
import SettingsBackupRestoreIcon from "@mui/icons-material/SettingsBackupRestore";
import { useState } from "react";

export default function PowerMenu() {
    const [anchorEl, setAnchorEl] = useState<null | HTMLElement>(null);
    const open = Boolean(anchorEl);

    const handleClick = (event: React.MouseEvent<HTMLElement>) => {
        setAnchorEl(event.currentTarget);
    };
    const handleClose = () => setAnchorEl(null);

    const handleAction = (action: string) => {
        handleClose();
        switch (action) {
            case "restart_node":
                alert("Restarting node... (TODO)");
                break;
            case "restart_container":
                alert("Restarting container... (TODO)");
                break;
            case "restart_device":
                alert("Rebooting device... (TODO)");
                break;
        }
    };

    return (
        <>
            <Tooltip title="Power Menu">
                <IconButton color="inherit" onClick={handleClick}>
                    <PowerSettingsNewIcon />
                </IconButton>
            </Tooltip>
            <Menu anchorEl={anchorEl} open={open} onClose={handleClose}>
                <MenuItem onClick={() => handleAction("restart_node")}>
                    <ListItemIcon><RestartAltIcon fontSize="small" /></ListItemIcon>
                    <ListItemText>Restart Node</ListItemText>
                </MenuItem>
                <MenuItem onClick={() => handleAction("restart_container")}>
                    <ListItemIcon><SyncIcon fontSize="small" /></ListItemIcon>
                    <ListItemText>Restart Container</ListItemText>
                </MenuItem>
                <MenuItem onClick={() => handleAction("restart_device")}>
                    <ListItemIcon><SettingsBackupRestoreIcon fontSize="small" /></ListItemIcon>
                    <ListItemText>Restart Device</ListItemText>
                </MenuItem>
            </Menu>
        </>
    );
}
