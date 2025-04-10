import {
  Box,
  Breadcrumbs,
  Button,
  Card,
  CardContent,
  CardHeader,
  CircularProgress,
  Dialog,
  DialogTitle,
  DialogContent,
  DialogActions,
  Link,
  List,
  ListItem,
  ListItemButton,
  ListItemIcon,
  ListItemText,
  TextField,
  Typography
} from "@mui/material";
import FolderIcon from "@mui/icons-material/Folder";
import DescriptionIcon from "@mui/icons-material/Description";
import NavigateNextIcon from "@mui/icons-material/NavigateNext";
import { useEffect, useState } from "react";
import axios from "axios";
import ConfigEditor from "./ConfigBuilder";
import { API_BASE } from "./types";

interface FileEntry {
  name: string;
  is_dir: boolean;
}

export default function ConfigManager() {
  const [pathStack, setPathStack] = useState<string[]>([]);
  const [entries, setEntries] = useState<FileEntry[]>([]);
  const [loading, setLoading] = useState(false);

  const [fileContent, setFileContent] = useState("");
  const [editingFile, setEditingFile] = useState<string | null>(null);
  const [createOpen, setCreateOpen] = useState(false);

  const currentPath = pathStack.join("/");

  const fetchList = async () => {
    setLoading(true);
    try {
      const res = await axios.get(API_BASE + "/api/configs/list", {
        params: { path: currentPath },
      });
      setEntries(res.data.entries);
      setEditingFile(null);
      setFileContent("");
    } catch (err) {
      console.error("Failed to fetch list", err);
    } finally {
      setLoading(false);
    }
  };

  const handleOpenFolder = (name: string) => {
    setPathStack((prev) => [...prev, name]);
  };

  const handleNavigateUp = (index: number) => {
    setPathStack(pathStack.slice(0, index + 1));
  };

  const handleOpenFile = async (name: string) => {
    try {
      const filePath = [...pathStack, name].join("/");
      const res = await axios.get(API_BASE + "/api/configs/load", {
        params: { path: filePath },
      });
      setEditingFile(name);
      setFileContent(res.data.content);
    } catch (err) {
      console.error("Failed to load file", err);
    }
  };

  const handleSave = async () => {
    const filePath = [...pathStack, editingFile].join("/");
    try {
      await axios.post(API_BASE + "/api/configs/save", {
        path: filePath,
        content: fileContent,
      });
      alert("Saved!");
    } catch (err) {
      console.error("Failed to save file", err);
      alert("Save failed.");
    }
  };

  const handleSaveNewConfig = async (config: any) => {
    const configPath = [...pathStack, `${config.name}.json`].join("/");
    try {
      await axios.post(API_BASE + "/api/configs/save", {
        path: configPath,
        content: JSON.stringify(config, null, 2),
      });
      alert("Config saved!");
      setCreateOpen(false);
      fetchList();
    } catch (err) {
      console.error("Failed to save new config", err);
      alert("Failed to save config.");
    }
  };

  useEffect(() => {
    fetchList();
  }, [pathStack]);

  return (
    <Card>
      <CardHeader
        title="Configuration Manager"
        action={
          <Button variant="contained" onClick={() => setCreateOpen(true)}>
            New Config
          </Button>
        }
      />
      <CardContent>
        <Breadcrumbs separator={<NavigateNextIcon fontSize="small" />} sx={{ mb: 2 }}>
          <Link
            color={pathStack.length === 0 ? "text.primary" : "inherit"}
            underline="hover"
            onClick={() => setPathStack([])}
            sx={{ cursor: "pointer" }}
          >
            configs
          </Link>
          {pathStack.map((dir, index) => (
            <Link
              key={index}
              underline="hover"
              color={index === pathStack.length - 1 ? "text.primary" : "inherit"}
              onClick={() => handleNavigateUp(index)}
              sx={{ cursor: "pointer" }}
            >
              {dir}
            </Link>
          ))}
        </Breadcrumbs>

        {loading ? (
          <Box display="flex" justifyContent="center" alignItems="center" minHeight="100px">
            <CircularProgress />
          </Box>
        ) : (
          <>
            <List dense>
              {entries.map((entry) => (
                <ListItem key={entry.name} disablePadding>
                  <ListItemButton
                    onClick={() =>
                      entry.is_dir ? handleOpenFolder(entry.name) : handleOpenFile(entry.name)
                    }
                  >
                    <ListItemIcon>
                      {entry.is_dir ? <FolderIcon /> : <DescriptionIcon />}
                    </ListItemIcon>
                    <ListItemText primary={entry.name} />
                  </ListItemButton>
                </ListItem>
              ))}
            </List>

            {editingFile && (
              <Box mt={4}>
                <Typography variant="h6" gutterBottom>
                  Editing: {editingFile}
                </Typography>
                <TextField
                  fullWidth
                  multiline
                  minRows={12}
                  value={fileContent}
                  onChange={(e) => setFileContent(e.target.value)}
                />
                <Box mt={2}>
                  <Button variant="contained" onClick={handleSave}>
                    Save
                  </Button>
                </Box>
              </Box>
            )}
          </>
        )}
      </CardContent>

      {/* Dialog for new config creation */}
      <Dialog open={createOpen} onClose={() => setCreateOpen(false)} maxWidth="md" fullWidth>
        <DialogTitle>Create New Launch Config</DialogTitle>
        <DialogContent>
          <ConfigEditor onSave={handleSaveNewConfig} />
        </DialogContent>
        <DialogActions>
          <Button onClick={() => setCreateOpen(false)}>Cancel</Button>
        </DialogActions>
      </Dialog>
    </Card>
  );
}
