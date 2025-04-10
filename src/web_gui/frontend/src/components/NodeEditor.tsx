import {
    Box,
    IconButton,
    MenuItem,
    Select,
    Stack,
    TextField,
    Typography,
    Button
} from "@mui/material";
import DeleteIcon from "@mui/icons-material/Delete";
import { API_BASE, NodeConfig } from "../types";
import { useEffect, useState } from "react";
import axios from "axios";

export default function NodeEditor({
    block,
    onChange,
    onDelete
}: {
    block: NodeConfig;
    onChange: (updated: NodeConfig) => void;
    onDelete: () => void;
}) {
    const [packages, setPackages] = useState<string[]>([]);
    const [executables, setExecutables] = useState<string[]>([]);

    const [newParamKey, setNewParamKey] = useState("");
    const [newRemapKey, setNewRemapKey] = useState("");

    useEffect(() => {
        axios.get(API_BASE + "/api/ros/packages")
            .then(res => setPackages(res.data))
            .catch(console.error);
    }, []);

    useEffect(() => {
        if (!block.package) return;
        axios.get(API_BASE + "/api/ros/executables", { params: { package: block.package } })
            .then(res => setExecutables(res.data))
            .catch(console.error);
    }, [block.package]);

    const updateField = (updates: Partial<NodeConfig>) => {
        onChange({ ...block, ...updates });
    };

    const updateMap = (type: "parameters" | "remaps", key: string, value: string) => {
        onChange({
            ...block,
            [type]: {
                ...(block[type] || {}),
                [key]: value
            }
        });
    };

    const handleAddParam = () => {
        if (newParamKey.trim()) {
            updateMap("parameters", newParamKey.trim(), "");
            setNewParamKey("");
        }
    };

    const handleAddRemap = () => {
        if (newRemapKey.trim()) {
            updateMap("remaps", newRemapKey.trim(), "");
            setNewRemapKey("");
        }
    };

    return (
        <Box border={1} borderRadius={2} borderColor="grey.300" p={2}>
            <Stack spacing={2}>
                <Box display="flex" justifyContent="space-between" alignItems="center">
                    <Typography fontWeight={600}>ROS Node</Typography>
                    <IconButton onClick={onDelete}><DeleteIcon /></IconButton>
                </Box>

                <Select
                    fullWidth
                    displayEmpty
                    value={block.package}
                    onChange={(e) => updateField({ package: e.target.value, executable: "", parameters: {}, remaps: {} })}
                >
                    <MenuItem value="">Select Package</MenuItem>
                    {packages.map((pkg) => (
                        <MenuItem key={pkg} value={pkg}>{pkg}</MenuItem>
                    ))}
                </Select>

                <Select
                    fullWidth
                    displayEmpty
                    value={block.executable}
                    onChange={(e) => updateField({ executable: e.target.value })}
                    disabled={!block.package}
                >
                    <MenuItem value="">Select Executable</MenuItem>
                    {executables.map((exe) => (
                        <MenuItem key={exe} value={exe}>{exe}</MenuItem>
                    ))}
                </Select>

                <TextField label="Node Name" value={block.name} fullWidth onChange={(e) => updateField({ name: e.target.value })} />
                <TextField label="Namespace" value={block.namespace || ""} fullWidth onChange={(e) => updateField({ namespace: e.target.value })} />

                {/* Parameters */}
                <Typography variant="subtitle2">Parameters</Typography>
                {Object.entries(block.parameters || {}).map(([k, v]) => (
                    <TextField
                        key={k}
                        fullWidth
                        label={k}
                        value={v}
                        onChange={(e) => updateMap("parameters", k, e.target.value)}
                        sx={{ mb: 1 }}
                    />
                ))}
                <Box display="flex" gap={1}>
                    <TextField
                        fullWidth
                        label="New Parameter Key"
                        value={newParamKey}
                        onChange={(e) => setNewParamKey(e.target.value)}
                        onKeyDown={(e) => e.key === "Enter" && handleAddParam()}
                    />
                    <Button variant="contained" onClick={handleAddParam}>Add</Button>
                </Box>

                {/* Remaps */}
                <Typography variant="subtitle2" mt={2}>Remaps</Typography>
                {Object.entries(block.remaps || {}).map(([k, v]) => (
                    <TextField
                        key={k}
                        fullWidth
                        label={k}
                        value={v}
                        onChange={(e) => updateMap("remaps", k, e.target.value)}
                        sx={{ mb: 1 }}
                    />
                ))}
                <Box display="flex" gap={1}>
                    <TextField
                        fullWidth
                        label="New Remap Key"
                        value={newRemapKey}
                        onChange={(e) => setNewRemapKey(e.target.value)}
                        onKeyDown={(e) => e.key === "Enter" && handleAddRemap()}
                    />
                    <Button variant="contained" onClick={handleAddRemap}>Add</Button>
                </Box>
            </Stack>
        </Box>
    );
}
